#include "s5_avoid.h"
#include "../subsystem2/motion_config.h"

#define FRONT_MARGIN_MM 15u
#define MAX_LATERAL_OFFSET_MM 400u
#define NESTED_PASS_MARGIN_MM 130u
#define NESTED_PASS_LIMIT_MM 600u
#define MIN_LEFT_STEP_MM 20u
#define MOTION_SETTLE_MS 260u
#define PIVOT_PROOF_MAX_AGE_MS 1500u
#define SCAN_TIMEOUT_MS 14000u
#define POST_SCAN_RANGE_TIMEOUT_MS 1500u
#define RESCAN_INTERVAL_MS 1000u
#define RESCAN_RANGE_CHANGE_MM 30u
#define PHASE_TIMEOUT_MS 9000u
#define LONG_MOVE_TIMEOUT_MS 15000u
#define MAX_FALL_TIMEOUT_PAUSES 3u
#define RETURN_STOP_MM S5_TURN_CLEARANCE_MM
#define RETURN_RESERVE_MM (RETURN_STOP_MM + 10u)
#define ARC_FORWARD_MM ((S2_ARC_RADIUS_MM * 866u + 999u) / 1000u)
#define ARC_SIDEWAYS_MM (S2_ARC_RADIUS_MM / 2u)
#define ARC_ENTRY_MM (ARC_FORWARD_MM + S5_TURN_CLEARANCE_MM)
#define DEPARTURE_DIAGONAL_MIN_MM 20u
#define EDGE_OFFSET_DEADBAND_MM 20u

_Static_assert(S2_ARC_ANGLE_DEG == 60u, "Return offset projections use a 60 degree arc");
_Static_assert(S5_FOLLOW_CLEAR_MM + S5_PASS_UNCERTAIN_DEPTH_MM + S5_PASS_BODY_ALLOWANCE_MM <= S5_MAX_PASS_MM,
               "The conservative default depth must fit the bypass budget");
_Static_assert(S5_CLEAR_MM + NESTED_PASS_MARGIN_MM <= NESTED_PASS_LIMIT_MM,
               "The maximum nested pass must fit the nested bypass budget");
_Static_assert(S5_MAX_PASS_MM < 1024u, "Total pass distance must fit the 10-bit fields");
_Static_assert(S5_CHASSIS_WIDTH_MM >= EFFECTIVE_TRACK_MM, "Include the outer wheel edges in chassis width");
_Static_assert(S5_OUTWARD_LIMIT_MM < 64u && S5_OUTWARD_LIMIT_MM % S5_OUTWARD_STEP_MM == 0,
               "The outward correction must fit whole bounded steps");
_Static_assert(MAX_LATERAL_OFFSET_MM <= 511u, "Lateral offset must fit in 9 bits");
_Static_assert(MAX_LATERAL_OFFSET_MM + S5_SEARCH_MARGIN_MM <= S5_MAX_LEG_MM && S5_MAX_LEG_MM <= 1023u,
               "Movement and tracked offsets must fit in 10 bits");
_Static_assert(S5_NESTED_LIMIT < 4u, "Nesting depth must fit in two bits");

static bool at_or_after(uint32_t time, uint32_t reference)
{
    return (uint32_t)(time - reference) < UINT32_C(0x80000000);
}

static S5_AVOID_OUTPUT go(S5_AVOID *state, S5_PHASE phase, uint32_t now)
{
    state->phase = phase;
    state->phase_at = now;
    state->line_samples = 0;
    state->last_line_at = now;
    state->last_range_at = now;
    state->range_seen = false;
    state->echo_pauses = 0;
    state->leg_progress_mm = 0;
    state->issued_mm = state->waiting_echo = state->short_approach = 0;
    return (S5_AVOID_OUTPUT){.action = S5_WAIT};
}

static bool confirmed_forward(S5_AVOID *state, const S5_AVOID_INPUT *in)
{
    if (in->range_mm == S5_OPEN_RANGE) return true; /* The range gate already confirmed two pings. */
    if (in->range_at == state->last_range_at) return false;
    state->last_range_at = in->range_at;
    bool confirmed = state->range_seen;
    state->range_seen = true;
    return confirmed;
}

void s5_avoid_abort(S5_AVOID *state, S5_FAILURE reason, uint32_t now)
{
    if (state->phase == S5_FAILED) return;
    state->failure = reason;
    state->failed_phase = state->phase;
    go(state, S5_FAILED, now);
}

static S5_AVOID_OUTPUT fail(S5_AVOID *state, S5_FAILURE reason, uint32_t now)
{
    s5_avoid_abort(state, reason, now);
    return (S5_AVOID_OUTPUT){.action = S5_STOP};
}

void s5_avoid_begin(S5_AVOID *state, uint32_t now)
{
    *state = (S5_AVOID){.phase = S5_PROFILE_PATH, .phase_at = now};
}

unsigned s5_reentry_detection_mm(unsigned offset_mm, unsigned sensor_to_axle_mm)
{
    unsigned margin = S5_SEARCH_MARGIN_MM + sensor_to_axle_mm;
    return offset_mm > margin ? offset_mm - margin : 0u;
}

static unsigned diagonal_distance_mm(unsigned sideways_mm)
{
    return (sideways_mm * 1000u + 865u) / 866u;
}

static unsigned departure_diagonal_mm(unsigned offset_mm)
{
    return diagonal_distance_mm(offset_mm - 2u * ARC_SIDEWAYS_MM);
}

static unsigned departure_forward_mm(unsigned offset_mm)
{
    return 2u * ARC_FORWARD_MM + (departure_diagonal_mm(offset_mm) + 1u) / 2u;
}

unsigned s5_return_tape_distance_mm(const S5_AVOID *state)
{
    if (!state->rounded_return) return state->offset_mm;
    return diagonal_distance_mm(state->offset_mm - ARC_SIDEWAYS_MM);
}

unsigned s5_planned_pass_mm(const S5_PROFILE *profile, unsigned forward_mm)
{
    /* The width bounds the box depth for this course. Reserve enough travel
       for the rear of the chassis to get beyond the box before turning. */
    unsigned depth = profile->width_mm;
    if (!depth || depth == S5_UNKNOWN_WIDTH) {
        depth = S5_PASS_UNCERTAIN_DEPTH_MM;
    } else if (profile->width_status != S5_WIDTH_OK) {
        depth += S5_UNCERTAIN_EDGE_MM;
        if (depth < S5_PASS_UNCERTAIN_DEPTH_MM) depth = S5_PASS_UNCERTAIN_DEPTH_MM;
    }
    unsigned gap = profile->nearest_mm;
    if (s5_range_measured(forward_mm) && forward_mm > gap) gap = forward_mm;
    return gap + depth + S5_PASS_BODY_ALLOWANCE_MM;
}

static unsigned bypass_offset(int lateral_extent, bool edge_valid)
{
    if (!edge_valid) {
        /* A partial profile retains its last observed edge and an uncertainty gap. */
        unsigned extent = lateral_extent > 0 ? (unsigned)lateral_extent : 0u;
        return S5_SIDE_ALLOWANCE_MM + extent + S5_UNCERTAIN_EDGE_MM;
    }
    int offset = lateral_extent + (int)S5_SIDE_ALLOWANCE_MM;
    /* Even the shortest sidestep must leave the forward-mounted IR sensors
       on the approach side of the tape when the car turns to cross it. */
    return offset > (int)S5_MIN_SIDE_SHIFT_MM ? (unsigned)offset : S5_MIN_SIDE_SHIFT_MM;
}

static unsigned side_range(unsigned heading, unsigned background, unsigned offset)
{
    /* Prefer the 45-degree sample. A farther background beyond the edge can
       guide the turn when that bearing still intersects the obstacle. */
    unsigned required = offset + FRONT_MARGIN_MM + S5_MANEUVER_STOP_MM;
    if (heading <= S5_MANEUVER_STOP_MM) return heading;
    if (s5_range_available(heading) && heading >= required) return heading;
    if (s5_range_available(background)) return background;
    return heading;
}

S5_BYPASS s5_plan_bypass(const S5_PROFILE *profile)
{
    S5_BYPASS plan = {.failure = S5_FAIL_TARGET};
    if (!s5_range_measured(profile->nearest_mm) || profile->nearest_mm > S5_FOLLOW_CLEAR_MM)
        return plan;

    unsigned left_offset = bypass_offset(-profile->left_edge_mm, profile->left_edge_valid);
    unsigned right_offset = bypass_offset(profile->right_edge_mm, profile->right_edge_valid);
    unsigned left_path = side_range(profile->left_path_mm, profile->left_mm, left_offset);
    unsigned right_path = side_range(profile->right_path_mm, profile->right_mm, right_offset);
    bool left_room = !profile->left_clipped && left_offset <= MAX_LATERAL_OFFSET_MM;
    bool right_room = !profile->right_clipped && right_offset <= MAX_LATERAL_OFFSET_MM;
    bool left = left_room && s5_range_available(left_path) &&
                left_path >= left_offset + FRONT_MARGIN_MM + S5_MANEUVER_STOP_MM;
    bool right = right_room && s5_range_available(right_path) &&
                 right_path >= right_offset + FRONT_MARGIN_MM + S5_MANEUVER_STOP_MM;
    /* If neither reading supports the planned distance, try left unless
       there is evidence that the left side is already too close. */
    bool probe_left = !left && !right && left_room &&
                      (!s5_range_available(left_path) || left_path > S5_MANEUVER_STOP_MM);
    if (!left && !right && !probe_left) {
        if ((left_room && !s5_range_available(left_path)) ||
            (right_room && !s5_range_available(right_path))) plan.failure = S5_FAIL_ECHO;
        else if (profile->left_clipped && profile->right_clipped) plan.failure = S5_FAIL_BOTH_LIMITS;
        else if (profile->left_clipped) plan.failure = S5_FAIL_LEFT_LIMIT;
        else if (profile->right_clipped) plan.failure = S5_FAIL_RIGHT_LIMIT;
        else plan.failure = left_room || right_room ? S5_FAIL_SIDE : S5_FAIL_ROOM;
        return plan;
    }
    /* Verify turning room here. The controller checks live range again before moving. */
    if (profile->front_mm < S5_DEPARTURE_CLEARANCE_MM) {
        plan.failure = S5_FAIL_ROOM;
        return plan;
    }

    /* Within the scan's uncertainty use spare clearance as the tiebreak.
       A clearly off-centre box should take the shorter of two usable routes. */
    bool use_left = probe_left || (left && (!right || left_path - left_offset >= right_path - right_offset));
    if (left && right && profile->left_edge_valid && profile->right_edge_valid) {
        if (left_offset + EDGE_OFFSET_DEADBAND_MM < right_offset) use_left = true;
        else if (right_offset + EDGE_OFFSET_DEADBAND_MM < left_offset) use_left = false;
    }
    plan.probe_left = probe_left;
    plan.side = use_left ? S5_LEFT : S5_RIGHT;
    plan.failure = S5_FAIL_NONE;
    plan.offset_mm = use_left ? left_offset : right_offset;
    return plan;
}

static unsigned leg_distance(const S5_AVOID *state, const S5_AVOID_INPUT *in)
{
    switch (state->phase) {
    case S5_SHIFT_OUT:
        if (state->rounded_departure) return departure_diagonal_mm(state->offset_mm);
        return state->widening ? S5_OUTWARD_STEP_MM : state->offset_mm;
    case S5_PASS: return state->next_pass_mm;
    case S5_FIND_LINE: return s5_return_tape_distance_mm(state) + (state->depth ? 0u : S5_SEARCH_MARGIN_MM);
    case S5_CENTER_LINE: return in->sensor_to_axle_mm;
    default: return 0;
    }
}

static unsigned accepted_progress(const S5_AVOID *state, const S5_AVOID_INPUT *in)
{
    if (in->motion != S5_MOTION_PENDING) return 0u;
    return in->travel_mm < state->issued_mm ? in->travel_mm : state->issued_mm;
}

static S5_AVOID_OUTPUT avoid_on_leg(S5_AVOID *state, const S5_AVOID_INPUT *in)
{
    if (state->depth == S5_NESTED_LIMIT) return fail(state, S5_FAIL_CLOSE, in->now);
    unsigned travelled = accepted_progress(state, in);
    unsigned progress = state->leg_progress_mm + travelled;
    if (progress > S5_MAX_LEG_MM) return fail(state, S5_FAIL_MOTION, in->now);

    unsigned line_goal = state->phase == S5_CENTER_LINE ? in->sensor_to_axle_mm : state->offset_mm;
    state->interrupted[state->depth++] = (S5_INTERRUPTED_LEG){
        .phase = state->phase, .side = state->side, .offset_mm = state->offset_mm,
        .next_pass_mm = state->next_pass_mm, .passed_mm = state->passed_mm,
        .progress_mm = progress, .line_goal_mm = line_goal,
        .additional_outward_mm = state->additional_outward_mm,
        .rolling_route = state->rolling_route,
    };
    state->side = S5_NO_SIDE;
    state->scan_retried = state->probe_left = false;
    state->reverse_available = travelled >= S5_REVERSE_MM;
    state->reverse_long = travelled >= S5_LONG_REVERSE_MM;
    state->offset_mm = state->next_pass_mm = state->passed_mm = 0;
    state->widening = false;
    state->rounded_departure = false;
    state->rounded_return = false;
    state->rolling_route = false;
    state->additional_outward_mm = 0;
    go(state, S5_PROFILE_PATH, in->now);
    return (S5_AVOID_OUTPUT){.action = S5_STOP};
}

static bool returning_to_line(S5_PHASE phase)
{
    return phase == S5_FIND_LINE || phase == S5_CENTER_LINE;
}

static bool passed_parent_line(const S5_AVOID *state)
{
    const S5_INTERRUPTED_LEG *leg = &state->interrupted[state->depth - 1u];
    return returning_to_line(leg->phase) && leg->progress_mm + state->passed_mm > leg->line_goal_mm;
}

/* Credit travel parallel to the interrupted leg. The two sideways moves have
   already put the car back on that leg when a nested bypass finishes. */
static S5_AVOID_OUTPUT resume_leg(S5_AVOID *state, const S5_AVOID_INPUT *in, bool bypassed)
{
    bool face_back = bypassed && passed_parent_line(state);
    S5_INTERRUPTED_LEG leg = state->interrupted[--state->depth];
    unsigned progress = leg.progress_mm + (bypassed ? state->passed_mm : 0u);
    if (progress > S5_MAX_LEG_MM) return fail(state, S5_FAIL_MOTION, in->now);
    state->side = leg.side;
    state->offset_mm = leg.offset_mm;
    state->next_pass_mm = leg.next_pass_mm;
    state->passed_mm = leg.passed_mm;
    state->additional_outward_mm = leg.additional_outward_mm;
    state->rolling_route = leg.rolling_route;
    state->reverse_available = state->reverse_long = state->scan_retried = state->probe_left = false;
    S5_PHASE phase = leg.phase;
    if (bypassed && returning_to_line(phase)) {
        /* Resume tape search from the side of the estimated crossing that we reached. */
        if (face_back) state->side = leg.side == S5_LEFT ? S5_RIGHT : S5_LEFT;
        unsigned offset = face_back ? progress - leg.line_goal_mm : leg.line_goal_mm - progress;
        if (offset + S5_SEARCH_MARGIN_MM > S5_MAX_LEG_MM)
            return fail(state, S5_FAIL_LINE, in->now);
        state->offset_mm = offset;
        progress = 0;
        phase = S5_FIND_LINE;
    }
    go(state, phase, in->now);
    state->leg_progress_mm = progress;
    return (S5_AVOID_OUTPUT){.action = S5_WAIT};
}

static S5_AVOID_OUTPUT finish_forward_leg(S5_AVOID *state, const S5_AVOID_INPUT *in)
{
    switch (state->phase) {
    case S5_SHIFT_OUT: {
        if (state->rounded_departure) return go(state, S5_ALIGN_OUTSIDE, in->now);
        unsigned offset = state->leg_progress_mm;
        if (state->widening) offset += state->offset_mm;
        if (offset + S5_SEARCH_MARGIN_MM > S5_MAX_LEG_MM)
            return fail(state, S5_FAIL_LINE, in->now);
        state->offset_mm = offset;
        return go(state, S5_ALIGN_OUTSIDE, in->now);
    }
    case S5_PASS: {
        unsigned passed = state->passed_mm + state->leg_progress_mm;
        if (passed > S5_MAX_LEG_MM) return fail(state, S5_FAIL_MOTION, in->now);
        state->passed_mm = passed;
        /* Leave room for the forward-mounted IR sensor if a nested bypass
           ends very close to the true line. Continue past it, then face back. */
        if (state->depth == 1u && returning_to_line(state->interrupted[0].phase)) {
            const S5_INTERRUPTED_LEG *leg = &state->interrupted[0];
            unsigned position = leg->progress_mm + passed;
            unsigned margin = in->sensor_to_axle_mm + S5_SEARCH_MARGIN_MM;
            unsigned far_side = leg->line_goal_mm + margin;
            if (position + margin > leg->line_goal_mm && position < far_side) {
                unsigned extra = far_side - position;
                if (passed + extra > NESTED_PASS_LIMIT_MM) return fail(state, S5_FAIL_LINE, in->now);
                state->next_pass_mm = extra;
                return go(state, S5_PASS, in->now);
            }
        }
        state->rounded_return = state->rolling_route;
        return go(state, S5_TURN_IN, in->now);
    }
    case S5_FIND_LINE:
        return state->depth ? go(state, S5_ALIGN_LINE, in->now) : fail(state, S5_FAIL_LINE, in->now);
    case S5_CENTER_LINE: return go(state, S5_ALIGN_LINE, in->now);
    default: return fail(state, S5_FAIL_MOTION, in->now);
    }
}

static S5_AVOID_OUTPUT wait_for_path(S5_AVOID *state, S5_FAILURE reason, const S5_AVOID_INPUT *in)
{
    state->failure = reason;
    state->failed_phase = state->phase;
    state->blocked_range_mm = S5_UNKNOWN_RANGE;
    if (in->range_valid && in->now - in->range_at <= S5_RANGE_MAX_AGE_MS &&
        in->range_mm <= (state->depth ? S5_CLEAR_MM : S5_FOLLOW_CLEAR_MM))
        state->blocked_range_mm = in->range_mm;
    return go(state, S5_WAIT_PATH, in->now);
}

static void clear_path_failure(S5_AVOID *state)
{
    state->failure = S5_FAIL_NONE;
    state->failed_phase = S5_OFF;
}

S5_AVOID_OUTPUT s5_avoid_step(S5_AVOID *state, const S5_AVOID_INPUT *in)
{
    S5_PHASE phase = state->phase;
    S5_AVOID_OUTPUT wait = {.action = S5_WAIT};
    if (phase == S5_OFF || phase == S5_COMPLETE || phase == S5_FAILED) return wait;

    uint32_t elapsed = in->now - state->phase_at;
    unsigned clear_mm = state->depth ? S5_CLEAR_MM : S5_FOLLOW_CLEAR_MM;
    bool range_fresh = in->range_valid && in->now - in->range_at <= S5_RANGE_MAX_AGE_MS;
    bool proof_fresh = in->profile_valid && in->profile.pivot_confirmed &&
                       in->now - in->profile_at <= PIVOT_PROOF_MAX_AGE_MS;
    if (in->motion == S5_MOTION_FAILED) return fail(state, S5_FAIL_MOTION, in->now);
    if (phase == S5_WAIT_PATH) {
        /* The scan task keeps ranging at 90 degrees while stopped. A new
           sweep can begin once a current forward range justifies it. */
        if (!in->stopped) return wait;
        if (in->retry_scan) {
            clear_path_failure(state);
            state->scan_retried = true;
            return go(state, S5_PROFILE_PATH, in->now);
        }
        if (!range_fresh || !at_or_after(in->range_at, state->phase_at)) {
            state->range_seen = false;
            return wait;
        }
        if (in->range_mm > clear_mm) {
            if (!confirmed_forward(state, in)) return wait;
            clear_path_failure(state);
            return state->depth ? resume_leg(state, in, false) : go(state, S5_COMPLETE, in->now);
        }
        state->range_seen = false;
        /* Allow one automatic retry if the forward echo returns or the
           measured clearance improves. A clear path can always resume above. */
        unsigned blocked = state->blocked_range_mm;
        if (state->scan_retried || state->failure == S5_FAIL_BOTH_LIMITS ||
            (s5_range_measured(blocked) && in->range_mm < blocked + RESCAN_RANGE_CHANGE_MM)) return wait;
        if (elapsed >= RESCAN_INTERVAL_MS) {
            clear_path_failure(state);
            state->scan_retried = true;
            return go(state, S5_PROFILE_PATH, in->now);
        }
        return wait;
    }
    bool forward = phase == S5_SHIFT_OUT || phase == S5_PASS ||
                   phase == S5_FIND_LINE || phase == S5_CENTER_LINE;
    bool returning = !state->depth && returning_to_line(phase);
    bool arcing = (state->rounded_departure && (phase == S5_TURN_OUT || phase == S5_ALIGN_OUTSIDE)) ||
                  (state->rounded_return && (phase == S5_TURN_IN || phase == S5_ALIGN_LINE));
    bool moving = in->motion == S5_MOTION_QUEUED || in->motion == S5_MOTION_PENDING;
    bool probing = phase == S5_TURN_OUT && state->probe_left && !state->rolling_route;
    if (state->waiting_echo) {
        if (elapsed >= PHASE_TIMEOUT_MS) return fail(state, S5_FAIL_ECHO, in->now);
        if (!range_fresh) state->range_seen = false;
        if (in->stopped && range_fresh && elapsed >= MOTION_SETTLE_MS &&
            at_or_after(in->range_at, state->phase_at + MOTION_SETTLE_MS) && confirmed_forward(state, in)) {
            state->waiting_echo = false;
            state->phase_at = in->now - MOTION_SETTLE_MS;
        }
        return wait;
    }
    bool fall_timeout_on_straight = forward && in->echo_fall_timeout;
    if (!range_fresh && (in->echo_quiet || fall_timeout_on_straight) &&
        ((forward && in->motion != S5_MOTION_DONE) ||
         (in->motion == S5_MOTION_NONE && phase != S5_PROFILE_PATH && phase != S5_BACK_UP &&
          !(probing && proof_fresh)))) {
        if (fall_timeout_on_straight) {
            if (state->echo_pauses >= MAX_FALL_TIMEOUT_PAUSES) return fail(state, S5_FAIL_ECHO, in->now);
            ++state->echo_pauses;
        }
        unsigned progress = state->leg_progress_mm + accepted_progress(state, in);
        if (progress > S5_MAX_LEG_MM) return fail(state, S5_FAIL_MOTION, in->now);
        state->leg_progress_mm = progress;
        state->issued_mm = state->short_approach = 0;
        state->waiting_echo = true;
        state->range_seen = false;
        state->last_range_at = in->now;
        state->phase_at = in->now;
        return (S5_AVOID_OUTPUT){.action = moving ? S5_STOP : S5_WAIT};
    }
    unsigned timeout = PHASE_TIMEOUT_MS;
    if (phase == S5_PROFILE_PATH) timeout = SCAN_TIMEOUT_MS;
    else if (state->issued_mm > MAX_LATERAL_OFFSET_MM + S5_SEARCH_MARGIN_MM) timeout = LONG_MOVE_TIMEOUT_MS;
    if (elapsed >= timeout) {
        S5_FAILURE reason = phase == S5_CHECK_LINE && range_fresh ? S5_FAIL_LINE :
                            phase == S5_PROFILE_PATH && !in->profile_valid ? S5_FAIL_SCAN_TIMEOUT :
                            !range_fresh ? S5_FAIL_ECHO : S5_FAIL_PHASE_TIMEOUT;
        if (phase == S5_PROFILE_PATH ||
            (((phase == S5_TURN_OUT && !state->widening) || phase == S5_BACK_UP) &&
             in->motion == S5_MOTION_NONE))
            return wait_for_path(state, reason, in);
        return fail(state, reason, in->now);
    }
    if (phase == S5_PROFILE_PATH) {
        /* The scan belongs to this stop and has completed with the servo centred. */
        if (!in->profile_valid || in->profile_at == state->phase_at ||
            !at_or_after(in->profile_at, state->phase_at) || !in->stopped) return wait;
        if (range_fresh && in->range_mm > clear_mm) {
            if (s5_range_measured(in->profile.nearest_mm) && !confirmed_forward(state, in)) return wait;
            clear_path_failure(state);
            return state->depth ? resume_leg(state, in, false) : go(state, S5_COMPLETE, in->now);
        }
        state->range_seen = false;
        S5_PROFILE profile = in->profile;
        if (range_fresh && in->range_mm < profile.front_mm) profile.front_mm = in->range_mm;
        S5_BYPASS plan = s5_plan_bypass(&profile);
        if (s5_range_available(profile.nearest_mm) && !range_fresh &&
            !(state->depth && plan.probe_left && proof_fresh)) {
            if (in->now - in->profile_at >= POST_SCAN_RANGE_TIMEOUT_MS)
                return wait_for_path(state, S5_FAIL_ECHO, in);
            return wait;
        }
        if (plan.side != S5_NO_SIDE) {
            unsigned pass = state->depth ? profile.nearest_mm + NESTED_PASS_MARGIN_MM :
                            s5_planned_pass_mm(&profile, in->range_mm);
            if (pass > S5_MAX_PASS_MM) return wait_for_path(state, S5_FAIL_ROOM, in);
            clear_path_failure(state);
            state->side = plan.side;
            state->probe_left = plan.probe_left;
            state->offset_mm = plan.offset_mm;
            state->next_pass_mm = pass;
            state->rolling_route = state->rounded_departure = !state->depth;
            if (state->rolling_route) {
                unsigned minimum_offset = 2u * ARC_SIDEWAYS_MM + DEPARTURE_DIAGONAL_MIN_MM;
                if (state->offset_mm < minimum_offset) state->offset_mm = minimum_offset;
                unsigned advance = departure_forward_mm(state->offset_mm);
                if (state->next_pass_mm < advance) state->next_pass_mm = advance;
            }
            return go(state, S5_TURN_OUT, in->now);
        }
        S5_FAILURE reason = plan.failure;
        if (!state->scan_retried && (reason == S5_FAIL_TARGET || reason == S5_FAIL_ECHO)) {
            state->scan_retried = true;
            go(state, S5_PROFILE_PATH, in->now);
            return (S5_AVOID_OUTPUT){.action = S5_RESCAN};
        }
        bool clipped = reason >= S5_FAIL_LEFT_LIMIT;
        /* Change vantage point only if the tracked segment permits retracing and
           a clipped target should still be in profiling range after the move. */
        bool retrace_in_range = range_fresh && in->range_mm <= clear_mm - S5_REVERSE_MM;
        bool on_retrace = state->depth || (in->line_centered && !in->line_junction);
        if (state->reverse_available && range_fresh && on_retrace && (!clipped || retrace_in_range)) {
            state->reverse_available = false;
            state->reverse_long = state->reverse_long && reason == S5_FAIL_ROOM &&
                                  profile.front_mm <= clear_mm - S5_LONG_REVERSE_MM &&
                                  range_fresh && in->range_mm <= clear_mm - S5_LONG_REVERSE_MM;
            return go(state, S5_BACK_UP, in->now);
        }
        return wait_for_path(state, range_fresh ? reason : S5_FAIL_ECHO, in);
    }

    if (phase == S5_BACK_UP) {
        if (!state->depth && (in->line_junction || !in->line_centered))
            return fail(state, S5_FAIL_LINE, in->now);
        if (!range_fresh)
            return in->motion == S5_MOTION_NONE ? wait_for_path(state, S5_FAIL_ECHO, in) :
                                                  fail(state, S5_FAIL_ECHO, in->now);
        unsigned reverse_mm = state->reverse_long ? S5_LONG_REVERSE_MM : S5_REVERSE_MM;
        if (in->motion == S5_MOTION_DONE) {
            if (state->depth) state->interrupted[state->depth - 1u].progress_mm -= reverse_mm;
            return go(state, S5_PROFILE_PATH, in->now);
        }
        if (in->motion != S5_MOTION_NONE || !in->stopped || elapsed < MOTION_SETTLE_MS ||
            !at_or_after(in->range_at, state->phase_at + MOTION_SETTLE_MS)) return wait;
        return (S5_AVOID_OUTPUT){.action = S5_REVERSE,
                                 .distance_mm = reverse_mm};
    }

    if (phase == S5_CHECK_LINE) {
        if (!in->stopped || !range_fresh || elapsed < MOTION_SETTLE_MS ||
            !at_or_after(in->range_at, state->phase_at + MOTION_SETTLE_MS)) return wait;
        if (state->depth) return resume_leg(state, in, true);
        if (in->line_junction) return fail(state, S5_FAIL_LINE, in->now);
        /* Two light outer sensors also occur on blank floor. Require actual
           tape under the centre or an outer sensor after the turn. */
        if (at_or_after(in->line_at, state->phase_at + MOTION_SETTLE_MS) && in->line_at != state->last_line_at) {
            state->last_line_at = in->line_at;
            state->line_samples = (in->line_center_seen || in->line_hit) ? state->line_samples + 1u : 0u;
        }
        if (state->line_samples < 2u) return wait;
        if (in->range_mm <= clear_mm) {
            s5_avoid_begin(state, in->now);
            return wait;
        }
        return go(state, S5_COMPLETE, in->now);
    }
    unsigned stop_distance = returning ? RETURN_STOP_MM :
                             state->short_approach ? S5_MANEUVER_STOP_MM : S5_SCAN_STOP_MM;
    if (moving && arcing && (!range_fresh || in->range_mm <= RETURN_STOP_MM))
        return fail(state, range_fresh ? S5_FAIL_CLOSE : S5_FAIL_ECHO, in->now);
    if (moving && forward && range_fresh && in->range_mm <= stop_distance)
        return (state->widening || state->rounded_departure || state->rounded_return || phase == S5_PASS) ?
               fail(state, S5_FAIL_CLOSE, in->now) : avoid_on_leg(state, in);
    if (moving && (range_fresh ? in->range_mm < S5_TURN_CLEARANCE_MM : !(probing || in->echo_quiet)))
        return fail(state, range_fresh ? S5_FAIL_CLOSE : S5_FAIL_ECHO, in->now);

    bool settled_search = in->motion == S5_MOTION_NONE && in->stopped && elapsed >= MOTION_SETTLE_MS;
    if (returning && phase == S5_FIND_LINE &&
        (in->motion == S5_MOTION_PENDING || in->motion == S5_MOTION_DONE || settled_search)) {
        if (in->near_original_line && at_or_after(in->line_at, state->phase_at) && in->line_at != state->last_line_at) {
            state->last_line_at = in->line_at;
            state->line_samples = in->line_hit ? state->line_samples + 1u : 0u;
        }
        if (state->line_samples >= 2) {
            go(state, state->rounded_return ? S5_ALIGN_LINE : S5_CENTER_LINE, in->now);
            return (S5_AVOID_OUTPUT){.action = S5_STOP};
        }
        if (state->line_samples == 1 && (settled_search || in->motion == S5_MOTION_DONE)) return wait;
    }

    if (moving) return wait;
    unsigned target = leg_distance(state, in);
    if (in->motion == S5_MOTION_DONE) {
        if (forward) {
            unsigned travelled = state->leg_progress_mm + state->issued_mm;
            if (travelled > S5_MAX_LEG_MM) return fail(state, S5_FAIL_MOTION, in->now);
            state->leg_progress_mm = travelled;
            state->issued_mm = state->short_approach = 0;
            if (travelled >= target) return finish_forward_leg(state, in);
            state->phase_at = in->now;
            return wait;
        }
        switch (phase) {
        case S5_TURN_OUT: return go(state, S5_SHIFT_OUT, in->now);
        case S5_ALIGN_OUTSIDE: {
            unsigned travelled = state->rounded_departure ? departure_forward_mm(state->offset_mm) : 0u;
            state->rounded_departure = false;
            state->widening = false;
            go(state, S5_PASS, in->now);
            state->leg_progress_mm = travelled;
            return wait;
        }
        case S5_TURN_IN: return go(state, S5_FIND_LINE, in->now);
        case S5_ALIGN_LINE: return go(state, S5_CHECK_LINE, in->now);
        default: return fail(state, S5_FAIL_MOTION, in->now);
        }
    }

    if (!in->stopped || elapsed < MOTION_SETTLE_MS) return wait;
    if (range_fresh ? !at_or_after(in->range_at, state->phase_at + MOTION_SETTLE_MS) :
                      !(probing && proof_fresh)) return wait;
    if (!forward) {
        if (range_fresh && phase == S5_TURN_OUT && in->range_mm < S5_DEPARTURE_CLEARANCE_MM)
            return state->widening ? fail(state, S5_FAIL_ROOM, in->now) :
                                     wait_for_path(state, S5_FAIL_ROOM, in);
        if (range_fresh && in->range_mm < S5_TURN_CLEARANCE_MM) return fail(state, S5_FAIL_CLOSE, in->now);
        bool outward = phase == S5_TURN_OUT || phase == S5_ALIGN_LINE;
        if (phase == S5_ALIGN_LINE && state->depth && passed_parent_line(state)) outward = false;
        bool left = outward ? state->side == S5_LEFT : state->side == S5_RIGHT;
        if (arcing) {
            if (range_fresh && in->range_mm >= ARC_ENTRY_MM)
                return (S5_AVOID_OUTPUT){.action = left ? S5_ARC_LEFT : S5_ARC_RIGHT};
            if (state->rolling_route) {
                if (phase != S5_TURN_OUT) return fail(state, S5_FAIL_CLOSE, in->now);
                if (state->reverse_available && in->line_centered && !in->line_junction &&
                    in->range_mm <= clear_mm - S5_REVERSE_MM) {
                    state->reverse_available = state->reverse_long = false;
                    return go(state, S5_BACK_UP, in->now);
                }
                return wait_for_path(state, S5_FAIL_ROOM, in);
            }
            /* The first turn can fall back while still parallel. Once the
               diagonal heading is established, finish it with the matching arc. */
            if (phase == S5_TURN_OUT) state->rounded_departure = false;
            else if (phase == S5_TURN_IN) state->rounded_return = false;
            else return fail(state, S5_FAIL_CLOSE, in->now);
        }
        return (S5_AVOID_OUTPUT){.action = left ? S5_PIVOT_LEFT : S5_PIVOT_RIGHT};
    }
    if (state->leg_progress_mm >= target) return finish_forward_leg(state, in);
    unsigned distance = target - state->leg_progress_mm;
    if (state->rounded_departure && phase == S5_SHIFT_OUT && in->range_mm < distance + ARC_ENTRY_MM)
        return fail(state, S5_FAIL_CLOSE, in->now);
    if (phase == S5_PASS && !state->depth && !state->passed_mm && !state->leg_progress_mm &&
        in->range_mm <= S5_SCAN_STOP_MM) {
        /* Start with the measured edge. Widen only when the live forward echo
           shows the first parallel lane is still obstructed. */
        if (in->range_mm < S5_DEPARTURE_CLEARANCE_MM ||
            state->additional_outward_mm >= S5_OUTWARD_LIMIT_MM ||
            state->offset_mm + S5_OUTWARD_STEP_MM > MAX_LATERAL_OFFSET_MM)
            return fail(state, S5_FAIL_CLOSE, in->now);
        state->additional_outward_mm += S5_OUTWARD_STEP_MM;
        state->widening = true;
        state->probe_left = false;
        return go(state, S5_TURN_OUT, in->now);
    }
    if (returning) {
        unsigned available = in->range_mm > RETURN_RESERVE_MM ? in->range_mm - RETURN_RESERVE_MM : 0u;
        unsigned tape = s5_return_tape_distance_mm(state);
        unsigned contact = tape > in->sensor_to_axle_mm ? tape - in->sensor_to_axle_mm : 0u;
        if (!available || (phase == S5_FIND_LINE && !state->leg_progress_mm && available < contact))
            return fail(state, S5_FAIL_CLOSE, in->now);
        if (distance > available) distance = available;
    } else if (in->range_mm <= S5_SCAN_STOP_MM) {
        if (state->widening || state->rounded_departure || phase == S5_PASS)
            return fail(state, S5_FAIL_CLOSE, in->now);
        unsigned buffer = S5_MANEUVER_STOP_MM + FRONT_MARGIN_MM;
        if (phase != S5_SHIFT_OUT || !state->probe_left || state->leg_progress_mm ||
            in->range_mm < buffer + MIN_LEFT_STEP_MM) return avoid_on_leg(state, in);
        unsigned safe_step = in->range_mm - buffer;
        if (distance > safe_step) distance = safe_step;
        state->short_approach = true;
    } else if (in->range_mm < distance + S5_MANEUVER_STOP_MM) {
        /* Approach far returns only to the scan threshold. A later command
           continues the leg if the object has moved since that reading. */
        distance = in->range_mm - S5_SCAN_STOP_MM + 1u;
    }
    if (!state->depth && phase == S5_FIND_LINE) {
        state->last_line_at = in->line_at;
    }
    state->issued_mm = distance;
    return (S5_AVOID_OUTPUT){.action = S5_ADVANCE, .distance_mm = distance};
}
