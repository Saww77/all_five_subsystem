/* Host tests for the production controller and planner. */
#include "../../app_program/vehicle_controller/vehicle_core.h"
#include <assert.h>
#include <stdio.h>

static void step(VC_MODEL *model, VC_INPUT *input, VC_OUTPUT *output, unsigned ms)
{
    input->avoidance.now += ms;
    if (input->avoidance.range_valid) input->avoidance.range_at = input->avoidance.now;
    vc_core_step(model, input, output);
}

static VC_INPUT track_input(void)
{
    return (VC_INPUT){
        .ready = true, .line_valid = true, .link_ok = true,
        .left_rpm = 30, .right_rpm = 30,
        .avoidance = {.stopped = true, .range_valid = true, .range_mm = 1000,
                      .sensor_to_axle_mm = 60},
    };
}

static void profile(VC_MODEL *model, VC_INPUT *in, VC_OUTPUT *out, S5_PROFILE scan)
{
    in->start = true;
    step(model, in, out, 20);
    assert(model->state == VC_FOLLOW_LINE && !out->stop);
    in->start = false;
    in->avoidance.range_mm = S5_SCAN_STOP_MM + 1;
    step(model, in, out, 20);
    assert(out->left_rpm == 20 && out->right_rpm == 20);

    S5_RANGE_GATE gate = {0};
    assert(s5_accept_range(&gate, S5_SCAN_STOP_MM + 1) && !gate.blocked);
    assert(s5_accept_range(&gate, S5_SCAN_STOP_MM) && gate.blocked);
    assert(s5_accept_range(&gate, S5_CLEAR_MM) && gate.blocked);
    assert(s5_accept_range(&gate, S5_CLEAR_MM + 1) && !gate.blocked);
    in->obstacle_blocked = true;
    in->avoidance.range_mm = S5_SCAN_STOP_MM;
    step(model, in, out, 20);
    assert(out->stop && model->state == VC_AVOIDING && model->avoidance.phase == S5_PROFILE_PATH);

    in->avoidance.profile = scan;
    in->avoidance.profile_valid = true;
    in->avoidance.profile_at = in->avoidance.now - 20;
    step(model, in, out, 20);
    assert(model->avoidance.phase == S5_PROFILE_PATH); /* ignore pre-stop scan */
    in->avoidance.profile_at = in->avoidance.now + 2000;
    step(model, in, out, 2000);
}

static S5_PROFILE obstacle(bool left)
{
    return (S5_PROFILE){.nearest_mm = S5_SCAN_STOP_MM, .front_mm = S5_SCAN_STOP_MM, .width_mm = 90,
                        .left_mm = S5_UNKNOWN_RANGE, .right_mm = S5_UNKNOWN_RANGE,
                        .left_path_mm = left ? 850 : 600, .right_path_mm = left ? 600 : 850,
                        .left_edge_mm = -50, .right_edge_mm = 40,
                        .left_edge_valid = true, .right_edge_valid = true};
}

static void start_move(VC_MODEL *model, VC_INPUT *in, VC_OUTPUT *out,
                       S5_ACTION action, unsigned travel)
{
    in->avoidance.motion = S5_MOTION_NONE;
    step(model, in, out, 260);
    assert(out->maneuver.action == action && out->maneuver.distance_mm == travel);
    in->avoidance.motion = S5_MOTION_PENDING;
}

static void end_move(VC_MODEL *model, VC_INPUT *in, VC_OUTPUT *out)
{
    in->avoidance.motion = S5_MOTION_DONE;
    step(model, in, out, 500);
    in->avoidance.motion = S5_MOTION_NONE;
}

static void reach_inward_turn(VC_MODEL *model, VC_INPUT *in, VC_OUTPUT *out, bool left, bool open_side)
{
    S5_ACTION outward = left ? S5_PIVOT_LEFT : S5_PIVOT_RIGHT;
    S5_ACTION inward = left ? S5_PIVOT_RIGHT : S5_PIVOT_LEFT;
    S5_PROFILE scan = obstacle(left);
    if (open_side) {
        scan.width_mm = S5_UNKNOWN_WIDTH;
        if (left) {
            scan.left_path_mm = S5_OPEN_RANGE;
            scan.right_clipped = true;
            scan.right_edge_valid = false;
        } else {
            scan.right_path_mm = S5_OPEN_RANGE;
            scan.left_clipped = true;
            scan.left_edge_valid = false;
        }
    }
    profile(model, in, out, scan);
    assert(model->avoidance.side == (left ? S5_LEFT : S5_RIGHT));

    start_move(model, in, out, outward, 0);
    end_move(model, in, out);
    in->avoidance.range_mm = open_side ? S5_OPEN_RANGE : 1000;
    in->obstacle_blocked = false;
    start_move(model, in, out, S5_ADVANCE, left ? 185 : 175);
    end_move(model, in, out);
    start_move(model, in, out, inward, 0);
    end_move(model, in, out);
    start_move(model, in, out, S5_ADVANCE, 350);
    end_move(model, in, out);
    start_move(model, in, out, inward, 0);
    end_move(model, in, out);
    assert(model->state == VC_RECOVER_LINE && model->avoidance.phase == S5_FIND_LINE);
}

static void test_rejoin(bool left, bool retry, bool open_side)
{
    VC_MODEL model = {0};
    VC_INPUT in = track_input();
    VC_OUTPUT out;
    reach_inward_turn(&model, &in, &out, left, open_side);
    S5_ACTION outward = left ? S5_PIVOT_LEFT : S5_PIVOT_RIGHT;
    S5_ACTION inward = left ? S5_PIVOT_RIGHT : S5_PIVOT_LEFT;
    if (retry) {
        in.avoidance.range_mm = 240;
        step(&model, &in, &out, 260);
        assert(out.maneuver.action == S5_WAIT && model.avoidance.phase == S5_ALIGN_RETRY);
        start_move(&model, &in, &out, outward, 0);
        end_move(&model, &in, &out);
        in.avoidance.range_mm = 1000;
        start_move(&model, &in, &out, S5_ADVANCE, 60);
        end_move(&model, &in, &out);
        start_move(&model, &in, &out, inward, 0);
        end_move(&model, &in, &out);
    }

    start_move(&model, &in, &out, S5_ADVANCE, left ? 225 : 215);
    in.avoidance.line_hit = true;
    in.avoidance.line_at = in.avoidance.now + 20;
    step(&model, &in, &out, 20);
    assert(model.avoidance.line_samples == 0); /* a marking too early is not the original line */
    in.avoidance.near_original_line = true;
    in.avoidance.line_junction = true; /* both outer sensors may cross together */
    in.avoidance.line_at = in.avoidance.now + 20;
    step(&model, &in, &out, 20);
    assert(!out.stop);
    step(&model, &in, &out, 20);
    assert(!out.stop); /* the same reading can't confirm the line twice */
    in.avoidance.line_at = in.avoidance.now + 20;
    step(&model, &in, &out, 20);
    assert(out.stop && out.maneuver.action == S5_STOP && model.avoidance.phase == S5_CENTER_LINE);

    in.avoidance.line_hit = in.avoidance.line_junction = false;
    start_move(&model, &in, &out, S5_ADVANCE, 60);
    end_move(&model, &in, &out);
    start_move(&model, &in, &out, outward, 0);
    end_move(&model, &in, &out);
    assert(model.avoidance.phase == S5_CHECK_LINE);
    in.avoidance.range_valid = false;
    step(&model, &in, &out, 260);
    assert(model.state == VC_RECOVER_LINE && out.maneuver.action == S5_WAIT);
    in.avoidance.range_valid = true;
    step(&model, &in, &out, 20);
    assert(model.state == VC_RECOVER_LINE); /* crossing evidence cannot authorize handoff */
    in.avoidance.line_centered = true;
    in.avoidance.line_at = in.avoidance.now + 20;
    step(&model, &in, &out, 20);
    assert(model.state == VC_FOLLOW_LINE && model.avoidance.phase == S5_COMPLETE);
    step(&model, &in, &out, 20);
    assert(!out.stop && out.left_rpm == 30 && out.right_rpm == 30);
}

static void test_abort_paths(void)
{
    enum { OPERATOR_STOP, STALE_LINE, NO_ECHO, MOTOR_REJECTED, NO_LINE };
    for (unsigned cause = OPERATOR_STOP; cause <= NO_LINE; ++cause) {
        VC_MODEL model = {0};
        VC_INPUT in = track_input();
        VC_OUTPUT out;
        reach_inward_turn(&model, &in, &out, false, false);
        start_move(&model, &in, &out, S5_ADVANCE, 215);
        switch (cause) {
        case OPERATOR_STOP: in.stop = true; break;
        case STALE_LINE: in.line_age_ms = VC_LINE_MAX_AGE_MS + 1; break;
        case NO_ECHO: in.avoidance.range_valid = false; break;
        case MOTOR_REJECTED: in.avoidance.motion = S5_MOTION_FAILED; break;
        case NO_LINE: in.avoidance.motion = S5_MOTION_DONE; break;
        }
        step(&model, &in, &out, 20);
        assert(out.stop && !vc_core_running(&model) && model.avoidance.phase == S5_FAILED);
        in = track_input();
        in.start = true;
        step(&model, &in, &out, 20);
        assert(out.stop && !vc_core_running(&model)); /* repositioning needs actual IR evidence */
    }
}

static void test_link_loss_does_not_end_a_run(void)
{
    VC_MODEL model = {0};
    VC_INPUT in = track_input();
    VC_OUTPUT out;
    in.start = true;
    step(&model, &in, &out, 20);
    assert(model.state == VC_FOLLOW_LINE);
    in.start = false;
    in.link_ok = false;
    step(&model, &in, &out, 20);
    assert(model.state == VC_FOLLOW_LINE && !out.stop && out.left_rpm == 30);
    in.obstacle_blocked = true;
    in.avoidance.range_mm = 200;
    step(&model, &in, &out, 20);
    assert(model.state == VC_AVOIDING && model.avoidance.phase == S5_PROFILE_PATH);
    in.avoidance.profile = obstacle(true);
    in.avoidance.profile_valid = true;
    in.avoidance.profile_at = in.avoidance.now + 600;
    step(&model, &in, &out, 600);
    start_move(&model, &in, &out, S5_PIVOT_LEFT, 0);
    in.stop = true;
    step(&model, &in, &out, 20);
    assert(out.stop && model.state == VC_IDLE);
}

static void test_unsafe_profile_and_scan_timeout(void)
{
    VC_MODEL model = {0};
    VC_INPUT in = track_input();
    VC_OUTPUT out;
    S5_PROFILE scan = obstacle(false);
    scan.left_path_mm = 150;
    scan.right_path_mm = 190;
    profile(&model, &in, &out, scan);
    assert(out.maneuver.action == S5_WAIT && model.avoidance.phase == S5_WAIT_PATH &&
           model.avoidance.failure == S5_FAIL_SIDE && model.state == VC_AVOIDING);
    step(&model, &in, &out, 1000);
    assert(out.maneuver.action == S5_WAIT && model.avoidance.phase == S5_WAIT_PATH);
    in.avoidance.range_mm += 30;
    step(&model, &in, &out, 20);
    assert(out.maneuver.action == S5_WAIT && model.avoidance.phase == S5_PROFILE_PATH);

    model = (VC_MODEL){0};
    in = track_input();
    in.start = true;
    step(&model, &in, &out, 20);
    in.start = false;
    in.obstacle_blocked = true;
    step(&model, &in, &out, 20);
    step(&model, &in, &out, 14000);
    assert(out.maneuver.action == S5_WAIT && model.avoidance.phase == S5_WAIT_PATH &&
           model.avoidance.failure == S5_FAIL_SCAN_TIMEOUT && model.state == VC_AVOIDING);
    in.avoidance.range_valid = false;
    step(&model, &in, &out, 1000);
    assert(out.maneuver.action == S5_WAIT && model.avoidance.phase == S5_WAIT_PATH);
    in.avoidance.range_valid = true;
    in.avoidance.range_mm = 196;
    step(&model, &in, &out, 20);
    assert(out.maneuver.action == S5_WAIT && model.avoidance.phase == S5_PROFILE_PATH);
}

static void test_reentry_limit(void)
{
    S5_AVOID model = {.phase = S5_FIND_LINE, .side = S5_LEFT,
                     .offset_mm = 185, .passed_mm = S5_MAX_PASS_MM};
    S5_AVOID_INPUT in = {.now = 300, .range_at = 300, .stopped = true,
                         .range_valid = true, .range_mm = 240};
    S5_AVOID_OUTPUT out = s5_avoid_step(&model, &in);
    assert(out.action == S5_STOP && model.phase == S5_FAILED);
}

static void test_width_and_side_clearance(void)
{
    S5_PROFILE scan = obstacle(true);
    S5_BYPASS plan = s5_plan_bypass(&scan);
    assert(plan.side == S5_LEFT);
    assert(plan.offset_mm == 185 && plan.pass_mm == 350);
    assert(s5_reentry_detection_mm(plan.offset_mm, 60) == 85);
    scan.left_path_mm = scan.right_path_mm; /* Equal clearance defaults to T. */
    assert(s5_plan_bypass(&scan).side == S5_LEFT);
    scan.front_mm = 114;
    plan = s5_plan_bypass(&scan);
    assert(plan.side == S5_NO_SIDE && plan.failure == S5_FAIL_ROOM);
    scan.front_mm = S5_SCAN_STOP_MM;
    scan.width_mm = S5_UNKNOWN_WIDTH;
    plan = s5_plan_bypass(&scan);
    assert(plan.side == S5_LEFT && plan.offset_mm == 185);
    scan = obstacle(true);
    scan.left_path_mm = S5_UNKNOWN_RANGE;
    assert(s5_plan_bypass(&scan).side == S5_RIGHT);

    scan.front_mm = 215;
    scan.left_path_mm = 150;
    scan.right_path_mm = 222;
    plan = s5_plan_bypass(&scan);
    assert(plan.side == S5_NO_SIDE && plan.failure == S5_FAIL_SIDE);
}

static void test_bypass_with_estimated_edges(void)
{
    S5_PROFILE scan = obstacle(true);
    scan.left_edge_valid = scan.right_edge_valid = false;
    scan.left_edge_mm = -35;
    scan.right_edge_mm = 20;
    scan.width_mm = 28;
    scan.left_path_mm = 650;
    scan.right_path_mm = 730;

    S5_BYPASS plan = s5_plan_bypass(&scan);
    assert(plan.side == S5_RIGHT && plan.failure == S5_FAIL_NONE);
    assert(plan.offset_mm == 188 && plan.pass_mm == 350);

    scan.left_path_mm = S5_OPEN_RANGE;
    scan.right_path_mm = S5_UNKNOWN_RANGE;
    plan = s5_plan_bypass(&scan);
    assert(plan.side == S5_LEFT && plan.offset_mm == 195 && plan.pass_mm == 350);

    scan.left_path_mm = 150;
    scan.right_path_mm = 188 + 15 + S5_MANEUVER_STOP_MM - 1;
    plan = s5_plan_bypass(&scan);
    assert(plan.side == S5_NO_SIDE && plan.failure == S5_FAIL_SIDE);

    scan.left_clipped = scan.right_clipped = true;
    scan.left_path_mm = scan.right_path_mm = S5_OPEN_RANGE;
    plan = s5_plan_bypass(&scan);
    assert(plan.side == S5_NO_SIDE && plan.failure == S5_FAIL_BOTH_LIMITS);
}

static void test_default_left_with_unconfirmed_clearance(void)
{
    S5_PROFILE scan = {
        .nearest_mm = 202, .front_mm = 190, .width_mm = 145, .angle_offset_deg = 65,
        .left_mm = S5_UNKNOWN_RANGE, .right_mm = S5_UNKNOWN_RANGE,
        .left_path_mm = S5_UNKNOWN_RANGE, .right_path_mm = 207,
        .left_edge_mm = -90,
    };
    S5_BYPASS plan = s5_plan_bypass(&scan);
    assert(plan.side == S5_LEFT && plan.probe_left && plan.offset_mm == 305);

    S5_AVOID state = {.phase = S5_PROFILE_PATH, .phase_at = 100};
    S5_AVOID_INPUT in = {.profile = scan, .now = 500, .profile_at = 490, .stopped = true,
                         .profile_valid = true, .range_mm = S5_UNKNOWN_RANGE};
    assert(s5_avoid_step(&state, &in).action == S5_WAIT && state.phase == S5_PROFILE_PATH);
    in.profile.pivot_confirmed = true;
    in.echo_quiet = true;
    assert(s5_avoid_step(&state, &in).action == S5_WAIT && state.phase == S5_TURN_OUT);
    S5_AVOID expired = state;
    in.now = 2200;
    assert(s5_avoid_step(&expired, &in).action == S5_WAIT);
    in.now = 760;
    assert(s5_avoid_step(&state, &in).action == S5_PIVOT_LEFT);
    in.motion = S5_MOTION_PENDING;
    in.now = 900;
    assert(s5_avoid_step(&state, &in).action == S5_WAIT && state.phase == S5_TURN_OUT);

    S5_AVOID danger = state;
    S5_AVOID_INPUT close = in;
    close.range_valid = true;
    close.range_at = close.now;
    close.range_mm = S5_TURN_CLEARANCE_MM - 1u;
    assert(s5_avoid_step(&danger, &close).action == S5_STOP && danger.phase == S5_FAILED);

    in.motion = S5_MOTION_DONE;
    assert(s5_avoid_step(&state, &in).action == S5_WAIT && state.phase == S5_SHIFT_OUT);
    in.now = 1160;
    in.motion = S5_MOTION_NONE;
    S5_AVOID waiting = state;
    assert(s5_avoid_step(&waiting, &in).action == S5_WAIT && waiting.waiting_echo);
    in.echo_quiet = false;
    in.range_valid = true;
    in.range_mm = 860;
    in.range_at = in.now;
    S5_AVOID_OUTPUT move = s5_avoid_step(&state, &in);
    assert(move.action == S5_ADVANCE && move.distance_mm == 305);
    state.issued_mm = 0;
    in.range_mm = 207;
    move = s5_avoid_step(&state, &in);
    assert(move.action == S5_ADVANCE && move.distance_mm == 42);
    in.motion = S5_MOTION_PENDING;
    in.range_mm = 180;
    assert(s5_avoid_step(&state, &in).action == S5_WAIT);
    in.range_mm = 150;
    assert(s5_avoid_step(&state, &in).action == S5_STOP && state.depth == 1);

    scan.left_path_mm = 80;
    assert(s5_plan_bypass(&scan).side == S5_NO_SIDE);
    scan.left_path_mm = 207;
    scan.width_mm = S5_UNKNOWN_WIDTH;
    assert(s5_plan_bypass(&scan).probe_left);
    scan.front_mm = S5_DEPARTURE_CLEARANCE_MM - 1u;
    assert(s5_plan_bypass(&scan).failure == S5_FAIL_ROOM);
}

static void test_lateral_bypass_at_start(void)
{
    /* Reproduce the sweep with short 45-degree readings and usable space at R. */
    S5_PROFILE scan = {.nearest_mm = 218, .front_mm = 120, .width_mm = 399,
                       .left_mm = S5_UNKNOWN_RANGE, .right_mm = 925,
                       .left_path_mm = 234, .right_path_mm = 230, .left_clipped = true,
                       .right_edge_mm = 234, .right_edge_valid = true};
    S5_BYPASS plan = s5_plan_bypass(&scan);
    assert(plan.side == S5_RIGHT && plan.offset_mm == 369 && plan.pass_mm == 348);

    VC_MODEL model = {0};
    VC_INPUT in = track_input();
    VC_OUTPUT out;
    in.start = in.obstacle_blocked = true;
    in.avoidance.range_mm = 220;
    assert(vc_core_start_block(&model, &in) == VC_START_READY);
    step(&model, &in, &out, 20);
    assert(out.stop && model.state == VC_AVOIDING && model.avoidance.phase == S5_PROFILE_PATH);

    in.start = false;
    in.avoidance.profile = scan;
    in.avoidance.profile_valid = true;
    in.avoidance.profile_at = in.avoidance.now + 2000;
    step(&model, &in, &out, 2000);
    start_move(&model, &in, &out, S5_PIVOT_RIGHT, 0);
    end_move(&model, &in, &out);
    assert(model.avoidance.phase == S5_SHIFT_OUT);

    VC_MODEL blocked = model;
    VC_INPUT near = in;
    near.avoidance.range_mm = 300;
    start_move(&blocked, &near, &out, S5_ADVANCE, 81);
    end_move(&blocked, &near, &out);
    assert(blocked.avoidance.phase == S5_SHIFT_OUT && blocked.avoidance.leg_progress_mm == 81);
    near.avoidance.range_mm = 219;
    step(&blocked, &near, &out, 260);
    assert(out.stop && blocked.avoidance.phase == S5_PROFILE_PATH && blocked.avoidance.depth == 1);

    in.avoidance.range_mm = 1000;
    start_move(&model, &in, &out, S5_ADVANCE, 369);
    end_move(&model, &in, &out);
    start_move(&model, &in, &out, S5_PIVOT_LEFT, 0);
    end_move(&model, &in, &out);
    start_move(&model, &in, &out, S5_ADVANCE, 348);
    end_move(&model, &in, &out);
    start_move(&model, &in, &out, S5_PIVOT_LEFT, 0);
    end_move(&model, &in, &out);
    start_move(&model, &in, &out, S5_ADVANCE, 409);
}

static void test_stationary_retry(void)
{
    VC_MODEL model = {0};
    VC_INPUT in = track_input();
    VC_OUTPUT out;
    S5_PROFILE scan = obstacle(true);
    scan.left_path_mm = 80;
    scan.right_path_mm = S5_UNKNOWN_RANGE;
    scan.width_mm = S5_UNKNOWN_WIDTH;
    profile(&model, &in, &out, scan);
    assert(out.maneuver.action == S5_RESCAN && model.avoidance.phase == S5_PROFILE_PATH);
    step(&model, &in, &out, 20);
    assert(out.maneuver.action == S5_WAIT);
    in.avoidance.profile = obstacle(true);
    in.avoidance.profile_at = in.avoidance.now + 2000;
    step(&model, &in, &out, 2000);
    assert(model.avoidance.phase == S5_TURN_OUT && model.avoidance.side == S5_LEFT);
    start_move(&model, &in, &out, S5_PIVOT_LEFT, 0);
}

static void test_clipped_scan(bool can_retrace)
{
    VC_MODEL model = {0};
    VC_INPUT in = track_input();
    VC_OUTPUT out;
    in.avoidance.line_centered = true;
    in.avoidance.can_retrace = can_retrace;
    S5_PROFILE scan = obstacle(true);
    scan.width_mm = S5_UNKNOWN_WIDTH;
    scan.left_edge_valid = false;
    scan.left_clipped = true;
    scan.left_path_mm = 234;
    scan.right_path_mm = 230;
    profile(&model, &in, &out, scan);
    if (!can_retrace) {
        assert(out.maneuver.action == S5_WAIT && model.avoidance.phase == S5_WAIT_PATH);
        assert(model.avoidance.failure == S5_FAIL_LEFT_LIMIT);
        return;
    }
    assert(out.maneuver.action == S5_WAIT && model.avoidance.phase == S5_BACK_UP);
    start_move(&model, &in, &out, S5_REVERSE, S5_REVERSE_MM);
    end_move(&model, &in, &out);
    assert(model.avoidance.phase == S5_PROFILE_PATH);
    in.avoidance.range_mm += S5_REVERSE_MM;
    in.avoidance.profile_at = in.avoidance.now + 2000;
    step(&model, &in, &out, 2000);
    assert(out.maneuver.action == S5_WAIT && model.avoidance.phase == S5_WAIT_PATH);
    assert(model.avoidance.failure == S5_FAIL_LEFT_LIMIT);
}

static void test_departure_room_after_retrace(void)
{
    S5_AVOID state = {.phase = S5_PROFILE_PATH, .phase_at = 100,
                      .reverse_available = true, .reverse_long = true};
    S5_AVOID_INPUT in = {.now = 500, .profile_at = 450, .range_at = 500, .range_mm = 110,
                         .profile_valid = true, .stopped = true, .range_valid = true,
                         .line_centered = true, .profile = obstacle(true)};
    /* The newer forward measurement leaves too little room to pivot. */
    assert(s5_avoid_step(&state, &in).action == S5_WAIT && state.phase == S5_BACK_UP);
    in.now = in.range_at = 800;
    S5_AVOID_OUTPUT out = s5_avoid_step(&state, &in);
    assert(out.action == S5_REVERSE && out.distance_mm == S5_LONG_REVERSE_MM);
    in.motion = S5_MOTION_DONE;
    assert(s5_avoid_step(&state, &in).action == S5_WAIT && state.phase == S5_PROFILE_PATH);
    in.now = in.range_at = in.profile_at = 950;
    in.motion = S5_MOTION_NONE;
    in.range_mm = 230;
    in.profile.front_mm = in.profile.nearest_mm = 230;
    assert(s5_avoid_step(&state, &in).action == S5_WAIT && state.phase == S5_TURN_OUT);
}

static void test_close_left_obstacle_with_right_clearance(bool has_retrace)
{
    S5_AVOID state = {.phase = S5_PROFILE_PATH, .phase_at = 100,
                      .reverse_available = has_retrace, .reverse_long = has_retrace};
    S5_AVOID_INPUT in = {
        .now = 500, .profile_at = 490, .range_at = 500, .range_mm = 64,
        .profile_valid = true, .stopped = true, .range_valid = true, .line_centered = true,
        .profile = {.nearest_mm = 64, .front_mm = 58, .width_mm = 78, .left_clipped = true,
                    .left_path_mm = 68, .right_path_mm = 839, .right_mm = S5_UNKNOWN_RANGE},
    };
    S5_AVOID_OUTPUT out = s5_avoid_step(&state, &in);
    assert(out.action == S5_WAIT);
    if (!has_retrace) {
        assert(state.phase == S5_WAIT_PATH && state.failure == S5_FAIL_ROOM);
        in.now = in.range_at = 8000;
        assert(s5_avoid_step(&state, &in).action == S5_WAIT && state.phase == S5_WAIT_PATH);
        in.now = in.range_at = 8100;
        in.range_mm = 124;
        assert(s5_avoid_step(&state, &in).action == S5_WAIT && state.phase == S5_PROFILE_PATH);
        return;
    }
    assert(state.phase == S5_BACK_UP);
    in.now = in.range_at = 800;
    out = s5_avoid_step(&state, &in);
    assert(out.action == S5_REVERSE && out.distance_mm == 60);

    in.now = in.range_at = 1500;
    in.motion = S5_MOTION_DONE;
    in.range_mm = 124;
    assert(s5_avoid_step(&state, &in).action == S5_WAIT && state.phase == S5_PROFILE_PATH);
    in.now = in.range_at = in.profile_at = 2500;
    in.motion = S5_MOTION_NONE;
    in.profile.nearest_mm = 124;
    in.profile.front_mm = 118;
    assert(s5_avoid_step(&state, &in).action == S5_WAIT && state.phase == S5_TURN_OUT);
    in.now = in.range_at = 2800;
    out = s5_avoid_step(&state, &in);
    assert(out.action == S5_PIVOT_RIGHT);
}

static void complete_nested_bypass(VC_MODEL *model, VC_INPUT *in, VC_OUTPUT *out,
                                   bool face_back)
{
    assert(model->avoidance.phase == S5_PROFILE_PATH && model->avoidance.depth == 1);
    assert(!in->avoidance.stopped);
    step(model, in, out, 20);
    assert(out->maneuver.action == S5_WAIT && model->avoidance.phase == S5_PROFILE_PATH);

    in->avoidance.stopped = true;
    in->avoidance.range_mm = 150;
    in->avoidance.profile = obstacle(false);
    in->avoidance.profile.nearest_mm = in->avoidance.profile.front_mm = 150;
    in->avoidance.profile_valid = true;
    in->avoidance.profile_at = in->avoidance.now + 600;
    step(model, in, out, 600);
    assert(model->avoidance.phase == S5_TURN_OUT && model->avoidance.side == S5_RIGHT);
    start_move(model, in, out, S5_PIVOT_RIGHT, 0);
    end_move(model, in, out);
    in->avoidance.range_mm = 1000;
    start_move(model, in, out, S5_ADVANCE, 175);
    end_move(model, in, out);
    start_move(model, in, out, S5_PIVOT_LEFT, 0);
    end_move(model, in, out);
    start_move(model, in, out, S5_ADVANCE, 280);
    end_move(model, in, out);
    start_move(model, in, out, S5_PIVOT_LEFT, 0);
    end_move(model, in, out);
    start_move(model, in, out, S5_ADVANCE, 175);
    end_move(model, in, out);
    start_move(model, in, out, face_back ? S5_PIVOT_LEFT : S5_PIVOT_RIGHT, 0);
    end_move(model, in, out);
    in->avoidance.profile_valid = false;
    step(model, in, out, 260); /* No IR hit on the temporary corridor. */
    assert(model->avoidance.depth == 0);
}

static void test_obstacle_during_pass(void)
{
    VC_MODEL model = {.state = VC_AVOIDING,
                      .avoidance = {.phase = S5_PASS, .side = S5_LEFT, .offset_mm = 185, .next_pass_mm = 350}};
    VC_INPUT in = track_input();
    VC_OUTPUT out;
    start_move(&model, &in, &out, S5_ADVANCE, 350);
    in.avoidance.range_mm = 200;
    in.avoidance.travel_mm = 20;
    in.avoidance.stopped = false;
    step(&model, &in, &out, 100);
    assert(out.stop && model.avoidance.phase == S5_PROFILE_PATH && model.avoidance.depth == 1);
    in.avoidance.motion = S5_MOTION_NONE;

    VC_MODEL cancelled = model;
    VC_INPUT stopped = in;
    stopped.stop = true;
    step(&cancelled, &stopped, &out, 20);
    assert(out.stop && cancelled.state == VC_IDLE && cancelled.avoidance.phase == S5_FAILED);

    complete_nested_bypass(&model, &in, &out, false);
    assert(model.avoidance.phase == S5_PASS && model.avoidance.side == S5_LEFT);
    assert(model.avoidance.offset_mm == 185 && model.avoidance.leg_progress_mm == 300);
    start_move(&model, &in, &out, S5_ADVANCE, 50);
    end_move(&model, &in, &out);
    assert(model.avoidance.phase == S5_TURN_IN && model.avoidance.passed_mm == 350);
    start_move(&model, &in, &out, S5_PIVOT_RIGHT, 0);
}

static void test_nested_route_clears_before_the_scan_finishes(void)
{
    VC_MODEL model = {.state = VC_AVOIDING,
                      .avoidance = {.phase = S5_PASS, .side = S5_LEFT, .offset_mm = 185, .next_pass_mm = 350}};
    VC_INPUT in = track_input();
    VC_OUTPUT out;
    start_move(&model, &in, &out, S5_ADVANCE, 350);
    in.avoidance.range_mm = 200;
    in.avoidance.travel_mm = 80;
    step(&model, &in, &out, 20);
    assert(out.stop && model.avoidance.depth == 1);

    in.avoidance.motion = S5_MOTION_NONE;
    in.avoidance.range_mm = 1000;
    in.avoidance.profile = (S5_PROFILE){.nearest_mm = S5_OPEN_RANGE};
    in.avoidance.profile_valid = true;
    in.avoidance.profile_at = in.avoidance.now + 600;
    step(&model, &in, &out, 600);
    assert(model.state == VC_AVOIDING && model.avoidance.phase == S5_PASS && !model.avoidance.depth);
    start_move(&model, &in, &out, S5_ADVANCE, 270);
}

static void test_obstacle_during_line_search(void)
{
    VC_MODEL model = {.state = VC_RECOVER_LINE,
                      .avoidance = {.phase = S5_FIND_LINE, .side = S5_LEFT, .offset_mm = 175}};
    VC_INPUT in = track_input();
    VC_OUTPUT out;
    start_move(&model, &in, &out, S5_ADVANCE, 215);
    in.avoidance.range_mm = 200;
    in.avoidance.travel_mm = 40;
    in.avoidance.stopped = false;
    step(&model, &in, &out, 100);
    assert(out.stop && model.avoidance.phase == S5_PROFILE_PATH && model.avoidance.depth == 1);
    in.avoidance.motion = S5_MOTION_NONE;
    complete_nested_bypass(&model, &in, &out, true);

    assert(model.state == VC_RECOVER_LINE && model.avoidance.phase == S5_FIND_LINE);
    assert(model.avoidance.side == S5_RIGHT && model.avoidance.offset_mm == 145);
    start_move(&model, &in, &out, S5_ADVANCE, 185);
    in.avoidance.near_original_line = in.avoidance.line_hit = true;
    in.avoidance.line_at = in.avoidance.now + 20;
    step(&model, &in, &out, 20);
    in.avoidance.line_at = in.avoidance.now + 20;
    step(&model, &in, &out, 20);
    assert(out.stop && model.avoidance.phase == S5_CENTER_LINE);
    in.avoidance.line_hit = false;
    start_move(&model, &in, &out, S5_ADVANCE, 60);
    end_move(&model, &in, &out);
    start_move(&model, &in, &out, S5_PIVOT_RIGHT, 0);
}

static void test_next_obstacle_on_return(void)
{
    VC_MODEL model = {.state = VC_RECOVER_LINE,
                      .avoidance = {.phase = S5_CHECK_LINE, .phase_at = 100, .side = S5_LEFT}};
    VC_INPUT in = track_input();
    VC_OUTPUT out;
    in.avoidance.now = 400;
    in.avoidance.line_at = 420;
    in.avoidance.line_centered = true;
    in.avoidance.range_mm = 200;
    in.obstacle_blocked = true;
    step(&model, &in, &out, 20);
    assert(model.state == VC_AVOIDING && model.avoidance.phase == S5_PROFILE_PATH);

    in.avoidance.profile = obstacle(false);
    in.avoidance.profile_at = 600;
    in.avoidance.profile_valid = true;
    step(&model, &in, &out, 180);
    assert(model.avoidance.side == S5_RIGHT && model.avoidance.phase == S5_TURN_OUT);
    start_move(&model, &in, &out, S5_PIVOT_RIGHT, 0);
}

static void test_reverse_and_rescan(bool path_clears)
{
    VC_MODEL model = {0};
    VC_INPUT in = track_input();
    VC_OUTPUT out;
    in.avoidance.line_centered = in.avoidance.can_retrace = true;
    S5_PROFILE blocked = obstacle(true);
    blocked.front_mm = blocked.nearest_mm = 90;
    profile(&model, &in, &out, blocked);
    assert(model.avoidance.phase == S5_BACK_UP);
    start_move(&model, &in, &out, S5_REVERSE, S5_REVERSE_MM);
    end_move(&model, &in, &out);
    assert(model.avoidance.phase == S5_PROFILE_PATH);
    step(&model, &in, &out, 20);
    assert(model.avoidance.phase == S5_PROFILE_PATH); /* previous scan can't be reused */

    if (path_clears) {
        in.avoidance.profile = (S5_PROFILE){.nearest_mm = S5_OPEN_RANGE};
        in.avoidance.range_mm = S5_CLEAR_MM + 1;
        in.obstacle_blocked = false;
    }
    in.avoidance.profile_at = in.avoidance.now + 2000;
    step(&model, &in, &out, 2000);
    if (!path_clears) {
        assert(out.maneuver.action == S5_WAIT && model.avoidance.phase == S5_WAIT_PATH);
        return;
    }
    assert(model.state == VC_FOLLOW_LINE && model.avoidance.phase == S5_COMPLETE);
}

int main(void)
{
    test_rejoin(true, false, false);
    test_rejoin(false, true, false);
    test_rejoin(true, false, true);
    test_rejoin(false, false, true);
    test_abort_paths();
    test_link_loss_does_not_end_a_run();
    test_unsafe_profile_and_scan_timeout();
    test_reentry_limit();
    test_width_and_side_clearance();
    test_bypass_with_estimated_edges();
    test_default_left_with_unconfirmed_clearance();
    test_lateral_bypass_at_start();
    test_stationary_retry();
    test_clipped_scan(false);
    test_clipped_scan(true);
    test_departure_room_after_retrace();
    test_close_left_obstacle_with_right_clearance(true);
    test_close_left_obstacle_with_right_clearance(false);
    test_obstacle_during_pass();
    test_nested_route_clears_before_the_scan_finishes();
    test_obstacle_during_line_search();
    test_next_obstacle_on_return();
    test_reverse_and_rescan(true);
    test_reverse_and_rescan(false);
    puts("PASS default left pivot, post-turn range, nested bypass, line recovery and cancellation");
}
