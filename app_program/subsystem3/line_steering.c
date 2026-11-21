#include "line_steering.h"

#define EDGE_DARKNESS 160
#define WIDE_MARKING_DARKNESS 800
#define STRAIGHT_ENTER 165
#define STRAIGHT_RELEASE 80
#define CURVE_DARKNESS 350
#define CURVE_IMBALANCE 315
#define CURVE_CONFIRM_SAMPLES 3
#define CURVE_HOLD_SAMPLES 10
#define CURVE_TIGHT_SAMPLES 24
#define CURVE_RECENTER_SAMPLES 2
#define SHORT_GAP_SAMPLES 10

static int absolute(int value)
{
    return value < 0 ? -value : value;
}

static void clear_curve(LF_STATE *state)
{
    state->candidate_side = 0;
    state->candidate_samples = 0;
    state->curve_side = 0;
    state->strong_samples = 0;
    state->light_samples = 0;
    state->centered_samples = 0;
}

static void confirm_curve(LF_STATE *state, int strong_side)
{
    if (strong_side == 0 || state->candidate_side != strong_side) {
        state->candidate_side = (int8_t)strong_side;
        state->candidate_samples = 0;
    }
    if (strong_side != 0 && ++state->candidate_samples == CURVE_CONFIRM_SAMPLES) {
        state->curve_side = (int8_t)strong_side;
        state->strong_samples = CURVE_CONFIRM_SAMPLES;
    }
}

static int track_demand(LF_STATE *state, int error, bool light)
{
    if (light) {
        state->filtered_error = 0;
        return 0;
    }
    int previous = state->filtered_error;
    state->filtered_error += (int16_t)((error - previous) / 4);
    int magnitude = absolute(state->filtered_error);
    int side = state->filtered_error < 0 ? -1 : 1;
    bool already_steering = state->correction_rpm * side > 0;
    int threshold = already_steering ? STRAIGHT_RELEASE : STRAIGHT_ENTER;
    if (magnitude < threshold) { return 0; }
    if (!already_steering &&
        (previous * side < STRAIGHT_ENTER || error * side < STRAIGHT_RELEASE)) {
        return 0;
    }
    return side * (magnitude >= 320 ? 4 : 2);
}

static int curve_demand(LF_STATE *state, int error, int strong_side, bool light)
{
    int side = state->curve_side;
    if (light) {
        state->centered_samples = 0;
        if (state->light_samples < S3_BLIND_CURVE_LIMIT) { ++state->light_samples; }
        if (state->strong_samples < CURVE_HOLD_SAMPLES) {
            /* A short edge touch can finish with both sensors on white. */
            if (state->light_samples >= SHORT_GAP_SAMPLES) {
                clear_curve(state);
                state->filtered_error = 0;
                return 0;
            }
            return side * 4;
        }
        /* A sustained bend followed by an edge loss calls for a tight arc
         * toward the last seen edge. The caller stops if it stays blind. */
        return side * (state->light_samples < SHORT_GAP_SAMPLES ? 8 : S3_CRUISE_RPM);
    }

    state->light_samples = 0;
    if (side * error < 120) {
        if (++state->centered_samples >= CURVE_RECENTER_SAMPLES) {
            clear_curve(state);
            state->filtered_error = 0;
        }
        return 0;
    }
    state->centered_samples = 0;
    if (strong_side != side) {
        if (state->strong_samples > 0) { --state->strong_samples; }
        return side * 4;
    }
    if (state->strong_samples < CURVE_TIGHT_SAMPLES) { ++state->strong_samples; }
    if (state->strong_samples >= CURVE_TIGHT_SAMPLES) { return side * S3_CRUISE_RPM; }
    return side * (state->strong_samples >= CURVE_HOLD_SAMPLES ? 8 : 6);
}

void line_decide(int left, int center, int right, LF_STATE *state, LF_CMD *command)
{
    int demand = 0;
    int error = right - left;
    int peak = left > right ? left : right;
    bool light = peak < EDGE_DARKNESS;
    int strong_side = 0;

    /* GP27 belongs to barcode sensing. */
    (void)center;
    if (peak >= CURVE_DARKNESS && absolute(error) >= CURVE_IMBALANCE) {
        strong_side = error < 0 ? -1 : 1;
    }
    if (left >= WIDE_MARKING_DARKNESS && right >= WIDE_MARKING_DARKNESS) {
        clear_curve(state);
        state->filtered_error = 0;
    } else {
        if (state->curve_side == 0) { confirm_curve(state, strong_side); }
        demand = state->curve_side != 0 ? curve_demand(state, error, strong_side, light)
                                        : track_demand(state, error, light);
    }

    if (state->light_samples >= S3_BLIND_CURVE_LIMIT) {
        *command = (LF_CMD){.stop = true};
        return;
    }

    if (state->slew_hold) {
        state->slew_hold = false;
    } else if (demand != state->correction_rpm) {
        state->correction_rpm += demand > state->correction_rpm
                               ? S3_CORRECTION_STEP_RPM : -S3_CORRECTION_STEP_RPM;
        state->slew_hold = true;
    }
    int left_rpm = S3_CRUISE_RPM + state->correction_rpm;
    int right_rpm = S3_CRUISE_RPM - state->correction_rpm;
    *command = (LF_CMD){
        .left = left_rpm > S3_MAX_OUTER_RPM ? S3_MAX_OUTER_RPM : left_rpm,
        .right = right_rpm > S3_MAX_OUTER_RPM ? S3_MAX_OUTER_RPM : right_rpm
    };
}
