#include <assert.h>
#include <stdio.h>

#include "../../app_program/subsystem3/line_steering.h"
#include "../../app_program/vehicle_controller/vehicle_core.h"

static void check_steering(void)
{
    LF_STATE state = {0};
    LF_CMD out;

    for (int sample = 0; sample < 40; ++sample) {
        int noise = sample % 2 == 0 ? 60 : -60;
        line_decide(250 + noise, sample % 2 == 0 ? 1000 : 0, 250 - noise, &state, &out);
        assert(out.left == S3_CRUISE_RPM && out.right == S3_CRUISE_RPM && !out.stop);
    }
    line_decide(40, 420, 980, &state, &out);
    line_decide(250, 420, 250, &state, &out);
    assert(out.left == S3_CRUISE_RPM && out.right == S3_CRUISE_RPM);
    assert(state.curve_side == 0);
}

static void check_steering_reversal(void)
{
    LF_STATE state = {0};
    LF_CMD out;

    for (unsigned i = 0; i < 12u; ++i) {
        line_decide(130, 0, 475, &state, &out);
    }
    assert(out.left > out.right);
    unsigned straight = 0;
    for (unsigned i = 0; i < 30u; ++i) {
        line_decide(380, 0, 130, &state, &out);
        if (out.left == out.right) {
            ++straight;
        } else if (out.left < out.right) {
            assert(straight >= 8u);
            return;
        }
    }
    assert(0 && "persistent error must still steer after the neutral interval");
}

static void check_curve(int side)
{
    LF_STATE state = {0};
    LF_CMD out;
    int left = side < 0 ? 950 : 80;
    int right = side > 0 ? 950 : 80;

    for (int sample = 0; sample < 35; ++sample) {
        int before = state.correction_rpm;
        line_decide(left, 600, right, &state, &out);
        assert((state.correction_rpm - before) * side >= 0);
        assert((state.correction_rpm - before) * side <= S3_CORRECTION_STEP_RPM);
        assert(out.left >= 0 && out.left <= S3_MAX_OUTER_RPM);
        assert(out.right >= 0 && out.right <= S3_MAX_OUTER_RPM);
        assert(!(out.left & 1) && !(out.right & 1) && !out.stop);
    }
    assert(state.curve_side == side);
    assert((side < 0 ? out.left : out.right) == 0);

    for (unsigned sample = 0; sample < 30u; ++sample) {
        line_decide(30, 950, 30, &state, &out);
        assert(!out.stop && state.curve_side == side);
        assert(side * (out.left - out.right) > 0);
    }

    /* A recovered line returns to neutral without flipping the correction. */
    for (int sample = 0; sample < 20; ++sample) {
        line_decide(430, 950, 430, &state, &out);
        assert(!out.stop && side * (out.left - out.right) >= 0);
    }
    assert(state.curve_side == 0);
    assert(out.left == S3_CRUISE_RPM && out.right == S3_CRUISE_RPM);
}

static void check_line_drop(void)
{
    LF_STATE state = {0};
    LF_CMD out;

    for (unsigned sample = 0; sample < 10u; ++sample) {
        line_decide(80, 600, 950, &state, &out);
    }
    assert(state.curve_side == 1);
    for (unsigned sample = 0; sample < 30u; ++sample) {
        line_decide(30, 1000, 30, &state, &out);
        assert(!out.stop);
    }
    assert(out.left == S3_CRUISE_RPM && out.right == S3_CRUISE_RPM);

    for (unsigned sample = 0; sample < 35u; ++sample) {
        line_decide(950, 0, 80, &state, &out);
    }
    for (unsigned sample = 0; sample < S3_BLIND_CURVE_LIMIT - 1u; ++sample) {
        line_decide(20, 0, 20, &state, &out);
        assert(!out.stop);
    }
    for (unsigned sample = 0; sample < 120u; ++sample) {
        line_decide(20, 0, 20, &state, &out);
        assert(out.stop && out.left == 0 && out.right == 0);
    }

    state = (LF_STATE){0};
    for (int sample = 0; sample < 30; ++sample) {
        line_decide(810, 0, 1000, &state, &out);
    }
    assert(out.left == out.right && !out.stop);
}

static void check_start_stop(void)
{
    VC_MODEL model = {0};
    VC_OUTPUT out;
    VC_INPUT input = {
        .start = true, .ready = true, .link_required = true, .line_valid = true,
        .left_rpm = 30, .right_rpm = 30
    };
    vc_core_step(&model, &input, &out);
    assert(!model.running && out.stop);
    input.link_ok = true;
    vc_core_step(&model, &input, &out);
    assert(model.running && !out.stop && out.left_rpm == 30 && out.right_rpm == 30);
    input.start = false;
    input.link_ok = false;
    vc_core_step(&model, &input, &out);
    assert(model.running && !out.stop);
    input.line_age_ms = VC_LINE_MAX_AGE_MS + 1u;
    vc_core_step(&model, &input, &out);
    assert(!model.running && out.stop && model.reason == VC_REASON_STALE_LINE);
    input.line_age_ms = 0;
    input.link_ok = true;
    vc_core_step(&model, &input, &out);
    assert(out.stop);
    input.start = true;
    input.stop = true;
    vc_core_step(&model, &input, &out);
    assert(!model.running && out.stop && model.reason == VC_REASON_STOP);
    input.start = false;
    input.stop = false;
    vc_core_step(&model, &input, &out);
    assert(out.stop);
    input.start = true;
    input.motor_faults = 1;
    vc_core_step(&model, &input, &out);
    assert(out.stop && model.reason == VC_REASON_MOTOR);
}

int main(void)
{
    check_steering();
    check_steering_reversal();
    check_curve(-1);
    check_curve(1);
    check_line_drop();
    check_start_stop();
    puts("PASS slow steering, both curves, line recovery and timeout, MQTT start and STOP priority");
    return 0;
}
