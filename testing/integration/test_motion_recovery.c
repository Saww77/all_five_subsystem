#include <assert.h>
#include <stdio.h>

#include "../../app_program/subsystem3/line_steering.h"
#include "../../app_program/vehicle_controller/vehicle_core.h"

static void check_steering(void)
{
    LF_STATE state = {0};
    LF_CMD out;

    for (int sample = 0; sample < 20; ++sample) {
        int noise = sample % 2 == 0 ? 25 : -25;
        line_decide(250 + noise, sample % 2 == 0 ? 1000 : 0, 250 - noise, &state, &out);
        assert(out.left == S3_CRUISE_RPM && out.right == S3_CRUISE_RPM && !out.stop);
    }
    for (int sample = 0; sample < 8; ++sample) {
        int before = state.correction_rpm;
        line_decide(80, 600, 950, &state, &out);
        assert(state.correction_rpm > 0 && state.correction_rpm <= S3_MAX_CORRECTION_RPM);
        assert(state.correction_rpm - before <= S3_CORRECTION_STEP_RPM);
        assert(out.left + out.right == 2 * S3_CRUISE_RPM);
        assert(out.right > 0 && !(out.left & 1) && !(out.right & 1));
    }
    line_decide(30, 0, 30, &state, &out);
    assert(state.correction_rpm == S3_MAX_CORRECTION_RPM - S3_CORRECTION_STEP_RPM);
    for (int sample = 0; sample < 5; ++sample) {
        line_decide(30, 1000, 30, &state, &out);
    }
    assert(out.left == S3_CRUISE_RPM && out.right == S3_CRUISE_RPM);

    line_decide(900, 0, 90, &state, &out);
    assert(out.left < out.right);
    line_decide(900, 0, 1000, &state, &out);
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
    check_start_stop();
    puts("PASS steering deadband and recentering, MQTT start, link loss, stale line and STOP priority");
    return 0;
}
