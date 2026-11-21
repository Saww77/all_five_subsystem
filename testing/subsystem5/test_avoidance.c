/* Host tests for the production controller and planner. */
#include "../../app_program/vehicle_controller/vehicle_core.h"
#include "../../app_program/subsystem2/motion_config.h"
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
    in->avoidance.range_mm = scan.nearest_mm > S5_SCAN_STOP_MM ? scan.nearest_mm : S5_SCAN_STOP_MM;
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
    if (out->maneuver.action != action || out->maneuver.distance_mm != travel)
        fprintf(stderr, "phase=%u range=%u motion=%u/%u expected=%u/%u\n",
                model->avoidance.phase, in->avoidance.range_mm, out->maneuver.action,
                out->maneuver.distance_mm, action, travel);
    assert(out->maneuver.action == action && out->maneuver.distance_mm == travel);
    in->avoidance.motion = S5_MOTION_PENDING;
}

static void end_move(VC_MODEL *model, VC_INPUT *in, VC_OUTPUT *out)
{
    in->avoidance.motion = S5_MOTION_DONE;
    step(model, in, out, 500);
    in->avoidance.motion = S5_MOTION_NONE;
}

static void return_check(VC_MODEL *model, VC_INPUT *in, VC_OUTPUT *out, unsigned clearance)
{
    assert(model->avoidance.phase == S5_VERIFY_RETURN);
    in->avoidance.return_received = true;
    in->avoidance.return_at = in->avoidance.now + 20;
    in->avoidance.return_clear_mm = clearance;
    step(model, in, out, 20);
    assert(out->maneuver.action == S5_WAIT || out->maneuver.action == S5_STOP ||
           out->maneuver.action == S5_RESCAN);
    in->avoidance.return_received = false;
}

static void clear_rear_and_recheck(VC_MODEL *model, VC_INPUT *in, VC_OUTPUT *out)
{
    unsigned passed = model->avoidance.passed_mm;
    return_check(model, in, out, S5_OPEN_RANGE);
    assert(model->avoidance.phase == S5_PASS && model->avoidance.rear_clearance_pending);
    start_move(model, in, out, S5_ADVANCE, S5_REAR_CLEARANCE_MM);
    in->avoidance.travel_mm = 100; /* The old buffer ended while the chassis was beside the box. */
    step(model, in, out, 1000);
    assert(!out->stop && out->maneuver.action == S5_WAIT && model->avoidance.phase == S5_PASS);
    in->avoidance.travel_mm = 0;
    end_move(model, in, out);
    assert(model->avoidance.phase == S5_VERIFY_RETURN &&
           model->avoidance.passed_mm == passed + S5_REAR_CLEARANCE_MM);
    in->avoidance.return_received = true;
    step(model, in, out, 20); /* The reading from before the extra move is stale. */
    assert(model->avoidance.phase == S5_VERIFY_RETURN && out->maneuver.action == S5_WAIT);
    return_check(model, in, out, S5_OPEN_RANGE);
    assert(model->avoidance.phase == S5_TURN_IN);
}

static void reach_inward_turn(VC_MODEL *model, VC_INPUT *in, VC_OUTPUT *out, bool left, bool open_side)
{
    S5_ACTION outward = left ? S5_ARC_LEFT : S5_ARC_RIGHT;
    S5_ACTION inward = left ? S5_ARC_RIGHT : S5_ARC_LEFT;
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
    start_move(model, in, out, S5_ADVANCE, left ? 81 : 70);
    end_move(model, in, out);
    start_move(model, in, out, inward, 0);
    end_move(model, in, out);
    start_move(model, in, out, S5_ADVANCE, S5_SCAN_STOP_MM + 35 - (left ? 215 : 209));
    end_move(model, in, out);
    clear_rear_and_recheck(model, in, out);
    start_move(model, in, out, inward, 0);
    end_move(model, in, out);
    assert(model->state == VC_RECOVER_LINE && model->avoidance.phase == S5_FIND_LINE);
}

static void test_rejoin(bool left, bool open_side)
{
    VC_MODEL model = {0};
    VC_INPUT in = track_input();
    VC_OUTPUT out;
    reach_inward_turn(&model, &in, &out, left, open_side);
    S5_ACTION outward = left ? S5_ARC_LEFT : S5_ARC_RIGHT;
    unsigned travel = s5_return_tape_distance_mm(&model.avoidance) + S5_SEARCH_MARGIN_MM;
    start_move(&model, &in, &out, S5_ADVANCE, travel);
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
    assert(out.stop && out.maneuver.action == S5_STOP && model.avoidance.phase == S5_ALIGN_LINE);

    in.avoidance.line_hit = in.avoidance.line_junction = false;
    start_move(&model, &in, &out, outward, 0);
    end_move(&model, &in, &out);
    assert(model.avoidance.phase == S5_CHECK_LINE);
    in.avoidance.range_valid = false;
    step(&model, &in, &out, 260);
    assert(model.state == VC_RECOVER_LINE && out.maneuver.action == S5_WAIT);
    in.avoidance.range_valid = true;
    step(&model, &in, &out, 20);
    assert(model.state == VC_RECOVER_LINE); /* crossing evidence cannot authorize handoff */
    in.avoidance.line_center_seen = true;
    in.avoidance.line_at = in.avoidance.now + 20;
    step(&model, &in, &out, 20);
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
        unsigned travel = s5_return_tape_distance_mm(&model.avoidance) + S5_SEARCH_MARGIN_MM;
        start_move(&model, &in, &out, S5_ADVANCE, travel);
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
    in.avoidance.range_mm = 230;
    step(&model, &in, &out, 20);
    assert(model.state == VC_AVOIDING && model.avoidance.phase == S5_PROFILE_PATH);
    in.avoidance.profile = obstacle(true);
    in.avoidance.profile_valid = true;
    in.avoidance.profile_at = in.avoidance.now + 600;
    step(&model, &in, &out, 600);
    start_move(&model, &in, &out, S5_ARC_LEFT, 0);
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
                         .range_valid = true, .range_mm = 220, .sensor_to_axle_mm = 60};
    S5_AVOID_OUTPUT out = s5_avoid_step(&model, &in);
    assert(out.action == S5_STOP && model.phase == S5_FAILED);
}

static void test_width_and_side_clearance(void)
{
    S5_PROFILE scan = obstacle(true);
    S5_BYPASS plan = s5_plan_bypass(&scan);
    assert(plan.side == S5_LEFT);
    assert(plan.offset_mm == S5_SIDE_ALLOWANCE_MM + 50 && plan.pass_mm == S5_SCAN_STOP_MM + 35);
    assert(s5_reentry_detection_mm(plan.offset_mm, 60) == 70);
    scan.left_path_mm = scan.right_path_mm;
    assert(s5_plan_bypass(&scan).side == S5_RIGHT); /* Shorter shift leaves more space. */
    scan.left_edge_mm = -scan.right_edge_mm;
    assert(s5_plan_bypass(&scan).side == S5_LEFT);
    scan.front_mm = 114;
    plan = s5_plan_bypass(&scan);
    assert(plan.side == S5_NO_SIDE && plan.failure == S5_FAIL_ROOM);
    scan = obstacle(true);
    scan.front_mm = S5_SCAN_STOP_MM;
    scan.width_mm = S5_UNKNOWN_WIDTH;
    plan = s5_plan_bypass(&scan);
    assert(plan.side == S5_LEFT && plan.offset_mm == S5_SIDE_ALLOWANCE_MM + 50);
    scan = obstacle(true);
    scan.left_path_mm = S5_UNKNOWN_RANGE;
    assert(s5_plan_bypass(&scan).side == S5_RIGHT);

    scan.front_mm = 215;
    scan.left_path_mm = 150;
    scan.right_path_mm = 222;
    plan = s5_plan_bypass(&scan);
    assert(plan.side == S5_NO_SIDE && plan.failure == S5_FAIL_SIDE);
}

static void test_centered_and_offset_boxes(void)
{
    S5_PROFILE scan = obstacle(true);
    scan.width_mm = 200;
    scan.left_path_mm = scan.right_path_mm = 450;
    scan.left_edge_mm = -100;
    scan.right_edge_mm = 100;
    S5_BYPASS plan = s5_plan_bypass(&scan);
    assert(plan.side == S5_LEFT && plan.offset_mm == 100 + S5_SIDE_ALLOWANCE_MM);

    scan.left_edge_mm = -150;
    scan.right_edge_mm = 50;
    scan.left_path_mm = 450;
    scan.right_path_mm = 400;
    plan = s5_plan_bypass(&scan);
    assert(plan.side == S5_RIGHT && plan.offset_mm == 50 + S5_SIDE_ALLOWANCE_MM);
    scan.left_edge_mm = -50;
    scan.right_edge_mm = 150;
    scan.left_path_mm = 400;
    scan.right_path_mm = S5_OPEN_RANGE; /* A distant return used to outweigh the shorter left route. */
    plan = s5_plan_bypass(&scan);
    assert(plan.side == S5_LEFT && plan.offset_mm == 50 + S5_SIDE_ALLOWANCE_MM);
    scan.left_path_mm = 100;
    scan.left_mm = S5_OPEN_RANGE;
    plan = s5_plan_bypass(&scan); /* A close return on the left rules it out even with distant background. */
    assert(plan.side == S5_RIGHT);

    scan.left_edge_mm = -220;
    scan.right_edge_mm = -20;
    plan = s5_plan_bypass(&scan);
    assert(plan.side == S5_RIGHT && plan.offset_mm == S5_SIDE_ALLOWANCE_MM - 20);
    scan.left_edge_mm = -250;
    scan.right_edge_mm = -50;
    plan = s5_plan_bypass(&scan);
    assert(plan.side == S5_RIGHT && plan.offset_mm == S5_MIN_SIDE_SHIFT_MM);
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
    unsigned right_offset = 20 + S5_SIDE_ALLOWANCE_MM + S5_UNCERTAIN_EDGE_MM;
    assert(plan.offset_mm == right_offset && plan.pass_mm == S5_SCAN_STOP_MM + 35);
    scan.width_mm = 342;
    assert(s5_plan_bypass(&scan).offset_mm == right_offset); /* Never use the whole width for one side. */

    scan.left_path_mm = S5_OPEN_RANGE;
    scan.right_path_mm = S5_UNKNOWN_RANGE;
    plan = s5_plan_bypass(&scan);
    assert(plan.side == S5_LEFT && plan.offset_mm == 35 + S5_SIDE_ALLOWANCE_MM + S5_UNCERTAIN_EDGE_MM &&
           plan.pass_mm == S5_SCAN_STOP_MM + 35);

    scan.left_path_mm = 150;
    scan.right_path_mm = right_offset + 15 + S5_MANEUVER_STOP_MM - 1;
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
    assert(plan.side == S5_LEFT && plan.probe_left &&
           plan.offset_mm == 90 + S5_SIDE_ALLOWANCE_MM + S5_UNCERTAIN_EDGE_MM);

    S5_AVOID state = {.phase = S5_PROFILE_PATH, .phase_at = 100, .depth = 1};
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
    assert(move.action == S5_ADVANCE && move.distance_mm == plan.offset_mm);
    state.issued_mm = 0;
    in.range_mm = 207;
    move = s5_avoid_step(&state, &in);
    assert(move.action == S5_ADVANCE && move.distance_mm == 42);
    in.motion = S5_MOTION_PENDING;
    in.range_mm = 180;
    assert(s5_avoid_step(&state, &in).action == S5_WAIT);
    in.range_mm = 150;
    assert(s5_avoid_step(&state, &in).action == S5_STOP && state.depth == 2);

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
    assert(plan.side == S5_RIGHT && plan.offset_mm == 234 + S5_SIDE_ALLOWANCE_MM && plan.pass_mm == 253);

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
    start_move(&model, &in, &out, S5_ARC_RIGHT, 0);
    end_move(&model, &in, &out);
    assert(model.avoidance.phase == S5_SHIFT_OUT);

    VC_MODEL clear = model;
    VC_INPUT roomy = in;
    roomy.avoidance.range_mm = 1000;
    start_move(&clear, &roomy, &out, S5_ADVANCE, 294);
    end_move(&clear, &roomy, &out);
    start_move(&clear, &roomy, &out, S5_ARC_LEFT, 0);
    end_move(&clear, &roomy, &out);
    step(&clear, &roomy, &out, 260);
    assert(out.maneuver.action == S5_WAIT && clear.avoidance.phase == S5_VERIFY_RETURN &&
           clear.avoidance.passed_mm == 321); /* Both arcs already reached the side check. */

    in.avoidance.range_mm = 300;
    step(&model, &in, &out, 260);
    assert(out.stop && model.avoidance.failure == S5_FAIL_CLOSE);
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
    start_move(&model, &in, &out, S5_ARC_LEFT, 0);
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
    assert(out.action == S5_WAIT && state.phase == S5_WAIT_PATH && state.failure == S5_FAIL_ROOM);
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
    start_move(model, in, out, S5_ADVANCE, S5_SIDE_ALLOWANCE_MM + 40);
    end_move(model, in, out);
    start_move(model, in, out, S5_PIVOT_LEFT, 0);
    end_move(model, in, out);
    start_move(model, in, out, S5_ADVANCE, 280);
    end_move(model, in, out);
    start_move(model, in, out, S5_PIVOT_LEFT, 0);
    end_move(model, in, out);
    start_move(model, in, out, S5_ADVANCE, S5_SIDE_ALLOWANCE_MM + 40);
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
    assert(model.avoidance.phase == S5_VERIFY_RETURN && model.avoidance.passed_mm == 350);
    clear_rear_and_recheck(&model, &in, &out);
    in.avoidance.range_mm = 180;
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
    in.avoidance.range_mm = S5_TURN_CLEARANCE_MM;
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
    in.avoidance.line_center_seen = true;
    in.avoidance.range_mm = 200;
    in.obstacle_blocked = true;
    step(&model, &in, &out, 20);
    in.avoidance.line_at = 440;
    step(&model, &in, &out, 20);
    assert(model.state == VC_AVOIDING && model.avoidance.phase == S5_PROFILE_PATH);

    in.avoidance.range_mm = 230;
    in.avoidance.profile = obstacle(false);
    in.avoidance.profile_at = 600;
    in.avoidance.profile_valid = true;
    step(&model, &in, &out, 160);
    assert(model.avoidance.side == S5_RIGHT && model.avoidance.phase == S5_TURN_OUT);
    start_move(&model, &in, &out, S5_ARC_RIGHT, 0);
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
        in.avoidance.range_mm = S5_FOLLOW_CLEAR_MM + 1;
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

static void test_rounded_bypass(bool left)
{
    VC_MODEL model = {0};
    VC_INPUT in = track_input();
    VC_OUTPUT out;
    S5_PROFILE scan = obstacle(left);
    /* Confirm four rolling turns and the shorter first side check in either direction. */
    scan.nearest_mm = S5_FOLLOW_STOP_MM;
    scan.front_mm = S5_FOLLOW_STOP_MM;
    scan.width_mm = 200;
    scan.left_edge_mm = -100;
    scan.right_edge_mm = 100;
    S5_BYPASS plan = s5_plan_bypass(&scan);
    assert(plan.offset_mm == 100 + S5_SIDE_ALLOWANCE_MM);
    profile(&model, &in, &out, scan);
    S5_ACTION outward = left ? S5_ARC_LEFT : S5_ARC_RIGHT;
    S5_ACTION inward = left ? S5_ARC_RIGHT : S5_ARC_LEFT;
    in.avoidance.range_mm = S5_FOLLOW_STOP_MM;
    start_move(&model, &in, &out, outward, 0);
    end_move(&model, &in, &out);
    in.avoidance.range_mm = S5_OPEN_RANGE;
    in.obstacle_blocked = false;
    start_move(&model, &in, &out, S5_ADVANCE, 139);
    end_move(&model, &in, &out);
    start_move(&model, &in, &out, inward, 0);
    end_move(&model, &in, &out);
    assert(model.avoidance.offset_mm == plan.offset_mm && model.avoidance.leg_progress_mm == 244);
    unsigned first_pass = S5_FOLLOW_STOP_MM + 35;
    start_move(&model, &in, &out, S5_ADVANCE, first_pass - 244);
    end_move(&model, &in, &out);
    assert(model.avoidance.passed_mm == first_pass);

    in.avoidance.return_received = true;
    in.avoidance.return_at = in.avoidance.now - 1u;
    in.avoidance.return_clear_mm = S5_OPEN_RANGE;
    step(&model, &in, &out, 20);
    assert(model.avoidance.phase == S5_VERIFY_RETURN && out.maneuver.action == S5_WAIT);
    for (unsigned i = 0; i < 2; ++i) {
        return_check(&model, &in, &out, 110);
        assert(model.avoidance.phase == S5_PASS);
        start_move(&model, &in, &out, S5_ADVANCE, 60);
        end_move(&model, &in, &out);
    }
    clear_rear_and_recheck(&model, &in, &out);
    in.avoidance.range_valid = false;
    step(&model, &in, &out, 260);
    assert(model.avoidance.phase == S5_TURN_IN && out.maneuver.action == S5_WAIT);
    in.avoidance.range_valid = true;
    assert(model.avoidance.rounded_return);
    start_move(&model, &in, &out, inward, 0);
    end_move(&model, &in, &out);
    unsigned approach = s5_return_tape_distance_mm(&model.avoidance);
    assert(approach == 197);
    start_move(&model, &in, &out, S5_ADVANCE, approach + S5_SEARCH_MARGIN_MM);
    in.avoidance.near_original_line = true;
    in.avoidance.line_hit = true;
    for (unsigned i = 0; i < 2; ++i) {
        in.avoidance.line_at = in.avoidance.now + 20;
        step(&model, &in, &out, 20);
    }
    assert(model.avoidance.phase == S5_ALIGN_LINE && out.maneuver.action == S5_STOP);
    start_move(&model, &in, &out, outward, 0);
    end_move(&model, &in, &out);
    in.avoidance.line_hit = false;
    in.avoidance.line_centered = true;
    in.avoidance.line_at = in.avoidance.now + 260;
    step(&model, &in, &out, 260);
    assert(model.state == VC_RECOVER_LINE && model.avoidance.line_samples == 0);
    in.avoidance.line_center_seen = true;
    in.avoidance.line_at = in.avoidance.now + 20;
    step(&model, &in, &out, 20);
    step(&model, &in, &out, 20);
    assert(model.state == VC_RECOVER_LINE && model.avoidance.line_samples == 1);
    in.avoidance.line_at = in.avoidance.now + 20;
    step(&model, &in, &out, 20);
    assert(model.state == VC_FOLLOW_LINE);
}

static void test_recovery_uses_available_room_and_hands_off_on_edge(void)
{
    VC_MODEL model = {.state = VC_RECOVER_LINE, .avoidance = {
        .phase = S5_FIND_LINE, .side = S5_LEFT, .offset_mm = 220,
    }};
    VC_INPUT in = track_input();
    VC_OUTPUT out;
    in.avoidance.range_mm = 300;
    /* Room to reach the tape's IR crossing, but not to drive the entire 260 mm search. */
    start_move(&model, &in, &out, S5_ADVANCE, 190);
    in.avoidance.near_original_line = true;
    in.avoidance.range_mm = 170; /* Still enough room to centre the axle. */
    in.avoidance.line_hit = true;
    for (unsigned i = 0; i < 2; ++i) {
        in.avoidance.line_at = in.avoidance.now + 20;
        step(&model, &in, &out, 20);
    }
    assert(out.stop && model.avoidance.phase == S5_CENTER_LINE);
    start_move(&model, &in, &out, S5_ADVANCE, 60);
    end_move(&model, &in, &out);
    start_move(&model, &in, &out, S5_PIVOT_LEFT, 0);
    end_move(&model, &in, &out);
    in.avoidance.range_mm = 1500;
    in.avoidance.line_at = in.avoidance.now + 260; /* Tape still under one outer IR sensor. */
    step(&model, &in, &out, 260);
    in.avoidance.line_at = in.avoidance.now + 20;
    step(&model, &in, &out, 20);
    assert(model.state == VC_FOLLOW_LINE);
}

static void test_blank_floor_after_alignment_stops(void)
{
    VC_MODEL model = {.state = VC_RECOVER_LINE, .avoidance = {.phase = S5_CHECK_LINE}};
    VC_INPUT in = track_input();
    VC_OUTPUT out;
    in.avoidance.line_centered = true;
    in.avoidance.line_at = 300;
    step(&model, &in, &out, 300);
    assert(model.state == VC_RECOVER_LINE && out.maneuver.action == S5_WAIT);
    step(&model, &in, &out, 9000);
    assert(out.stop && model.state == VC_FAULT && model.avoidance.failure == S5_FAIL_LINE);
}

static void test_tape_already_under_sensor_at_start_of_search(void)
{
    VC_MODEL model = {.state = VC_RECOVER_LINE, .avoidance = {
        .phase = S5_FIND_LINE, .side = S5_LEFT, .offset_mm = 90,
    }};
    VC_INPUT in = track_input();
    VC_OUTPUT out;
    in.avoidance.near_original_line = true;
    in.avoidance.line_hit = true;
    in.avoidance.line_at = 260;
    step(&model, &in, &out, 260);
    assert(out.maneuver.action == S5_WAIT && model.avoidance.line_samples == 1);
    step(&model, &in, &out, 20);
    assert(out.maneuver.action == S5_WAIT && model.avoidance.line_samples == 1);
    in.avoidance.line_at = 300;
    step(&model, &in, &out, 20);
    assert(out.stop && model.avoidance.phase == S5_CENTER_LINE);
}

static void test_return_and_pivot_interlocks(void)
{
    assert(s5_return_clearance_mm(200, 300) == 141);
    assert(s5_return_clearance_mm(S5_OPEN_RANGE, 300) == 259);
    assert(s5_return_clearance_mm(S5_OPEN_RANGE, S5_UNKNOWN_RANGE) == S5_UNKNOWN_RANGE);
    S5_AVOID state = {.side = S5_LEFT, .phase = S5_VERIFY_RETURN, .phase_at = 100,
                      .offset_mm = 225, .passed_mm = S5_MAX_PASS_MM};
    S5_AVOID_INPUT in = {.now = 200, .return_at = 190, .stopped = true,
                        .return_received = true, .return_clear_mm = 110};
    assert(s5_avoid_step(&state, &in).action == S5_STOP && state.failure == S5_FAIL_CLOSE);
    state = (S5_AVOID){.phase = S5_VERIFY_RETURN, .phase_at = 100, .offset_mm = 225,
                       .passed_mm = 460, .rear_clearance_pending = true};
    S5_AVOID_OUTPUT blocked_again = s5_avoid_step(&state, &in);
    assert(blocked_again.action == S5_WAIT && state.phase == S5_PASS && !state.rear_clearance_pending);
    assert(state.next_pass_mm == S5_MAX_PASS_MM - 460 - S5_REAR_CLEARANCE_MM);
    state = (S5_AVOID){.phase = S5_VERIFY_RETURN, .phase_at = 100, .offset_mm = 225};
    in.return_clear_mm = S5_UNKNOWN_RANGE;
    assert(s5_avoid_step(&state, &in).action == S5_RESCAN);
    assert(state.phase == S5_VERIFY_RETURN && state.phase_at == 100 && state.return_scan_retried);
    in.return_received = false;
    assert(s5_avoid_step(&state, &in).action == S5_WAIT);
    in.return_received = true;
    S5_AVOID retry_failed = state;
    in.now = in.return_at = 800;
    assert(s5_avoid_step(&retry_failed, &in).action == S5_STOP && retry_failed.failure == S5_FAIL_ECHO);
    in.now = in.return_at = 4600;
    assert(s5_avoid_step(&state, &in).action == S5_STOP && state.failure == S5_FAIL_ECHO);
    in.now = in.return_at = 800;
    for (unsigned room = S5_SIDE_ALLOWANCE_MM - 1; room <= S5_SIDE_ALLOWANCE_MM; ++room) {
        state = (S5_AVOID){.phase = S5_VERIFY_RETURN, .phase_at = 100, .offset_mm = 220,
                           .rear_clearance_pending = true};
        in.sensor_to_axle_mm = 60;
        in.return_clear_mm = 220 + room;
        assert(s5_avoid_step(&state, &in).action == S5_WAIT && state.phase == S5_TURN_IN);
        assert(state.rounded_return == (room == S5_SIDE_ALLOWANCE_MM));
    }
    state = (S5_AVOID){.phase = S5_TURN_IN, .side = S5_LEFT};
    in.range_valid = true;
    in.range_at = in.now = 500;
    in.range_mm = S5_TURN_CLEARANCE_MM - 1;
    in.motion = S5_MOTION_PENDING;
    assert(s5_avoid_step(&state, &in).action == S5_STOP && state.failure == S5_FAIL_CLOSE);
}

static void test_side_scan_recovery_preserves_rear_clearance(void)
{
    VC_MODEL model = {.state = VC_AVOIDING, .avoidance = {
        .phase = S5_VERIFY_RETURN, .side = S5_RIGHT, .offset_mm = 220, .passed_mm = 265,
    }};
    VC_INPUT in = track_input();
    VC_OUTPUT out;
    in.avoidance.return_received = true;
    in.avoidance.return_clear_mm = S5_UNKNOWN_RANGE;
    in.avoidance.return_at = 20;
    step(&model, &in, &out, 20);
    assert(out.maneuver.action == S5_RESCAN && !model.avoidance.rear_clearance_pending);

    clear_rear_and_recheck(&model, &in, &out);
    assert(model.avoidance.passed_mm == 265 + S5_REAR_CLEARANCE_MM &&
           !model.avoidance.return_scan_retried && model.avoidance.rounded_return);
}

static void test_side_scan_from_ahead(bool rear_move_done)
{
    unsigned passed = rear_move_done ? 265 + S5_REAR_CLEARANCE_MM : 265;
    VC_MODEL model = {.state = VC_AVOIDING, .avoidance = {
        .phase = S5_VERIFY_RETURN, .side = S5_RIGHT, .offset_mm = 220, .passed_mm = passed,
        .rolling_route = true, .rear_clearance_pending = rear_move_done,
    }};
    VC_INPUT in = track_input();
    VC_OUTPUT out;
    return_check(&model, &in, &out, S5_UNKNOWN_RANGE);
    assert(out.maneuver.action == S5_RESCAN);
    return_check(&model, &in, &out, S5_UNKNOWN_RANGE);
    assert(model.avoidance.phase == S5_PASS && model.avoidance.side_probe_active);

    VC_MODEL blocked = model;
    VC_INPUT close = in;
    close.avoidance.range_mm = S5_SCAN_STOP_MM;
    step(&blocked, &close, &out, 260);
    assert(out.stop && blocked.avoidance.failure == S5_FAIL_CLOSE);

    VC_MODEL quiet = model;
    VC_INPUT missed = in;
    missed.avoidance.range_valid = false;
    missed.avoidance.echo_quiet = true;
    step(&quiet, &missed, &out, 260);
    assert(out.maneuver.action == S5_WAIT && quiet.avoidance.waiting_echo);
    step(&quiet, &missed, &out, 9000);
    assert(out.stop && quiet.avoidance.failure == S5_FAIL_ECHO);

    start_move(&model, &in, &out, S5_ADVANCE, 40);
    end_move(&model, &in, &out);
    assert(model.avoidance.passed_mm == passed + 40 && model.avoidance.rear_clearance_pending == rear_move_done);

    VC_MODEL unavailable = model;
    VC_INPUT no_return = in;
    return_check(&unavailable, &no_return, &out, S5_UNKNOWN_RANGE);
    assert(out.maneuver.action == S5_RESCAN);
    return_check(&unavailable, &no_return, &out, S5_UNKNOWN_RANGE);
    assert(out.stop && unavailable.avoidance.failure == S5_FAIL_ECHO);

    if (!rear_move_done) clear_rear_and_recheck(&model, &in, &out);
    else return_check(&model, &in, &out, S5_OPEN_RANGE);
    assert(model.avoidance.phase == S5_TURN_IN && model.avoidance.rounded_return);
    in.avoidance.range_mm = 180;
    step(&model, &in, &out, 260);
    assert(out.stop && model.avoidance.failure == S5_FAIL_CLOSE); /* No pivot on a tight return. */
}

static void test_compact_offset_still_rolls(void)
{
    VC_MODEL model = {0};
    VC_INPUT in = track_input();
    VC_OUTPUT out;
    S5_PROFILE scan = obstacle(false);
    scan.left_edge_mm = -240;
    scan.right_edge_mm = -50;
    profile(&model, &in, &out, scan);
    assert(model.avoidance.side == S5_RIGHT && model.avoidance.offset_mm == 120);
    start_move(&model, &in, &out, S5_ARC_RIGHT, 0);
    end_move(&model, &in, &out);
    in.avoidance.range_mm = S5_OPEN_RANGE;
    start_move(&model, &in, &out, S5_ADVANCE, 24);
    end_move(&model, &in, &out);
    start_move(&model, &in, &out, S5_ARC_LEFT, 0);
    end_move(&model, &in, &out);
    start_move(&model, &in, &out, S5_ADVANCE, 275 - 186);
    end_move(&model, &in, &out);
    clear_rear_and_recheck(&model, &in, &out);
    start_move(&model, &in, &out, S5_ARC_LEFT, 0);
}

static void test_rounded_departure_interlocks(void)
{
    S5_AVOID state = {.phase = S5_ALIGN_OUTSIDE, .side = S5_LEFT, .rounded_departure = true};
    S5_AVOID_INPUT in = {.now = 300, .range_at = 300, .range_valid = true, .range_mm = 180,
                         .stopped = true};
    assert(s5_avoid_step(&state, &in).action == S5_STOP && state.failure == S5_FAIL_CLOSE);

    state = (S5_AVOID){.phase = S5_TURN_OUT, .side = S5_LEFT, .rounded_departure = true};
    in.range_mm = S5_FOLLOW_STOP_MM;
    assert(s5_avoid_step(&state, &in).action == S5_ARC_LEFT);
    in.motion = S5_MOTION_PENDING;
    in.range_valid = false;
    assert(s5_avoid_step(&state, &in).action == S5_STOP && state.failure == S5_FAIL_ECHO);
}

static void test_widen_only_when_the_initial_lane_is_blocked(void)
{
    unsigned offset = 100 + S5_SIDE_ALLOWANCE_MM;
    VC_MODEL model = {.state = VC_AVOIDING, .avoidance = {
        .phase = S5_PASS, .side = S5_RIGHT, .offset_mm = offset, .next_pass_mm = 360,
    }};
    VC_INPUT in = track_input();
    VC_OUTPUT out;
    for (unsigned extra = S5_OUTWARD_STEP_MM; extra <= S5_OUTWARD_LIMIT_MM; extra += S5_OUTWARD_STEP_MM) {
        in.avoidance.range_mm = 210;
        step(&model, &in, &out, 260);
        assert(model.avoidance.phase == S5_TURN_OUT && model.avoidance.widening);
        VC_MODEL cancelled = model;
        in.stop = true;
        step(&cancelled, &in, &out, 20);
        assert(out.stop && cancelled.avoidance.phase == S5_FAILED);
        in.stop = false;

        start_move(&model, &in, &out, S5_PIVOT_RIGHT, 0);
        end_move(&model, &in, &out);
        in.avoidance.range_mm = 1000;
        start_move(&model, &in, &out, S5_ADVANCE, S5_OUTWARD_STEP_MM);
        end_move(&model, &in, &out);
        start_move(&model, &in, &out, S5_PIVOT_LEFT, 0);
        end_move(&model, &in, &out);
        assert(model.avoidance.offset_mm == offset + extra);
        assert(model.avoidance.phase == S5_PASS && !model.avoidance.widening);

        VC_MODEL clear_lane = model;
        start_move(&clear_lane, &in, &out, S5_ADVANCE, 360);
        in.avoidance.motion = S5_MOTION_NONE;
    }
    in.avoidance.range_mm = 210;
    step(&model, &in, &out, 260);
    assert(out.stop && model.avoidance.failure == S5_FAIL_CLOSE);
}

int main(void)
{
    test_rejoin(true, false);
    test_rejoin(false, false);
    test_rejoin(true, true);
    test_rejoin(false, true);
    test_abort_paths();
    test_link_loss_does_not_end_a_run();
    test_unsafe_profile_and_scan_timeout();
    test_reentry_limit();
    test_width_and_side_clearance();
    test_centered_and_offset_boxes();
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
    test_rounded_bypass(true);
    test_rounded_bypass(false);
    test_return_and_pivot_interlocks();
    test_side_scan_recovery_preserves_rear_clearance();
    test_side_scan_from_ahead(false);
    test_side_scan_from_ahead(true);
    test_compact_offset_still_rolls();
    test_rounded_departure_interlocks();
    test_recovery_uses_available_room_and_hands_off_on_edge();
    test_blank_floor_after_alignment_stops();
    test_tape_already_under_sensor_at_start_of_search();
    test_widen_only_when_the_initial_lane_is_blocked();
    puts("PASS rounded and tight-space bypasses, side clearance, tape confirmation and cancellation");
}
