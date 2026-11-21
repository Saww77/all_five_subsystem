#include <assert.h>
#include <stdio.h>
#include "fake_kernel.h"

#include "vehicle_controller/controller.c"
#include "subsystem1/route_config.c"

ID controller_events = 1;
volatile uint8_t car_faults;
static int last_move;
static int last_turn;
static bool braking;
static bool impact_armed;
static int requested_speed[2];
static unsigned scan_requests;
static line_sample_t line;
static obstacle_t profile;
static bool profile_valid;
static unsigned calibration_result;
static unsigned calibration_request;
static UINT pending_buttons;
static bool gp20_held;

/* These adapters are used by the RTOS entry point. The test exercises the
 * same state handlers directly, so an attempt to run the scheduler is an error. */
uint32_t app_us(void) { assert(false); return 0; }
void hardware_watchdog_feed(void) { assert(false); }
range_t sonar_range(void) { assert(false); return (range_t){0}; }
line_sample_t line_sample(void) { assert(false); return (line_sample_t){0}; }
terrain_sample_t terrain_sample(void) { assert(false); return (terrain_sample_t){0}; }
motion_sample_t motion_sample(void) { assert(false); return (motion_sample_t){0}; }
move_result_t motion_update(uint32_t now) { (void)now; assert(false); return MOVE_FAILED; }
bool app_periodic(ID events, unsigned period_ms)
{
    (void)events;
    (void)period_ms;
    assert(false);
    return false;
}
bool app_start(FP entry, INT priority, unsigned stack)
{
    (void)entry;
    (void)priority;
    (void)stack;
    assert(false);
    return false;
}
INT tk_wai_flg(ID id, UINT pattern, UINT mode, UINT *found, int timeout)
{
    (void)id;
    (void)pattern;
    (void)mode;
    (void)found;
    (void)timeout;
    assert(false);
    return -1;
}

static range_t beam(uint32_t now, unsigned mm)
{
    return (range_t){.at_us = now, .range_mm = mm, .valid = true};
}

void car_fault(uint8_t faults) { car_faults |= faults; braking = true; }
void motion_stop(void) { braking = true; }
void motion_velocity(int left, int right)
{
    assert(left >= 0 && right >= 0);
    requested_speed[0] = left;
    requested_speed[1] = right;
    braking = false;
}
bool motion_move(int mm) { last_move = mm; braking = false; return true; }
bool motion_turn(int degrees) { last_turn = degrees; braking = false; return true; }
bool hardware_button_held(void) { return true; }
bool hardware_estop_held(void) { return gp20_held; }
INT tk_set_flg(ID id, UINT pattern)
{
    assert(id == controller_events);
    pending_buttons |= pattern;
    return 0;
}
void line_read_barcode(bool enabled) { (void)enabled; }
void line_calibrate(unsigned surface) { calibration_request = surface; }
unsigned line_calibration(void) { return calibration_result; }
void terrain_set_armed(bool armed) { impact_armed = armed; }
void terrain_begin_run(void) { }
int tm_printf(UB *format, ...) { (void)format; return 0; }
void sonar_scan(void) { ++scan_requests; }
bool sonar_take_profile(obstacle_t *output)
{
    *output = profile;
    bool had = profile_valid;
    profile_valid = false;
    return had;
}

static void set_letter(char letter)
{
    line.letter = (uint8_t)(letter - 'A' + 1);
    ++line.barcode_id;
}

static void check_junction(void)
{
    uint32_t now = 1000000u;
    line = (line_sample_t){.calibrated = true, .visible = true, .time_us = now};
    motion_sample_t motion = {.travelled_mm = 350};
    terrain_sample_t terrain = {.ready = true};
    routes_t routes = routes_default();
    assert(routes_set(&routes, 'A', ROUTE_LEFT));
    assert(routes_set(&routes, 'X', ROUTE_RIGHT));
    route_config_init();
    assert(route_config_apply(&routes));

    mission.phase = CAR_READY;
    start_button(now, line, terrain, motion, beam(now, 0));
    assert(mission.phase == CAR_FOLLOW && impact_armed);
    set_letter('A');
    line.barcode_at_mm = motion.travelled_mm;
    navigate(line, motion, now);
    assert(mission.route == ROUTE_LEFT && mission.phase == CAR_FOLLOW && last_turn == 0);

    /* Make A go straight over MQTT while the left turn is already queued. */
    assert(routes_set(&routes, 'A', ROUTE_STRAIGHT));
    assert(route_config_apply(&routes));
    line.junction = true;
    navigate(line, motion, now);
    assert(mission.phase == CAR_JUNCTION && mission.nav == NAV_APPROACH);
    assert(last_move == IR_TO_AXLE_MM && last_turn == 0);
    junction_step(MOVE_COMPLETE, now);
    assert(last_turn == 90);
    junction_step(MOVE_COMPLETE, now);
    assert(mission.phase == CAR_REACQUIRE && mission.route == ROUTE_NONE && braking);

    begin_follow(now, line, motion.travelled_mm);
    set_letter('A');
    navigate(line, motion, now);
    assert(mission.phase == CAR_FOLLOW && mission.route == ROUTE_NONE && !braking);
    begin_follow(now, line, motion.travelled_mm);
    set_letter('X');
    line.junction = false;
    navigate(line, motion, now);
    motion.travelled_mm += JUNCTION_WINDOW_MM + 1;
    navigate(line, motion, now);
    assert(mission.phase == CAR_STOPPED && braking && !impact_armed && (car_faults & FAULT_PATH));
}

static void check_avoidance(void)
{
    mission = (mission_t){.phase = CAR_FOLLOW, .run_started = true};
    car_faults = 0;
    scan_requests = 0;
    braking = false;

    update_range(beam(2000000, 205), 2000000);
    assert(mission.phase == CAR_SCAN && braking && scan_requests == 0);
    scan_step(2010000, (motion_sample_t){.speed_mm_s = 40}, line, beam(2000000, 205), false);
    assert(scan_requests == 0);
    scan_step(2100000, (motion_sample_t){0}, line, beam(2000000, 205), false);
    assert(scan_requests == 1);
    profile = (obstacle_t){.plan = PLAN_LEFT, .offset_mm = 190, .forward_mm = 365};
    profile_valid = true;
    scan_step(3000000, (motion_sample_t){.travelled_mm = 170}, line, beam(3000000, 205), false);
    assert(mission.phase == CAR_SCAN && profile_valid);
    scan_step(3000000, (motion_sample_t){.travelled_mm = 170}, line, beam(3000000, 205), true);
    assert(mission.phase == CAR_DETOUR && last_turn == 90);
    range_t range = beam(3000000, 650);
    detour_step(MOVE_COMPLETE, 3000000, range, line, 170);
    assert(mission.range_needed && mission.detour == DETOUR_OUT);
    detour_step(MOVE_IDLE, 3100000, range, line, 170);
    assert(mission.range_needed);
    range = beam(3100000, 650);
    detour_step(MOVE_IDLE, 3100000, range, line, 170);
    assert(!mission.range_needed && last_move == 190);
    detour_step(MOVE_COMPLETE, 6000000, range, line, 360);
    assert(last_turn == -90 && mission.detour == DETOUR_FACE);
    detour_step(MOVE_COMPLETE, 7500000, range, line, 440);
    assert(mission.range_needed && mission.detour == DETOUR_PAST);
    range = beam(7600000, 650);
    detour_step(MOVE_IDLE, 7600000, range, line, 440);
    assert(last_move == 365);
    detour_step(MOVE_COMPLETE, 11500000, range, line, 805);
    assert(last_turn == -90 && mission.detour == DETOUR_INWARD);
    detour_step(MOVE_COMPLETE, 12900000, range, line, 890);
    range = beam(13010000, 0); /* A fresh quiet measurement on an open side. */
    detour_step(MOVE_IDLE, 13010000, range, line, 890);
    assert(last_move == 255 && mission.detour == DETOUR_CROSS);
    line.time_us = 15000000;
    for (unsigned i = 0; i < 3; ++i) {
        ++line.sequence;
        detour_step(MOVE_RUNNING, line.time_us, range, line, 1026);
    }
    assert(last_move == IR_TO_AXLE_MM - 3 && mission.center_on_line);
    detour_step(MOVE_COMPLETE, 15800000, range, line, 1090);
    assert(last_turn == 90 && mission.detour == DETOUR_RESTORE);
    detour_step(MOVE_COMPLETE, 17500000, range, line, 1175);
    assert(mission.phase == CAR_REACQUIRE && braking);
    line.time_us = 17700000;
    line.error = 30;
    for (unsigned i = 0; i < 4; ++i) {
        ++line.sequence;
        recover(MOVE_IDLE, line, line.time_us, 1175u);
    }
    assert(mission.phase == CAR_FOLLOW && impact_armed);

    mission.phase = CAR_DETOUR;
    mission.detour = DETOUR_PAST;
    update_range(beam(18000000, 80), 18000000);
    assert(mission.phase == CAR_STOPPED && braking && !impact_armed && (car_faults & FAULT_PATH));

    car_faults = 0;
    mission.phase = CAR_FOLLOW;
    range = beam(18100000, 0);
    range.fault = true;
    update_range(range, 18100000);
    assert(mission.phase == CAR_STOPPED && (car_faults & FAULT_NO_ECHO));
}

static void check_tracking(void)
{
    mission = (mission_t){0};
    line = (line_sample_t){.visible = true};
    for (int i = 0; i < 30; ++i) {
        line.error = i % 2 == 0 ? -40 : 40;
        steering(line, 10000u * (unsigned)i);
        assert(requested_speed[0] == LINE_SPEED_MM_S && requested_speed[1] == LINE_SPEED_MM_S);
    }
    line.error = 510;
    steering(line, 400000);
    assert(mission.steering == 3 && requested_speed[0] < requested_speed[1]);
    line.barcode_busy = true;
    for (unsigned i = 0; i < 12; ++i) { steering(line, 450000u + 10000u * i); }
    assert(mission.steering <= BARCODE_MAX_STEER);
    assert(requested_speed[0] + requested_speed[1] == 2 * LINE_SPEED_MM_S);
    line.visible = false;
    for (unsigned i = 0; i < 6; ++i) { steering(line, 600000u + 10000u * i); }
    assert(mission.steering == 0 && requested_speed[0] == requested_speed[1]);
}

static void check_setup(void)
{
    mission = (mission_t){0};
    line = (line_sample_t){0};
    car_faults = 0;
    braking = true;
    impact_armed = false;
    pending_buttons = 0;
    calibration_request = 0;
    calibration_mode = true;
    terrain_sample_t terrain = {.ready = true};
    motion_sample_t motion = {0};

    publish_status(motion, line, terrain, (range_t){0});
    assert(controller_status().plan == 1u);

    controller_button_pressed(PIN_START);
    assert(pending_buttons == 0u && calibration_request == 0u);
    start_button(500000u, line, terrain, motion, beam(500000u, 0));
    assert(mission.phase == CAR_SETUP && braking && !impact_armed);

    gp20_held = true;
    controller_button_pressed(PIN_ESTOP);
    assert(pending_buttons == EVENT_CALIBRATE && car_faults == 0);
    capture_button(1000000u);
    assert(calibration_request == CAL_WHITE && mission.phase == CAR_SETUP);
    assert(braking && !impact_armed);
    calibration_result = CAL_WHITE;
    setup_step(1150000u);
    publish_status(motion, line, terrain, (range_t){0});
    assert(controller_status().plan == 2u);

    capture_button(2000000u);
    assert(calibration_request == CAL_BLACK && mission.phase == CAR_SETUP);
    assert(braking && !impact_armed);
    calibration_result = CAL_INVALID;
    setup_step(2150000u);
    publish_status(motion, line, terrain, (range_t){0});
    assert(controller_status().plan == 4u && braking && !impact_armed);

    capture_button(3000000u);
    calibration_result = CAL_WHITE;
    setup_step(3150000u);
    capture_button(4000000u);
    calibration_result = CAL_COMPLETE;
    setup_step(4150000u);
    assert(mission.phase == CAR_READY && braking && !impact_armed && !calibration_mode);

    pending_buttons = 0;
    controller_button_pressed(PIN_START);
    assert(pending_buttons == EVENT_START && car_faults == 0);
    start_button(5000000u, line, terrain, motion, beam(5000000u, 0));
    assert(mission.phase == CAR_READY && braking && !impact_armed);
    gp20_held = false;
    line = (line_sample_t){.calibrated = true, .visible = true, .time_us = 6000000u};
    start_button(6000000u, line, terrain, motion, beam(5000000u, 0));
    assert(mission.phase == CAR_READY && braking);
    line.time_us = 6300000u;
    start_button(6300000u, line, terrain, motion, beam(6300000u, 0));
    assert(mission.phase == CAR_FOLLOW && impact_armed);
    navigate(line, motion, 6300000u);
    assert(!braking);

    pending_buttons = 0;
    controller_button_pressed(PIN_ESTOP);
    assert((car_faults & FAULT_ESTOP) != 0u && pending_buttons == 0u && braking);
}

int main(void)
{
    check_setup();
    check_tracking();
    check_junction();
    check_avoidance();
    puts("PASS braked IR setup, line steering, MQTT remapping at junction, detour, line recovery and obstacle stop");
    return 0;
}
