#include "controller.h"
#include "../app.h"
#include "../car_config.h"
#include "../hardware.h"
#include "../subsystem2/motion.h"
#include "../subsystem1/routes.h"
#include "../subsystem3/line.h"
#include "../subsystem4/terrain.h"
#include "../subsystem5/obstacle.h"

enum { NAV_APPROACH, NAV_TURN };
enum { DETOUR_TURN, DETOUR_OUT, DETOUR_FACE, DETOUR_PAST, DETOUR_INWARD,
       DETOUR_CROSS, DETOUR_RESTORE, DETOUR_REVERSE };

typedef struct {
    uint32_t last_start;
    uint32_t last_capture;
    uint32_t line_at;
    uint32_t state_at;
    uint32_t run_origin;
    uint32_t follow_origin;
    uint32_t nav_origin;
    uint32_t range_after;
    uint32_t crossing_origin;
    int16_t last_error;
    int8_t steering;
    uint8_t phase : 3;
    uint8_t setup : 2;
    uint8_t nav : 1;
    uint8_t detour : 3;
    uint8_t route : 3;
    uint8_t retries : 1;
    uint8_t hunt : 2;
    uint8_t lock_on : 3;
    uint8_t barcode_id;
    uint8_t last_line_sequence;
    uint8_t observed_barcode : 5;
    bool range_needed : 1;
    bool center_on_line : 1;
    bool run_started : 1;
    bool scan_requested : 1;
    obstacle_t obstacle;
} mission_t;

static car_status_t shared;
static mission_t mission;
/* Written by the controller and read by the GPIO ISR on the same core. */
static volatile bool calibration_mode = true;

void controller_button_pressed(unsigned pin)
{
    bool calibrating = calibration_mode;

    if (pin == PIN_ESTOP) {
        if (calibrating) {
            (void)tk_set_flg(controller_events, EVENT_CALIBRATE);
        } else {
            car_fault(FAULT_ESTOP);
        }
    } else if (pin == PIN_START && !calibrating) {
        (void)tk_set_flg(controller_events, EVENT_START);
    }
}

static int clamp(int value, int lo, int hi)
{
    return value < lo ? lo : value > hi ? hi : value;
}

static int absolute(int value)
{
    return value < 0 ? -value : value;
}

static void transition(unsigned phase, uint32_t now)
{
    calibration_mode = phase == CAR_SETUP;
    mission.phase = (uint8_t)phase;
    mission.state_at = now;
    terrain_set_armed(phase >= CAR_FOLLOW && phase <= CAR_REACQUIRE);
    line_read_barcode(phase == CAR_FOLLOW);
}

static void stop_fault(uint8_t fault, uint32_t now)
{
    car_fault(fault);
    motion_stop();
    transition(CAR_STOPPED, now);
}

static void begin_scan(uint32_t now, bool new_obstacle)
{
    if (new_obstacle) {
        mission.retries = 0u;
        mission.route = ROUTE_NONE;
        mission.hunt = 0u;
    }
    motion_stop();
    mission.scan_requested = false;
    transition(CAR_SCAN, now);
}

static bool line_fresh(line_sample_t line, uint32_t now)
{
    return line.calibrated && now - line.time_us < 40000u;
}

static bool range_fresh(range_t range, uint32_t now)
{
    return range.valid && !range.fault && now - range.at_us <= RANGE_MAX_AGE_US;
}

static void begin_follow(uint32_t now, line_sample_t line, uint32_t distance_mm)
{
    mission.steering = 0;
    mission.last_error = 0;
    mission.line_at = now;
    mission.follow_origin = distance_mm;
    mission.lock_on = 0;
    mission.barcode_id = line.barcode_id;
    transition(CAR_FOLLOW, now);
}

static void capture_button(uint32_t now)
{
    if (mission.phase != CAR_SETUP || mission.setup == SETUP_WAIT ||
        now - mission.last_capture < 230000u || !hardware_estop_held()) {
        return;
    }
    mission.last_capture = now;
    motion_stop();
    if (mission.setup == SETUP_BLACK) {
        line_calibrate(CAL_BLACK);
        tm_printf((UB *)"Sampling black\n");
    } else {
        line_calibrate(CAL_WHITE);
        tm_printf((UB *)"Sampling white\n");
    }
    mission.setup = SETUP_WAIT;
}

static void start_button(uint32_t now, line_sample_t line, terrain_sample_t terrain,
                         motion_sample_t motion, range_t range)
{
    if ((mission.phase != CAR_READY && mission.phase != CAR_STOPPED) ||
        now - mission.last_start < 230000u || !hardware_button_held()) {
        return;
    }
    mission.last_start = now;

    if (!terrain.ready) {
        tm_printf((UB *)"Start blocked: leave the car upright and still until the IMU settles.\n");
        return;
    }
    if (!line_fresh(line, now) || !line.visible) {
        tm_printf((UB *)"Start blocked: align the IR line sensors on the track.\n");
        return;
    }
    if (!range_fresh(range, now)) {
        tm_printf((UB *)"Start blocked: check the ultrasonic trigger and echo.\n");
        return;
    }
    /* Local start acknowledges course faults after repositioning. */
    const uint8_t recoverable = FAULT_ESTOP | FAULT_LINE | FAULT_PATH | FAULT_NO_ECHO;
    UW key;
    DI(key);
    bool safe = !hardware_estop_held() && (car_faults & FATAL_FAULTS & ~recoverable) == 0u;
    if (safe) { car_faults &= (uint8_t)~recoverable; }
    EI(key);
    if (!safe) {
        tm_printf((UB *)"Start blocked by fault mask %02x. Check GP20 and dashboard.\n", car_faults);
        return;
    }
#if APP_TEST
    tm_printf((UB *)"Bench: scanning with motor outputs inhibited.\n");
    begin_scan(now, true);
    return;
#endif
    mission.run_origin = motion.travelled_mm;
    mission.run_started = true;
    mission.route = ROUTE_NONE;
    mission.obstacle = (obstacle_t){0};
    mission.observed_barcode = 0u;
    terrain_begin_run();
    if (range.range_mm != 0u && range.range_mm <= STOP_RANGE_MM) {
        mission.follow_origin = motion.travelled_mm;
        begin_scan(now, true);
    } else {
        begin_follow(now, line, motion.travelled_mm);
    }
}

static void setup_step(uint32_t now)
{
    if (mission.setup != SETUP_WAIT) { return; }
    unsigned result = line_calibration();
    if (result == CAL_WHITE) {
        mission.setup = SETUP_BLACK;
        tm_printf((UB *)"Place all three sensors over matte black. Press GP20.\n");
    } else if (result == CAL_COMPLETE) {
        transition(CAR_READY, now);
        tm_printf((UB *)"Align on the track with GP27 over the barcode lane. Press GP21.\n");
    } else if (result == CAL_INVALID) {
        mission.setup = SETUP_BAD;
        tm_printf((UB *)"IR contrast too low. Place all sensors over white and press GP20.\n");
    }
}

static void steering(line_sample_t line, uint32_t now)
{
    int base = LINE_SPEED_MM_S;
    int error = 0;
    int demand = 0;

    if (line.visible) {
        bool reacquired = now - mission.line_at > 15000u;
        mission.line_at = now;
        if (!line.junction && absolute(line.error) > LINE_DEADBAND) {
            error = line.error - (line.error < 0 ? -LINE_DEADBAND : LINE_DEADBAND);
            int derivative = reacquired ? 0 : clamp((error - mission.last_error) / 36, -4, 4);
            int limit = line.barcode_busy ? BARCODE_MAX_STEER : LINE_MAX_STEER;
            demand = clamp(error / 34 + derivative, -limit, limit);
        }
    } else {
        /* Traverse small gaps while smoothly returning to straight steering. */
        base = LINE_MIN_MM_S + 6;
    }
    mission.steering += (int8_t)clamp(demand - mission.steering, -3, 3);
    mission.last_error = (int16_t)error;
    motion_velocity(clamp(base - mission.steering, LINE_MIN_MM_S, DRIVE_MAX_MM_S),
                    clamp(base + mission.steering, LINE_MIN_MM_S, DRIVE_MAX_MM_S));
}

static bool start_turn(int degrees, uint32_t now)
{
    if (motion_turn(degrees)) { return true; }
    stop_fault(FAULT_MOTION, now);
    return false;
}

static bool start_move(int mm, uint32_t now)
{
    if (motion_move(mm)) { return true; }
    stop_fault(FAULT_MOTION, now);
    return false;
}

static void reacquire(uint32_t now)
{
    motion_stop();
    mission.hunt = 0u;
    mission.lock_on = 0u;
    transition(CAR_REACQUIRE, now);
}

static void depart_junction(uint32_t now)
{
    int degrees = mission.route == ROUTE_LEFT ? 90 : mission.route == ROUTE_RIGHT ? -90 : 180;
    mission.nav = NAV_TURN;
    (void)start_turn(degrees, now);
}

static void navigate(line_sample_t line, motion_sample_t motion, uint32_t now)
{
    if (line.barcode_id != mission.barcode_id) {
        mission.barcode_id = line.barcode_id;
        mission.observed_barcode = line.letter;
        char letter = line.letter != 0u ? (char)('A' + line.letter - 1) : 0;
        unsigned action = route_config_action(letter);
        if (action != ROUTE_NONE && mission.route == ROUTE_NONE) {
            mission.route = (uint8_t)action;
            mission.nav_origin = line.barcode_at_mm;
        }
    }
    if (mission.route != ROUTE_NONE && line_fresh(line, now)) {
        unsigned since = motion.travelled_mm - mission.nav_origin;
        bool measured_offset = false;
#if BARCODE_TO_JUNCTION_MM > 0
        measured_offset = since >= BARCODE_TO_JUNCTION_MM;
#endif
        if (line.junction || measured_offset) {
            if (mission.route == ROUTE_STRAIGHT) {
                mission.route = ROUTE_NONE;
                steering(line, now);
                return;
            }
            transition(CAR_JUNCTION, now);
            motion_stop();
            if (line.junction) {
                mission.nav = NAV_APPROACH;
                (void)start_move(IR_TO_AXLE_MM, now);
            } else {
                depart_junction(now);
            }
            return;
        }
        if (since >= JUNCTION_WINDOW_MM ||
            (!line.visible && now - mission.line_at > LINE_LOSS_US)) {
            stop_fault(FAULT_PATH, now);
            return;
        }
    }
    if (!line_fresh(line, now)) {
        stop_fault(FAULT_LINE, now);
    } else if (!line.visible && now - mission.line_at > LINE_LOSS_US) {
        reacquire(now);
    } else {
        steering(line, now);
    }
}

static void junction_step(move_result_t result, uint32_t now)
{
    if (result != MOVE_COMPLETE) { return; }

    if (mission.nav == NAV_APPROACH) {
        depart_junction(now);
    } else {
        mission.route = ROUTE_NONE;
        reacquire(now);
    }
}

static void scan_step(uint32_t now, motion_sample_t motion, line_sample_t line,
                      range_t range, bool profile_event)
{
    obstacle_t output;

    if (!mission.scan_requested) {
        if (absolute(motion.speed_mm_s) <= 12 && absolute(motion.yaw_deg_s) <= 10) {
            mission.scan_requested = true;
            mission.state_at = now;
            sonar_scan();
        } else if (now - mission.state_at > 450000u) {
            stop_fault(FAULT_MOTION, now);
        }
        return;
    }
    if (!profile_event) {
        if (now - mission.state_at > 5400000u) { stop_fault(FAULT_PATH, now); }
        return;
    }
    if (!sonar_take_profile(&output)) {
        stop_fault(FAULT_PATH, now);
        return;
    }
    mission.obstacle = output;
#if APP_TEST
    tm_printf((UB *)"Bench: range=%u mm width=%u mm left=%u mm right=%u mm plan=%u\n",
              output.range_mm, output.width_mm, output.left_mm, output.right_mm, output.plan);
    transition(CAR_READY, now);
    return;
#endif
    if (!range_fresh(range, now)) {
        stop_fault(FAULT_PATH | FAULT_NO_ECHO, now);
        return;
    }
    if (output.plan == PLAN_CONTINUE) {
        if (range.range_mm == 0u || range.range_mm <= STOP_RANGE_MM) {
            stop_fault(FAULT_PATH, now);
        } else if (line.visible && line_fresh(line, now)) {
            begin_follow(now, line, motion.travelled_mm);
        } else {
            reacquire(now);
        }
    } else if (output.plan == PLAN_LEFT || output.plan == PLAN_RIGHT) {
        mission.detour = DETOUR_TURN;
        mission.center_on_line = false;
        mission.range_needed = false;
        transition(CAR_DETOUR, now);
        (void)start_turn(output.plan == PLAN_LEFT ? 90 : -90, now);
    } else if (output.plan == PLAN_REVERSE && !mission.retries &&
               motion.travelled_mm - mission.follow_origin >= 100u) {
        mission.retries = 1u;
        mission.range_needed = false;
        mission.detour = DETOUR_REVERSE;
        transition(CAR_DETOUR, now);
        (void)start_move(-80, now);
    } else {
        stop_fault(FAULT_PATH, now);
    }
}

static void wait_forward_beam(uint32_t now)
{
    mission.range_needed = true;
    mission.range_after = now + 90000u;
}

static void detour_step(move_result_t result, uint32_t now, range_t range,
                        line_sample_t line, uint32_t distance_mm)
{
    int side = mission.obstacle.plan == PLAN_LEFT ? 1 : -1;

    if (mission.range_needed) {
        if ((int32_t)(now - (mission.range_after + 340000u)) > 0) {
            stop_fault(FAULT_PATH, now);
        } else if (range_fresh(range, now) &&
                   (int32_t)(range.at_us - mission.range_after) >= 0) {
            mission.range_needed = false;
            unsigned distance = mission.detour == DETOUR_OUT ? mission.obstacle.offset_mm
                              : mission.detour == DETOUR_PAST ? mission.obstacle.forward_mm
                              : mission.obstacle.offset_mm + 65u;
            if (range.range_mm != 0u && range.range_mm < distance + HARD_RANGE_MM) {
                stop_fault(FAULT_PATH, now);
            } else {
                if (mission.detour == DETOUR_CROSS) { mission.crossing_origin = distance_mm; }
                (void)start_move((int)distance, now);
            }
        }
        return;
    }
    int crossing_min = (int)mission.obstacle.offset_mm - IR_TO_AXLE_MM - 18;
    if (mission.detour == DETOUR_CROSS && !mission.center_on_line &&
        line_fresh(line, now) && line.visible &&
        (int)(distance_mm - mission.crossing_origin) >= crossing_min) {
        if (line.sequence != mission.last_line_sequence) {
            mission.last_line_sequence = line.sequence;
            ++mission.lock_on;
        }
        if (mission.lock_on >= 3u) {
            motion_stop();
            mission.center_on_line = true;
            (void)start_move(IR_TO_AXLE_MM - 3, now);
            return;
        }
    } else {
        mission.lock_on = 0u;
    }
    if (result != MOVE_COMPLETE) { return; }

    switch (mission.detour) {
    case DETOUR_TURN:
    case DETOUR_FACE:
    case DETOUR_INWARD:
        ++mission.detour;
        wait_forward_beam(now);
        break;
    case DETOUR_OUT:
    case DETOUR_PAST:
        ++mission.detour;
        (void)start_turn(-side * 90, now);
        break;
    case DETOUR_CROSS:
        if (mission.center_on_line) {
            mission.detour = DETOUR_RESTORE;
            (void)start_turn(side * 90, now);
        } else {
            stop_fault(FAULT_LINE, now);
        }
        break;
    case DETOUR_RESTORE:
        reacquire(now);
        break;
    case DETOUR_REVERSE:
        begin_scan(now, false);
        break;
    default:
        stop_fault(FAULT_PATH, now);
        break;
    }
}

static void recover(move_result_t result, line_sample_t line, uint32_t now, uint32_t distance_mm)
{
    const int turns[] = { 18, -36, 36 };

    if (!line_fresh(line, now) || now - mission.state_at > 2800000u) {
        stop_fault(FAULT_LINE, now);
        return;
    }
    if (line.visible && absolute(line.error) <= 620) {
        if (line.sequence != mission.last_line_sequence) {
            mission.last_line_sequence = line.sequence;
            ++mission.lock_on;
        }
        if (mission.lock_on >= 4u) {
            motion_stop();
            begin_follow(now, line, distance_mm);
            return;
        }
    } else {
        mission.lock_on = 0u;
    }
    if (result == MOVE_RUNNING || mission.lock_on != 0u ||
        now - mission.state_at < 70000u) {
        return;
    }
    if (mission.hunt >= 3u) {
        stop_fault(FAULT_LINE, now);
        return;
    }
    int side = mission.last_error < 0 ? -1 : 1;
    int turn = turns[mission.hunt++];
    (void)start_turn(side * turn, now);
}

static bool forward_segment(void)
{
    if (mission.phase == CAR_JUNCTION) {
        return mission.nav == NAV_APPROACH;
    }
    return mission.phase == CAR_DETOUR && !mission.range_needed &&
           (mission.detour == DETOUR_OUT || mission.detour == DETOUR_PAST ||
            mission.detour == DETOUR_CROSS);
}

static void update_range(range_t range, uint32_t now)
{
    if (range.fault) {
        if (mission.phase >= CAR_FOLLOW && mission.phase <= CAR_REACQUIRE &&
            mission.phase != CAR_SCAN) {
            stop_fault(FAULT_PATH | FAULT_NO_ECHO, now);
        }
        return;
    }
    if (mission.phase == CAR_FOLLOW && range.range_mm != 0u &&
        range.range_mm <= STOP_RANGE_MM) {
        begin_scan(now, true);
    } else if (forward_segment() && range.range_mm != 0u && range.range_mm <= HARD_RANGE_MM) {
        stop_fault(FAULT_PATH, now);
    }
}

static void publish_status(motion_sample_t motion, line_sample_t line, terrain_sample_t terrain,
                           range_t range)
{
    car_status_t next = {
        .distance_mm = mission.run_started ? motion.travelled_mm - mission.run_origin : 0u,
        .front_mm = range.range_mm,
        .max_hump_mm = terrain.max_hump_mm,
        .obstacle_width_mm = mission.obstacle.width_mm,
        .speed_mm_s = clamp(motion.speed_mm_s, -512, 511),
        .yaw_deg_s = terrain.yaw_deg_s,
        .phase = mission.phase,
        .terrain = terrain.event,
        .barcode = mission.observed_barcode,
        /* While calibrating, the obstacle-plan field carries the setup step. */
        .plan = mission.phase == CAR_SETUP ? mission.setup + 1u : mission.obstacle.plan,
        .line = line.visible,
        .magnetic_rate = terrain.magnetic_rate,
    };
    UW key;
    DI(key);
    next.faults = car_faults;
    shared = next;
    EI(key);
}

car_status_t controller_status(void)
{
    UW key;
    car_status_t copy;

    DI(key);
    copy = shared;
    EI(key);
    return copy;
}

static void controller_task(INT argument, void *context)
{
    UINT events;
    uint32_t last_control = app_us();
    const UINT controller_mask = EVENT_TICK | EVENT_STOP | EVENT_START |
                                 EVENT_RANGE | EVENT_CALIBRATE | EVENT_PROFILE;

    (void)argument;
    (void)context;
    tm_printf((UB *)"Place all three IR sensors over white. GP20 captures. GP21 starts after calibration.\n");
    for (;;) {
        (void)tk_wai_flg(controller_events, controller_mask, TWF_ORW | TWF_BITCLR, &events, TMO_FEVR);
        uint32_t now = app_us();
        range_t range = sonar_range();
        line_sample_t line = line_sample();
        terrain_sample_t terrain = terrain_sample();
        motion_sample_t motion = motion_sample();
        bool range_update = (events & EVENT_RANGE) != 0u;

        if ((events & EVENT_STOP) != 0u || (car_faults & FATAL_FAULTS) != 0u) {
            motion_stop();
            if (mission.phase != CAR_SETUP) { transition(CAR_STOPPED, now); }
        }
        if ((events & EVENT_CALIBRATE) != 0u && (events & EVENT_STOP) == 0u) {
            capture_button(now);
        }
        if ((events & EVENT_START) != 0u && (events & EVENT_STOP) == 0u) {
            start_button(now, line, terrain, motion, range);
        }
        if (range_update) { update_range(range, now); }
        if ((events & EVENT_PROFILE) != 0u && mission.phase == CAR_SCAN && mission.scan_requested) {
            scan_step(now, motion, line, range, true);
        }
        if ((events & EVENT_TICK) == 0u) {
            publish_status(motion, line, terrain, range);
            continue;
        }

        /* Cyclic callbacks can coalesce. Enforce a physical minimum interval
         * and let the motion owner fault if a control deadline was missed. */
        if (now - last_control < 7000u) { continue; }
        last_control = now;
        hardware_watchdog_feed();
        if (mission.phase >= CAR_FOLLOW && mission.phase <= CAR_REACQUIRE &&
            mission.phase != CAR_SCAN && !range_fresh(range, now)) {
            stop_fault(FAULT_PATH | FAULT_NO_ECHO, now);
        }
        move_result_t result = motion_update(now);
        motion = motion_sample();
        if (result == MOVE_FAILED && mission.phase != CAR_SETUP) {
            transition(CAR_STOPPED, now);
        }
        switch (mission.phase) {
        case CAR_SETUP: setup_step(now); break;
        case CAR_FOLLOW: navigate(line, motion, now); break;
        case CAR_JUNCTION: junction_step(result, now); break;
        case CAR_SCAN: scan_step(now, motion, line, range, false); break;
        case CAR_DETOUR: detour_step(result, now, range, line, motion.travelled_mm); break;
        case CAR_REACQUIRE: recover(result, line, now, motion.travelled_mm); break;
        default: break;
        }
        publish_status(motion, line, terrain, range);
    }
}

bool controller_start(void)
{
    return app_periodic(controller_events, 10u) && app_start((FP)controller_task, 3, 1600);
}
