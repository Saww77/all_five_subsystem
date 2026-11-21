/* Host exercise of the production dispatcher with a simulated motion owner. */
#include <assert.h>
#include <stdio.h>
#include "../../app_program/vehicle_controller/vehicle_controller.c"

static uint32_t clock_ms;
static unsigned event_flags, stop_count, command_count, scan_count, clear_count;
static unsigned right_turn_count, left_turn_count;
static unsigned connection = 7;
static bool motor_busy;
static S2_STATUS motor;
static S2_REPLY motor_reply;
static uint16_t queued_command;

ID tk_cre_mtx(const T_CMTX *config) { assert(config->mtxatr == TA_INHERIT); return 1; }
ER tk_del_mtx(ID id) { assert(id == 1); return E_OK; }
ER tk_loc_mtx(ID id, TMO time) { assert(id == 1 && time == TMO_FEVR); return E_OK; }
ER tk_unl_mtx(ID id) { assert(id == 1); return E_OK; }
ID tk_cre_flg(const T_CFLG *config) { assert(!config->iflgptn); return 2; }
ER tk_set_flg(ID id, UINT bits) { assert(id == 2); event_flags |= bits; return E_OK; }
ER tk_get_otm(SYSTIM *time) { time->lo = clock_ms; return E_OK; }
INT tm_printf(const UB *format, ...) { (void)format; return 0; }
ID app_start_task(FP entry, PRI priority, UW cpu, const char *name)
{
    (void)entry; (void)priority; (void)cpu; (void)name;
    assert(0); /* The test drives the worker directly. */
    return E_OBJ;
}
ER tk_wai_flg(ID id, UINT wanted, UINT mode, UINT *seen, TMO timeout)
{
    (void)id; (void)wanted; (void)mode; (void)seen; (void)timeout;
    assert(0);
    return E_OBJ;
}

unsigned s1_connection_status(void) { return connection; }
uint32_t s2_core_timing_fault_gap_us(void) { return 0; }
bool s2_core_distance_reached(unsigned mm, const S2_STATUS *status)
{ (void)mm; (void)status; return false; }
unsigned s2_core_distance_travelled_mm(const S2_STATUS *status)
{ (void)status; return 55; }
bool s2_core_maneuver_finished(unsigned arg, const S2_STATUS *status)
{ (void)arg; return s2_get_bits(status, 0, 2) == 0; }
bool s2_core_arc_finished(bool right, const S2_STATUS *status)
{ (void)right; return s2_get_bits(status, 0, 2) == 0; }
bool motion_get_feedback(S2_STATUS *status, S2_REPLY *reply, bool *busy)
{
    *status = motor;
    if (reply) *reply = motor_reply;
    *busy = motor_busy;
    return true;
}

void motion_stop(void)
{
    ++stop_count;
    queued_command = 0;
    motor_busy = true;
}

bool motion_set_wheel_rpm_tracked(unsigned left, unsigned right, uint16_t *command)
{
    assert(!queued_command && left && right);
    ++command_count;
    queued_command = (uint16_t)(1 + left + right * 128);
    *command = queued_command;
    motor_busy = true;
    return true;
}

bool motion_clear_faults(void) { ++clear_count; return true; }
bool motion_maneuver(unsigned mm, bool right, uint16_t *command)
{
    assert(!queued_command);
    if (!mm) {
        if (right) ++right_turn_count;
        else ++left_turn_count;
    }
    *command = queued_command = (uint16_t)(1u + (mm << 4));
    motor_busy = true;
    return true;
}
bool motion_maneuver_reverse(unsigned mm, uint16_t *command)
{ (void)mm; (void)command; assert(0); return false; }
bool motion_arc(bool right, uint16_t *command)
{
    assert(!queued_command);
    *command = queued_command = (uint16_t)(S2_ARC | ((unsigned)right << 4));
    motor_busy = true;
    return true;
}
void subsystem5_request_scan(void) { ++scan_count; }

static void motor_acknowledge(unsigned reason)
{
    assert(queued_command);
    motor_reply.command = queued_command;
    queued_command = 0;
    ++motor_reply.sequence;
    motor_reply.reason = reason;
    motor_reply.accepted = reason == S2_REPLY_OK;
    motor_busy = motor_reply.accepted;
    s2_put_bits(&motor, 0, 2, motor_reply.accepted);
    vehicle_notify_feedback();
}

static void motor_stopped(void)
{
    motor_busy = false;
    s2_put_bits(&motor, 0, 2, 0);
    vehicle_notify_feedback();
}

static TMO work(void)
{
    assert(event_flags == VC_WAKE);
    event_flags = 0;
    return vehicle_service_once();
}

static unsigned vehicle_state(void)
{
    VEHICLE_SNAPSHOT snapshot;
    assert(vehicle_get_snapshot(&snapshot));
    return snapshot.state;
}

int main(void)
{
    uint32_t request;
    VEHICLE_SNAPSHOT snapshot;
    assert(vehicle_init() == E_OK);
    vehicle_enable_obstacle_monitor();
    vehicle_submit_line(0, 0, 0, 30, 30, FALSE, 0);
    assert(work() == TMO_FEVR);
    assert(vehicle_get_snapshot(&snapshot) && snapshot.obstacle_no_echo && snapshot.start_block == VC_START_READY);

    vehicle_submit_obstacle_range(S5_OPEN_RANGE);
    assert(vehicle_get_snapshot(&snapshot) && snapshot.obstacle_no_echo); /* A single open ping is insufficient. */
    assert(vehicle_request_start());
    vehicle_request_stop();
    assert(work() == TMO_FEVR && stop_count == 1 && !command_count);
    assert(!vehicle_request_start());
    motor_stopped();
    work();
    assert(vehicle_request_start());
    vehicle_submit_obstacle_range(S5_UNKNOWN_RANGE);
    assert(work() > 0 && vehicle_state() == VC_AVOIDING && !command_count);
    assert(scan_count == 1 && vehicle_obstacle_scan_request(&request, NULL));
    clock_ms += 20;
    vehicle_finish_obstacle_scan(NULL, request);
    assert(work() > 0 && scan_count == 2 && vehicle_obstacle_scan_request(&request, NULL));
    clock_ms += 20;
    vehicle_finish_obstacle_scan(NULL, request);
    assert(work() > 0 && !command_count && vehicle_state() == VC_AVOIDING);
    assert(vehicle_get_snapshot(&snapshot) && snapshot.avoidance_phase == S5_WAIT_PATH &&
           snapshot.obstacle_no_echo && snapshot.start_block == VC_START_READY);

    vehicle_submit_obstacle_range(S5_OPEN_RANGE);
    assert(!event_flags && !command_count);
    vehicle_submit_obstacle_range(S5_OPEN_RANGE);
    assert(work() > 0 && vehicle_state() == VC_FOLLOW_LINE && !command_count);
    vehicle_submit_line(0, 0, 0, 30, 30, FALSE, 0);
    assert(work() > 0 && command_count == 1);
    vehicle_submit_line(0, 0, 0, 24, 24, FALSE, 0);
    work();
    assert(command_count == 1);
    motor_acknowledge(S2_REPLY_SETTLING);
    work();
    assert(command_count == 1); /* Wait before retrying a settling rejection. */
    clock_ms += VC_REFRESH_MS;
    vehicle_submit_line(0, 0, 0, 30, 30, FALSE, 0);
    work();
    assert(command_count == 2);
    motor_acknowledge(S2_REPLY_OK);
    work();

    vehicle_submit_obstacle_range(S5_SCAN_STOP_MM);
    work();
    assert(vehicle_state() == VC_AVOIDING && stop_count == 2 && scan_count == 2);
    assert(!vehicle_obstacle_scan_request(&request, NULL));
    motor_stopped();
    work();
    assert(scan_count == 3 && vehicle_obstacle_scan_request(&request, NULL));
    assert(vehicle_obstacle_scan_current(request));

    connection = 0;
    vehicle_notify_feedback();
    assert(work() > 0 && vehicle_state() == VC_AVOIDING && vehicle_obstacle_scan_current(request));
    vehicle_request_stop();
    assert(work() == TMO_FEVR && vehicle_state() == VC_IDLE);
    assert(!vehicle_obstacle_scan_current(request));
    S5_PROFILE stale = {.nearest_mm = S5_SCAN_STOP_MM};
    vehicle_finish_obstacle_scan(&stale, request);
    assert(!event_flags && command_count == 2);

    /* A cancelled detour can be reset even with an empty motor fault mask. */
    connection = 7;
    vehicle_submit_obstacle_range(600);
    assert(vehicle_request_reset());
    assert(work() > 0 && pending.reset && !vehicle_request_start());
    motor_stopped();
    assert(work() == TMO_FEVR && !pending.reset && !clear_count);
    assert(vehicle_get_snapshot(&snapshot) && snapshot.state == VC_IDLE &&
           snapshot.avoidance_phase == S5_OFF && !snapshot.reset_pending);
    assert(command_count == 2);

    s2_put_bits(&motor, 5, 4, 4);
    vehicle_notify_feedback();
    assert(work() == TMO_FEVR && vehicle_state() == VC_FAULT);
    assert(vehicle_request_reset());
    assert(work() > 0 && !clear_count);
    motor_stopped();
    assert(work() > 0 && clear_count == 1 && pending.reset);
    s2_put_bits(&motor, 5, 4, 0);
    vehicle_notify_feedback();
    assert(work() == TMO_FEVR && !pending.reset);
    assert(vehicle_state() == VC_IDLE && command_count == 2);

    assert(vehicle_request_reset());
    work();
    clock_ms += VC_RESET_TIMEOUT_MS;
    vehicle_notify_feedback();
    assert(work() == TMO_FEVR && !pending.reset && motor_busy);
    motor_stopped();
    work();
    vehicle_submit_line(108, 0, 26, 30, 30, FALSE, 0);
    vehicle_submit_obstacle_range(606);
    assert(vehicle_request_start());
    work();
    motor_acknowledge(S2_REPLY_OK);
    work();
    vehicle_submit_obstacle_range(208);
    work();
    motor_stopped();
    work();
    assert(vehicle_obstacle_scan_request(&request, NULL));

    S5_PROFILE unresolved = {.nearest_mm = 208, .front_mm = 203, .width_mm = S5_UNKNOWN_WIDTH,
                             .angle_offset_deg = 60, .left_mm = S5_UNKNOWN_RANGE, .right_mm = S5_UNKNOWN_RANGE,
                             .left_path_mm = 80, .right_path_mm = S5_UNKNOWN_RANGE};
    uint32_t first_request = request;
    clock_ms += 20;
    vehicle_finish_obstacle_scan(&unresolved, request);
    assert(work() > 0 && vehicle_obstacle_scan_request(&request, NULL) && request != first_request);
    clock_ms += 20;
    vehicle_finish_obstacle_scan(&unresolved, request);
    assert(work() > 0 && vehicle_state() == VC_AVOIDING);
    assert(vehicle_get_snapshot(&snapshot) && snapshot.avoidance_phase == S5_WAIT_PATH &&
           snapshot.avoidance_failure == S5_FAIL_ECHO && snapshot.avoidance_failed_phase == S5_PROFILE_PATH);
    vehicle_submit_obstacle_range(606);
    vehicle_request_stop();
    work();
    assert(vehicle_get_snapshot(&snapshot) && snapshot.state == VC_IDLE && snapshot.obstacle_profile &&
           snapshot.start_block == VC_START_STOP_PENDING);
    assert(snapshot.near_mm == 208 && snapshot.obstacle_forward_mm == 606 && snapshot.width_mm == S5_UNKNOWN_WIDTH);
    motor_stopped();
    work();
    assert(vehicle_request_reset());
    work();
    motor_stopped();
    work();
    assert(vehicle_get_snapshot(&snapshot) && !snapshot.obstacle_profile &&
           snapshot.avoidance_failure == S5_FAIL_NONE && !snapshot.reset_pending);
    /* START with a close obstacle requests avoidance directly from standstill. */
    vehicle_submit_line(108, 0, 26, 30, 30, FALSE, 0);
    vehicle_submit_obstacle_range(208);
    unsigned before_start = command_count;
    assert(vehicle_request_start());
    assert(work() > 0 && vehicle_state() == VC_AVOIDING && command_count == before_start);
    assert(vehicle_obstacle_scan_request(&request, NULL));
    S5_PROFILE saved = {
        .nearest_mm = 196, .front_mm = 196, .width_mm = 294, .angle_offset_deg = 60,
        .left_path_mm = 208, .right_path_mm = 651, .left_edge_mm = -254, .right_edge_mm = 40,
        .left_edge_valid = true, .right_edge_valid = true,
    };
    unsigned scans_before = scan_count;
    clock_ms += 20;
    vehicle_submit_obstacle_range(S5_UNKNOWN_RANGE);
    vehicle_finish_obstacle_scan(&saved, request);
    assert(work() > 0 && vehicle_state() == VC_AVOIDING && !right_turn_count);
    assert(vehicle_get_snapshot(&snapshot) && snapshot.obstacle_profile && snapshot.obstacle_no_echo);
    clock_ms += 500;
    vehicle_submit_line(108, 0, 26, 30, 30, FALSE, 0);
    assert(work() > 0 && scan_count == scans_before && !right_turn_count);
    vehicle_submit_obstacle_range(196);
    assert(work() > 0 && scan_count == scans_before && !right_turn_count);

    clock_ms += 260;
    vehicle_submit_line(108, 0, 26, 30, 30, FALSE, 0);
    vehicle_submit_obstacle_range(196);
    assert(work() > 0 && right_turn_count == 1);
    assert(vehicle_get_snapshot(&snapshot) && snapshot.avoidance_phase == S5_TURN_OUT &&
           snapshot.width_mm == 294 && snapshot.avoidance_failure == S5_FAIL_NONE);
    motor_acknowledge(S2_REPLY_OK);
    work();
    motor_stopped();
    work();
    clock_ms += 260;
    vehicle_submit_line(108, 0, 26, 30, 30, FALSE, 0);
    vehicle_submit_obstacle_range(1000);
    assert(work() > 0 && queued_command && model.avoidance.phase == S5_SHIFT_OUT);
    motor_acknowledge(S2_REPLY_OK);
    work();

    scans_before = scan_count;
    unsigned stops_before = stop_count;
    vehicle_submit_obstacle_range(200);
    assert(work() > 0 && stop_count == stops_before + 1 && scan_count == scans_before);
    assert(!vehicle_obstacle_scan_request(&request, NULL));
    assert(model.avoidance.depth == 1 && model.avoidance.interrupted[0].progress_mm == 55);
    motor_stopped();
    assert(work() > 0 && scan_count == scans_before + 1 && vehicle_obstacle_scan_request(&request, NULL));
    assert(vehicle_get_snapshot(&snapshot) && snapshot.state == VC_AVOIDING);
    S5_PROFILE default_left = {.nearest_mm = 202, .front_mm = 190, .width_mm = 145, .angle_offset_deg = 65,
                               .left_mm = S5_UNKNOWN_RANGE, .right_mm = S5_UNKNOWN_RANGE,
                               .left_path_mm = S5_UNKNOWN_RANGE, .right_path_mm = 207,
                               .left_edge_mm = -90, .pivot_confirmed = true};
    clock_ms += 20;
    vehicle_submit_obstacle_range(S5_UNKNOWN_RANGE);
    vehicle_finish_obstacle_scan(&default_left, request);
    assert(work() > 0 && model.avoidance.phase == S5_TURN_OUT);
    assert(vehicle_get_snapshot(&snapshot) && snapshot.avoidance_side == S5_LEFT && !left_turn_count);
    clock_ms += 260;
    vehicle_submit_line(108, 0, 26, 30, 30, FALSE, 0);
    assert(work() > 0 && left_turn_count == 1 && scan_count == scans_before + 1);
    motor_acknowledge(S2_REPLY_OK);
    work();
    motor_stopped();
    work();
    clock_ms += 260;
    vehicle_submit_line(108, 0, 26, 30, 30, FALSE, 0);
    assert(work() > 0 && model.avoidance.phase == S5_SHIFT_OUT && !queued_command);
    vehicle_submit_obstacle_range(860);
    assert(work() > 0 && (queued_command >> 4) == 305);
    motor_acknowledge(S2_REPLY_OK);
    work();
    S5_PING missed = s5_classify_echo(S5_ECHO_QUIET, 0);
    clock_ms += 90;
    vehicle_submit_obstacle_ping(missed);
    assert(!event_flags);
    assert(vehicle_get_snapshot(&snapshot) && snapshot.obstacle_forward_mm == 860 && !snapshot.obstacle_no_echo);
    clock_ms += 90;
    vehicle_submit_line(108, 0, 26, 30, 30, FALSE, 0);
    stop_count = 0;
    vehicle_submit_obstacle_ping(missed);
    assert(work() > 0 && stop_count == 1 && model.avoidance.waiting_echo);
    assert(model.avoidance.leg_progress_mm == 55 && vehicle_state() == VC_AVOIDING);
    motor_stopped();
    work();
    clock_ms += 260;
    vehicle_submit_line(108, 0, 26, 30, 30, FALSE, 0);
    vehicle_submit_obstacle_range(860);
    assert(work() > 0 && !model.avoidance.waiting_echo);
    vehicle_notify_feedback();
    assert(work() > 0 && (queued_command >> 4) == 250);

    /* Reproduce the completed parallel leg with the motor still settling. */
    vehicle_request_stop();
    work();
    model = (VC_MODEL){.state = VC_AVOIDING, .avoidance = {
        .phase = S5_VERIFY_RETURN, .side = S5_RIGHT, .phase_at = clock_ms,
        .offset_mm = 235, .passed_mm = 300,
    }};
    vehicle_notify_feedback();
    scans_before = scan_count;
    work();
    assert(scan_count == scans_before && !vehicle_obstacle_scan_request(&request, NULL));
    motor_stopped();
    work();
    S5_SIDE check_side = S5_NO_SIDE;
    assert(scan_count == scans_before + 1 && vehicle_obstacle_scan_request(&request, &check_side));
    assert(check_side == S5_RIGHT);
    vehicle_finish_obstacle_scan(NULL, request);
    assert(vehicle_obstacle_scan_current(request) && !pending.return_received);
    vehicle_request_stop();
    work();
    assert(!vehicle_obstacle_scan_current(request));
    vehicle_finish_obstacle_return_check(S5_OPEN_RANGE, request);
    assert(!pending.return_received && !queued_command);

    puts("PASS MQTT loss, local STOP, left pivot, one missed echo and resume after echo recovery");
}
