#include "motion.h"
#include "../hardware.h"
#include "../car_config.h"

#define POSITION_LIMIT_MM 1800
#define STALL_US 450000u

typedef struct {
    uint32_t last_edge;
    int32_t position_um;
    int32_t speed_q8 : 22;
    int32_t goal_speed : 9;
    bool reverse : 1;
    int16_t integral;
    int8_t last_error;
} wheel_t;

static wheel_t wheels[2];
static const uint16_t distance_per_edge[2] = { LEFT_UM_PER_EDGE, RIGHT_UM_PER_EDGE };
static uint32_t last_update;
static uint32_t travelled_mm;
static uint16_t travel_remainder;
static uint32_t forward_um;
static uint32_t move_start;
static uint32_t move_timeout;
static uint32_t goal_um[2]; /* modular signed positions */
static bool positioned;
static bool timing_started;

static int clamp(int value, int lo, int hi)
{
    if (value < lo) { return lo; }
    return value > hi ? hi : value;
}

static int sign(int value)
{
    return (value > 0) - (value < 0);
}

static unsigned magnitude(int value)
{
    return value < 0 ? (unsigned)(-(int64_t)value) : (unsigned)value;
}

void motion_stop(void)
{
    positioned = false;
    wheels[0].goal_speed = 0;
    wheels[1].goal_speed = 0;
    wheels[0].integral = 0;
    wheels[1].integral = 0;
    wheels[0].last_error = 0;
    wheels[1].last_error = 0;
    hardware_brake(true);
}

void motion_velocity(int left_mm_s, int right_mm_s)
{
    if (!positioned) {
        wheels[0].goal_speed = (int16_t)clamp(left_mm_s, -DRIVE_MAX_MM_S, DRIVE_MAX_MM_S);
        wheels[1].goal_speed = (int16_t)clamp(right_mm_s, -DRIVE_MAX_MM_S, DRIVE_MAX_MM_S);
        hardware_brake(false);
    }
}

static bool relative_move(int left_mm, int right_mm)
{
    if (positioned || (car_faults & FATAL_FAULTS) != 0u) {
        return false;
    }
    motion_stop();
    goal_um[0] = (uint32_t)wheels[0].position_um + (uint32_t)(left_mm * 1000);
    goal_um[1] = (uint32_t)wheels[1].position_um + (uint32_t)(right_mm * 1000);
    move_start = app_us();
    move_timeout = 2200000u + 17500u * magnitude(left_mm);
    positioned = true;
    hardware_brake(false);
    return true;
}

bool motion_move(int mm)
{
    return mm != 0 && magnitude(mm) <= POSITION_LIMIT_MM && relative_move(mm, mm);
}

bool motion_turn(int degrees_left)
{
    if (degrees_left == 0 || magnitude(degrees_left) > 180u) {
        return false;
    }
    int product = TRACK_MM * 31416 * degrees_left;
    int wheel_mm = (product + sign(product) * 1800000) / 3600000;
    return wheel_mm != 0 && relative_move(-wheel_mm, wheel_mm);
}

motion_sample_t motion_sample(void)
{
    /* All callers are on the control core. The controller is the sole writer.
     * Read both wheels in one critical section so lower-priority tasks see one update. */
    UW key;
    motion_sample_t sample;
    int left;
    int right;

    DI(key);
    left = wheels[0].speed_q8 / 256;
    right = wheels[1].speed_q8 / 256;
    sample.speed_mm_s = (int16_t)((left + right) / 2);
    sample.yaw_deg_s = (int16_t)((right - left) * 573 / (10 * TRACK_MM));
    sample.forward_um = forward_um;
    sample.travelled_mm = travelled_mm;
    EI(key);
    return sample;
}

static int wheel_duty(wheel_t *wheel, unsigned dt, uint32_t now)
{
    int target = wheel->goal_speed;
    int direction = sign(target);
    bool reverse_target = direction < 0;
    int speed = wheel->speed_q8 / 256;
    int duty;
    int error;

    /* Brake before changing inferred encoder direction. A single channel
     * cannot detect the sign of externally pushed or slipping wheels. */
    if (target == 0 || (reverse_target != wheel->reverse && magnitude(speed) > 18u)) {
        wheel->integral = 0;
        wheel->last_error = 0;
        wheel->last_edge = now;
        return 0;
    }
    if (wheel->reverse != reverse_target) {
        wheel->reverse = reverse_target;
        wheel->last_edge = now;
        wheel->speed_q8 = 0;
        speed = 0;
    }
    if (now - wheel->last_edge > STALL_US) {
        car_fault(FAULT_MOTION);
        return 0;
    }

    error = (int)magnitude(target) - speed * direction;
    int integral = clamp(wheel->integral + error * (int)(dt / 1000u), -18500, 30000);
    int derivative = clamp(error - wheel->last_error, -36, 36);
    wheel->last_error = (int8_t)clamp(error, -127, 127);
    duty = 135 + (int)magnitude(target) * 2 + 3 * error + integral / 100 - derivative / 2;
    if ((duty < 900 || error < 0) && (duty > 0 || error > 0)) {
        wheel->integral = integral;
    }
    return direction * clamp(duty, 0, 900);
}

move_result_t motion_update(uint32_t now)
{
    uint8_t count[2];
    unsigned dt = now - last_update;
    bool done = true;
    int duty[2];
    int32_t forward_step = 0;
    unsigned travel_step = 0;

    bool intact = hardware_take_edges(count);
    if (!timing_started) {
        timing_started = true;
        dt = CONTROL_US;
        for (unsigned i = 0; i < 2; ++i) {
            wheels[i].last_edge = now;
        }
    }
    last_update = now;
    if (dt == 0u || dt > 35000u) {
        if (positioned || wheels[0].goal_speed || wheels[1].goal_speed) { car_fault(FAULT_TIMING); }
        dt = CONTROL_US;
    }
    for (unsigned i = 0; i < 2; ++i) {
        wheel_t *wheel = &wheels[i];
        unsigned edges = count[i];
        unsigned distance = edges * distance_per_edge[i];
        if (!intact) {
            car_fault(FAULT_MOTION);
            distance = 0;
        } else if (edges > 0u) {
            wheel->last_edge = now;
        }
        wheel->position_um = (int32_t)((uint32_t)wheel->position_um
                                        + (uint32_t)((int)distance * (wheel->reverse ? -1 : 1)));
        forward_step += (int32_t)distance * (wheel->reverse ? -1 : 1);
        travel_step += distance;
        int instantaneous_q8 = (int)((uint64_t)distance * 256000u / dt) * (wheel->reverse ? -1 : 1);
        wheel->speed_q8 += (instantaneous_q8 - wheel->speed_q8) / 4;
        if (positioned) {
            int32_t remaining = (int32_t)(goal_um[i] - (uint32_t)wheel->position_um);
            if (magnitude(remaining) < 1800u) {
                wheel->goal_speed = 0;
                if (magnitude(wheel->speed_q8) > 22u * 256u) { done = false; }
            } else {
                done = false;
                wheel->goal_speed = (int16_t)(sign(remaining)
                    * clamp((int)(magnitude(remaining) / 430u), 43, 115));
            }
        }
        duty[i] = wheel_duty(wheel, dt, now);
    }
    forward_um += (uint32_t)(forward_step / 2);
    /* Keep half-micrometre contributions until the next full millimetre. */
    travel_step += travel_remainder;
    travelled_mm += travel_step / 2000u;
    travel_remainder = (uint16_t)(travel_step % 2000u);
    if ((car_faults & FATAL_FAULTS) != 0u ||
        (positioned && now - move_start >= move_timeout)) {
        if (positioned && now - move_start >= move_timeout) { car_fault(FAULT_MOTION); }
        motion_stop();
        return MOVE_FAILED;
    }
    if (positioned && done) {
        motion_stop();
        return MOVE_COMPLETE;
    }
    hardware_drive(duty[0], duty[1]);
    return positioned ? MOVE_RUNNING : MOVE_IDLE;
}
