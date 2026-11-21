#include "obstacle.h"
#include "../hardware.h"
#include "../car_config.h"

#define SCAN_REQUEST 1u
#define SONAR_INTERVAL_US 62000u
#define SERVO_MS_PER_DEG 3u
#define COARSE_POINTS 5u
#define FINE_POINTS 9u

static ID scan_events;
static range_t range;
static obstacle_t latest;
static bool profile_ready;
static unsigned angle_now = 90u;
static uint32_t last_ping;

static scan_point_t measure(unsigned angle, bool extra_sample)
{
    unsigned difference = angle >= angle_now ? angle - angle_now : angle_now - angle;
    unsigned attempts = extra_sample ? 2u : 1u;
    unsigned nearest = 0u;
    bool fault = false;

    if (difference != 0u) {
        hardware_servo(angle);
        angle_now = angle;
        (void)tk_dly_tsk(38u + difference * SERVO_MS_PER_DEG);
    }
    for (unsigned i = 0; i < attempts; ++i) {
        uint16_t ticket;
        uint16_t duration;
        unsigned elapsed = app_us() - last_ping;
        if (elapsed < SONAR_INTERVAL_US) {
            (void)tk_dly_tsk((SONAR_INTERVAL_US - elapsed + 999u) / 1000u);
        }
        last_ping = app_us();
        echo_result_t result = hardware_ping(&ticket)
                             ? hardware_echo(ticket, &duration) : ECHO_ERROR;
        if (result == ECHO_VALID) {
            unsigned mm = ((unsigned)duration * 343u + 1000u) / 2000u;
            if (mm <= 4095u && (nearest == 0u || mm < nearest)) { nearest = mm; }
        } else if (result == ECHO_ERROR) {
            fault = true;
        }
    }
    return (scan_point_t){.mm = nearest, .angle = angle, .fault = nearest == 0u && fault};
}

static obstacle_t sweep(void)
{
    scan_point_t coarse[COARSE_POINTS];
    scan_point_t fine[FINE_POINTS];
    unsigned closest = 540u;
    unsigned mid = 90u;
    unsigned count = 0u;

    for (unsigned i = 0; i < COARSE_POINTS; ++i) {
        coarse[i] = measure(30u + 30u * i, true);
        if (coarse[i].fault) { return (obstacle_t){.plan = PLAN_STOP}; }
        if (coarse[i].mm != 0u && coarse[i].mm < closest) {
            closest = coarse[i].mm;
            mid = coarse[i].angle;
        }
    }
    if (closest < 540u) {
        unsigned first = mid < 50u ? 30u : mid > 130u ? 110u : mid - 20u;
        for (unsigned i = 0u; i < FINE_POINTS; ++i) {
            scan_point_t point = measure(first + 5u * i, false);
            if (!point.fault) { fine[count++] = point; }
        }
    }
    return obstacle_profile(coarse, fine, count);
}

void sonar_scan(void)
{
    UW key;

    DI(key);
    profile_ready = false;
    range.valid = false;
    EI(key);
    (void)tk_set_flg(scan_events, SCAN_REQUEST);
}

range_t sonar_range(void)
{
    UW key;
    range_t copy;

    DI(key);
    copy = range;
    EI(key);
    return copy;
}

bool sonar_take_profile(obstacle_t *profile)
{
    bool ready;
    UW key;

    DI(key);
    ready = profile_ready;
    if (ready) {
        *profile = latest;
        profile_ready = false;
    }
    EI(key);
    return ready;
}

static void sonar_task(INT argument, void *context)
{
    UINT flags;
    UW key;

    (void)argument;
    (void)context;
    for (;;) {
        ER event = tk_wai_flg(scan_events, SCAN_REQUEST, TWF_ORW | TWF_BITCLR, &flags,
                             SONAR_INTERVAL_US / 1000u);
        UINT notify = EVENT_RANGE;
        if (event >= E_OK) {
            obstacle_t profile = sweep();
            /* Restore the forward beam before returning drive authority. */
            scan_point_t center = measure(90u, false);
            DI(key);
            range = (range_t){.at_us = last_ping, .range_mm = center.mm,
                              .valid = true, .fault = center.fault};
            latest = profile;
            profile_ready = true;
            EI(key);
            notify |= EVENT_PROFILE;
        } else {
            scan_point_t center = measure(90u, false);
            DI(key);
            range = (range_t){.at_us = last_ping, .range_mm = center.mm,
                              .valid = true, .fault = center.fault};
            EI(key);
        }
        (void)tk_set_flg(controller_events, notify);
    }
}

bool sonar_start(void)
{
    scan_events = app_events();
    return scan_events > 0 && app_start((FP)sonar_task, 5, 1200);
}
