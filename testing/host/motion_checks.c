#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include "fake_kernel.h"
#include "subsystem2/motion.c"

volatile uint8_t car_faults;
static bool braking = true;
static uint32_t now;
static int pwm[2];
static uint8_t synthetic[2];
static bool overflow;

uint32_t app_us(void) { return now; }
void car_fault(uint8_t faults) { car_faults |= faults; hardware_brake(true); }
void hardware_brake(bool stop) {
    braking = stop || car_faults;
    if (braking) { pwm[0] = pwm[1] = 0; }
}
void hardware_drive(int left, int right) {
    if (!braking) { pwm[0] = left; pwm[1] = right; }
}
bool hardware_take_edges(uint8_t count[2]) {
    count[0] = synthetic[0];
    count[1] = synthetic[1];
    return !overflow;
}

static move_result_t tick(unsigned left, unsigned right, unsigned us)
{
    synthetic[0] = (uint8_t)left;
    synthetic[1] = (uint8_t)right;
    now += us;
    return motion_update(now);
}

int main(void)
{
    assert(tick(0, 0, 10000) == MOVE_IDLE);
    motion_velocity(110, 110);
    for (unsigned i = 0; i < 30; ++i) {
        (void)tick(3, 3, 10000);
    }
    motion_sample_t sample = motion_sample();
    assert(sample.speed_mm_s >= 95 && sample.speed_mm_s <= 105);
    assert(sample.travelled_mm >= 28 && sample.travelled_mm <= 32);
    assert(abs(pwm[0] - pwm[1]) <= 8);
    motion_stop();
    assert(braking && pwm[0] == 0 && pwm[1] == 0);

    for (unsigned i = 0; i < 15u; ++i) { (void)tick(0, 0, 10000); }
    assert(motion_move(30));
    move_result_t result = MOVE_RUNNING;
    for (unsigned i = 0; i < 40u && result == MOVE_RUNNING; ++i) {
        unsigned left = i < 29u ? 3u : 0u;
        result = tick(left, left, 10000);
    }
    assert(result == MOVE_COMPLETE && braking);
    assert(motion_turn(90));
    (void)tick(0, 0, 10000);
    assert(pwm[0] < 0 && pwm[1] > 0);
    motion_stop();

    motion_velocity(100, 100);
    assert(tick(3, 3, 40000) == MOVE_FAILED && braking && (car_faults & FAULT_TIMING));
    car_faults = 0;
    motion_velocity(80, 80);
    (void)tick(2, 2, 10000);
    for (unsigned i = 0; i < 48; ++i) {
        (void)tick(0, 0, 10000);
    }
    assert(braking && (car_faults & FAULT_MOTION));

    car_faults = 0;
    motion_velocity(100, 100);
    overflow = true;
    assert(tick(100, 100, 10000) == MOVE_FAILED && braking);

    puts("PASS encoder speed, equal-wheel PID, encoder targets, brake, missed deadline and stall");
    return 0;
}
