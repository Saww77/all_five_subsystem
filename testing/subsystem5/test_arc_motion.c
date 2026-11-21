/* Exercise the production motor core with a deterministic clock and encoders. */
#include <assert.h>
#include <stdio.h>

#define APP_H
#define EXPORT
#define E_OK 0
#define DI(saved) ((saved) = 0)
#define EI(saved) ((void)(saved))
typedef int ER;
typedef int INT;
typedef unsigned UW;
#include "../../app_program/subsystem2/motion_core.c"

static uint64_t time_us;
static unsigned duty[32];

uint64_t s2_hw_time_us(void) { return time_us; }
void s2_hw_pwm_init(void) {}
ER s2_hw_encoder_init(void) { return E_OK; }
void s2_hw_pwm_write(unsigned pin, unsigned level) { duty[pin] = level; }

static void sample(unsigned left, unsigned right)
{
    for (unsigned i = 0; i < left; ++i)
        subsystem2_encoder_gpio_irq(LEFT_ENC_A, S2_GPIO_EDGE_RISE);
    for (unsigned i = 0; i < right; ++i)
        subsystem2_encoder_gpio_irq(RIGHT_ENC_A, S2_GPIO_EDGE_RISE);
    time_us += S2_PERIOD_MS * 1000u;
    s2_core_service();
}

static void run_arc(bool right)
{
    s2_core_init();
    assert(s2_core_set_speed(0));
    assert(s2_core_arc(right));
    assert(!s2_core_arc(!right));
    sample(0, 0);
    assert(duty[LEFT_IN_B] == 0 && duty[RIGHT_IN_B] == 0);
    assert(duty[LEFT_IN_A] > 0 && duty[RIGHT_IN_A] > 0);
    assert(right ? duty[LEFT_IN_A] > duty[RIGHT_IN_A] : duty[RIGHT_IN_A] > duty[LEFT_IN_A]);

    S2_STATUS status;
    s2_core_snapshot(&status);
    assert(!s2_core_arc_finished(right, &status));

    /* Model unequal drivetrain drag and carry fractional encoder pulses. */
    const unsigned pins[] = {LEFT_IN_A, RIGHT_IN_A};
    const double load[] = {48.0, 92.0};
    const double pulses_per_rpm[] = {LEFT_CPR_X10 / 60000.0, RIGHT_CPR_X10 / 60000.0};
    double speed[] = {0.0, 0.0};
    double fraction[] = {0.2, 0.8};
    for (unsigned tick = 0; tick < 800 && s2_core_busy(); ++tick) {
        unsigned pulses[2];
        for (unsigned wheel = 0; wheel < 2; ++wheel) {
            double effort = duty[pins[wheel]] - load[wheel];
            double target = effort > 0 ? effort / 6.0 : 0.0;
            speed[wheel] += (target - speed[wheel]) / 7.0;
            fraction[wheel] += speed[wheel] * pulses_per_rpm[wheel];
            pulses[wheel] = (unsigned)fraction[wheel];
            fraction[wheel] -= pulses[wheel];
        }
        sample(pulses[0], pulses[1]);
    }
    s2_core_snapshot(&status);
    assert(s2_core_arc_finished(right, &status));
    assert(!s2_core_arc_finished(!right, &status));
    assert(s2_get_bits(&status, 2, 3) == 0);
    assert(duty[LEFT_IN_A] == 0 && duty[RIGHT_IN_A] == 0);
}

int main(void)
{
    run_arc(false);
    run_arc(true);
    assert(s2_core_arc(true));
    sample(2, 1);
    s2_core_stop();
    S2_STATUS status;
    s2_core_snapshot(&status);
    assert(!s2_core_arc_finished(true, &status));
    assert(duty[LEFT_IN_A] == 0 && duty[RIGHT_IN_A] == 0);
    puts("PASS rolling turns in both directions, encoder completion and interrupted arc");
}
