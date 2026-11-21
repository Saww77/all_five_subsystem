#ifndef APP_H
#define APP_H

#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include <bsp/libbsp.h>

enum {
    EVENT_TICK = 1u, EVENT_STOP = 2u, EVENT_START = 4u,
    EVENT_RANGE = 8u, EVENT_CALIBRATE = 16u, EVENT_PROFILE = 32u
};
enum {
    FAULT_ESTOP = 1u, FAULT_COLLISION = 2u, FAULT_MOTION = 4u,
    FAULT_IMU = 8u, FAULT_LINE = 16u, FAULT_PATH = 32u,
    FAULT_TIMING = 64u, FAULT_NO_ECHO = 128u
};
#define FATAL_FAULTS (FAULT_ESTOP | FAULT_COLLISION | FAULT_MOTION | \
                      FAULT_IMU | FAULT_LINE | FAULT_PATH | FAULT_TIMING)

extern ID controller_events;
extern volatile uint8_t car_faults;
bool app_start(FP entry, PRI priority, unsigned stack);
bool app_periodic(ID events, unsigned period_ms);
ID app_events(void);
uint32_t app_us(void);
void car_fault(uint8_t faults);
void car_stop_command(void);

#endif
