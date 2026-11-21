#ifndef FAKE_KERNEL_H
#define FAKE_KERNEL_H

#include <stdbool.h>
#include <stdint.h>

#define APP_H
typedef unsigned UW;
typedef unsigned UINT;
typedef int INT;
typedef int ID;
typedef unsigned char UB;
typedef void (*FP)(INT, void *);

#define DI(key) ((key) = 0u)
#define EI(key) ((void)(key))
#define TMO_FEVR (-1)
#define TWF_ORW 1u
#define TWF_BITCLR 2u

enum {
    EVENT_TICK = 1u, EVENT_STOP = 2u, EVENT_START = 4u,
    EVENT_RANGE = 8u, EVENT_CALIBRATE = 16u, EVENT_PROFILE = 32u
};
enum {
    FAULT_ESTOP = 1u, FAULT_COLLISION = 2u, FAULT_MOTION = 4u, FAULT_IMU = 8u,
    FAULT_LINE = 16u, FAULT_PATH = 32u, FAULT_TIMING = 64u, FAULT_NO_ECHO = 128u
};
#define FATAL_FAULTS 127u

extern ID controller_events;
extern volatile uint8_t car_faults;
uint32_t app_us(void);
void car_fault(uint8_t faults);
bool app_start(FP entry, INT priority, unsigned stack);
bool app_periodic(ID events, unsigned period_ms);
INT tk_set_flg(ID id, UINT pattern);
INT tk_wai_flg(ID id, UINT pattern, UINT mode, UINT *found, int timeout);
int tm_printf(UB *format, ...);

#endif
