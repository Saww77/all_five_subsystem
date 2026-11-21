#ifndef HARDWARE_H
#define HARDWARE_H

#include "app.h"

bool hardware_init(void);
void hardware_watchdog_start(void);
void hardware_watchdog_feed(void);
void hardware_brake(bool engaged);
void hardware_drive(int left, int right);
bool hardware_take_edges(uint8_t count[2]);
bool hardware_button_held(void);
bool hardware_estop_held(void);
void hardware_servo(unsigned angle);
bool hardware_ping(uint16_t *generation);
typedef enum { ECHO_QUIET, ECHO_VALID, ECHO_ERROR } echo_result_t;
echo_result_t hardware_echo(uint16_t generation, uint16_t *pulse_us);

#endif
