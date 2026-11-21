#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>

enum {
    CAR_SETUP, CAR_READY, CAR_FOLLOW, CAR_JUNCTION, CAR_SCAN,
    CAR_DETOUR, CAR_REACQUIRE, CAR_STOPPED
};
enum { SETUP_WHITE, SETUP_BLACK, SETUP_WAIT, SETUP_BAD };

typedef struct {
    uint32_t distance_mm;
    uint16_t front_mm : 12;
    uint16_t max_hump_mm : 10;
    uint16_t obstacle_width_mm : 10;
    int16_t speed_mm_s : 10;
    int16_t yaw_deg_s : 10;
    uint8_t faults;
    uint8_t phase : 3;
    uint8_t terrain : 3;
    uint8_t barcode : 5;
    uint8_t plan : 3;
    bool line : 1;
    bool magnetic_rate : 1;
} car_status_t;

/* Called by the GPIO IRQ for the two active-low board buttons. */
void controller_button_pressed(unsigned pin);
bool controller_start(void);
car_status_t controller_status(void);

#endif
