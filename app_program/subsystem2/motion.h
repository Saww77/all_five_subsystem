#ifndef MOTION_H
#define MOTION_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint32_t travelled_mm;
    uint32_t forward_um; /* Signed displacement in modular two's complement. */
    int16_t speed_mm_s;
    int16_t yaw_deg_s;
} motion_sample_t;

typedef enum { MOVE_IDLE, MOVE_RUNNING, MOVE_COMPLETE, MOVE_FAILED } move_result_t;

void motion_stop(void);
void motion_velocity(int left_mm_s, int right_mm_s);
bool motion_move(int mm);
bool motion_turn(int degrees_left);
motion_sample_t motion_sample(void);
move_result_t motion_update(uint32_t now);

#endif
