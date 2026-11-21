#ifndef OBSTACLE_H
#define OBSTACLE_H

#include <stdbool.h>
#include <stdint.h>

typedef enum { PLAN_STOP, PLAN_CONTINUE, PLAN_LEFT, PLAN_RIGHT, PLAN_REVERSE } plan_t;
typedef struct {
    uint32_t at_us;
    uint16_t range_mm : 12;
    bool valid : 1;
    bool fault : 1;
} range_t;

typedef struct {
    int16_t lateral_mm : 9;
    uint16_t range_mm : 10;
    uint16_t width_mm : 10;
    uint16_t left_mm : 9;
    uint16_t right_mm : 9;
    uint16_t forward_mm : 9;
    uint16_t offset_mm : 9;
    uint8_t plan : 3;
} obstacle_t;

typedef struct {
    uint16_t mm : 12;
    bool fault : 1;
    uint8_t angle : 8;
} scan_point_t;

obstacle_t obstacle_profile(const scan_point_t coarse[5], const scan_point_t *fine,
                            unsigned count);
bool sonar_start(void);
void sonar_scan(void);
range_t sonar_range(void);
bool sonar_take_profile(obstacle_t *profile);

#endif
