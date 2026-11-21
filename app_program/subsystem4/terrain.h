#ifndef TERRAIN_H
#define TERRAIN_H

#include <stdbool.h>
#include <stdint.h>

enum {
    TERRAIN_STILL, TERRAIN_CRUISING, TERRAIN_ACCELERATING, TERRAIN_TURNING,
    TERRAIN_CLIMB, TERRAIN_DESCENT, TERRAIN_TILT, TERRAIN_IMPACT
};

typedef struct {
    uint16_t max_hump_mm : 10;
    uint16_t event : 3;
    bool ready : 1;
    bool magnetic_rate : 1;
    int16_t yaw_deg_s : 10;
} terrain_sample_t;

typedef struct {
    int32_t height_um;
    uint32_t start_mm;
    uint16_t peak_mm;
    uint16_t max_mm;
    uint8_t count;
    uint8_t phase : 2;
} hump_t;

int terrain_atan(int rise, int run);
void terrain_hump_reset(hump_t *hump);
void terrain_hump_update(hump_t *hump, int slope_millideg, unsigned distance_mm,
                         uint32_t position_mm, bool valid, bool moving);

bool terrain_impact_sample(bool armed, int norm_sq, int up, int left, uint8_t *impacts);

bool terrain_start(void);
void terrain_set_armed(bool armed);
terrain_sample_t terrain_sample(void);
void terrain_begin_run(void);

#endif
