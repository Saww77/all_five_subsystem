#include "terrain.h"

/* atan with integer arithmetic. On the 0..45-degree octant the quadratic
 * correction is within roughly 0.25 degrees of atan. */
int terrain_atan(int rise, int run)
{
    int sign = rise < 0 ? -1 : 1;
    unsigned y = (unsigned)(rise < 0 ? -rise : rise);
    unsigned x = (unsigned)(run < 0 ? -run : run);

    if (x == 0u && y == 0u) { return 0; }
    bool steep = y > x;
    unsigned ratio = steep ? (unsigned)((uint64_t)x * 1024u / y) : (unsigned)((uint64_t)y * 1024u / x);
    int angle = (int)(ratio * (45000u + (1024u - ratio) * 15u) / 1024u);
    if (steep) { angle = 90000 - angle; }
    if (run < 0) { angle = 180000 - angle; }
    return sign * angle;
}

void terrain_hump_reset(hump_t *hump)
{
    hump->phase = 0;
    hump->height_um = 0;
    hump->peak_mm = 0;
    hump->count = 0;
}

void terrain_hump_update(hump_t *hump, int slope_millideg, unsigned distance_mm,
                         uint32_t position_mm, bool valid, bool moving)
{
    if (!valid || slope_millideg > 35000 || slope_millideg < -35000) {
        terrain_hump_reset(hump);
        return;
    }
    if (!moving && !(hump->phase == 2u && distance_mm == 0u &&
                     slope_millideg > -1800 && slope_millideg < 1800)) {
        return;
    }
    if (hump->phase == 0u) {
        if (slope_millideg > 3200) {
            if (hump->count == 0u) { hump->start_mm = position_mm; }
            if (++hump->count >= 4u) {
                hump->phase = 1u;
                hump->count = 0u;
            }
        } else {
            terrain_hump_reset(hump);
            return;
        }
    }
    if (position_mm - hump->start_mm > 750u) {
        terrain_hump_reset(hump);
        return;
    }

    /* sin(theta) ~= theta in radians on shallow slopes. Cubic correction
     * reduces the approximation error at the allowed 35-degree bound. */
    int millirad = slope_millideg * 1745 / 100000;
    int sin_milli = millirad - millirad * millirad * millirad / 6000000;
    hump->height_um += (int32_t)distance_mm * sin_milli;
    if (hump->height_um > (int32_t)hump->peak_mm * 1000) {
        hump->peak_mm = (uint16_t)((unsigned)hump->height_um / 1000u);
    }
    if (hump->phase == 1u && slope_millideg < -2500) {
        hump->phase = 2u;
    }
    if (hump->phase == 2u) {
        if (slope_millideg > -1800 && slope_millideg < 1800) {
            if (++hump->count >= 8u) {
                int peak = hump->peak_mm;
                if (peak >= 4 && hump->height_um < peak * 750 &&
                    hump->height_um > -12000 && hump->peak_mm > hump->max_mm) {
                    hump->max_mm = hump->peak_mm;
                }
                terrain_hump_reset(hump);
            }
        } else {
            hump->count = 0u;
        }
    }
}
