#include "terrain.h"

bool terrain_impact_sample(bool armed, int norm_sq, int up, int left, uint8_t *impacts)
{
    if (!armed) {
        *impacts = 0u;
        return false;
    }
    if (norm_sq > 4200000 || norm_sq < 110000) {
        if (*impacts < 2u) { ++*impacts; }
    } else {
        *impacts = 0u;
    }
    return *impacts >= 2u || up < 280 || left < -850 || left > 850;
}
