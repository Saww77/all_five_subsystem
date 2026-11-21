#include "telemetry.h"
#include <string.h>

static void bits(uint8_t *bytes, unsigned *offset, uint32_t value, unsigned count)
{
    unsigned position = *offset;
    *offset += count;
    while (count != 0u) {
        unsigned low = position & 7u;
        unsigned available = 8u - low;
        unsigned part = count < available ? count : available;
        uint32_t mask = (1u << part) - 1u;
        bytes[position / 8u] |= (uint8_t)((value & mask) << low);
        value >>= part;
        count -= part;
        position += part;
    }
}

void telemetry_encode(car_status_t status, uint8_t bytes[TELEMETRY_BYTES])
{
    unsigned offset = 0;

    memset(bytes, 0, TELEMETRY_BYTES);
    bits(bytes, &offset, TELEMETRY_VERSION, 4);
    bits(bytes, &offset, status.phase, 3);
    bits(bytes, &offset, status.terrain, 3);
    bits(bytes, &offset, status.barcode, 5);
    bits(bytes, &offset, status.plan, 3);
    bits(bytes, &offset, status.line, 1);
    bits(bytes, &offset, status.magnetic_rate, 1);
    bits(bytes, &offset, status.faults, 8);
    bits(bytes, &offset, status.distance_mm > 0xffffffu ? 0xffffffu : status.distance_mm, 24);
    bits(bytes, &offset, status.front_mm, 12);
    bits(bytes, &offset, status.max_hump_mm, 10);
    bits(bytes, &offset, status.obstacle_width_mm, 10);
    bits(bytes, &offset, (uint32_t)status.speed_mm_s, 10);
    bits(bytes, &offset, (uint32_t)status.yaw_deg_s, 10);
}
