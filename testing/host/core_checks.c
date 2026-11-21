#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "subsystem1/routes.h"
#include "subsystem1/telemetry.h"
#include "subsystem3/barcode.h"
#include "subsystem4/terrain.h"
#include "subsystem5/obstacle.h"

/* Reflectance runs sampled from Barcode Sample.pdf, scaled to the
 * 141 mm print size in Barcode & Line Specification. */
static const uint16_t printed[4][29] = {
    {3146,8866,3146,2860,9152,2860,9152,2860,3146,2860,9152,2860,3146,2860,3146,
     8580,3146,2860,9152,2860,3146,8866,3146,2860,9152,2860,9152,2860,3146},
    {3411,8528,3411,2843,9097,2843,9097,2843,3411,2843,9097,2843,3411,2274,3411,
     8528,3411,2843,9097,2843,3411,8528,3411,2274,9097,2843,9097,2843,3411},
    {3146,8866,3146,2860,9152,2860,9152,2860,3146,2860,3146,8866,9152,2574,9152,
     2860,3146,2860,3146,2860,3146,8866,3146,2860,9152,2860,9152,2860,3146},
    {3411,8528,3411,2843,9097,2843,9097,2843,3411,2274,3411,9097,9097,2843,9097,
     2843,3411,2274,3411,2843,3411,8528,3411,2274,9097,2843,9097,2843,3411}
};

static unsigned read_print(const uint16_t widths[29], bool reverse, unsigned step,
                           char expected)
{
    barcode_t scanner = {0};
    unsigned end = 18000;
    unsigned element = 0;
    uint32_t x = 0;
    unsigned readings = 0;
    unsigned index = 0;

    while (element <= 30u) {
        if (x >= end) {
            ++element;
            if (element <= 29u) {
                end += widths[reverse ? 29u - element : element - 1u];
            } else {
                end += 17000u;
            }
        }
        uint16_t reflectance = element <= 29u && (element & 1u) ? 885u : 145u;
        char letter = barcode_sample(&scanner, reflectance, UINT32_MAX - 60000u + x);
        if (letter != 0) {
            if (letter != expected) {
                fprintf(stderr, "decoded %c, expected %d, reverse %d, step %u\n",
                        letter, expected, reverse, step);
            }
            assert(letter == expected);
            ++readings;
        }
        x += step + (index++ % 4u) * 95u;
    }
    return readings;
}

static void check_barcode(void)
{
    for (unsigned page = 0; page < 4; ++page) {
        for (unsigned direction = 0; direction < 2; ++direction) {
            for (unsigned step = 100u; step <= 820u; step += 120u) {
                assert(read_print(printed[page], direction != 0, step, page < 2u ? 'A' : 'Z') == 1u);
            }
        }
    }
    static const char *const patterns[] = {
        "wnnnnwnnw", "nnwnnwnnw", "wnwnnwnnn", "nnnnwwnnw", "wnnnwwnnn",
        "nnwnwwnnn", "nnnnnwwnw", "wnnnnwwnn", "nnwnnwwnn", "nnnnwwwnn",
        "wnnnnnnww", "nnwnnnnww", "wnwnnnnwn", "nnnnwnnww", "wnnnwnnwn",
        "nnwnwnnwn", "nnnnnnwww", "wnnnnnwwn", "nnwnnnwwn", "nnnnwnwwn",
        "wwnnnnnnw", "nwwnnnnnw", "wwwnnnnnn", "nwnnwnnnw", "wwnnwnnnn", "nwwnwnnnn"
    };
    for (unsigned letter = 0; letter < 26u; ++letter) {
        uint16_t trace[29];
        memcpy(trace, printed[0], sizeof(trace));
        for (unsigned i = 0; i < 9u; ++i) {
            trace[10u + i] = patterns[letter][i] == 'w' ? 9000u : 3000u;
        }
        for (unsigned direction = 0; direction < 2u; ++direction) {
            assert(read_print(trace, direction != 0, 570u, (char)('A' + letter)) == 1u);
        }
    }
    uint16_t malformed[29];
    memcpy(malformed, printed[0], sizeof(malformed));
    malformed[24] = 20000;
    assert(read_print(malformed, false, 420, 0) == 0);
}

static void check_terrain(void)
{
    hump_t hump = {0};
    uint32_t position = 0u;
    uint8_t impacts = 1u;
    assert(!terrain_impact_sample(false, 0, 0, 0, &impacts) && impacts == 0u);
    assert(!terrain_impact_sample(false, 6000000, 800, 1000, &impacts));
    assert(!terrain_impact_sample(true, 4900000, 1000, 0, &impacts));
    assert(terrain_impact_sample(true, 4900000, 1000, 0, &impacts));
    assert(!terrain_impact_sample(false, 4900000, 1000, 0, &impacts) && impacts == 0u);
    assert(terrain_impact_sample(true, 1000000, 100, 0, &impacts));

    assert(terrain_atan(1000, 1000) == 45000);
    assert(terrain_atan(1000, -1000) == 135000);
    assert(terrain_atan(-1000, -1000) == -135000);
    for (unsigned i = 0; i < 150; ++i) {
        terrain_hump_update(&hump, 11000, 1, ++position, true, true);
    }
    assert(hump.phase == 1u);
    for (unsigned i = 0; i < 150; ++i) {
        terrain_hump_update(&hump, -11000, 1, ++position, true, true);
    }
    for (unsigned i = 0; i < 10; ++i) {
        terrain_hump_update(&hump, 0, 0, position, true, false);
    }
    assert(hump.phase == 0u && hump.max_mm >= 27 && hump.max_mm <= 29);
    terrain_hump_update(&hump, 42000, 1, ++position, true, true);
    assert(hump.phase == 0u && hump.max_mm >= 27);
}

static void check_profile(void)
{
    scan_point_t sparse[] = {
        {.angle = 30}, {.angle = 60}, {.angle = 90, .mm = 210}, {.angle = 120}, {.angle = 150}
    };
    scan_point_t detailed[] = {
        {.angle = 80, .mm = 216}, {.angle = 90, .mm = 210}, {.angle = 100, .mm = 216}
    };
    obstacle_t profile = obstacle_profile(sparse, detailed, 3);
    assert(profile.plan == PLAN_LEFT);
    assert(profile.range_mm == 210 && profile.lateral_mm == 0);
    assert(profile.width_mm > 95 && profile.forward_mm > 280);
    assert(profile.left_mm == 0 && profile.right_mm == 0);
    sparse[0].mm = 126;
    profile = obstacle_profile(sparse, detailed, 3);
    assert(profile.plan == PLAN_LEFT);
    sparse[4].mm = 120;
    profile = obstacle_profile(sparse, detailed, 3);
    assert(profile.plan == PLAN_REVERSE);
    sparse[2].mm = 1900;
    assert(obstacle_profile(sparse, NULL, 0).plan == PLAN_CONTINUE);
    sparse[2].mm = 0;
    assert(obstacle_profile(sparse, NULL, 0).plan == PLAN_STOP);
}

static void check_wire(void)
{
    routes_t mapping = routes_default();
    assert(routes_valid(&mapping));
    for (char letter = 'A'; letter <= 'Z'; ++letter) {
        assert(routes_set(&mapping, letter, (unsigned)(letter - 'A') % 5u));
    }
    assert(routes_valid(&mapping));
    for (char letter = 'A'; letter <= 'Z'; ++letter) {
        assert(routes_get(&mapping, letter) == (uint8_t)((letter - 'A') % 5));
    }
    assert(!routes_set(&mapping, '@', ROUTE_LEFT));
    assert(!routes_set(&mapping, 'Z', 7));
    mapping.bytes[9] |= 7u << 5;
    assert(!routes_valid(&mapping));

    const uint8_t expected[TELEMETRY_BYTES] = {
        0x53,0xe9,0x2d,0x61,0x45,0x23,0x61,0x1a,0x1a,0xb0,0x34,0xb9,0x01
    };
    car_status_t snapshot = {.phase = CAR_DETOUR, .terrain = 2, .barcode = 26,
        .plan = 3, .line = true, .magnetic_rate = true, .faults = 0x12,
        .distance_mm = 0x123456, .front_mm = 0x1a6, .max_hump_mm = 26,
        .obstacle_width_mm = 300, .speed_mm_s = -109, .yaw_deg_s = 6};
    uint8_t bytes[TELEMETRY_BYTES];

    telemetry_encode(snapshot, bytes);
    assert(memcmp(bytes, expected, sizeof(bytes)) == 0);
}

int main(void)
{
    check_barcode();
    check_terrain();
    check_profile();
    check_wire();
    puts("PASS Code 39 print traces, terrain, obstacle geometry, route map and telemetry");
    return 0;
}
