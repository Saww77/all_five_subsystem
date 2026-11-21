#include "obstacle.h"
#include "../car_config.h"

#define PROFILE_NEAR_MM 540

/* sin(angle) in thousandths, sampled at the scanner's 5-degree increments. */
static const int sine[] = {
    0, 87, 174, 259, 342, 423, 500, 574, 643, 707, 766, 819, 866,
    906, 940, 966, 985, 996, 1000
};

static int lateral(scan_point_t point)
{
    int angle = (int)point.angle - 90;
    int offset = angle < 0 ? -angle : angle;
    int mm = point.mm * sine[offset / 5] / 1000;
    return angle < 0 ? -mm : mm;
}

static int forward(scan_point_t point)
{
    int angle = (int)point.angle - 90;
    int offset = angle < 0 ? -angle : angle;
    return point.mm * sine[(90 - offset) / 5] / 1000;
}

static unsigned allowance(scan_point_t side, scan_point_t diagonal)
{
    /* Empty echoes are unobserved. Valid echoes limit the route. */
    unsigned clearance = 0;
    const scan_point_t points[2] = { side, diagonal };

    for (unsigned i = 0; i < 2u; ++i) {
        if (points[i].mm != 0u) {
            int x = lateral(points[i]);
            unsigned mm = (unsigned)(x < 0 ? -x : x);
            if (clearance == 0u || mm < clearance) { clearance = mm; }
        }
    }
    return clearance > 511u ? 511u : clearance;
}

obstacle_t obstacle_profile(const scan_point_t coarse[5], const scan_point_t *fine,
                            unsigned count)
{
    obstacle_t result = {0};
    unsigned closest = 0;
    unsigned nearest_bearing_mm = 0;
    int x_min = 0;
    int x_max = 0;
    int y_max = 0;
    int near_x = 0;
    bool hit = false;
    const scan_point_t *sets[2] = { coarse, fine };
    const unsigned lengths[2] = {5u, count};

    for (unsigned s = 0; s < 2u; ++s) {
        for (unsigned i = 0; i < lengths[s]; ++i) {
            scan_point_t p = sets[s][i];
            if (p.mm == 0u || p.mm > PROFILE_NEAR_MM ||
                p.angle < 30u || p.angle > 150u) { continue; }
            int x = lateral(p);
            int y = forward(p);
            if (y < 70 || x < -250 || x > 250) { continue; }
            if (!hit || p.mm < closest) {
                closest = p.mm;
                near_x = x;
                nearest_bearing_mm = (unsigned)y;
            }
            if (x < x_min) { x_min = x; }
            if (x > x_max) { x_max = x; }
            if (y > y_max) { y_max = y; }
            hit = true;
        }
    }
    if (!hit) {
        result.plan = coarse[2].mm > STOP_RANGE_MM ? PLAN_CONTINUE : PLAN_STOP;
        return result;
    }
    result.range_mm = closest;
    result.lateral_mm = near_x;
    /* Add an angular-beam margin to the measured front-surface bounds. */
    int beam = (int)nearest_bearing_mm / 7 + 15;
    x_min -= beam;
    x_max += beam;
    result.width_mm = (unsigned)(x_max - x_min);
    result.left_mm = allowance(coarse[4], coarse[3]);
    result.right_mm = allowance(coarse[0], coarse[1]);

    unsigned left_offset = (unsigned)(x_max + CAR_HALF_WIDTH_MM + SIDE_MARGIN_MM);
    unsigned right_offset = (unsigned)(-x_min + CAR_HALF_WIDTH_MM + SIDE_MARGIN_MM);
    bool left_safe = left_offset <= MAX_DETOUR_MM &&
                     (result.left_mm == 0u || result.left_mm >= left_offset + CAR_HALF_WIDTH_MM);
    bool right_safe = right_offset <= MAX_DETOUR_MM &&
                      (result.right_mm == 0u || result.right_mm >= right_offset + CAR_HALF_WIDTH_MM);
    unsigned left_cost = left_offset + (result.left_mm == 0u ? 45u : 0u);
    unsigned right_cost = right_offset + (result.right_mm == 0u ? 45u : 0u);
    unsigned forward_distance = (unsigned)y_max + 155u;
    if (forward_distance > 510u) {
        result.plan = PLAN_STOP;
    } else if (left_safe && (!right_safe || left_cost <= right_cost ||
               (result.left_mm == 0u && result.right_mm == 0u))) {
        result.plan = PLAN_LEFT;
        result.offset_mm = left_offset;
    } else if (right_safe) {
        result.plan = PLAN_RIGHT;
        result.offset_mm = right_offset;
    } else {
        result.plan = PLAN_REVERSE;
    }
    result.forward_mm = forward_distance > 510u ? 0u : forward_distance;
    return result;
}
