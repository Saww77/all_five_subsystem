#include "line_steering.h"

#define EDGE_DARKNESS 160
#define ERROR_DEADBAND 80
#define ERROR_PER_STEP 145

static int clamp(int value, int lower, int upper)
{
    if (value < lower) { return lower; }
    if (value > upper) { return upper; }
    return value;
}

void line_decide(int left, int center, int right, LF_STATE *state, LF_CMD *command)
{
    int demand = 0;
    int error = right - left;

    /* GP27 is the barcode reader. Its bars must not pull the car sideways. */
    (void)center;
    if ((left < EDGE_DARKNESS && right < EDGE_DARKNESS) ||
        (left >= 850 && right >= 850)) {
        /* Both steering sensors are off the edges or over a broad marking.
         * Release the correction immediately at the next slew step. */
        state->filtered_error = 0;
    } else {
        state->filtered_error = (int16_t)((state->filtered_error + error) / 2);
        int filtered = state->filtered_error;
        int magnitude = filtered < 0 ? -filtered : filtered;
        if (magnitude > ERROR_DEADBAND) {
            int steps = 1 + (magnitude - ERROR_DEADBAND) / ERROR_PER_STEP;
            demand = clamp(steps * S3_CORRECTION_STEP_RPM, 0, S3_MAX_CORRECTION_RPM);
            if (filtered < 0) { demand = -demand; }
        }
    }

    int change = clamp(demand - state->correction_rpm,
                       -S3_CORRECTION_STEP_RPM, S3_CORRECTION_STEP_RPM);
    state->correction_rpm = (int8_t)(state->correction_rpm + change);
    *command = (LF_CMD){
        .left = S3_CRUISE_RPM + state->correction_rpm,
        .right = S3_CRUISE_RPM - state->correction_rpm
    };
}
