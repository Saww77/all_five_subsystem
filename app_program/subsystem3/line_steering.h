#ifndef LINE_STEERING_H
#define LINE_STEERING_H

#include <stdbool.h>
#include <stdint.h>

/* Wheel targets use the team's even-RPM Subsystem 2 API. */
#define S3_CRUISE_RPM 22
#define S3_MAX_CORRECTION_RPM 8
#define S3_CORRECTION_STEP_RPM 2

typedef struct {
    int16_t filtered_error;
    int8_t correction_rpm;
} LF_STATE;

typedef struct {
    int left;
    int right;
    bool stop;
} LF_CMD;

void line_decide(int left, int center, int right, LF_STATE *state, LF_CMD *command);

#endif
