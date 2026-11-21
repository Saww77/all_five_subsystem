#ifndef LINE_H
#define LINE_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint32_t time_us;
    uint32_t barcode_at_mm;
    uint8_t sequence;
    int16_t error : 11;
    bool visible : 1;
    bool junction : 1;
    bool barcode_busy : 1;
    bool calibrated : 1;
    uint8_t letter : 5;
    uint8_t barcode_id;
} line_sample_t;

enum { CAL_WHITE = 1, CAL_BLACK, CAL_COMPLETE, CAL_INVALID };

bool line_start(void);
void line_calibrate(unsigned surface);
unsigned line_calibration(void);
void line_read_barcode(bool enabled);
line_sample_t line_sample(void);

#endif
