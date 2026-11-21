#ifndef BARCODE_H
#define BARCODE_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint32_t edge_um;
    uint8_t width[19];  /* 29 five-bit widths in mm for one Code 39 frame. */
    uint8_t count : 5;
    bool black : 1;
    bool candidate : 1;
    bool active : 1;
    uint8_t stable : 2;
    bool observed_white : 1;
} barcode_t;

void barcode_reset(barcode_t *reader);
char barcode_sample(barcode_t *reader, uint16_t darkness, uint32_t position_um);
bool barcode_active(const barcode_t *reader);

#endif
