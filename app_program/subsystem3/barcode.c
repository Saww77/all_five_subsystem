#include "barcode.h"
#include <string.h>

static const uint16_t letters[] = {
    0x109, 0x049, 0x148, 0x019, 0x118, 0x058, 0x00d, 0x10c, 0x04c,
    0x01c, 0x103, 0x043, 0x142, 0x013, 0x112, 0x052, 0x007, 0x106,
    0x046, 0x016, 0x181, 0x0c1, 0x1c0, 0x091, 0x190, 0x0d0
};
#define CODE39_STOP 0x094u

void barcode_reset(barcode_t *reader)
{
    memset(reader, 0, sizeof(*reader));
}

static unsigned run(const uint8_t widths[19], unsigned index, bool reverse)
{
    unsigned bit = 5u * (reverse ? 28u - index : index);
    unsigned byte = bit / 8u;
    unsigned shift = bit % 8u;
    unsigned value = widths[byte];

    if (shift > 3u) { value |= (unsigned)widths[byte + 1u] << 8; }
    return (value >> shift) & 31u;
}

static void store_run(uint8_t widths[19], unsigned index, unsigned mm)
{
    unsigned bit = 5u * index;
    unsigned byte = bit / 8u;
    unsigned shift = bit % 8u;
    unsigned value = widths[byte];

    if (shift > 3u) { value |= (unsigned)widths[byte + 1u] << 8; }
    value = (value & ~(31u << shift)) | (mm << shift);
    widths[byte] = (uint8_t)value;
    if (shift > 3u) { widths[byte + 1u] = (uint8_t)(value >> 8); }
}

static unsigned symbol(const uint8_t widths[19], unsigned start, bool reverse,
                       unsigned *narrow)
{
    unsigned widest_narrow = 0;
    unsigned least_wide = 255;
    unsigned small_sum = 0;
    unsigned wide_sum = 0;
    unsigned pattern = 0;
    unsigned wide_count = 0;

    /* Find the lower bound of the three widest runs. Validate the gap
     * between classes after classification, including rounded edge widths. */
    unsigned largest[3] = {0};
    for (unsigned i = 0; i < 9; ++i) {
        unsigned w = run(widths, start + i, reverse);
        for (unsigned j = 0; j < 3; ++j) {
            if (w > largest[j]) {
                unsigned displaced = largest[j];
                largest[j] = w;
                w = displaced;
            }
        }
    }

    for (unsigned i = 0; i < 9; ++i) {
        unsigned w = run(widths, start + i, reverse);
        pattern <<= 1;
        if (w >= largest[2]) {
            pattern |= 1u;
            ++wide_count;
            wide_sum += w;
            if (w < least_wide) { least_wide = w; }
        } else {
            small_sum += w;
            if (w > widest_narrow) { widest_narrow = w; }
        }
    }
    if (wide_count != 3u || least_wide * 10u < widest_narrow * 16u ||
        largest[0] * 10u > least_wide * 18u ||
        wide_sum * 2u > small_sum * 5u || wide_sum * 4u < small_sum * 3u) {
        return 0;
    }
    *narrow = (small_sum + 3u) / 6u;
    return pattern;
}

static char decode(const uint8_t widths[19], bool reverse)
{
    unsigned narrow[3];
    unsigned start = symbol(widths, 0, reverse, &narrow[0]);
    unsigned body = symbol(widths, 10, reverse, &narrow[1]);
    unsigned stop = symbol(widths, 20, reverse, &narrow[2]);

    if (start != CODE39_STOP || stop != CODE39_STOP || body == 0u) {
        return 0;
    }
    for (unsigned i = 0; i < 2u; ++i) {
        unsigned gap = run(widths, 9u + 10u * i, reverse);
        if (gap * 2u < narrow[i] || gap * 2u > narrow[i] * 3u ||
            narrow[i] * 2u > narrow[i + 1u] * 3u ||
            narrow[i + 1u] * 2u > narrow[i] * 3u) {
            return 0;
        }
    }
    for (unsigned i = 0; i < sizeof(letters) / sizeof(letters[0]); ++i) {
        if (body == letters[i]) { return (char)('A' + i); }
    }
    return 0;
}

char barcode_sample(barcode_t *reader, uint16_t darkness, uint32_t position_um)
{
    bool black = darkness > 620u || (darkness >= 380u && reader->black);
    unsigned width = position_um - reader->edge_um;

    if (black != reader->candidate) {
        reader->candidate = black;
        reader->stable = 1u;
    } else if (reader->stable < 2u) {
        ++reader->stable;
    }
    if (reader->stable == 2u && black != reader->black) {
        reader->black = black;
        reader->edge_um = position_um;
        if (!reader->active) {
            if (black && reader->observed_white && width >= 7000u) {
                reader->active = true;
                reader->count = 0u;
            }
        } else if (width < 1000u || width > 22000u || reader->count >= 29u) {
            reader->active = false;
        } else {
            store_run(reader->width, reader->count++, (width + 500u) / 1000u);
        }
        reader->observed_white = !black;
        return 0;
    }
    if (!black && !reader->observed_white && reader->stable >= 2u) {
        reader->observed_white = true;
        reader->edge_um = position_um;
        width = 0;
    }
    if (reader->active && !black && reader->count == 29u && width >= 8000u) {
        reader->active = false;
        char letter = decode(reader->width, false);
        return letter != 0 ? letter : decode(reader->width, true);
    }
    if (reader->active && width > 24000u) {
        reader->active = false;
    }
    return 0;
}

bool barcode_active(const barcode_t *reader)
{
    return reader->active && reader->count >= 3u;
}
