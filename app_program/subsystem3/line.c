#include "line.h"
#include "barcode.h"
#include "../app.h"
#include "../car_config.h"
#include "../subsystem2/motion.h"

#define CAL_SAMPLES 24u
#define DARK_ENTER 180u
#define DARK_EXIT  105u
#define CAL_SPAN   300

static ID timer;
static line_sample_t sample;
static volatile uint8_t calibration;
static volatile uint8_t requested;
static volatile uint8_t barcode_epoch;
static volatile bool scan_enabled;
static bool calibrated;

static unsigned sample_peak(unsigned left, unsigned right)
{
    return left > right ? left : right;
}

static int absolute(int value)
{
    return value < 0 ? -value : value;
}

static uint64_t read_channels(ID adc)
{
    uint64_t packed = 0;
    SZ received;
    UW raw;

    for (INT channel = 0; channel < 3; ++channel) {
        if (tk_srea_dev(adc, channel, &raw, 1, &received) < E_OK || received != 1) {
            return UINT64_MAX;
        }
        packed |= (uint64_t)(raw & 4095u) << (12 * channel);
    }
    return packed;
}

static unsigned darkness(int raw, uint16_t white, uint16_t black)
{
    int normalized = (raw - white) * 1000 / ((int)black - white);
    return normalized < 0 ? 0u : normalized > 1000 ? 1000u : (unsigned)normalized;
}

void line_calibrate(unsigned surface)
{
    requested = (uint8_t)surface;
    calibration = 0;
}

unsigned line_calibration(void)
{
    return calibration;
}

void line_read_barcode(bool enabled)
{
    UW key;

    DI(key);
    if (scan_enabled != enabled) {
        scan_enabled = enabled;
        ++barcode_epoch;
    }
    EI(key);
}

line_sample_t line_sample(void)
{
    UW key;
    line_sample_t copy;

    DI(key);
    copy = sample;
    EI(key);
    return copy;
}

static void line_task(INT argument, void *context)
{
    ID adc = tk_opn_dev((const UB *)"adca", TD_READ);
    uint16_t white[3] = {0};
    uint16_t black[3] = {0};
    uint32_t sums[3] = {0};
    unsigned collecting = 0;
    unsigned acquired = 0;
    int filtered = 0;
    unsigned broad_count = 0;
    uint8_t scan_epoch = 0;
    barcode_t scanner = {0};
    line_sample_t next = {0};
    uint32_t last_sample = app_us();
    UINT event;
    UW key;

    (void)argument;
    (void)context;
    if (adc < E_OK) {
        car_fault(FAULT_LINE);
        return;
    }
    for (;;) {
        (void)tk_wai_flg(timer, EVENT_TICK, TWF_ORW | TWF_BITCLR, &event, TMO_FEVR);
        uint32_t now = app_us();
        uint64_t packed = read_channels(adc);
        if (packed == UINT64_MAX) {
            if (now - last_sample > 60000u) { car_fault(FAULT_LINE); }
            continue;
        }
        uint16_t raw[3] = {
            (uint16_t)(packed & 4095u),
            (uint16_t)((packed >> 12) & 4095u),
            (uint16_t)((packed >> 24) & 4095u)
        };
        bool scan;
        uint8_t epoch;

        DI(key);
        if (requested != 0u) {
            collecting = requested;
            requested = 0u;
            acquired = 0u;
            for (unsigned i = 0; i < 3; ++i) { sums[i] = 0u; }
        }
        scan = scan_enabled;
        epoch = barcode_epoch;
        EI(key);
        if (collecting != 0u) {
            for (unsigned i = 0; i < 3; ++i) { sums[i] += raw[i]; }
            if (++acquired == CAL_SAMPLES) {
                bool valid = true;
                for (unsigned i = 0; i < 3; ++i) {
                    if (collecting == CAL_WHITE) {
                        white[i] = (uint16_t)(sums[i] / CAL_SAMPLES);
                    } else {
                        black[i] = (uint16_t)(sums[i] / CAL_SAMPLES);
                        valid = valid && absolute((int)black[i] - white[i]) >= CAL_SPAN;
                    }
                }
                calibrated = collecting == CAL_BLACK && valid;
                calibration = collecting == CAL_WHITE ? CAL_WHITE
                            : valid ? CAL_COMPLETE : CAL_INVALID;
                collecting = 0u;
            }
        }
        next.calibrated = calibrated;
        if (calibrated) {
            unsigned left = darkness(raw[0], white[0], black[0]);
            unsigned right = darkness(raw[2], white[2], black[2]);
            unsigned center = darkness(raw[1], white[1], black[1]);
            int error = ((int)left - (int)right) * 1000 / (int)(left + right + 260u);
            bool was_visible = next.visible;
            next.visible = sample_peak(left, right) >= (was_visible ? DARK_EXIT : DARK_ENTER);
            if (next.visible) {
                filtered = was_visible ? filtered + (error - filtered) / 4 : error;
            } else {
                filtered -= filtered / 4;
            }
            next.error = (int16_t)filtered;
            if (left >= 820u && right >= 820u) {
                if (broad_count < 5u) { ++broad_count; }
            } else {
                broad_count = 0u;
            }
            next.junction = broad_count == 5u;

            if (!scan || epoch != scan_epoch || now - last_sample > 20000u) {
                barcode_reset(&scanner);
            }
            scan_epoch = epoch;
            if (scan) {
                motion_sample_t motion = motion_sample();
                char letter = barcode_sample(&scanner, (uint16_t)center, motion.forward_um);
                if (letter != 0) {
                    next.letter = (uint8_t)(letter - 'A' + 1);
                    next.barcode_at_mm = motion.travelled_mm;
                    ++next.barcode_id;
                }
            }
            next.barcode_busy = scan && barcode_active(&scanner);
        }
        last_sample = now;
        next.time_us = now;
        ++next.sequence;
        DI(key);
        sample = next;
        EI(key);
    }
}

bool line_start(void)
{
    timer = app_events();
    return timer > 0 && app_periodic(timer, 5u) && app_start((FP)line_task, 6, 960);
}
