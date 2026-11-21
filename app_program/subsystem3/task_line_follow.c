/*
 * task_line_follow.c: line following (Buddy 3).
 * line_sense()  reads the 3 sensors -> darkness 0..1000
 * line_decide() PURE logic: darkness in, wheel RPM out (easy to unit-test)
 * line_follow_task() runs both every LINE_PERIOD_MS and drives Motion Control
 */
#include "subsystem3.h"
#include "../vehicle_controller/vehicle_controller.h"


/* Calibration: raw ADC on white paper and on the black line (L, C, R).
   Measure on your track; the live check below prints the raw values. */
LOCAL const INT white_raw[3] = { 130, 130, 130 };
LOCAL const INT black_raw[3] = { 3200, 3200, 3200 };


LOCAL INT darkness(INT raw, INT white, INT black)
{
    INT d;
    if (black <= white) { return 0; }
    d = ((raw - white) * 1000) / (black - white);
    if (d < 0)    { d = 0; }
    if (d > 1000) { d = 1000; }
    return d;
}


/* Only the outer sensors are required for steering and START. */
EXPORT ER line_sense(INT *left, INT *center, INT *right)
{
    INT left_raw = ir_read(0);
    INT right_raw = ir_read(2);
    if (left_raw < 0 || right_raw < 0) { return E_IO; }
    *left = darkness(left_raw, white_raw[0], black_raw[0]);
    *right = darkness(right_raw, white_raw[2], black_raw[2]);
    INT barcode_raw = ir_read(1);
    *center = barcode_raw < 0 ? -1 : darkness(barcode_raw, white_raw[1], black_raw[1]);
    return E_OK;
}

LOCAL ID owner_id;
#define S3_START_RPM 30
#define S3_START_MS 250u

LOCAL void tick_release(void *exinf)
{
    (void)exinf;
    (void)tk_wup_tsk(owner_id);
}

/* Preserve the team's existing v2 telemetry encoding for A through D. */
LOCAL unsigned barcode_id(char letter)
{
    return letter >= 'A' && letter <= 'D' ? (unsigned)(letter - 'A' + 1) : 0u;
}

EXPORT void line_follow_task(INT stacd, void *exinf)
{
    LF_STATE steering = {0};
    SYSTIM time;
    UW previous_tick = 0;
    UW follow_started = 0;
    unsigned last_barcode = 0;
    BOOL was_following = FALSE;
    T_CCYC cyc = {0};

    (void)stacd;
    (void)exinf;
    owner_id = tk_get_tid();
    cyc.cycatr = TA_HLNG | TA_STA;
    cyc.cychdr = (FP)tick_release;
    cyc.cyctim = LINE_PERIOD_MS;
    cyc.cycphs = LINE_PERIOD_MS;
    if (tk_cre_cyc(&cyc) < E_OK) {
        tm_printf((UB *)"[S3] could not start line sampling\n");
        vehicle_invalidate_line();
        return;
    }

    for (;;) {
        if (tk_slp_tsk(TMO_FEVR) < E_OK) { continue; }
        (void)tk_can_wup(owner_id);
        (void)tk_get_otm(&time);
        INT left, center, right;
        if (line_sense(&left, &center, &right) < E_OK) {
            vehicle_invalidate_line();
            steering = (LF_STATE){0};
            barcode_reset();
            continue;
        }

        BOOL following = vehicle_is_following();
        if (!following || !was_following) {
            last_barcode = 0;
            steering = (LF_STATE){0};
            follow_started = time.lo;
        }
        if (!following || !was_following || center < 0 ||
            time.lo - previous_tick > 2u * LINE_PERIOD_MS) {
            barcode_reset();
        }
        if (following && center >= 0 && time.lo - follow_started >= S3_START_MS) {
            barcode_feed(center);
            char letter = barcode_take();
            if (letter != 0) { last_barcode = barcode_id(letter); }
        }
        previous_tick = time.lo;
        was_following = following;

        LF_CMD command;
        line_decide(left, center, right, &steering, &command);
        /* Use the original target briefly to get through static friction,
         * then settle to the slower line-following target. */
        if (time.lo - follow_started < S3_START_MS) {
            command.left += S3_START_RPM - S3_CRUISE_RPM;
            command.right += S3_START_RPM - S3_CRUISE_RPM;
        }
        vehicle_submit_line(left, center < 0 ? 0 : center, right,
                            command.left, command.right, command.stop, last_barcode);
    }
}
