#include "terrain.h"
#include "../app.h"
#include "../car_config.h"
#include "../subsystem2/motion.h"
#include "../../device/include/dev_i2c.h"

#define ACCEL_PRIMARY 0x19u
#define ACCEL_ALTERNATE 0x18u
#define MAG   0x1eu
#define CAL_COUNT 64u
#define ARM_SETTLE_SAMPLES 15u

static terrain_sample_t shared;
static ID timer;
static volatile uint8_t run_epoch;
static volatile bool mission_armed;

static int absolute(int x)
{
    return x < 0 ? -x : x;
}

static bool burst(ID dev, UW address, UB reg, UB *data, SZ bytes)
{
    SZ actual;
    T_I2C_EXEC transfer = {
        .sadr = address, .snd_size = 1, .snd_data = &reg,
        .rcv_size = bytes, .rcv_data = data
    };

    return tk_swri_dev(dev, TDN_I2C_EXEC, &transfer, sizeof(transfer), &actual) >= E_OK
           && actual == sizeof(transfer);
}

static UW setup_accelerometer(ID dev)
{
    const UW addresses[] = {ACCEL_PRIMARY, ACCEL_ALTERNATE};

    for (unsigned index = 0; index < 2u; ++index) {
        UW address = addresses[index];
        UB identity;
        /* Block data update, high-resolution ±4 g, then 100 Hz XYZ. */
        if (burst(dev, address, 0x0fu, &identity, 1) && identity == 0x33u &&
            i2c_write_reg(dev, address, 0x23u, 0x98u) >= E_OK &&
            i2c_write_reg(dev, address, 0x20u, 0x57u) >= E_OK) {
            return address;
        }
    }
    return 0u;
}

static bool setup_magnetometer(ID dev)
{
    UB identity[3];

    return burst(dev, MAG, 0x0au, identity, 3) &&
           identity[0] == 'H' && identity[1] == '4' && identity[2] == '3' &&
           i2c_write_reg(dev, MAG, 0x00u, 0x18u) >= E_OK &&
           i2c_write_reg(dev, MAG, 0x01u, 0x20u) >= E_OK &&
           i2c_write_reg(dev, MAG, 0x02u, 0x00u) >= E_OK;
}

static int16_t word(UB high, UB low)
{
    return (int16_t)((uint16_t)high << 8 | low);
}

static int wrapped_delta(int next, int previous)
{
    int delta = next - previous;
    if (delta > 180000) { return delta - 360000; }
    if (delta < -180000) { return delta + 360000; }
    return delta;
}

void terrain_set_armed(bool armed)
{
    mission_armed = armed;
}

void terrain_begin_run(void)
{
    ++run_epoch;
}

terrain_sample_t terrain_sample(void)
{
    terrain_sample_t copy;
    UW key;

    DI(key);
    copy = shared;
    EI(key);
    return copy;
}

static void terrain_task(INT argument, void *context)
{
    ID imu = tk_opn_dev((const UB *)"iica", TD_READ | TD_WRITE);
    uint32_t previous_position = 0u;
    int odometry_remainder = 0;
    uint32_t last_accel = app_us();
    uint32_t next_mag = 0u;
    uint32_t mag_time = 0u;
    bool magnetic_rate_valid = false;
    int last_heading = 0;
    int magnetic_turn = 0;
    int bias[3] = {0};
    int acc[3] = {0};
    int baseline = 0;
    int last_speed = 0;
    int longitudinal_q8 = 0;
    unsigned samples = 0u;
    uint8_t impacts = 0u;
    uint8_t settled_samples = 0u;
    uint8_t epoch = 0u;
    hump_t hump = {0};
    terrain_sample_t next = {0};
    UINT flags;
    UW key;

    (void)argument;
    (void)context;
    UW accel_address = 0u;
    for (unsigned attempt = 0; imu > 0 && attempt < 3u; ++attempt) {
        accel_address = setup_accelerometer(imu);
        if (accel_address != 0u) { break; }
        (void)tk_dly_tsk(100u);
    }
    if (accel_address == 0u) {
        car_fault(FAULT_IMU);
        return;
    }
    bool mag_available = setup_magnetometer(imu);
    last_accel = app_us();
    for (;;) {
        UB raw[7];
        (void)tk_wai_flg(timer, EVENT_TICK, TWF_ORW | TWF_BITCLR, &flags, TMO_FEVR);
        uint32_t now = app_us();
        if (!burst(imu, accel_address, 0xa7u, raw, 7) || !(raw[0] & 8u)) {
            if (now - last_accel > 70000u) {
                car_fault(FAULT_IMU);
                (void)tk_dly_tsk(250u);
            }
            continue;
        }
        unsigned dt_us = now - last_accel;
        last_accel = now;
        for (unsigned i = 0; i < 3u; ++i) {
            acc[i] = (word(raw[2u + 2u * i], raw[1u + 2u * i]) / 16) * 2;
        }
        int fwd = acc[IMU_FORWARD_AXIS] * IMU_FORWARD_SIGN;
        int left = acc[IMU_LEFT_AXIS] * IMU_LEFT_SIGN;
        int up = acc[IMU_UP_AXIS] * IMU_UP_SIGN;
        int norm_sq = fwd * fwd + left * left + up * up;
        if (samples < CAL_COUNT) {
            if (up < 750 || up > 1250 || absolute(fwd) > 350 || absolute(left) > 350) {
                samples = 0u;
                bias[0] = bias[1] = bias[2] = 0;
                continue;
            }
            if (samples != 0u && (absolute(fwd - bias[0] / (int)samples) > 95 ||
                absolute(left - bias[1] / (int)samples) > 95 ||
                absolute(up - bias[2] / (int)samples) > 95)) {
                samples = 0u;
                bias[0] = bias[1] = bias[2] = 0;
                continue;
            }
            bias[0] += fwd;
            bias[1] += left;
            bias[2] += up;
            if (++samples == CAL_COUNT) {
                baseline = terrain_atan(bias[0] / (int)samples, bias[2] / (int)samples);
                longitudinal_q8 = baseline * 256;
            }
        }
        if (samples < CAL_COUNT) { continue; }
        motion_sample_t motion = motion_sample();
        if (epoch != run_epoch) {
            epoch = run_epoch;
            hump = (hump_t){0};
            previous_position = motion.forward_um;
            odometry_remainder = 0;
        }
        int32_t delta_um = (int32_t)(motion.forward_um - previous_position);
        previous_position = motion.forward_um;
        int progress = delta_um + odometry_remainder;
        unsigned distance = progress > 0 ? (unsigned)progress / 1000u : 0u;
        odometry_remainder = progress > 0 ? progress % 1000 : 0;
        int speed = motion.speed_mm_s;
        int yaw = motion.yaw_deg_s;
        bool armed = mission_armed;
        bool impact = terrain_impact_sample(armed, norm_sq, up, left, &impacts);
        if (impact) { car_fault(FAULT_COLLISION); }

        /* The driver has to put the car down and let it settle before arming. */
        bool upright = absolute(fwd - bias[0] / (int)CAL_COUNT) <= 180 &&
                       absolute(left - bias[1] / (int)CAL_COUNT) <= 180 &&
                       absolute(up - bias[2] / (int)CAL_COUNT) <= 180;
        if (!armed && upright && absolute(speed) <= 18 && dt_us < 30000u) {
            if (settled_samples < ARM_SETTLE_SAMPLES) { ++settled_samples; }
        } else {
            settled_samples = 0u;
        }
        next.ready = settled_samples == ARM_SETTLE_SAMPLES;
        /* Subtract the measured change in wheel speed from longitudinal specific force. */
        int dynamic = dt_us >= 6000u && dt_us <= 25000u
                      ? (speed - last_speed) * 102000 / (int)dt_us : 0;
        last_speed = speed;
        if (dynamic < -180) { dynamic = -180; }
        if (dynamic > 180) { dynamic = 180; }
        int current_angle = terrain_atan(fwd - dynamic, up);
        longitudinal_q8 += (current_angle * 256 - longitudinal_q8) / 5;
        int slope = longitudinal_q8 / 256 - baseline;
        bool gravity_like = norm_sq > 630000 && norm_sq < 1660000;
        bool driving = armed && speed > 18 && absolute(yaw) < 28 && delta_um >= 0;
        terrain_hump_update(&hump, slope, distance, motion.travelled_mm,
                             gravity_like && distance <= 8u, driving);
        if (mag_available && ((int32_t)(now - next_mag) >= 0 || next_mag == 0u)) {
            UB magnetic[6];
            next_mag = now + 40000u;
            magnetic_rate_valid = false;
            if (burst(imu, MAG, 0x03u, magnetic, 6)) {
                int field[3] = {
                    word(magnetic[0], magnetic[1]) - IMU_MAG_X_OFFSET,
                    word(magnetic[4], magnetic[5]) - IMU_MAG_Y_OFFSET,
                    word(magnetic[2], magnetic[3]) * 11 / 10 - IMU_MAG_Z_OFFSET
                };
                int mx = field[IMU_FORWARD_AXIS] * IMU_FORWARD_SIGN;
                int my = field[IMU_LEFT_AXIS] * IMU_LEFT_SIGN;
                int mz = field[IMU_UP_AXIS] * IMU_UP_SIGN;
                if (absolute(mx) < 2000 && absolute(my) < 2000 && absolute(mz) < 2000 &&
                    up >= 650 && mx * mx + my * my + mz * mz > 10000) {
                    int heading = terrain_atan(my * up - mz * left, mx * up - mz * fwd);
                    unsigned elapsed = now - mag_time;
                    if (mag_time != 0u && elapsed >= 30000u && elapsed <= 75000u) {
                        magnetic_turn = -wrapped_delta(heading, last_heading) * 1000 / (int)elapsed;
                        magnetic_rate_valid = true;
                    }
                    mag_time = now;
                    last_heading = heading;
                }
            }
        }

        next.magnetic_rate = magnetic_rate_valid && now - mag_time < 85000u &&
                             absolute(magnetic_turn - yaw) <= 42;
        int selected_yaw = next.magnetic_rate ? (magnetic_turn + yaw) / 2 : yaw;
        if (selected_yaw > 511) { selected_yaw = 511; }
        if (selected_yaw < -512) { selected_yaw = -512; }
        next.yaw_deg_s = selected_yaw;
        next.max_hump_mm = hump.max_mm;
        if (impact) {
            next.event = TERRAIN_IMPACT;
        } else if (absolute(slope) > 26000 || absolute(left - bias[1] / (int)samples) > 420) {
            next.event = TERRAIN_TILT;
        } else if (hump.phase == 2u) {
            next.event = TERRAIN_DESCENT;
        } else if (hump.phase == 1u) {
            next.event = TERRAIN_CLIMB;
        } else if (absolute(yaw) > 28) {
            next.event = TERRAIN_TURNING;
        } else if (absolute(dynamic) > 65) {
            next.event = TERRAIN_ACCELERATING;
        } else {
            next.event = absolute(speed) > 18 ? TERRAIN_CRUISING : TERRAIN_STILL;
        }
        DI(key);
        shared = next;
        EI(key);
    }
}

bool terrain_start(void)
{
    timer = app_events();
    return timer > 0 && app_periodic(timer, 10u) && app_start((FP)terrain_task, 4, 1280);
}
