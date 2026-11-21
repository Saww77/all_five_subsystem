#ifndef CAR_CONFIG_H
#define CAR_CONFIG_H

/* RP2040 GPIO numbers. Motor and encoder wiring follows the tested team image. */
#define PIN_ENCODER_LEFT   2u
#define PIN_ENCODER_LEFT_B 3u
#define PIN_ENCODER_RIGHT  0u
#define PIN_ENCODER_RIGHT_B 1u
#define PIN_MOTOR_LEFT_A   8u
#define PIN_MOTOR_LEFT_B   9u
#define PIN_MOTOR_RIGHT_A  10u
#define PIN_MOTOR_RIGHT_B  11u
#define PIN_SERVO          12u
#define PIN_TRIGGER        16u
#define PIN_ECHO           17u
#define PIN_HEARTBEAT      19u
#define PIN_ESTOP          20u
#define PIN_START          21u

/* One rising edge channel per wheel. Defaults are provisional bench values.
 * Derive um per rising edge by rolling each wheel through 10 revolutions. */
#define LEFT_UM_PER_EDGE   331
#define RIGHT_UM_PER_EDGE  333
#define TRACK_MM           108
#define LEFT_MOTOR_SIGN    1
#define RIGHT_MOTOR_SIGN   1
#define DRIVE_MAX_MM_S     180
#define CONTROL_US         10000u

/* GP26/27/28 = ADC0/1/2. Keep GP26 left and GP28 right for line sensing.
 * Mount the two line sensors just inside opposite edges of the 18 mm line.
 * GP27 retains barcode duty. Mount it over the parallel barcode, about 35 mm to the left. */
#define BARCODE_PROGRESS_MAP 1
/* Prefer optical junction detection. To enable an encoder fallback, measure
 * from the barcode end (subtract the 8 mm quiet zone) to axle alignment. */
#define BARCODE_TO_JUNCTION_MM 0
#define JUNCTION_WINDOW_MM   145
#define IR_TO_AXLE_MM     60
#define LINE_SPEED_MM_S    45u
#define LINE_MIN_MM_S      26
#define LINE_MAX_STEER     15
#define BARCODE_MAX_STEER  10
#define LINE_DEADBAND      70
#define LINE_LOSS_US       160000u

/* Map the sensor axes to vehicle forward, left, and up. Indices are 0..2. */
#define IMU_FORWARD_AXIS   1u
#define IMU_LEFT_AXIS      0u
#define IMU_UP_AXIS        2u
#define IMU_FORWARD_SIGN   -1
#define IMU_LEFT_SIGN      1
#define IMU_UP_SIGN        1
#define IMU_MAG_X_OFFSET   0
#define IMU_MAG_Y_OFFSET   0
#define IMU_MAG_Z_OFFSET   0

/* Angle 90 is forward; higher logical angles are to the left. */
#define SERVO_REVERSED     0
#define SERVO_CENTER_US    1500
#define SERVO_US_PER_60    560
#define STOP_RANGE_MM      250u
#define HARD_RANGE_MM      105u
#define RANGE_MAX_AGE_US   250000u
#define CAR_HALF_WIDTH_MM  80
#define SIDE_MARGIN_MM     55
#define MAX_DETOUR_MM      440

#endif
