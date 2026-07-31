#ifndef CONTROL_CONFIG_H_
#define CONTROL_CONFIG_H_

#define K230_IMAGE_WIDTH                  (640)
#define K230_IMAGE_HEIGHT                 (360)
#define BALL_POSITION_LIMIT_CENTI_CM      (1250)
#define BALL_MIN_SCORE                    (35U)

#define BALL_DEAD_ZONE_ENTER_CM           (0.20f)
#define BALL_DEAD_ZONE_EXIT_CM            (0.35f)
#define BALL_HOLD_VELOCITY_CM_S           (0.50f)

#define POSITION_PID_KP                   (1.80f)
#define POSITION_PID_KI                   (0.08f)
#define POSITION_PID_KD                   (0.25f)
#define POSITION_PID_INTEGRAL_LIMIT       (12.0f)
#define BALL_TARGET_VELOCITY_LIMIT_CM_S   (10.0f)

#define VELOCITY_PID_KP                   (100.0f)
#define VELOCITY_PID_KI                   (25.0f)
#define VELOCITY_PID_KD                   (0.80f)
#define VELOCITY_PID_INTEGRAL_LIMIT       (20.0f)
#define MOTOR_PULSE_RATE_LIMIT_PPS        (1200.0f)
#define MOTOR_COMMAND_MIN_PULSES          (2U)
#define MOTOR_COMMAND_MAX_PULSES          (96U)

#define BALL_VELOCITY_FILTER_ALPHA        (0.35f)
#define CONTROL_DT_DEFAULT_MS             (33U)
#define CONTROL_DT_MAX_MS                 (120U)
#define VISION_LOST_TIMEOUT_MS            (250U)

#define ZDT_MOTOR_ID                      (1U)
#define ZDT_MOTOR_CHECKSUM                (0x6BU)
#define ZDT_MOTOR_SPEED_RPM               (20U)
#define ZDT_MOTOR_ACCEL_LEVEL             (20U)
#define ZDT_STARTUP_TEST_ENABLED           (0U)
#define ZDT_STARTUP_TEST_DURATION_MS       (300U)
#define ZDT_MOTOR_MIN_PULSE_MS             (12U)
#define ZDT_MOTOR_MAX_PULSE_MS             (45U)

#define ZDT_DIRECTION_RIGHT_RAISE         (0x01U)
#define ZDT_DIRECTION_LEFT_LOWER          (0x00U)

#endif
