#include "ball_controller.h"

#include "control_config.h"

typedef struct {
    float position_integral;
    float velocity_integral;
    float previous_velocity_error;
    float filtered_velocity;
    int16_t previous_position_centi_cm;
    uint32_t previous_time_ms;
    uint8_t initialized;
    uint8_t holding_center;
} CascadeController;

static CascadeController g_controller;

static float clamp_float(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static float absolute_float(float value)
{
    return (value < 0.0f) ? -value : value;
}

int16_t BallController_MapPixels(int16_t cx_pixels)
{
    int32_t numerator;

    if (cx_pixels <= -320) {
        return -BALL_POSITION_LIMIT_CENTI_CM;
    }
    if (cx_pixels >= 319) {
        return BALL_POSITION_LIMIT_CENTI_CM;
    }
    if (cx_pixels < 0) {
        numerator = ((int32_t)(-cx_pixels) *
                     BALL_POSITION_LIMIT_CENTI_CM) + 160;
        return (int16_t)(-(numerator / 320));
    }

    numerator = ((int32_t)cx_pixels * BALL_POSITION_LIMIT_CENTI_CM) + 159;
    return (int16_t)(numerator / 319);
}

void BallController_Reset(void)
{
    g_controller.position_integral = 0.0f;
    g_controller.velocity_integral = 0.0f;
    g_controller.previous_velocity_error = 0.0f;
    g_controller.filtered_velocity = 0.0f;
    g_controller.previous_position_centi_cm = 0;
    g_controller.previous_time_ms = 0U;
    g_controller.initialized = 0U;
    g_controller.holding_center = 0U;
}

void BallController_Init(void)
{
    BallController_Reset();
}

BallControlCommand BallController_Update(const BallSample *sample,
                                         uint32_t now_ms)
{
    BallControlCommand command = {
        BALL_CONTROL_STOP, ZDT_PIPE_RAISE, 0U, 0U, 0
    };
    uint32_t dt_ms = CONTROL_DT_DEFAULT_MS;
    float dt_seconds;
    float position_cm;
    float raw_velocity;
    float position_error;
    float desired_velocity;
    float velocity_error;
    float velocity_error_derivative;
    float pulse_rate;
    float effort;
    uint16_t duration_ms;

    if ((sample == 0) || (sample->score < BALL_MIN_SCORE)) {
        BallController_Reset();
        return command;
    }

    command.position_centi_cm = BallController_MapPixels(sample->cx_pixels);
    position_cm = (float)command.position_centi_cm / 100.0f;

    if (g_controller.initialized != 0U) {
        dt_ms = now_ms - g_controller.previous_time_ms;
        if ((dt_ms == 0U) || (dt_ms > CONTROL_DT_MAX_MS)) {
            dt_ms = CONTROL_DT_DEFAULT_MS;
            g_controller.filtered_velocity = 0.0f;
            g_controller.position_integral = 0.0f;
            g_controller.velocity_integral = 0.0f;
        }
    }
    dt_seconds = (float)dt_ms / 1000.0f;

    if (g_controller.initialized == 0U) {
        raw_velocity = 0.0f;
        g_controller.filtered_velocity = 0.0f;
        g_controller.initialized = 1U;
    } else {
        raw_velocity =
            ((float)(command.position_centi_cm -
                     g_controller.previous_position_centi_cm) / 100.0f) /
            dt_seconds;
        g_controller.filtered_velocity += BALL_VELOCITY_FILTER_ALPHA *
            (raw_velocity - g_controller.filtered_velocity);
    }
    g_controller.previous_position_centi_cm = command.position_centi_cm;
    g_controller.previous_time_ms = now_ms;

    if (g_controller.holding_center != 0U) {
        if (absolute_float(position_cm) <= BALL_DEAD_ZONE_EXIT_CM) {
            return command;
        }
        g_controller.holding_center = 0U;
    }
    if ((absolute_float(position_cm) <= BALL_DEAD_ZONE_ENTER_CM) &&
        (absolute_float(g_controller.filtered_velocity) <=
         BALL_HOLD_VELOCITY_CM_S)) {
        g_controller.holding_center = 1U;
        g_controller.position_integral = 0.0f;
        g_controller.velocity_integral = 0.0f;
        return command;
    }

    position_error = -position_cm;
    g_controller.position_integral = clamp_float(
        g_controller.position_integral + (position_error * dt_seconds),
        -POSITION_PID_INTEGRAL_LIMIT, POSITION_PID_INTEGRAL_LIMIT);
    desired_velocity = (POSITION_PID_KP * position_error) +
        (POSITION_PID_KI * g_controller.position_integral) -
        (POSITION_PID_KD * g_controller.filtered_velocity);
    desired_velocity = clamp_float(
        desired_velocity, -BALL_TARGET_VELOCITY_LIMIT_CM_S,
        BALL_TARGET_VELOCITY_LIMIT_CM_S);

    velocity_error = desired_velocity - g_controller.filtered_velocity;
    g_controller.velocity_integral = clamp_float(
        g_controller.velocity_integral + (velocity_error * dt_seconds),
        -VELOCITY_PID_INTEGRAL_LIMIT, VELOCITY_PID_INTEGRAL_LIMIT);
    velocity_error_derivative =
        (velocity_error - g_controller.previous_velocity_error) / dt_seconds;
    g_controller.previous_velocity_error = velocity_error;

    pulse_rate = (VELOCITY_PID_KP * velocity_error) +
        (VELOCITY_PID_KI * g_controller.velocity_integral) +
        (VELOCITY_PID_KD * velocity_error_derivative);
    pulse_rate = clamp_float(pulse_rate, -MOTOR_PULSE_RATE_LIMIT_PPS,
                             MOTOR_PULSE_RATE_LIMIT_PPS);

    effort = absolute_float(pulse_rate) / MOTOR_PULSE_RATE_LIMIT_PPS;
    duration_ms = (uint16_t)(ZDT_MOTOR_MIN_PULSE_MS +
        (effort * (float)(ZDT_MOTOR_MAX_PULSE_MS -
                           ZDT_MOTOR_MIN_PULSE_MS)));
    if (duration_ms < ZDT_MOTOR_MIN_PULSE_MS) {
        return command;
    }
    if (duration_ms > ZDT_MOTOR_MAX_PULSE_MS) {
        duration_ms = ZDT_MOTOR_MAX_PULSE_MS;
    }

    command.action = BALL_CONTROL_MOVE;
    command.direction = (pulse_rate > 0.0f) ?
        ZDT_PIPE_RAISE : ZDT_PIPE_LOWER;
    command.speed_rpm = ZDT_MOTOR_SPEED_RPM;
    command.duration_ms = duration_ms;
    return command;
}
