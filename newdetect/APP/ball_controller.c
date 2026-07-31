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
    uint8_t fine_trim_active;
    float fine_trim_tilt_bias_mdeg;
    BallControllerTelemetry telemetry;
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

static int16_t float_to_centi(float value)
{
    value *= 100.0f;
    if (value > 32767.0f) {
        return 32767;
    }
    if (value < -32768.0f) {
        return -32768;
    }
    return (int16_t)value;
}

static void update_telemetry(float position_cm, float predicted_position_cm,
                             float position_error, float desired_velocity,
                             float velocity_error, float pulse_rate,
                             const BallControlCommand *command)
{
    g_controller.telemetry.position_centi_cm = float_to_centi(position_cm);
    g_controller.telemetry.predicted_position_centi_cm =
        float_to_centi(predicted_position_cm);
    g_controller.telemetry.filtered_velocity_centi_cm_s =
        float_to_centi(g_controller.filtered_velocity);
    g_controller.telemetry.position_error_centi_cm =
        float_to_centi(position_error);
    g_controller.telemetry.desired_velocity_centi_cm_s =
        float_to_centi(desired_velocity);
    g_controller.telemetry.velocity_error_centi_cm_s =
        float_to_centi(velocity_error);
    g_controller.telemetry.pulse_rate = (int16_t)pulse_rate;
    g_controller.telemetry.direction =
        (command->action == BALL_CONTROL_MOVE) ? (int8_t)command->direction :
                                                  0;
    g_controller.telemetry.speed_rpm = command->speed_rpm;
    g_controller.telemetry.burst_duration_ms = command->burst_duration_ms;
    g_controller.telemetry.control_mode = command->fine_trim ? 1U :
        (command->recovery ? 2U :
         (command->braking ? 4U :
          ((command->action == BALL_CONTROL_MOVE) ? 3U : 0U)));
    g_controller.telemetry.holding_center = g_controller.holding_center;
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
    g_controller.fine_trim_active = 0U;
    g_controller.fine_trim_tilt_bias_mdeg = 0.0f;
    g_controller.telemetry.position_centi_cm = 0;
    g_controller.telemetry.predicted_position_centi_cm = 0;
    g_controller.telemetry.filtered_velocity_centi_cm_s = 0;
    g_controller.telemetry.position_error_centi_cm = 0;
    g_controller.telemetry.desired_velocity_centi_cm_s = 0;
    g_controller.telemetry.velocity_error_centi_cm_s = 0;
    g_controller.telemetry.pulse_rate = 0;
    g_controller.telemetry.direction = 0;
    g_controller.telemetry.speed_rpm = 0U;
    g_controller.telemetry.burst_duration_ms = 0U;
    g_controller.telemetry.control_mode = 0U;
    g_controller.telemetry.holding_center = 0U;
    g_controller.telemetry.vision_active = 0U;
}

void BallController_Init(void)
{
    BallController_Reset();
}

void BallController_GetTelemetry(BallControllerTelemetry *telemetry)
{
    if (telemetry != 0) {
        *telemetry = g_controller.telemetry;
    }
}

BallControlCommand BallController_Update(const BallSample *sample,
                                         uint32_t now_ms)
{
    BallControlCommand command = {
        BALL_CONTROL_STOP, ZDT_PIPE_RAISE, 0U, 0U, 0U, 0U, 0U, 0, 0
    };
    uint32_t dt_ms = CONTROL_DT_DEFAULT_MS;
    float dt_seconds;
    float position_cm;
    float predicted_position_cm;
    float prediction_offset_cm;
    float brake_distance_cm;
    float raw_velocity;
    float position_error;
    float desired_velocity;
    float velocity_error;
    float velocity_error_derivative;
    float pulse_rate;
    float effort;
    float recovery_effort;
    float tilt_magnitude;
    uint16_t speed_rpm;
    uint8_t braking_active = 0U;
    ZdtPipeDirection fine_trim_direction = ZDT_PIPE_RAISE;

    if ((sample == 0) || (sample->score < BALL_MIN_SCORE)) {
        BallController_Reset();
        return command;
    }

    g_controller.telemetry.vision_active = 1U;

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
        if ((absolute_float(position_cm) <= BALL_DEAD_ZONE_EXIT_CM) &&
            (absolute_float(g_controller.filtered_velocity) <=
             BALL_HOLD_VELOCITY_EXIT_CM_S)) {
            update_telemetry(position_cm, position_cm, 0.0f, 0.0f, 0.0f,
                             0.0f, &command);
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
        g_controller.fine_trim_active = 0U;
        update_telemetry(position_cm, position_cm, 0.0f, 0.0f, 0.0f,
                         0.0f, &command);
        return command;
    }

    if ((g_controller.fine_trim_active != 0U) &&
        (absolute_float(g_controller.filtered_velocity) >=
         BALL_FINE_TRIM_RELEASE_VELOCITY_CM_S)) {
        g_controller.fine_trim_active = 0U;
        g_controller.fine_trim_tilt_bias_mdeg = 0.0f;
    }
    if ((g_controller.fine_trim_active == 0U) &&
        (absolute_float(position_cm) <= BALL_FINE_TRIM_ZONE_CM) &&
        (absolute_float(g_controller.filtered_velocity) <=
         BALL_FINE_TRIM_VELOCITY_CM_S)) {
        g_controller.fine_trim_active = 1U;
        g_controller.fine_trim_tilt_bias_mdeg = 0.0f;
    }
    if (g_controller.fine_trim_active != 0U) {
        if (((position_cm * g_controller.filtered_velocity) < 0.0f) &&
            (absolute_float(g_controller.filtered_velocity) >=
             BALL_FINE_TRIM_VELOCITY_CM_S)) {
            g_controller.fine_trim_tilt_bias_mdeg = 0.0f;
        } else if (absolute_float(g_controller.filtered_velocity) <=
                   BALL_FINE_TRIM_VELOCITY_CM_S) {
            g_controller.fine_trim_tilt_bias_mdeg = clamp_float(
                g_controller.fine_trim_tilt_bias_mdeg +
                (BALL_FINE_TRIM_TILT_RAMP_MDEG_PER_S * dt_seconds),
                0.0f, BALL_FINE_TRIM_TILT_BIAS_MAX_MDEG);
        }
    }
    fine_trim_direction = (position_cm < 0.0f) ?
        ZDT_PIPE_RAISE : ZDT_PIPE_LOWER;
    if (BALL_CONTROL_REVERSE_DIRECTION != 0U) {
        fine_trim_direction = (fine_trim_direction == ZDT_PIPE_RAISE) ?
            ZDT_PIPE_LOWER : ZDT_PIPE_RAISE;
    }

    prediction_offset_cm = clamp_float(
        g_controller.filtered_velocity * BALL_PREDICTION_TIME_S,
        -BALL_PREDICTION_MAX_CM, BALL_PREDICTION_MAX_CM);
    predicted_position_cm = position_cm + prediction_offset_cm;
    position_error = -predicted_position_cm;
    g_controller.position_integral = clamp_float(
        g_controller.position_integral + (position_error * dt_seconds),
        -POSITION_PID_INTEGRAL_LIMIT, POSITION_PID_INTEGRAL_LIMIT);
    desired_velocity = (POSITION_PID_KP * position_error) +
        (POSITION_PID_KI * g_controller.position_integral) -
        (POSITION_PID_KD * g_controller.filtered_velocity);

    if (((position_cm * g_controller.filtered_velocity) < 0.0f) &&
        (absolute_float(g_controller.filtered_velocity) >=
         BALL_BRAKE_MIN_VELOCITY_CM_S)) {
        brake_distance_cm = BALL_BRAKE_SAFETY_DISTANCE_CM +
            ((g_controller.filtered_velocity * g_controller.filtered_velocity) /
             (2.0f * BALL_BRAKE_DECELERATION_CM_S2));
        if (absolute_float(position_cm) <= brake_distance_cm) {
            desired_velocity = BALL_BRAKE_TARGET_VELOCITY_CM_S;
            braking_active = 1U;
            g_controller.position_integral = 0.0f;
        }
    }
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
    if ((absolute_float(g_controller.filtered_velocity) >=
         BALL_BRAKE_MIN_VELOCITY_CM_S) &&
        ((pulse_rate * g_controller.filtered_velocity) < 0.0f)) {
        braking_active = 1U;
    }

    if ((g_controller.fine_trim_active == 0U) &&
        (absolute_float(pulse_rate) < MOTOR_PULSE_RATE_STOP_THRESHOLD_PPS)) {
        update_telemetry(position_cm, predicted_position_cm, position_error,
                         desired_velocity, velocity_error, pulse_rate,
                         &command);
        return command;
    }

    if ((g_controller.fine_trim_active != 0U) &&
        (absolute_float(g_controller.filtered_velocity) >
         BALL_FINE_TRIM_VELOCITY_CM_S)) {
        update_telemetry(position_cm, predicted_position_cm, position_error,
                         desired_velocity, velocity_error, pulse_rate,
                         &command);
        return command;
    }

    effort = absolute_float(pulse_rate) / MOTOR_PULSE_RATE_LIMIT_PPS;
    speed_rpm = (uint16_t)(ZDT_MOTOR_MIN_SPEED_RPM +
        (effort * (float)(ZDT_MOTOR_DYNAMIC_MAX_SPEED_RPM -
                           ZDT_MOTOR_MIN_SPEED_RPM)) + 0.5f);
    if (speed_rpm > ZDT_MOTOR_DYNAMIC_MAX_SPEED_RPM) {
        speed_rpm = ZDT_MOTOR_DYNAMIC_MAX_SPEED_RPM;
    }
    command.action = BALL_CONTROL_MOVE;
    command.direction = (pulse_rate > 0.0f) ?
        ZDT_PIPE_RAISE : ZDT_PIPE_LOWER;
    if (BALL_CONTROL_REVERSE_DIRECTION != 0U) {
        command.direction = (command.direction == ZDT_PIPE_RAISE) ?
            ZDT_PIPE_LOWER : ZDT_PIPE_RAISE;
    }
    command.speed_rpm = speed_rpm;
    if (g_controller.fine_trim_active != 0U) {
        command.direction = fine_trim_direction;
        command.speed_rpm = ZDT_MOTOR_MIN_SPEED_RPM;
        command.burst_duration_ms = MOTOR_FINE_TRIM_BURST_MS;
        command.fine_trim = 1U;
        tilt_magnitude = PIPE_TILT_FINE_MIN_MDEG +
            (absolute_float(position_cm) / BALL_FINE_TRIM_ZONE_CM) *
            (PIPE_TILT_FINE_MAX_MDEG - PIPE_TILT_FINE_MIN_MDEG);
        tilt_magnitude += g_controller.fine_trim_tilt_bias_mdeg;
        tilt_magnitude = clamp_float(tilt_magnitude,
                                     PIPE_TILT_FINE_MIN_MDEG,
                                     PIPE_TILT_FINE_LIMIT_MDEG);
    } else if (absolute_float(g_controller.filtered_velocity) <=
               BALL_FINE_TRIM_VELOCITY_CM_S) {
        command.direction = fine_trim_direction;
        command.speed_rpm =
            (absolute_float(position_cm) >=
             MOTOR_RECOVERY_HIGH_SPEED_ZONE_CM) ?
            ZDT_MOTOR_MAX_SPEED_RPM : 2U;
        recovery_effort =
            (absolute_float(position_cm) - BALL_FINE_TRIM_ZONE_CM) /
            (BALL_POSITION_LIMIT_CENTI_CM / 100.0f -
             BALL_FINE_TRIM_ZONE_CM);
        recovery_effort = clamp_float(recovery_effort, 0.0f, 1.0f);
        command.burst_duration_ms = (uint16_t)(
            MOTOR_RECOVERY_BURST_MIN_MS +
            recovery_effort * (float)(MOTOR_RECOVERY_BURST_MAX_MS -
                                      MOTOR_RECOVERY_BURST_MIN_MS) +
            0.5f);
        command.recovery = 1U;
        tilt_magnitude = PIPE_TILT_RECOVERY_MIN_MDEG +
            recovery_effort * (PIPE_TILT_RECOVERY_MAX_MDEG -
                               PIPE_TILT_RECOVERY_MIN_MDEG);
    } else if (braking_active != 0U) {
        command.braking = 1U;
        tilt_magnitude = effort * PIPE_TILT_DYNAMIC_LIMIT_MDEG;
    } else {
        command.burst_duration_ms = 0U;
        tilt_magnitude = effort * PIPE_TILT_DYNAMIC_LIMIT_MDEG;
    }
    command.target_tilt_milli_degree =
        (int16_t)((command.direction == ZDT_PIPE_RAISE) ?
                  tilt_magnitude : -tilt_magnitude);
    update_telemetry(position_cm, predicted_position_cm, position_error,
                     desired_velocity, velocity_error, pulse_rate, &command);
    return command;
}
