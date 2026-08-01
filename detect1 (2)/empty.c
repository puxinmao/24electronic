/*
 * K230 visual ball balance + PB21 round-trip demonstration
 *
 * Reference configuration and balance parameters are retained from newdetect:
 *   - K230 UART1: PA9 RX, PA8 TX, 115200
 *   - CANFD0: PA13 RX, PA12 TX
 *   - PB21: active-low button; PA2: status LED
 *   - K230 frame: $BALL,x,y,score\r\n
 * Normal mode always balances the ball at image center (x = 320).
 * A debounced PB21 press starts ONE measured visual trajectory:
 *   center -> +5 cm -> -5 cm.
 * During the trajectory, the same visual cascade PID follows each target.
 * At -5 cm completion, the motor stops and holds its current pipe angle. It
 * does not resume continuous center balancing until the next power-up.
 *
 * Motor control deliberately avoids the old "short run / stop every image
 * frame" mechanism.  It receives a continuous RPM target, slews that target
 * every millisecond, and sends an asynchronous CAN F6 command at most every
 * 20 ms.  This separates motor motion from camera frame jitter.
 */

#include "ti_msp_dl_config.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

/* ---------------- newdetect balance parameters: do not retune here -------- */
#define K230_IMAGE_WIDTH                       (640)
#define K230_IMAGE_HEIGHT                      (360)
#define BALL_POSITION_LIMIT_CENTI_CM           (1250)
#define BALL_MIN_SCORE                         (35U)
#define BALL_DEAD_ZONE_ENTER_CM                (1.00f)
#define BALL_DEAD_ZONE_EXIT_CM                 (1.20f)
#define BALL_FINE_CONTROL_ZONE_CM              (1.20f)
#define BALL_FINE_CONTROL_MAX_SPEED_RPM        (1U)
#define BALL_HOLD_VELOCITY_CM_S                (0.50f)
#define BALL_OVERSHOOT_COMPENSATION_S          (0.12f)
#define BALL_OVERSHOOT_COMPENSATION_MAX_CM     (1.50f)
#define BALL_OVERSHOOT_BRAKE_ZONE_CM           (0.80f)
#define BALL_OVERSHOOT_BRAKE_MIN_CM_S          (0.30f)
#define BALL_OVERSHOOT_BRAKE_VELOCITY_CM_S     (0.80f)
#define BALL_OVERSHOOT_BRAKE_MAX_RPM           (2U)
#define POSITION_PID_KP                        (2.00f)
#define POSITION_PID_KI                        (0.08f)
#define POSITION_PID_KD                        (0.75f)
#define POSITION_PID_INTEGRAL_LIMIT            (12.0f)
#define BALL_TARGET_VELOCITY_LIMIT_CM_S        (12.0f)
#define VELOCITY_PID_KP                        (8.0f)
#define VELOCITY_PID_KI                        (0.0f)
#define VELOCITY_PID_KD                        (10.0f)
#define VELOCITY_PID_INTEGRAL_LIMIT            (20.0f)
#define MOTOR_PULSE_RATE_LIMIT_PPS             (2000.0f)
#define BALL_VELOCITY_FILTER_ALPHA             (0.35f)
#define CONTROL_DT_DEFAULT_MS                  (33U)
#define CONTROL_DT_MAX_MS                      (120U)
#define VISION_LOST_TIMEOUT_MS                 (250U)
#define ZDT_MOTOR_ID                           (1U)
#define ZDT_MOTOR_CHECKSUM                     (0x6BU)
#define ZDT_MOTOR_MIN_SPEED_RPM                (2U)
#define ZDT_MOTOR_MAX_SPEED_RPM                (3U)
#define ZDT_MOTOR_ACCEL_LEVEL                  (5U)
#define ZDT_DIRECTION_RIGHT_RAISE              (0x01U)
#define ZDT_DIRECTION_LEFT_LOWER               (0x00U)

/* --------------------- PB21 add-on round-trip parameters ------------------ */
/* These positions use the same 0.01 cm scale as the newdetect controller. */
#define TRAJECTORY_CENTER_CENTI_CM             (0)
#define TRAJECTORY_PLUS_CENTI_CM               (500)   /* +5.00 cm */
#define TRAJECTORY_MINUS_CENTI_CM              (-500)  /* -5.00 cm */
/* Turn thresholds preserve the original detect_and_count visual trajectory. */
#define TRAJECTORY_PLUS_TURN_CENTI_CM          (425)   /* +4.25 cm */
#define TRAJECTORY_MINUS_TURN_CENTI_CM         (-425)  /* -4.25 cm */
#define TRAJECTORY_CENTER_COMPLETE_CENTI_CM    (100)   /* +/-1.00 cm */
#define TRAJECTORY_CENTER_SPEED_CM_S_X100      (50)    /* 0.50 cm/s */
#define BUTTON_DEBOUNCE_MS                     (30U)

/* ------------------- Smooth non-blocking motor transport ------------------ */
#define MOTOR_CAN_EXT_ID                       ((uint32_t)ZDT_MOTOR_ID << 8U)
#define MOTOR_CAN_TX_BUFFER                    (0U)
#define MOTOR_COMMAND_PERIOD_MS                (20U)
#define MOTOR_HEARTBEAT_MS                     (100U)
#define MOTOR_DIRECTION_SETTLE_MS              (20U)
#define MOTOR_SLEW_RPM_PER_SEC                 (180U)

/* PB21 trajectory starts from rest.  Limit only its first outward leg so the
 * initial 0 -> +5 cm setpoint step does not launch the ball too hard. */
#define TRAJECTORY_START_MAX_SPEED_RPM          (2U)

#define K230_FRAME_MAX_LENGTH                  (40U)

typedef enum {
    PIPE_LOWER = -1,
    PIPE_RAISE = 1
} PipeDirection;

typedef enum {
    CONTROL_STOP = 0,
    CONTROL_MOVE
} ControlAction;

typedef struct {
    ControlAction action;
    PipeDirection direction;
    uint16_t speed_rpm;
    int16_t position_centi_cm;
    int16_t velocity_cm_s_x100;
} BallControlCommand;

typedef struct {
    int16_t cx_pixels;
    uint8_t score;
    uint8_t detected;
    /* Written last by the UART ISR; used to obtain a consistent snapshot. */
    uint32_t sequence;
} VisionSample;

typedef struct {
    float position_integral;
    float velocity_integral;
    float previous_velocity_error;
    float filtered_velocity;
    int16_t previous_position_centi_cm;
    uint32_t previous_time_ms;
    uint8_t initialized;
    uint8_t holding_target;
    uint8_t overshoot_brake_active;
} CascadeController;

typedef struct {
    int16_t desired_speed_rpm;
    int16_t applied_speed_rpm;
    int16_t last_sent_speed_rpm;
    uint16_t slew_fraction;
    uint8_t enabled;
    uint8_t can_ready;
    uint8_t sent_valid;
    uint32_t last_can_tx_ms;
    uint32_t last_command_check_ms;
    uint32_t reverse_ready_ms;
} MotorController;

typedef enum {
    TRAJECTORY_IDLE_CENTER = 0,
    TRAJECTORY_TO_PLUS,
    TRAJECTORY_TO_MINUS,
    TRAJECTORY_HOLD_STOP
} TrajectoryState;

static volatile VisionSample g_vision = {0};
static volatile uint32_t g_milliseconds = 0U;
static CascadeController g_controller = {0};
static MotorController g_motor = {0};
static TrajectoryState g_trajectory = TRAJECTORY_IDLE_CENTER;
static uint8_t g_last_button_sample = 1U;
static uint8_t g_stable_button_level = 1U;
static uint32_t g_button_change_ms = 0U;

void SysTick_Handler(void)
{
    g_milliseconds++;
}

static uint32_t milliseconds(void)
{
    return g_milliseconds;
}

static int16_t clamp_i16(int32_t value, int16_t minimum, int16_t maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return (int16_t)value;
}

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

static uint16_t absolute_i16(int16_t value)
{
    /* Promote before negating so INT16_MIN is handled correctly. */
    return (value < 0) ? (uint16_t)(-(int32_t)value) : (uint16_t)value;
}

static uint8_t time_reached(uint32_t now, uint32_t deadline)
{
    return ((int32_t)(now - deadline) >= 0) ? 1U : 0U;
}

/* ---------------------------- K230 packet reception ----------------------- */
static uint8_t parse_field(char **cursor, long *value, char delimiter)
{
    char *end;

    *value = strtol(*cursor, &end, 10);
    if (end == *cursor) {
        return 0U;
    }
    if (delimiter == '\0') {
        if (*end != '\0') {
            return 0U;
        }
    } else {
        if (*end != delimiter) {
            return 0U;
        }
        end++;
    }
    *cursor = end;
    return 1U;
}

static void parse_ball_frame(char *frame)
{
    char *cursor;
    long x;
    long y;
    long score;

    if ((frame == NULL) || (strncmp(frame, "$BALL,", 6U) != 0)) {
        return;
    }
    cursor = frame + 6;
    if ((parse_field(&cursor, &x, ',') == 0U) ||
        (parse_field(&cursor, &y, ',') == 0U) ||
        (parse_field(&cursor, &score, '\0') == 0U)) {
        return;
    }

    if ((x == -1L) && (y == -1L)) {
        g_vision.detected = 0U;
        g_vision.sequence++;
        return;
    }
    if ((x < 0L) || (x >= K230_IMAGE_WIDTH) ||
        (y < 0L) || (y >= K230_IMAGE_HEIGHT) ||
        (score < 0L) || (score > 100L)) {
        return;
    }

    g_vision.cx_pixels = (int16_t)(x - (K230_IMAGE_WIDTH / 2));
    g_vision.score = (uint8_t)score;
    g_vision.detected = 1U;
    g_vision.sequence++;
}

void UART_2_INST_IRQHandler(void)
{
    static char line[K230_FRAME_MAX_LENGTH];
    static uint8_t length = 0U;
    uint8_t byte;

    switch (DL_UART_getPendingInterrupt(UART_2_INST)) {
        case DL_UART_IIDX_RX:
            while (DL_UART_Main_isRXFIFOEmpty(UART_2_INST) == false) {
                byte = DL_UART_Main_receiveData(UART_2_INST);
                if (byte == '$') {
                    line[0] = '$';
                    length = 1U;
                } else if (length == 0U) {
                    /* Wait for the next frame start. */
                } else if (byte == '\r') {
                    /* CR is optional. */
                } else if (byte == '\n') {
                    line[length] = '\0';
                    parse_ball_frame(line);
                    length = 0U;
                } else if (length < (K230_FRAME_MAX_LENGTH - 1U)) {
                    line[length++] = (char)byte;
                } else {
                    length = 0U;
                }
            }
            break;
        default:
            break;
    }
}

static void k230_uart_init(void)
{
    NVIC_ClearPendingIRQ(UART_2_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_2_INST_INT_IRQN);
}

static uint8_t take_latest_vision(VisionSample *sample, uint32_t *last_sequence)
{
    uint32_t sequence;

    if ((sample == NULL) || (last_sequence == NULL)) {
        return 0U;
    }

    /* A sequence-before/after retry alone is not sufficient: the UART ISR
     * could update one payload field before it increments sequence.  Mask the
     * short UART interrupt while copying, so the main loop sees one complete
     * frame.  The copy is only a few CPU instructions, so RX FIFO overflow is
     * not a concern at 115200 baud. */
    NVIC_DisableIRQ(UART_2_INST_INT_IRQN);
    sequence = g_vision.sequence;
    sample->cx_pixels = g_vision.cx_pixels;
    sample->score = g_vision.score;
    sample->detected = g_vision.detected;
    NVIC_EnableIRQ(UART_2_INST_INT_IRQN);

    if (sequence == *last_sequence) {
        return 0U;
    }
    sample->sequence = sequence;
    *last_sequence = sequence;
    return 1U;
}

/* ------------------------- asynchronous CAN motor layer -------------------- */
static uint8_t motor_can_send_async(const uint8_t *data, uint8_t length)
{
    DL_MCAN_TxBufElement message = {0};
    uint8_t index;

    if ((g_motor.can_ready == 0U) || (data == NULL) ||
        (length == 0U) || (length > 8U)) {
        return 0U;
    }
    if ((DL_MCAN_getTxBufReqPend(MCAN0_INST) &
         (1UL << MOTOR_CAN_TX_BUFFER)) != 0U) {
        return 0U;
    }

    message.id = MOTOR_CAN_EXT_ID;
    message.rtr = 0U;
    message.xtd = 1U;
    message.esi = 0U;
    message.dlc = length;
    message.brs = 0U;
    message.fdf = 0U;
    message.efc = 0U;
    message.mm = 0U;
    for (index = 0U; index < length; index++) {
        message.data[index] = data[index];
    }
    DL_MCAN_writeMsgRam(MCAN0_INST, DL_MCAN_MEM_TYPE_BUF,
                        MOTOR_CAN_TX_BUFFER, &message);
    DL_MCAN_TXBufAddReq(MCAN0_INST, MOTOR_CAN_TX_BUFFER);
    return 1U;
}

static uint8_t motor_enable(void)
{
    const uint8_t command[5] = {0xF3U, 0xABU, 0x01U, 0x00U,
                                ZDT_MOTOR_CHECKSUM};

    if (motor_can_send_async(command, sizeof(command)) == 0U) {
        return 0U;
    }
    g_motor.enabled = 1U;
    return 1U;
}

static uint8_t motor_send_speed(int16_t signed_rpm)
{
    uint8_t command[7];
    uint16_t speed;

    if (signed_rpm == 0) {
        return 0U;
    }
    if ((g_motor.enabled == 0U) && (motor_enable() == 0U)) {
        return 0U;
    }

    speed = (uint16_t)absolute_i16(signed_rpm);
    command[0] = 0xF6U;
    command[1] = (signed_rpm > 0) ? ZDT_DIRECTION_RIGHT_RAISE :
                                     ZDT_DIRECTION_LEFT_LOWER;
    command[2] = (uint8_t)(speed >> 8U);
    command[3] = (uint8_t)(speed & 0xFFU);
    command[4] = ZDT_MOTOR_ACCEL_LEVEL;
    command[5] = 0x00U;
    command[6] = ZDT_MOTOR_CHECKSUM;
    return motor_can_send_async(command, sizeof(command));
}

static uint8_t motor_send_stop(void)
{
    const uint8_t command[4] = {0xFEU, 0x98U, 0x00U, ZDT_MOTOR_CHECKSUM};

    return motor_can_send_async(command, sizeof(command));
}

static void motor_init(void)
{
    uint32_t start = milliseconds();

    while ((uint32_t)(milliseconds() - start) < 50U) {
        if (DL_MCAN_getOpMode(MCAN0_INST) == DL_MCAN_OPERATION_MODE_NORMAL) {
            g_motor.can_ready = 1U;
            return;
        }
    }
}

static int16_t motor_slew_toward(int16_t current, int16_t target)
{
    uint16_t step;

    if (current == target) {
        g_motor.slew_fraction = 0U;
        return current;
    }
    g_motor.slew_fraction += MOTOR_SLEW_RPM_PER_SEC;
    step = (uint16_t)(g_motor.slew_fraction / 1000U);
    g_motor.slew_fraction %= 1000U;
    if (step == 0U) {
        return current;
    }
    if (current < target) {
        return ((int32_t)current + step > target) ? target :
                                                    (int16_t)(current + step);
    }
    return ((int32_t)current - step < target) ? target :
                                                (int16_t)(current - step);
}

static void motor_scheduler_tick(uint32_t now_ms)
{
    int16_t previous = g_motor.applied_speed_rpm;
    int16_t requested = g_motor.desired_speed_rpm;
    int16_t next;
    uint8_t changed;
    uint8_t heartbeat_due;

    /* First decelerate the applied command to zero before a requested
     * direction reversal. */
    if (((previous > 0) && (requested < 0)) ||
        ((previous < 0) && (requested > 0))) {
        requested = 0;
    }
    next = motor_slew_toward(previous, requested);

    /* Do not generate an opposite-direction command until a CAN stop frame
     * has actually been queued and its settle interval has elapsed.  This is
     * important when the single TX buffer was busy at the first stop attempt. */
    if (((next > 0) && (g_motor.last_sent_speed_rpm < 0)) ||
        ((next < 0) && (g_motor.last_sent_speed_rpm > 0)) ||
        ((next != 0) &&
         (time_reached(now_ms, g_motor.reverse_ready_ms) == 0U))) {
        next = 0;
        g_motor.slew_fraction = 0U;
    }
    g_motor.applied_speed_rpm = next;

    /* Do not use now_ms % period here: if the main loop happens to miss a
     * particular millisecond, a modulo-based scheduler can miss every send
     * slot.  An elapsed-time gate works with arbitrary loop jitter. */
    if ((uint32_t)(now_ms - g_motor.last_command_check_ms) <
        MOTOR_COMMAND_PERIOD_MS) {
        return;
    }
    g_motor.last_command_check_ms = now_ms;

    changed = (g_motor.sent_valid == 0U) ||
              (next != g_motor.last_sent_speed_rpm);
    heartbeat_due = ((uint32_t)(now_ms - g_motor.last_can_tx_ms) >=
                     MOTOR_HEARTBEAT_MS) ? 1U : 0U;
    if ((changed == 0U) && (heartbeat_due == 0U)) {
        return;
    }

    if (next == 0) {
        if ((g_motor.sent_valid != 0U) &&
            (g_motor.last_sent_speed_rpm != 0) &&
            (motor_send_stop() != 0U)) {
            g_motor.last_sent_speed_rpm = 0;
            g_motor.last_can_tx_ms = now_ms;
            g_motor.reverse_ready_ms = now_ms + MOTOR_DIRECTION_SETTLE_MS;
        }
    } else if (motor_send_speed(next) != 0U) {
        g_motor.last_sent_speed_rpm = next;
        g_motor.sent_valid = 1U;
        g_motor.last_can_tx_ms = now_ms;
    }
}

/* --------------------- newdetect cascade PID with setpoint ----------------- */
static int16_t map_pixels_to_position(int16_t cx_pixels)
{
    int32_t numerator;

    if (cx_pixels <= -320) {
        return -BALL_POSITION_LIMIT_CENTI_CM;
    }
    if (cx_pixels >= 319) {
        return BALL_POSITION_LIMIT_CENTI_CM;
    }
    if (cx_pixels < 0) {
        numerator = ((int32_t)(-cx_pixels) * BALL_POSITION_LIMIT_CENTI_CM) +
                    160;
        return (int16_t)(-(numerator / 320));
    }
    numerator = ((int32_t)cx_pixels * BALL_POSITION_LIMIT_CENTI_CM) + 159;
    return (int16_t)(numerator / 319);
}

static void controller_reset(void)
{
    memset(&g_controller, 0, sizeof(g_controller));
}

static BallControlCommand controller_update(const VisionSample *sample,
                                             uint32_t now_ms,
                                             int16_t target_centi_cm)
{
    BallControlCommand command = {CONTROL_STOP, PIPE_RAISE, 0U, 0, 0};
    uint32_t dt_ms = CONTROL_DT_DEFAULT_MS;
    float dt_seconds;
    float position_cm;
    float relative_position_cm;
    float predicted_relative_position_cm;
    float raw_velocity;
    float position_error;
    float desired_velocity;
    float velocity_error;
    float velocity_error_derivative;
    float pulse_rate;
    float effort;
    uint16_t speed_rpm;

    if ((sample == NULL) || (sample->score < BALL_MIN_SCORE)) {
        controller_reset();
        return command;
    }

    command.position_centi_cm = map_pixels_to_position(sample->cx_pixels);
    position_cm = (float)command.position_centi_cm / 100.0f;
    relative_position_cm =
        (float)(command.position_centi_cm - target_centi_cm) / 100.0f;

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
        g_controller.initialized = 1U;
        g_controller.filtered_velocity = 0.0f;
    } else {
        raw_velocity = ((float)(command.position_centi_cm -
                         g_controller.previous_position_centi_cm) / 100.0f) /
                       dt_seconds;
        g_controller.filtered_velocity += BALL_VELOCITY_FILTER_ALPHA *
            (raw_velocity - g_controller.filtered_velocity);
    }
    g_controller.previous_position_centi_cm = command.position_centi_cm;
    g_controller.previous_time_ms = now_ms;
    command.velocity_cm_s_x100 = clamp_i16(
        (int32_t)(g_controller.filtered_velocity * 100.0f), -32768, 32767);

    if (g_controller.holding_target != 0U) {
        if (absolute_float(relative_position_cm) <= BALL_DEAD_ZONE_EXIT_CM) {
            return command;
        }
        g_controller.holding_target = 0U;
    }
    if ((absolute_float(relative_position_cm) <= BALL_DEAD_ZONE_ENTER_CM) &&
        (absolute_float(g_controller.filtered_velocity) <=
         BALL_HOLD_VELOCITY_CM_S)) {
        g_controller.holding_target = 1U;
        g_controller.position_integral = 0.0f;
        g_controller.velocity_integral = 0.0f;
        return command;
    }

    if ((g_controller.overshoot_brake_active != 0U) &&
        ((relative_position_cm * g_controller.filtered_velocity) > 0.0f)) {
        g_controller.overshoot_brake_active = 0U;
        g_controller.position_integral = 0.0f;
        g_controller.velocity_integral = 0.0f;
        g_controller.previous_velocity_error = 0.0f;
    }

    predicted_relative_position_cm = relative_position_cm + clamp_float(
        g_controller.filtered_velocity * BALL_OVERSHOOT_COMPENSATION_S,
        -BALL_OVERSHOOT_COMPENSATION_MAX_CM,
        BALL_OVERSHOOT_COMPENSATION_MAX_CM);
    position_error = -predicted_relative_position_cm;

    if (((relative_position_cm * g_controller.filtered_velocity) < 0.0f) &&
        (absolute_float(g_controller.filtered_velocity) >=
         BALL_OVERSHOOT_BRAKE_MIN_CM_S) &&
        (absolute_float(predicted_relative_position_cm) <=
         BALL_OVERSHOOT_BRAKE_ZONE_CM) &&
        (g_controller.overshoot_brake_active == 0U)) {
        g_controller.overshoot_brake_active = 1U;
        g_controller.position_integral = 0.0f;
        g_controller.velocity_integral = 0.0f;
        g_controller.previous_velocity_error = 0.0f;
        return command;
    }

    g_controller.position_integral = clamp_float(
        g_controller.position_integral + (position_error * dt_seconds),
        -POSITION_PID_INTEGRAL_LIMIT, POSITION_PID_INTEGRAL_LIMIT);
    desired_velocity = (POSITION_PID_KP * position_error) +
        (POSITION_PID_KI * g_controller.position_integral) -
        (POSITION_PID_KD * g_controller.filtered_velocity);
    desired_velocity = clamp_float(desired_velocity,
        -BALL_TARGET_VELOCITY_LIMIT_CM_S, BALL_TARGET_VELOCITY_LIMIT_CM_S);

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
    speed_rpm = (uint16_t)(ZDT_MOTOR_MIN_SPEED_RPM +
        (effort * (float)(ZDT_MOTOR_MAX_SPEED_RPM - ZDT_MOTOR_MIN_SPEED_RPM)));
    if (speed_rpm > ZDT_MOTOR_MAX_SPEED_RPM) {
        speed_rpm = ZDT_MOTOR_MAX_SPEED_RPM;
    }
    if ((absolute_float(relative_position_cm) <= BALL_FINE_CONTROL_ZONE_CM) &&
        (speed_rpm > BALL_FINE_CONTROL_MAX_SPEED_RPM)) {
        speed_rpm = BALL_FINE_CONTROL_MAX_SPEED_RPM;
    }
    command.action = CONTROL_MOVE;
    command.direction = (pulse_rate > 0.0f) ? PIPE_RAISE : PIPE_LOWER;
    command.speed_rpm = speed_rpm;
    (void)position_cm;
    return command;
}

/* --------------------------- PB21 round trip ------------------------------ */
static uint8_t button_pressed(uint32_t now_ms)
{
    uint8_t sample = (DL_GPIO_readPins(KEY_PORT, KEY_PIN_0_PIN) != 0U) ?
                     1U : 0U;

    if (sample != g_last_button_sample) {
        g_last_button_sample = sample;
        g_button_change_ms = now_ms;
    }
    if (((uint32_t)(now_ms - g_button_change_ms) < BUTTON_DEBOUNCE_MS) ||
        (sample == g_stable_button_level)) {
        return 0U;
    }
    g_stable_button_level = sample;
    return (sample == 0U) ? 1U : 0U;
}

static int16_t trajectory_target(void)
{
    switch (g_trajectory) {
        case TRAJECTORY_TO_PLUS:
            return TRAJECTORY_PLUS_CENTI_CM;
        case TRAJECTORY_TO_MINUS:
            return TRAJECTORY_MINUS_CENTI_CM;
        case TRAJECTORY_IDLE_CENTER:
        case TRAJECTORY_HOLD_STOP:
        default:
            return TRAJECTORY_CENTER_CENTI_CM;
    }
}

static void trajectory_start(void)
{
    g_trajectory = TRAJECTORY_TO_PLUS;
    controller_reset();
    DL_GPIO_setPins(LED1_PORT, LED1_PIN_2_PIN);
}

static uint8_t trajectory_update(int16_t position_centi_cm,
                                 int16_t velocity_cm_s_x100)
{
    uint8_t transitioned = 0U;

    switch (g_trajectory) {
        case TRAJECTORY_TO_PLUS:
            if (position_centi_cm >= TRAJECTORY_PLUS_TURN_CENTI_CM) {
                g_trajectory = TRAJECTORY_TO_MINUS;
                transitioned = 1U;
            }
            break;
        case TRAJECTORY_TO_MINUS:
            if ((absolute_i16(position_centi_cm -
                              TRAJECTORY_MINUS_CENTI_CM) <=
                 TRAJECTORY_CENTER_COMPLETE_CENTI_CM) &&
                (absolute_i16(velocity_cm_s_x100) <=
                 TRAJECTORY_CENTER_SPEED_CM_S_X100)) {
                g_trajectory = TRAJECTORY_HOLD_STOP;
                DL_GPIO_clearPins(LED1_PORT, LED1_PIN_2_PIN);
                transitioned = 1U;
            }
            break;
        default:
            break;
    }

    if (transitioned != 0U) {
        controller_reset();
    }
    return transitioned;
}

int main(void)
{
    VisionSample sample;
    BallControlCommand command;
    uint32_t last_sequence = 0U;
    uint32_t last_valid_vision_ms = 0U;
    uint32_t last_tick;

    SYSCFG_DL_init();
    if (SysTick_Config(CPUCLK_FREQ / 1000U) != 0U) {
        while (1) {
            __WFI();
        }
    }
    k230_uart_init();
    motor_init();
    controller_reset();
    DL_GPIO_clearPins(LED1_PORT, LED1_PIN_2_PIN);
    last_tick = milliseconds();

    while (1) {
        uint32_t now_ms = milliseconds();

        if (now_ms == last_tick) {
            __WFI();
            continue;
        }
        last_tick = now_ms;

        if ((button_pressed(now_ms) != 0U) &&
            ((g_trajectory == TRAJECTORY_IDLE_CENTER) ||
             (g_trajectory == TRAJECTORY_HOLD_STOP))) {
            trajectory_start();
        }

        if (take_latest_vision(&sample, &last_sequence) != 0U) {
            if ((sample.detected != 0U) && (sample.score >= BALL_MIN_SCORE)) {
                last_valid_vision_ms = now_ms;
                if (g_trajectory == TRAJECTORY_HOLD_STOP) {
                    g_motor.desired_speed_rpm = 0;
                } else {
                    command = controller_update(&sample, now_ms,
                                                trajectory_target());
                    if (trajectory_update(command.position_centi_cm,
                                          command.velocity_cm_s_x100) != 0U) {
                        /* The command above used the old setpoint.  Stop first;
                         * the next visual frame will command the new direction. */
                        g_motor.desired_speed_rpm = 0;
                    } else if (command.action == CONTROL_MOVE) {
                        if ((g_trajectory == TRAJECTORY_TO_PLUS) &&
                            (command.speed_rpm >
                             TRAJECTORY_START_MAX_SPEED_RPM)) {
                            command.speed_rpm = TRAJECTORY_START_MAX_SPEED_RPM;
                        }
                        g_motor.desired_speed_rpm =
                            (command.direction == PIPE_RAISE) ?
                            (int16_t)command.speed_rpm :
                            -(int16_t)command.speed_rpm;
                    } else {
                        g_motor.desired_speed_rpm = 0;
                    }
                }
            } else {
                controller_reset();
                g_motor.desired_speed_rpm = 0;
                /* Keep the PB21 state.  A dropped camera frame must stop the
                 * motor safely, but must not turn a pending -5 cm destination
                 * back into the normal 0 cm target when vision returns. */
            }
        }

        if ((uint32_t)(now_ms - last_valid_vision_ms) >=
            VISION_LOST_TIMEOUT_MS) {
            g_motor.desired_speed_rpm = 0;
            controller_reset();
        }

        motor_scheduler_tick(now_ms);
    }
}
