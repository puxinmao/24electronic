#include "ti_msp_dl_config.h"
#include "stdint.h"
#include "usart.h"
#include "yb_protocol.h"


/*
 * 水管升降平衡控制。
 *
 * K230 发送 $BALL,x,y,score，其中 x=0..639。协议层将 x 转为相对中心的
 * cx=-320..319；本文件再精确映射为 -12.50..+12.50（单位：0.01）。
 *   x < 0：抬高水管；x > 0：降低水管；x = 0：保持。
 *
 * 电机只支持连续转速命令，因此 PD 输出被转换为“低速、短脉冲”动作。
 * 每一帧新的视觉坐标都会更新一次 PD，并通过 CAN 刷新方向、速度和命令持续时间。
 */
#define K230_CENTER_X_OFFSET_PIXELS      (0)
#define K230_LEFT_EDGE_OFFSET_PIXELS   (-320)
#define K230_RIGHT_EDGE_OFFSET_PIXELS    (319)
#define BALL_POSITION_LIMIT             (1250)  /* 12.50 * 100 */
#define BALL_POSITION_DEADBAND            (20)  /* Hold inside +/-0.20 cm of target. */
#define BALL_RESTART_DEADBAND             (30)  /* Correct again beyond +/-0.30 cm. */
#define BALL_VELOCITY_HOLD_DEADBAND         (3)  /* Hold below 0.03 cm per 20 ms. */
#define BALL_VELOCITY_RESTART_DEADBAND      (5)  /* Brake again above 0.05 cm per 20 ms. */
#define BALL_MIN_CONFIDENCE               (35)  /* K230 score: 0..100 */

/* 自动轨迹：O(0 cm) -> +5 cm -> -5 cm，最终稳定在 -5 cm 附近。 */
#define TRAJECTORY_ORIGIN_POSITION          (0)
#define TRAJECTORY_PLUS_POSITION          (500)  /* +5.00 cm */
#define TRAJECTORY_MINUS_POSITION        (-500)  /* -5.00 cm */
#define TRAJECTORY_FINAL_CONFIRM_FRAMES      (5)
#define TRAJECTORY_BUTTON_DEBOUNCE_MS       (30)
/*
 * 按键后目标直接设为 +5 cm，到位后直接切换为 -5 cm。
 * 全程使用同一个 PD 闭环；到达 -5 cm 后持续保持该目标。
 */

/*
 * 位置-速度 PD：command = Kp * (target-position) - Kd * velocity。
 * velocity 统一换算为每 20 ms 的位移，避免视觉帧间隔变化直接改变 D 项。
 */
#define PD_POSITION_KP_NUM                 (5)  /* Kp = 0.18 */
#define PD_VELOCITY_KD_NUM                (100)  /* Kd = 1.00 */
#define PD_GAIN_DEN                       (180)
#define PD_VELOCITY_TIME_BASE_MS           (20)
#define PD_VELOCITY_DEADBAND                (5)
#define PD_FRAME_INTERVAL_MAX_MS          (100)
#define PD_OUTPUT_LIMIT                   (300)
#define PD_OUTPUT_MIN                       (2)
/* Emm_V5 / ZDT_X CAN1_MAP 参数；须与电机内部设置相同。 */
#define MOTOR_ID                           (1)
#define MOTOR_CHECKSUM                   (0x6B)
#define MOTOR_DIR_CW                     (0x00)
#define MOTOR_DIR_CCW                    (0x01)

/*
 * 本机构当前接线/机械方向：抬高水管=CCW，降低水管=CW。
 * 首次上机时务必低速验证；若方向相反，只交换下面两个宏，不要改 PID 符号。
 */
#define MOTOR_DIRECTION_RAISE     MOTOR_DIR_CCW
#define MOTOR_DIRECTION_LOWER     MOTOR_DIR_CW

/* F6 为持续速度命令。采用短脉冲以减小每次水管位移。 */
#define MOTOR_ACCEL_LEVEL                 (20)
#define MOTOR_FULL_EFFORT                (120)
#define MOTOR_MIN_SPEED_RPM               (12)
#define MOTOR_MAX_SPEED_RPM               (32)
#define MOTOR_PULSE_MIN_MS                (20)
#define MOTOR_PULSE_MAX_MS                (45)
#define MOTOR_PULSE_GAP_MS                 (3)
#define MOTOR_DIRECTION_SETTLE_MS          (5)  /* Stop briefly before a reversal. */
#define MOTOR_CAN_TX_TIMEOUT_MS           (10)
#define VISION_LOST_TIMEOUT_MS           (300)

#define MOTOR_CAN_EXT_ID          ((uint32_t)MOTOR_ID << 8U)
#define MOTOR_CAN_TX_BUFFER                 (0U)

typedef enum {
    MOTOR_STATE_UNKNOWN = 0,
    MOTOR_STATE_STOPPED,
    MOTOR_STATE_RUNNING
} MotorState;

typedef struct {
    int previous_position;
    int target_position;
    uint8_t valid;
    uint8_t target_hold;
} BallPdController;

typedef enum {
    TRAJECTORY_WAIT_AT_ORIGIN = 0,
    TRAJECTORY_MOVE_TO_PLUS,
    TRAJECTORY_MOVE_TO_MINUS,
    TRAJECTORY_HOLD_AT_MINUS
} TrajectoryState;

typedef struct {
    TrajectoryState state;
    uint8_t confirm_frames;
} TrajectoryController;


static MotorState g_motor_state = MOTOR_STATE_UNKNOWN;
static uint8_t g_last_direction = MOTOR_DIRECTION_RAISE;
static uint8_t g_can_ready = 0U;
static uint8_t g_motor_enabled = 0U;
static uint16_t g_motor_pulse_ms_left = 0U;
static uint16_t g_motor_pulse_gap_ms_left = 0U;
static uint16_t g_active_speed_rpm = 0U;
static uint8_t g_pending_command_valid = 0U;
static uint8_t g_pending_direction = MOTOR_DIRECTION_RAISE;
static uint16_t g_pending_speed_rpm = 0U;
static uint16_t g_pending_pulse_ms = 0U;
static BallPdController g_pd = {0};
static uint8_t g_button_was_pressed = 0U;
static uint16_t g_button_debounce_ms = 0U;
static TrajectoryController g_trajectory = {
    TRAJECTORY_WAIT_AT_ORIGIN, 0U
};

static int clamp_int(int value, int min_value, int max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static int abs_int(int value)
{
    return (value < 0) ? -value : value;
}

/* cx=-320 映射为 -12.50；cx=0 映射为 0；cx=319 映射为 +12.50。 */
static int ball_x_pixels_to_position(int cx_pixels)
{
    cx_pixels = clamp_int(cx_pixels, K230_LEFT_EDGE_OFFSET_PIXELS,
                          K230_RIGHT_EDGE_OFFSET_PIXELS);

    if (cx_pixels < K230_CENTER_X_OFFSET_PIXELS) {
        return (cx_pixels * BALL_POSITION_LIMIT) /
               (-K230_LEFT_EDGE_OFFSET_PIXELS);
    }

    return (cx_pixels * BALL_POSITION_LIMIT) /
           K230_RIGHT_EDGE_OFFSET_PIXELS;
}

static void ball_pd_reset(void)
{
    g_pd.previous_position = 0;
    g_pd.target_position = 0;
    g_pd.valid = 0U;
    g_pd.target_hold = 0U;
}

/*
 * 输出为带符号的 PD 命令：
 *   正数 => 小球在中心左侧（x<0）=> 抬高水管；
 *   负数 => 小球在中心右侧（x>0）=> 降低水管。
 */
static int ball_pd_update(int raw_position, int target_position,
                          uint16_t frame_interval_ms)
{
    int error;
    int velocity;
    int command;
    int32_t numerator;

    raw_position = clamp_int(raw_position, -BALL_POSITION_LIMIT,
                             BALL_POSITION_LIMIT);

    if ((g_pd.valid == 0U) ||
        (g_pd.target_position != target_position) ||
        (frame_interval_ms == 0U) ||
        (frame_interval_ms > PD_FRAME_INTERVAL_MAX_MS)) {
        velocity = 0;
    } else {
        velocity = ((raw_position - g_pd.previous_position) *
                    PD_VELOCITY_TIME_BASE_MS) /
                   (int)frame_interval_ms;
    }
    if (abs_int(velocity) <= PD_VELOCITY_DEADBAND) {
        velocity = 0;
    }
    g_pd.previous_position = raw_position;
    g_pd.target_position = target_position;
    g_pd.valid = 1U;

    error = target_position - raw_position;

    if (g_pd.target_hold != 0U) {
        if ((abs_int(error) <= BALL_RESTART_DEADBAND) &&
            (abs_int(velocity) <= BALL_VELOCITY_RESTART_DEADBAND)) {
            return 0;
        }
        g_pd.target_hold = 0U;
    }

    if ((abs_int(error) <= BALL_POSITION_DEADBAND) &&
        (abs_int(velocity) <= BALL_VELOCITY_HOLD_DEADBAND)) {
        g_pd.target_hold = 1U;
        return 0;
    }

    numerator = ((int32_t)PD_POSITION_KP_NUM * error) -
                ((int32_t)PD_VELOCITY_KD_NUM * velocity);
    command = (int)(numerator / PD_GAIN_DEN);
    command = clamp_int(command, -PD_OUTPUT_LIMIT, PD_OUTPUT_LIMIT);

    if (abs_int(command) < PD_OUTPUT_MIN) {
        command = 0;
    }
    return command;
}

/* PD 输出绝对值同时决定 CAN 速度和本次速度命令的持续时间。 */
static uint16_t calculate_speed_rpm(int effort)
{
    int speed_range = MOTOR_MAX_SPEED_RPM - MOTOR_MIN_SPEED_RPM;
    int full_speed;

    effort = clamp_int(effort, PD_OUTPUT_MIN, MOTOR_FULL_EFFORT);
    full_speed = MOTOR_MIN_SPEED_RPM +
        ((effort - PD_OUTPUT_MIN) * speed_range) /
        (MOTOR_FULL_EFFORT - PD_OUTPUT_MIN);
    return (uint16_t)clamp_int(full_speed, MOTOR_MIN_SPEED_RPM,
                               MOTOR_MAX_SPEED_RPM);
}

static uint16_t calculate_pulse_ms(int effort)
{
    int pulse_range = MOTOR_PULSE_MAX_MS - MOTOR_PULSE_MIN_MS;
    int full_pulse;

    effort = clamp_int(effort, PD_OUTPUT_MIN, MOTOR_FULL_EFFORT);
    full_pulse = MOTOR_PULSE_MIN_MS +
        ((effort - PD_OUTPUT_MIN) * pulse_range) /
        (MOTOR_FULL_EFFORT - PD_OUTPUT_MIN);
    return (uint16_t)clamp_int(full_pulse, MOTOR_PULSE_MIN_MS,
                               MOTOR_PULSE_MAX_MS);
}

/* 近似延时，仅用于 CAN 请求等待与主循环约 1 ms 节拍。 */
static void motor_delay_us(uint32_t microseconds)
{
    volatile uint32_t i;

    while (microseconds-- > 0U) {
        for (i = 0U; i < 8U; i++) {
            __asm("nop");
        }
    }
}

static uint8_t motor_can_send(const uint8_t *data, uint8_t length)
{
    DL_MCAN_TxBufElement tx_msg = {0};
    uint16_t elapsed_20us;

    if ((!g_can_ready) || (data == 0) || (length == 0U) || (length > 8U)) {
        return 0U;
    }

    if ((DL_MCAN_getTxBufReqPend(MCAN0_INST) &
         (1UL << MOTOR_CAN_TX_BUFFER)) != 0U) {
        (void)DL_MCAN_txBufCancellationReq(MCAN0_INST, MOTOR_CAN_TX_BUFFER);
        motor_delay_us(100U);
    }

    tx_msg.id = MOTOR_CAN_EXT_ID;
    tx_msg.rtr = 0U;
    tx_msg.xtd = 1U;
    tx_msg.esi = 0U;
    tx_msg.dlc = length;
    tx_msg.brs = 0U;
    tx_msg.fdf = 0U;
    tx_msg.efc = 0U;
    tx_msg.mm = 0U;

    for (uint8_t i = 0U; i < length; i++) {
        tx_msg.data[i] = data[i];
    }

    DL_MCAN_writeMsgRam(MCAN0_INST, DL_MCAN_MEM_TYPE_BUF,
                        MOTOR_CAN_TX_BUFFER, &tx_msg);
    DL_MCAN_TXBufAddReq(MCAN0_INST, MOTOR_CAN_TX_BUFFER);

    for (elapsed_20us = 0U;
         elapsed_20us < (uint16_t)(MOTOR_CAN_TX_TIMEOUT_MS * 50U);
         elapsed_20us++) {
        if ((DL_MCAN_getTxBufReqPend(MCAN0_INST) &
             (1UL << MOTOR_CAN_TX_BUFFER)) == 0U) {
            return 1U;
        }
        motor_delay_us(20U);
    }

    (void)DL_MCAN_txBufCancellationReq(MCAN0_INST, MOTOR_CAN_TX_BUFFER);
    return 0U;
}

/* DATA = F3 AB enable sync 6B */
static uint8_t motor_enable(uint8_t enable)
{
    const uint8_t data[5] = { 0xF3, 0xAB, enable ? 0x01 : 0x00, 0x00,
                              MOTOR_CHECKSUM };
    uint8_t success = motor_can_send(data, sizeof(data));

    if (success != 0U) {
        g_motor_enabled = enable ? 1U : 0U;
    }
    return success;
}

/* DATA = F6 direction speed(H,L) acceleration sync 6B */
static uint8_t motor_set_speed(uint8_t direction, uint16_t speed_rpm)
{
    uint8_t data[7];

    if ((g_motor_enabled == 0U) && (motor_enable(1U) == 0U)) {
        return 0U;
    }

    data[0] = 0xF6;
    data[1] = direction;
    data[2] = (uint8_t)(speed_rpm >> 8);
    data[3] = (uint8_t)(speed_rpm & 0xFFU);
    data[4] = MOTOR_ACCEL_LEVEL;
    data[5] = 0x00;
    data[6] = MOTOR_CHECKSUM;
    return motor_can_send(data, sizeof(data));
}

/* DATA = FE 98 sync 6B：立即停止。 */
static void motor_stop(void)
{
    const uint8_t data[4] = { 0xFE, 0x98, 0x00, MOTOR_CHECKSUM };

    if (g_motor_state != MOTOR_STATE_STOPPED) {
        (void)motor_can_send(data, sizeof(data));
    }
    g_motor_state = MOTOR_STATE_STOPPED;
    g_motor_pulse_ms_left = 0U;
    g_active_speed_rpm = 0U;
    g_pending_command_valid = 0U;
}

static void motor_cancel_pending_command(void)
{
    g_pending_command_valid = 0U;
}

static void motor_start_pending_command(void)
{
    if ((g_pending_command_valid == 0U) ||
        (g_motor_state == MOTOR_STATE_RUNNING) ||
        (g_motor_pulse_gap_ms_left > 0U)) {
        return;
    }

    if (motor_set_speed(g_pending_direction, g_pending_speed_rpm) != 0U) {
        g_motor_state = MOTOR_STATE_RUNNING;
        g_last_direction = g_pending_direction;
        g_active_speed_rpm = g_pending_speed_rpm;
        g_motor_pulse_ms_left = g_pending_pulse_ms;
    } else {
        g_motor_enabled = 0U;
        g_motor_state = MOTOR_STATE_STOPPED;
    }
    g_pending_command_valid = 0U;
}

static void motor_control_tick_1ms(void)
{
    if (g_motor_pulse_ms_left > 0U) {
        g_motor_pulse_ms_left--;
        if (g_motor_pulse_ms_left == 0U) {
            motor_stop();
            g_motor_pulse_gap_ms_left = MOTOR_PULSE_GAP_MS;
        }
    } else if (g_motor_pulse_gap_ms_left > 0U) {
        g_motor_pulse_gap_ms_left--;
    }

    motor_start_pending_command();
}

static int task_trajectory_target_position(void)
{
    if (g_trajectory.state == TRAJECTORY_MOVE_TO_PLUS) {
        return TRAJECTORY_PLUS_POSITION;
    }
    if ((g_trajectory.state == TRAJECTORY_MOVE_TO_MINUS) ||
        (g_trajectory.state == TRAJECTORY_HOLD_AT_MINUS)) {
        return TRAJECTORY_MINUS_POSITION;
    }
    return TRAJECTORY_ORIGIN_POSITION;
}

static void task_trajectory_update_on_measurement(void)
{
    uint8_t required_frames;

    if (g_trajectory.state == TRAJECTORY_HOLD_AT_MINUS) {
        return;
    }

    if (g_trajectory.state == TRAJECTORY_MOVE_TO_PLUS) {
        required_frames = 1U;
    } else if (g_trajectory.state == TRAJECTORY_MOVE_TO_MINUS) {
        required_frames = TRAJECTORY_FINAL_CONFIRM_FRAMES;
    } else {
        return;
    }
    if (g_pd.target_hold == 0U) {
        g_trajectory.confirm_frames = 0U;
        return;
    }

    if (g_trajectory.confirm_frames < required_frames) {
        g_trajectory.confirm_frames++;
    }
    if (g_trajectory.confirm_frames < required_frames) {
        return;
    }

    g_trajectory.confirm_frames = 0U;
    ball_pd_reset();
    if (g_trajectory.state == TRAJECTORY_MOVE_TO_PLUS) {
        g_trajectory.state = TRAJECTORY_MOVE_TO_MINUS;
    } else if (g_trajectory.state == TRAJECTORY_MOVE_TO_MINUS) {
        g_trajectory.state = TRAJECTORY_HOLD_AT_MINUS;
        DL_GPIO_clearPins(LED1_PORT, LED1_PIN_2_PIN);
    }
}

static uint8_t trajectory_start_button_pressed(void)
{
    uint8_t pressed = (DL_GPIO_readPins(KEY_PORT, KEY_PIN_0_PIN) == 0U) ?
        1U : 0U;

    if (g_button_debounce_ms > 0U) {
        g_button_debounce_ms--;
        return 0U;
    }

    if ((pressed != 0U) && (g_button_was_pressed == 0U)) {
        g_button_was_pressed = 1U;
        g_button_debounce_ms = TRAJECTORY_BUTTON_DEBOUNCE_MS;
        return 1U;
    }

    if (pressed == 0U) {
        g_button_was_pressed = 0U;
    }
    return 0U;
}

static uint8_t task_trajectory_is_active(void)
{
    return ((g_trajectory.state == TRAJECTORY_MOVE_TO_PLUS) ||
            (g_trajectory.state == TRAJECTORY_MOVE_TO_MINUS) ||
            (g_trajectory.state == TRAJECTORY_HOLD_AT_MINUS)) ? 1U : 0U;
}

static void task_trajectory_start(void)
{
    g_trajectory.state = TRAJECTORY_MOVE_TO_PLUS;
    g_trajectory.confirm_frames = 0U;
    ball_pd_reset();
    motor_cancel_pending_command();
    motor_stop();
    g_motor_pulse_gap_ms_left = 0U;
    DL_GPIO_setPins(LED1_PORT, LED1_PIN_2_PIN);
}

static void motor_can_init(void)
{
    uint16_t elapsed_ms;

    for (elapsed_ms = 0U; elapsed_ms < 50U; elapsed_ms++) {
        if (DL_MCAN_getOpMode(MCAN0_INST) == DL_MCAN_OPERATION_MODE_NORMAL) {
            g_can_ready = 1U;
            return;
        }
        motor_delay_us(1000U);
    }
}

static uint8_t motor_direction_from_pd_command(int command)
{
    return (command > 0) ? MOTOR_DIRECTION_RAISE : MOTOR_DIRECTION_LOWER;
}

/* 每个有效新帧调用一次。target_position 可为 O、+5 cm 或 -5 cm。 */
static void motor_track_ball(const BallInfo *ball, int target_position,
                             uint16_t frame_interval_ms)
{
    int position;
    int command;
    uint8_t direction;
    uint16_t pulse_ms;
    uint16_t speed_rpm;

    if ((ball == 0) || (ball->score < BALL_MIN_CONFIDENCE)) {
        ball_pd_reset();
        motor_cancel_pending_command();
        motor_stop();
        return;
    }

    position = ball_x_pixels_to_position(ball->cx);
    command = ball_pd_update(position, target_position, frame_interval_ms);
    if (command == 0) {
        motor_cancel_pending_command();
        motor_stop();
        return;
    }

    direction = motor_direction_from_pd_command(command);
    speed_rpm = calculate_speed_rpm(abs_int(command));
    pulse_ms = calculate_pulse_ms(abs_int(command));

    if (g_motor_state == MOTOR_STATE_RUNNING) {
        if (direction != g_last_direction) {
            /* 速度项要求反向制动时，先发 CAN 停止帧再反向。 */
            motor_stop();
            g_pending_command_valid = 1U;
            g_pending_direction = direction;
            g_pending_speed_rpm = speed_rpm;
            g_pending_pulse_ms = pulse_ms;
            g_motor_pulse_gap_ms_left = MOTOR_DIRECTION_SETTLE_MS;
        } else {
            if ((speed_rpm != g_active_speed_rpm) &&
                (motor_set_speed(direction, speed_rpm) == 0U)) {
                g_motor_enabled = 0U;
                motor_stop();
                return;
            }
            g_active_speed_rpm = speed_rpm;
            if (g_motor_pulse_ms_left > pulse_ms) {
                g_motor_pulse_ms_left = pulse_ms;
            }
        }
        return;
    }

    if (g_motor_pulse_gap_ms_left > 0U) {
        g_pending_command_valid = 1U;
        g_pending_direction = direction;
        g_pending_speed_rpm = speed_rpm;
        g_pending_pulse_ms = pulse_ms;
        return;
    }

    if (motor_set_speed(direction, speed_rpm) != 0U) {
        g_motor_state = MOTOR_STATE_RUNNING;
        g_last_direction = direction;
        g_active_speed_rpm = speed_rpm;
        g_motor_pulse_ms_left = pulse_ms;
    } else {
        g_motor_enabled = 0U;
        g_motor_state = MOTOR_STATE_STOPPED;
    }
}

/* 从 UART RX 中断写入的结果取一个完整快照。 */
static uint8_t take_latest_ball(BallInfo *ball)
{
    volatile BallDetectResult *result = Pto_Get_Ball_Result();
    uint8_t detected;

    if ((result == 0) || (result->new_data == 0U)) {
        return 0U;
    }

    detected = result->detected;
    if (detected != 0U) {
        ball->cx = result->ball.cx;
        ball->score = result->ball.score;
    }
    result->new_data = 0U;
    return (detected != 0U) ? 1U : 2U;
}

int main(void)
{
    BallInfo ball;
    uint8_t ball_state;
    uint16_t vision_silence_ms = 0U;
    uint16_t frame_interval_ms = 0U;

    SYSCFG_DL_init();
    uart2_init();
    motor_can_init();

    while (1) {
        if ((task_trajectory_is_active() == 0U) &&
            (trajectory_start_button_pressed() != 0U)) {
            task_trajectory_start();
        }

        ball_state = take_latest_ball(&ball);
        if (ball_state == 1U) {
            frame_interval_ms = vision_silence_ms;
            if (frame_interval_ms == 0U) {
                frame_interval_ms = 1U;
            }
            vision_silence_ms = 0U;
            motor_track_ball(&ball, task_trajectory_target_position(),
                             frame_interval_ms);
            if (task_trajectory_is_active() != 0U) {
                task_trajectory_update_on_measurement();
            }
        } else if (ball_state == 2U) {
            vision_silence_ms = 0U;
            ball_pd_reset();
            motor_stop();
        } else {
            if (vision_silence_ms < VISION_LOST_TIMEOUT_MS) {
                vision_silence_ms++;
            }
            if (vision_silence_ms >= VISION_LOST_TIMEOUT_MS) {
                ball_pd_reset();
                motor_stop();
            }
        }

        motor_control_tick_1ms();
        motor_delay_us(1000U);
    }
}







