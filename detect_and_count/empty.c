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
 * 电机只支持连续转速命令，因此 PID 输出被转换为“低速、短脉冲”动作。
 * 每一帧新的视觉坐标都会更新一次 PID；一次脉冲不能被新帧延长，只能缩短或刹停，
 * 以使单次机械位移尽可能小。
 */
#define K230_CENTER_X_OFFSET_PIXELS      (0)
#define K230_LEFT_EDGE_OFFSET_PIXELS   (-320)
#define K230_RIGHT_EDGE_OFFSET_PIXELS    (319)
#define BALL_POSITION_SCALE              (100)   /* 100 = 1.00 位置单位 */
#define BALL_POSITION_LIMIT             (1250)  /* 12.50 * 100 */
#define BALL_POSITION_DEADBAND            (6)  /* Stop inside +/-0.10 cm of the center. */
#define BALL_RESTART_DEADBAND             (10)  /* Restart beyond +/-0.16 cm. */
#define BALL_TARGET_SLOW_ZONE            (200)  /* Slow down within 2.00 cm of balance */
#define BALL_MIN_CONFIDENCE               (35)  /* K230 score: 0..100 */

/* 自动轨迹：O(0 cm) -> +5 cm -> -5 cm，最终稳定在 -5 cm 附近。 */
#define TRAJECTORY_ORIGIN_POSITION          (0)
#define TRAJECTORY_PLUS_POSITION          (500)  /* +5.00 cm */
#define TRAJECTORY_START_TOLERANCE         (30)  /* Start only from O +/- 0.30 cm. */
#define TRAJECTORY_TARGET_TOLERANCE        (50)  /* Stop target within +/- 0.50 cm. */
#define TRAJECTORY_APPROACH_ZONE          (120)  /* Brake during final 1.20 cm. */
#define TRAJECTORY_APPROACH_PULSE_MS       (20)
#define TRAJECTORY_START_CONFIRM_FRAMES      (3)
#define TRAJECTORY_FINAL_CONFIRM_FRAMES      (5)
#define TRAJECTORY_TIMEOUT_MS             (4800)
/*
 * 参考轨迹采用“限速 + 跟随窗口”而非直接跳变：
 * - 每个新视觉帧设定值最多前进 0.12 cm；
 * - 设定值最多只允许领先实测小球 0.45 cm。
 *
 * 即使相机连续给出相同位置，设定值也会停下来等小球跟上，因而不会在
 * O -> +5 cm 起步阶段积累一个很大的 PID 误差并把小球直接冲到端部。
 */

/*
 * 离散 PID（每个有效 K230 帧执行一次；适用于稳定、较高的视觉帧率）。
 * P、I、D 均为定点分数，避免在 Cortex-M0+ 上使用软件浮点。
 * 初始值偏保守：先保证不明显过冲，再按下方说明逐步调大 Kp / Kd。
 */
#define PID_FILTER_DIVISOR                (1)   /* Use the current vision frame immediately. */
#define PID_KP_NUM                         (10) /* Kp = 0.95 */
#define PID_KI_NUM                          (0) /* Enable only after PD is stable. */
#define PID_KD_NUM                        (400) /* Kd = 2.40 */
#define PID_DERIVATIVE_DEADBAND             (8) /* Ignore <= 0.08 cm/frame vision jitter. */
#define PID_GAIN_DEN                      (100)
#define PID_INTEGRAL_LIMIT              (3000) /* Limits I contribution to 0.90 command. */
#define PID_OUTPUT_LIMIT                (1250)
#define PID_OUTPUT_MIN                    (30)  /* Start correcting from 0.30 command. */

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
#define MOTOR_ACCEL_LEVEL                  (3)
#define MOTOR_MIN_SPEED_RPM                (6)
#define MOTOR_CRUISE_SPEED_RPM             (8)
#define MOTOR_MAX_SPEED_RPM               (10)
#define MOTOR_PULSE_MIN_MS                (25)
#define MOTOR_PULSE_MAX_MS                (45)
#define MOTOR_PULSE_GAP_MS                 (3)
#define MOTOR_DIRECTION_SETTLE_MS          (5)  /* Stop briefly before a reversal. */
#define MOTOR_CAN_TX_TIMEOUT_MS           (10)
#define VISION_LOST_TIMEOUT_MS           (300)

#define MOTOR_CAN_EXT_ID          ((uint32_t)MOTOR_ID << 8U)
#define MOTOR_CAN_TX_BUFFER                 (0U)
#define MOTOR_DEBUG                          (0)

typedef enum {
    MOTOR_STATE_UNKNOWN = 0,
    MOTOR_STATE_STOPPED,
    MOTOR_STATE_RUNNING
} MotorState;

typedef struct {
    int filtered_position;
    int previous_error;
    int integral;
    uint8_t valid;
    uint8_t center_hold;
} BallPidController;

typedef enum {
    TRAJECTORY_WAIT_AT_ORIGIN = 0,
    TRAJECTORY_MOVE_TO_PLUS,
    TRAJECTORY_MOVE_TO_ORIGIN,
    TRAJECTORY_HOLD_AT_ORIGIN,
    TRAJECTORY_DONE,
    TRAJECTORY_TIMEOUT
} TrajectoryState;

typedef struct {
    TrajectoryState state;
    uint16_t elapsed_ms;
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
static BallPidController g_pid = {0};
static TrajectoryController g_trajectory = { TRAJECTORY_WAIT_AT_ORIGIN, 0U, 0U };

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

static void ball_pid_reset(void)
{
    g_pid.filtered_position = 0;
    g_pid.previous_error = 0;
    g_pid.integral = 0;
    g_pid.valid = 0U;
    g_pid.center_hold = 0U;
}

/*
 * 输出为带符号的 PID 命令：
 *   正数 => 小球在中心左侧（x<0）=> 抬高水管；
 *   负数 => 小球在中心右侧（x>0）=> 降低水管。
 */
static int ball_pid_update(int raw_position, int target_position)
{
    int error;
    int derivative;
    int next_integral;
    int command;
    int32_t numerator;

    raw_position = clamp_int(raw_position, -BALL_POSITION_LIMIT,
                             BALL_POSITION_LIMIT);

    if (g_pid.valid == 0U) {
        g_pid.filtered_position = raw_position;
        g_pid.previous_error = target_position - raw_position;
        g_pid.integral = 0;
        g_pid.valid = 1U;
    } else {
        g_pid.filtered_position +=
            (raw_position - g_pid.filtered_position) / PID_FILTER_DIVISOR;
    }

    error = target_position - g_pid.filtered_position;

    if (g_pid.center_hold != 0U) {
        if (abs_int(error) <= BALL_RESTART_DEADBAND) {
            g_pid.integral = 0;
            g_pid.previous_error = error;
            return 0;
        }
        g_pid.center_hold = 0U;
    }

    if (abs_int(error) <= BALL_POSITION_DEADBAND) {
        g_pid.center_hold = 1U;
        g_pid.integral = 0;
        g_pid.previous_error = error;
        return 0;
    }

    derivative = error - g_pid.previous_error;
    if (abs_int(derivative) <= PID_DERIVATIVE_DEADBAND) {
        derivative = 0;
    }
    next_integral = clamp_int(g_pid.integral + error,
                              -PID_INTEGRAL_LIMIT, PID_INTEGRAL_LIMIT);
    numerator = ((int32_t)PID_KP_NUM * error) +
                ((int32_t)PID_KI_NUM * next_integral) +
                ((int32_t)PID_KD_NUM * derivative);
    command = (int)(numerator / PID_GAIN_DEN);
    command = clamp_int(command, -PID_OUTPUT_LIMIT, PID_OUTPUT_LIMIT);
    if (!((command >= PID_OUTPUT_LIMIT) && (error > 0)) &&
        !((command <= -PID_OUTPUT_LIMIT) && (error < 0))) {
        g_pid.integral = next_integral;
    }
    g_pid.previous_error = error;

    if (abs_int(command) < PID_OUTPUT_MIN) {
        command = 0;
    }
    return command;
}

/*
 * 减速带相对“当前目标点”计算，而不是相对 O 点计算。
 * 这样 O -> +5 cm 和 +5 -> -5 cm 的中段始终采用巡航能力，只有接近每个目标时
 * 才逐步减速，避免之前在 O 点附近被过早限速。
 */
static uint16_t calculate_speed_rpm(int effort, int abs_error)
{
    int speed_range = MOTOR_MAX_SPEED_RPM - MOTOR_MIN_SPEED_RPM;
    int full_speed;

    (void)abs_error;

    effort = clamp_int(effort, PID_OUTPUT_MIN, PID_OUTPUT_LIMIT);
    full_speed = MOTOR_MIN_SPEED_RPM +
        ((effort - PID_OUTPUT_MIN) * speed_range) /
        (PID_OUTPUT_LIMIT - PID_OUTPUT_MIN);
    return (uint16_t)clamp_int(full_speed, MOTOR_MIN_SPEED_RPM,
                               MOTOR_MAX_SPEED_RPM);
}

static uint16_t calculate_pulse_ms(int effort, int abs_error)
{
    int pulse_range = MOTOR_PULSE_MAX_MS - MOTOR_PULSE_MIN_MS;
    int full_pulse;

    (void)abs_error;

    effort = clamp_int(effort, PID_OUTPUT_MIN, PID_OUTPUT_LIMIT);
    full_pulse = MOTOR_PULSE_MIN_MS +
        ((effort - PID_OUTPUT_MIN) * pulse_range) /
        (PID_OUTPUT_LIMIT - PID_OUTPUT_MIN);
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

#if MOTOR_DEBUG
static void motor_debug_can_frame(const uint8_t *data, uint8_t length)
{
    char debug_buf[80];
    uint8_t i;
    int offset;

    offset = sprintf(debug_buf, "Motor CAN TX: ID=%08lX DLC=%u DATA=",
                     (unsigned long)MOTOR_CAN_EXT_ID, (unsigned int)length);
    for (i = 0U; (i < length) && (offset < (int)(sizeof(debug_buf) - 5U)); i++) {
        offset += sprintf(&debug_buf[offset], "%02X ", data[i]);
    }
    sprintf(&debug_buf[offset], "\r\n");
    uart0_send_string(debug_buf);
}
#endif

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

#if MOTOR_DEBUG
    motor_debug_can_frame(data, length);
#endif
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
    if ((g_trajectory.state == TRAJECTORY_MOVE_TO_ORIGIN) ||
        (g_trajectory.state == TRAJECTORY_HOLD_AT_ORIGIN) ||
        (g_trajectory.state == TRAJECTORY_DONE)) {
        return TRAJECTORY_ORIGIN_POSITION;
    }
    return TRAJECTORY_ORIGIN_POSITION;
}

static void task_trajectory_update_on_measurement(int position)
{
    int target = task_trajectory_target_position();
    uint8_t required_frames = TRAJECTORY_FINAL_CONFIRM_FRAMES;

    if ((g_trajectory.state == TRAJECTORY_DONE) ||
        (g_trajectory.state == TRAJECTORY_TIMEOUT)) {
        return;
    }

    if (g_trajectory.state == TRAJECTORY_WAIT_AT_ORIGIN) {
        target = TRAJECTORY_ORIGIN_POSITION;
        required_frames = TRAJECTORY_START_CONFIRM_FRAMES;
    }

    if (abs_int(position - target) >
        ((g_trajectory.state == TRAJECTORY_WAIT_AT_ORIGIN) ?
         TRAJECTORY_START_TOLERANCE : TRAJECTORY_TARGET_TOLERANCE)) {
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
    ball_pid_reset();
    if (g_trajectory.state == TRAJECTORY_WAIT_AT_ORIGIN) {
        g_trajectory.state = TRAJECTORY_MOVE_TO_PLUS;
        g_trajectory.elapsed_ms = 0U;
    } else if (g_trajectory.state == TRAJECTORY_MOVE_TO_PLUS) {
        g_trajectory.state = TRAJECTORY_MOVE_TO_ORIGIN;
    } else if (g_trajectory.state == TRAJECTORY_MOVE_TO_ORIGIN) {
        g_trajectory.state = TRAJECTORY_HOLD_AT_ORIGIN;
    } else if (g_trajectory.state == TRAJECTORY_HOLD_AT_ORIGIN) {
        g_trajectory.state = TRAJECTORY_DONE;
    }
}

static void task_trajectory_tick_1ms(void)
{
    if ((g_trajectory.state != TRAJECTORY_MOVE_TO_PLUS) &&
        (g_trajectory.state != TRAJECTORY_MOVE_TO_ORIGIN) &&
        (g_trajectory.state != TRAJECTORY_HOLD_AT_ORIGIN)) {
        return;
    }

    if (g_trajectory.elapsed_ms < TRAJECTORY_TIMEOUT_MS) {
        g_trajectory.elapsed_ms++;
    }
    if (g_trajectory.elapsed_ms >= TRAJECTORY_TIMEOUT_MS) {
        g_trajectory.state = TRAJECTORY_TIMEOUT;
        ball_pid_reset();
        motor_cancel_pending_command();
        motor_stop();
    }
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

static uint8_t motor_direction_from_pid_command(int command)
{
    return (command > 0) ? MOTOR_DIRECTION_RAISE : MOTOR_DIRECTION_LOWER;
}

static int trajectory_goal_position(void)
{
#if TRAJECTORY_TEST_ENABLE
    switch (g_trajectory.state) {
        case TRAJECTORY_MOVE_TO_PLUS:
            return TRAJECTORY_PLUS_TARGET;
        case TRAJECTORY_MOVE_TO_MINUS:
        case TRAJECTORY_HOLD_AT_MINUS:
        case TRAJECTORY_DONE:
            return TRAJECTORY_MINUS_TARGET;
        case TRAJECTORY_WAIT_AT_ORIGIN:
        case TRAJECTORY_TIMEOUT:
        default:
            return TRAJECTORY_ORIGIN_POSITION;
    }
#else
    return TRAJECTORY_ORIGIN_POSITION;
#endif
}

/*
 * 目标斜坡发生在每一帧新视觉数据到达时，并加入“参考值不能领先小球太多”的
 * 跟随窗口。这样轨迹起步为小步试探：先给一小段设定值，确认小球开始响应后才
 * 继续前进；小球未跟上时立即冻结设定值等待反馈。
 */
static void trajectory_advance_reference(int measured_position)
{
#if TRAJECTORY_TEST_ENABLE
    int goal = trajectory_goal_position();
    int difference = goal - g_trajectory.reference_position;
    int next_reference;
    int reference_limit;

    if (difference == 0) {
        return;
    }

    if (difference > 0) {
        /* 向 + 方向时，PID 参考值不能超过“小球实测位置 + 0.45 cm”。 */
        reference_limit = measured_position + TRAJECTORY_REFERENCE_FOLLOW_WINDOW;
        next_reference = g_trajectory.reference_position +
            ((difference > TRAJECTORY_REFERENCE_STEP_PER_FRAME) ?
             TRAJECTORY_REFERENCE_STEP_PER_FRAME : difference);
        if (next_reference > reference_limit) {
            next_reference = reference_limit;
        }
        if (next_reference > g_trajectory.reference_position) {
            g_trajectory.reference_position = next_reference;
        }
    } else {
        /* 向 - 方向时，PID 参考值不能低于“小球实测位置 - 0.45 cm”。 */
        reference_limit = measured_position - TRAJECTORY_REFERENCE_FOLLOW_WINDOW;
        next_reference = g_trajectory.reference_position -
            (((-difference) > TRAJECTORY_REFERENCE_STEP_PER_FRAME) ?
             TRAJECTORY_REFERENCE_STEP_PER_FRAME : (-difference));
        if (next_reference < reference_limit) {
            next_reference = reference_limit;
        }
        if (next_reference < g_trajectory.reference_position) {
            g_trajectory.reference_position = next_reference;
        }
    }
#endif
}

static int trajectory_control_target_position(void)
{
#if TRAJECTORY_TEST_ENABLE
    return g_trajectory.reference_position;
#else
    return TRAJECTORY_ORIGIN_POSITION;
#endif
}

/* 每个有效视觉帧更新一次轨迹状态；连续帧确认抑制单帧误检触发折返。 */
static void trajectory_update_on_measurement(int position)
{
#if TRAJECTORY_TEST_ENABLE
    int target = trajectory_goal_position();
    int tolerance;
    uint8_t confirm_frames;

    if ((g_trajectory.state == TRAJECTORY_DONE) ||
        (g_trajectory.state == TRAJECTORY_TIMEOUT)) {
        return;
    }

    if (g_trajectory.state == TRAJECTORY_WAIT_AT_ORIGIN) {
        tolerance = TRAJECTORY_START_TOLERANCE;
        confirm_frames = TRAJECTORY_START_CONFIRM_FRAMES;
    } else {
        tolerance = TRAJECTORY_TARGET_TOLERANCE;
        confirm_frames = (g_trajectory.state == TRAJECTORY_HOLD_AT_MINUS) ?
            TRAJECTORY_FINAL_STABLE_FRAMES : TRAJECTORY_TURN_CONFIRM_FRAMES;
    }

    if (abs_int(position - target) > tolerance) {
        g_trajectory.consecutive_in_target_frames = 0U;
        return;
    }

    if (g_trajectory.consecutive_in_target_frames < confirm_frames) {
        g_trajectory.consecutive_in_target_frames++;
    }
    if (g_trajectory.consecutive_in_target_frames < confirm_frames) {
        return;
    }

    g_trajectory.consecutive_in_target_frames = 0U;
    ball_pid_reset();
    switch (g_trajectory.state) {
        case TRAJECTORY_WAIT_AT_ORIGIN:
            /*
             * 已在 O 点稳定，先以当前实测位置作为参考轨迹起点，再缓慢向 +5 cm
             * 前进；避免 O 点存在少量偏差时仍产生一次突发阶跃。
             */
            g_trajectory.reference_position = position;
            g_trajectory.state = TRAJECTORY_MOVE_TO_PLUS;
            g_trajectory.elapsed_ms = 0U;
            break;
        case TRAJECTORY_MOVE_TO_PLUS:
            /* 到达 +5 cm（误差 <= 1 cm）后立即折返。 */
            g_trajectory.state = TRAJECTORY_MOVE_TO_MINUS;
            break;
        case TRAJECTORY_MOVE_TO_MINUS:
            /* 首次进入 -5 cm 容差带，继续保持以验证稳定性。 */
            g_trajectory.state = TRAJECTORY_HOLD_AT_MINUS;
            break;
        case TRAJECTORY_HOLD_AT_MINUS:
            /* 连续稳定帧满足后，测试通过，仍维持 -5 cm 目标。 */
            g_trajectory.state = TRAJECTORY_DONE;
            break;
        default:
            break;
    }
#else
    (void)position;
#endif
}

/* 从开始向 +5 cm 运动起计时；超过 5 s 未完成时安全停止。 */
static void trajectory_tick_1ms(void)
{
#if TRAJECTORY_TEST_ENABLE
    if (trajectory_is_active() == 0U) {
        return;
    }

    if (g_trajectory.elapsed_ms < TRAJECTORY_MAX_TIME_MS) {
        g_trajectory.elapsed_ms++;
    }
    if (g_trajectory.elapsed_ms >= TRAJECTORY_MAX_TIME_MS) {
        g_trajectory.state = TRAJECTORY_TIMEOUT;
        g_trajectory.consecutive_in_target_frames = 0U;
        ball_pid_reset();
        motor_stop();
    }
#endif
}

/* 每个有效新帧调用一次。target_position 可为 O、+5 cm 或 -5 cm。 */
static void motor_track_ball(const BallInfo *ball, int target_position)
{
    int position;
    int command;
    int abs_error;
    uint8_t direction;
    uint16_t pulse_ms;
    uint16_t speed_rpm;

    if ((ball == 0) || (ball->score < BALL_MIN_CONFIDENCE)) {
        ball_pid_reset();
        motor_cancel_pending_command();
        motor_stop();
        return;
    }

    position = ball_x_pixels_to_position(ball->cx);
    command = ball_pid_update(position, target_position);
    if (command == 0) {
        motor_cancel_pending_command();
        motor_stop();
        return;
    }

    direction = motor_direction_from_pid_command(command);
    abs_error = abs_int(position - target_position);
    pulse_ms = calculate_pulse_ms(abs_int(command), abs_error);
    if (abs_error <= TRAJECTORY_APPROACH_ZONE) {
        pulse_ms = (pulse_ms > TRAJECTORY_APPROACH_PULSE_MS) ?
            TRAJECTORY_APPROACH_PULSE_MS : pulse_ms;
    }

    if (g_motor_state == MOTOR_STATE_RUNNING) {
        if (direction != g_last_direction) {
            /* 方向反转时先刹停；防止水管惯性和 PID 积分共同放大振荡。 */
            motor_stop();
            g_pending_command_valid = 1U;
            g_pending_direction = direction;
            g_pending_speed_rpm = calculate_speed_rpm(abs_int(command), abs_error);
            g_pending_pulse_ms = pulse_ms;
            g_motor_pulse_gap_ms_left = MOTOR_DIRECTION_SETTLE_MS;
        } else if (g_motor_pulse_ms_left > pulse_ms) {
            /* 新帧要求更小修正时，仅缩短当前脉冲，绝不延长它。 */
            g_motor_pulse_ms_left = pulse_ms;
        }
        return;
    }

    if (g_motor_pulse_gap_ms_left > 0U) {
        g_pending_command_valid = 1U;
        g_pending_direction = direction;
        g_pending_speed_rpm = calculate_speed_rpm(abs_int(command), abs_error);
        g_pending_pulse_ms = pulse_ms;
        return;
    }

    speed_rpm = calculate_speed_rpm(abs_int(command), abs_error);
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
    uint8_t count;

    if ((result == 0) || (result->new_data == 0U)) {
        return 0U;
    }

    count = result->count;
    if (count > 0U) {
        ball->cx = result->balls[0].cx;
        ball->cy = result->balls[0].cy;
        ball->w = result->balls[0].w;
        ball->score = result->balls[0].score;
    }
    result->new_data = 0U;
    return (count > 0U) ? 1U : 2U;
}

int main(void)
#if 0
·{
#endif
{
    BallInfo ball;
    uint8_t ball_state;
    uint16_t vision_silence_ms = 0U;

    SYSCFG_DL_init();
    uart0_init();
    uart1_init();
    uart2_init();
    motor_can_init();

    while (1) {
        ball_state = take_latest_ball(&ball);
        if (ball_state == 1U) {
            vision_silence_ms = 0U;
            motor_track_ball(&ball, TRAJECTORY_ORIGIN_POSITION);
        } else if (ball_state == 2U) {
            vision_silence_ms = 0U;
            ball_pid_reset();
            motor_stop();
        } else {
            if (vision_silence_ms < VISION_LOST_TIMEOUT_MS) {
                vision_silence_ms++;
            }
            if (vision_silence_ms >= VISION_LOST_TIMEOUT_MS) {
                ball_pid_reset();
                motor_stop();
            }
        }

        motor_control_tick_1ms();
        motor_delay_us(1000U);
    }
}







