#include "ti_msp_dl_config.h"
#include "stdio.h"
#include "stdint.h"
#include "usart.h"
#include "yb_protocol.h"

/*
 * K230 发送的钢球中心坐标约定：屏幕中心 = (0, 0)。
 * 当前实测 X 坐标约为 [-160, 160]；方向控制结果限制为 -1、0、1。
 * 为保留灵敏度，速度和脉冲长度仍按原始像素偏差连续计算。
 */
#define K230_X_HALF_RANGE           (160)
#define K230_X_DEADBAND_PIXELS      (12)    /* 中心 ±12 像素内不动作，抑制检测抖动 */
#define BALL_X_MIN                 (-1)
#define BALL_X_MAX                 (1)
#define BALL_X_DEADBAND            (0)     /* -1~1 控制范围中，只有 x=0 才停止 */

/*
 * 张大头 Emm_V5 / ZDT_X 系列通讯参数。
 * 电机屏幕/上位机中应确认：
 *   P_Serial = CAN1_MAP，ID_Addr = MOTOR_ID，Checksum = 0x6B。
 */
#define MOTOR_ID                   (1)
#define MOTOR_CHECKSUM             (0x6B)
#define MOTOR_DIR_CW               (0x00)
#define MOTOR_DIR_CCW              (0x01)
/*
 * 微调参数：F6 是持续转速命令，因此必须配合短脉冲和 FE 停止。
 * 兼顾灵敏度与稳定性：远离中心时速度/脉冲稍大，靠近中心时自动缩短脉冲。
 * 脉冲必须长于闭环驱动器的起步/加速时间；20~45 ms 过短时，电机可能来不及产生可见位移。
 * 若仍有过冲，优先减小 MOTOR_PULSE_MAX_MS；若响应仍慢，再增加 MOTOR_PULSE_MIN_MS。
 */
#define MOTOR_ACCEL_LEVEL          (5)     /* 温和加速，避免短脉冲尚未起转便被停止 */
#define MOTOR_MIN_SPEED_RPM        (3)
#define MOTOR_MAX_SPEED_RPM        (10)
#define MOTOR_SPEED_HYSTERESIS_RPM (1)
#define MOTOR_PULSE_MIN_MS         (90)    /* 靠近中心时也保证产生可见但有限的微调 */
#define MOTOR_PULSE_MAX_MS         (140)   /* 偏离最大时的单次调整上限，仍不是连续转动 */
#define MOTOR_PULSE_GAP_MS         (50)    /* 停下后等待画面稳定，再执行下一次微调 */
#define MOTOR_CAN_TX_TIMEOUT_MS    (10)    /* 等待 CAN 硬件完成发送的最长时间 */
#define VISION_LOST_TIMEOUT_MS     (1500)  /* K230 帧较稀疏：1.5 s 无视觉数据才停止 */

/*
 * 原机械方向沿用旧程序：球在左侧时电机 CW，球在右侧时电机 CCW。
 * 若实际调整方向相反，请将该宏改为 1，无须改控制算法。
 */
#define MOTOR_REVERSE_DIRECTION    (0)    /* 机械实际方向与原设定相反，交换 CW/CCW */

/* CAN1_MAP：扩展帧 ID = 电机地址 << 8。ID=1 时为 0x00000100。 */
#define MOTOR_CAN_EXT_ID            ((uint32_t)MOTOR_ID << 8U)
#define MOTOR_CAN_TX_BUFFER          (0U)

typedef enum {
    MOTOR_STATE_UNKNOWN = 0,
    MOTOR_STATE_STOPPED,
    MOTOR_STATE_RUNNING
} MotorState;

static MotorState g_motor_state = MOTOR_STATE_UNKNOWN;
static uint8_t    g_last_direction = MOTOR_DIR_CW;
static uint16_t   g_last_speed_rpm = 0;
static uint8_t    g_can_ready = 0;
static uint16_t   g_motor_pulse_ms_left = 0;
static uint16_t   g_motor_pulse_gap_ms_left = 0;

/* 将 K230 的横坐标限制为方向控制值：左=-1，中心=0，右=1。 */
static int map_k230_x_to_control(int raw_x)
{
    if (raw_x < -K230_X_DEADBAND_PIXELS) {
        return BALL_X_MIN;
    }
    if (raw_x > K230_X_DEADBAND_PIXELS) {
        return BALL_X_MAX;
    }
    return 0;
}
/* 返回与 |x| 成比例的速度：偏差越大，修正越快。 */
static uint16_t calculate_speed_rpm(int abs_raw_x)
{
    int active_range = K230_X_HALF_RANGE - K230_X_DEADBAND_PIXELS;
    int speed_range = MOTOR_MAX_SPEED_RPM - MOTOR_MIN_SPEED_RPM;
    int clamped_raw_x = abs_raw_x;
    int speed;

    if (clamped_raw_x > K230_X_HALF_RANGE) {
        clamped_raw_x = K230_X_HALF_RANGE;
    }
    if (active_range <= 0) {
        return MOTOR_MIN_SPEED_RPM;
    }

    speed = MOTOR_MIN_SPEED_RPM
            + ((clamped_raw_x - K230_X_DEADBAND_PIXELS) * speed_range) / active_range;

    if (speed < MOTOR_MIN_SPEED_RPM) {
        speed = MOTOR_MIN_SPEED_RPM;
    }
    if (speed > MOTOR_MAX_SPEED_RPM) {
        speed = MOTOR_MAX_SPEED_RPM;
    }
    return (uint16_t)speed;
}

/* 偏差越小时脉冲越短，避免小球刚接近中心又被推到另一侧。 */
static uint16_t calculate_pulse_ms(int abs_raw_x)
{
    int active_range = K230_X_HALF_RANGE - K230_X_DEADBAND_PIXELS;
    int pulse_range = MOTOR_PULSE_MAX_MS - MOTOR_PULSE_MIN_MS;
    int clamped_raw_x = abs_raw_x;
    int pulse_ms;

    if (clamped_raw_x > K230_X_HALF_RANGE) {
        clamped_raw_x = K230_X_HALF_RANGE;
    }
    if (active_range <= 0) {
        return MOTOR_PULSE_MIN_MS;
    }

    pulse_ms = MOTOR_PULSE_MIN_MS
             + ((clamped_raw_x - K230_X_DEADBAND_PIXELS) * pulse_range) / active_range;

    if (pulse_ms < MOTOR_PULSE_MIN_MS) {
        pulse_ms = MOTOR_PULSE_MIN_MS;
    }
    if (pulse_ms > MOTOR_PULSE_MAX_MS) {
        pulse_ms = MOTOR_PULSE_MAX_MS;
    }
    return (uint16_t)pulse_ms;
}

/*
 * 直接 CAN 控制（不再使用 PB2/PB3 的 UART3）。
 * CAN1_MAP 协议要求：500 kbps、经典 CAN、扩展帧、CAN ID = 电机 ID << 8。
 * 串口协议中的第一个“地址”字节不放入 CAN 数据区，例如：
 * Emm_V5 UART: 01 F6 direction speed(H,L) acceleration sync 6B
 * Emm_V5 CAN : ID=0x100, DLC=7, DATA=F6 direction speed(H,L) acceleration sync 6B
 */
static void motor_delay_us(uint32_t microseconds)
{
    volatile uint32_t i;

    /* 近似延时，仅用于视觉看门狗和 CAN 发送完成等待。 */
    while (microseconds-- > 0U) {
        for (i = 0; i < 8U; i++) {
            __asm("nop");
        }
    }
}

static void motor_debug_can_frame(const uint8_t *data, uint8_t length)
{
    char debug_buf[80];
    uint8_t i;
    int offset;

    offset = sprintf(debug_buf, "Motor CAN TX: ID=%08lX DLC=%u DATA=",
                     (unsigned long)MOTOR_CAN_EXT_ID, (unsigned int)length);
    for (i = 0; (i < length) && (offset < (int)(sizeof(debug_buf) - 5U)); i++) {
        offset += sprintf(&debug_buf[offset], "%02X ", data[i]);
    }
    sprintf(&debug_buf[offset], "\r\n");
    uart0_send_string(debug_buf);
}

/*
 * 发送一帧经典 CAN 扩展帧，并以“TX 请求是否清除”判断总线是否真正完成发送。
 * 若 SN65HVD230 断线、CANH/CANL 颠倒、RS 处于待机，或电机波特率不为 500k，
 * TX 请求会持续挂起；此时取消请求，避免一个失败帧堵住下一帧。
 */
static uint8_t motor_can_send(const uint8_t *data, uint8_t length)
{
    DL_MCAN_TxBufElement tx_msg = {0};
    DL_MCAN_ProtocolStatus protocol_status;
    uint16_t elapsed_20us;
    char debug_buf[88];

    if ((!g_can_ready) || (data == 0) || (length == 0U) || (length > 8U)) {
        return 0;
    }

    /* 上一次请求若仍在等待 ACK，先取消，确保 TX buffer 0 可复用。 */
    if ((DL_MCAN_getTxBufReqPend(MCAN0_INST) & (1UL << MOTOR_CAN_TX_BUFFER)) != 0U) {
        (void)DL_MCAN_txBufCancellationReq(MCAN0_INST, MOTOR_CAN_TX_BUFFER);
        motor_delay_us(100);
    }

    tx_msg.id  = MOTOR_CAN_EXT_ID;
    tx_msg.rtr = 0U;               /* data frame */
    tx_msg.xtd = 1U;               /* 29-bit extended CAN identifier */
    tx_msg.esi = 0U;
    tx_msg.dlc = length;
    tx_msg.brs = 0U;               /* classic CAN: no bitrate switching */
    tx_msg.fdf = 0U;               /* classic CAN, not CAN-FD */
    tx_msg.efc = 0U;
    tx_msg.mm  = 0U;

    for (uint8_t i = 0U; i < length; i++) {
        tx_msg.data[i] = data[i];
    }

    motor_debug_can_frame(data, length);
    DL_MCAN_writeMsgRam(MCAN0_INST, DL_MCAN_MEM_TYPE_BUF,
                        MOTOR_CAN_TX_BUFFER, &tx_msg);
    DL_MCAN_TXBufAddReq(MCAN0_INST, MOTOR_CAN_TX_BUFFER);

    for (elapsed_20us = 0U;
         elapsed_20us < (uint16_t)(MOTOR_CAN_TX_TIMEOUT_MS * 50U);
         elapsed_20us++) {
        if ((DL_MCAN_getTxBufReqPend(MCAN0_INST) &
             (1UL << MOTOR_CAN_TX_BUFFER)) == 0U) {
            uart0_send_string("Motor CAN TX OK (physical ACK)\r\n");
            return 1;
        }
        motor_delay_us(20);
    }

    DL_MCAN_getProtocolStatus(MCAN0_INST, &protocol_status);
    sprintf(debug_buf,
            "Motor CAN TX timeout: pending=%08lX lec=%lu passive=%lu warn=%lu busoff=%lu\r\n",
            (unsigned long)DL_MCAN_getTxBufReqPend(MCAN0_INST),
            (unsigned long)protocol_status.lastErrCode,
            (unsigned long)protocol_status.errPassive,
            (unsigned long)protocol_status.warningStatus,
            (unsigned long)protocol_status.busOffStatus);
    uart0_send_string(debug_buf);

    (void)DL_MCAN_txBufCancellationReq(MCAN0_INST, MOTOR_CAN_TX_BUFFER);
    return 0;
}

/* DATA = F3 AB enable sync 6B */
static uint8_t motor_enable(uint8_t enable)
{
    const uint8_t data[5] = {
        0xF3, 0xAB, enable ? 0x01 : 0x00, 0x00, MOTOR_CHECKSUM
    };

    return motor_can_send(data, sizeof(data));
}

/*
 * Emm_V5: DATA = F6 direction speed(H,L) acceleration sync 6B。
 * 注意：CAN 数据区只有 7 字节；速度单位就是 RPM，不能乘以 10。
 * 旧代码按 ZDT_X V2 格式发送了 8 字节，电机会把字段错位解析而不执行。
 */
static uint8_t motor_set_speed(uint8_t direction, uint16_t speed_rpm)
{
    uint8_t data[7];

    data[0] = 0xF6;
    data[1] = direction;
    data[2] = (uint8_t)(speed_rpm >> 8);
    data[3] = (uint8_t)(speed_rpm & 0xFF);
    data[4] = MOTOR_ACCEL_LEVEL;
    data[5] = 0x00;                /* 不使用多机同步 */
    data[6] = MOTOR_CHECKSUM;

    /* 每次新的速度请求先发送 F3 使能，确保 Hold 模式下电机已被逻辑使能。 */
    if (!motor_enable(1)) {
        return 0;
    }
    return motor_can_send(data, sizeof(data));
}

/* DATA = FE 98 sync 6B：立即停止 */
static void motor_stop(void)
{
    const uint8_t data[4] = { 0xFE, 0x98, 0x00, MOTOR_CHECKSUM };

    if (g_motor_state == MOTOR_STATE_STOPPED) {
        return;
    }

    (void)motor_can_send(data, sizeof(data));
    g_motor_state = MOTOR_STATE_STOPPED;
    g_last_speed_rpm = 0;
    g_motor_pulse_ms_left = 0U;
}

/*
 * F6 为持续转动命令。本控制器只让它保持当前计算出的短脉冲，随后发 FE 停止，
 * 并留出一段时间等待 K230 画面更新，防止一次命令把钢球推过中心。
 * 本函数在主循环中每约 1 ms 调用一次。
 */
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
}

/* 确认 MCAN 已进入正常模式；未就绪时只打印一次，电机不会误动作。 */
static void motor_can_init(void)
{
    uint16_t elapsed_ms;

    for (elapsed_ms = 0U; elapsed_ms < 50U; elapsed_ms++) {
        if (DL_MCAN_getOpMode(MCAN0_INST) == DL_MCAN_OPERATION_MODE_NORMAL) {
            g_can_ready = 1U;
            uart0_send_string("CAN ready: 500 kbps, XTD ID=0x00000100, PA12/PA13\r\n");
            return;
        }
        motor_delay_us(1000);
    }

    uart0_send_string("CAN init failed: MCAN is not in NORMAL mode\r\n");
}

/* 根据当前钢球 x 坐标更新电机状态。 */
static void motor_track_ball_x(int raw_x)
{
    char debug_buf[80];
    int x = map_k230_x_to_control(raw_x);
    int abs_raw_x = (raw_x < 0) ? -raw_x : raw_x;
    uint8_t direction;
    uint16_t speed_rpm;
    uint16_t pulse_ms;

    if (x == 0) {
        motor_stop();
        return;
    }

    /* x < 0：球在左；x > 0：球在右。 */
    direction = (x < 0) ? MOTOR_DIR_CW : MOTOR_DIR_CCW;
    if (MOTOR_REVERSE_DIRECTION) {
        direction = (direction == MOTOR_DIR_CW) ? MOTOR_DIR_CCW : MOTOR_DIR_CW;
    }
    speed_rpm = calculate_speed_rpm(abs_raw_x);
    pulse_ms = calculate_pulse_ms(abs_raw_x);

    /*
     * F6 会持续转动，不能随着每一帧 K230 数据重复下发。
     * 当前有一段微调正在执行时，保持该脉冲；脉冲结束后的等待期内也不再启动，
     * 直到相机已经看到这次调整的结果。
     */
    if (g_motor_state == MOTOR_STATE_RUNNING) {
        /* 目标方向若反转，先立即停住；下一次画面再从相反方向微调。 */
        if (direction != g_last_direction) {
            motor_stop();
            g_motor_pulse_gap_ms_left = MOTOR_PULSE_GAP_MS;
        }
        return;
    }
    if (g_motor_pulse_gap_ms_left > 0U) {
        return;
    }

    if (motor_set_speed(direction, speed_rpm)) {
        g_motor_state = MOTOR_STATE_RUNNING;
        g_last_direction = direction;
        g_last_speed_rpm = speed_rpm;
        g_motor_pulse_ms_left = pulse_ms;

        sprintf(debug_buf, "Motor micro-adjust: %s, speed=%u RPM, pulse=%u ms, raw_x=%d, x=%d\r\n",
                (direction == MOTOR_DIR_CW) ? "CW" : "CCW", speed_rpm,
                (unsigned int)pulse_ms, raw_x, x);
        uart0_send_string(debug_buf);
    } else {
        uart0_send_string("Motor command not accepted on CAN bus\r\n");
    }
}

int main(void)
{
    BallDetectResult *result;
    uint16_t vision_silence_ms = 0;

    SYSCFG_DL_init();
    uart0_init();
    uart1_init();                  /* 保留 UART3(PB2/PB3)，但本版本不再用于电机 */
    uart2_init();                  /* UART1: PA8/PA9 <- K230 */

    uart0_send_string("MSPM0G3507 Ball Detect + ZDT Direct CAN Motor\r\n");
    uart0_send_string("K230 raw x: -160..160 -> control: -1/0/1\r\n");
    uart0_send_string("Motor CAN: PA12=TX, PA13=RX, 500 kbps\r\n");
    motor_can_init();

    while (1)
    {
        result = Pto_Get_Ball_Result();

        if (result->new_data)
        {
            result->new_data = 0;
            vision_silence_ms = 0;

            if (result->count > 0)
            {
                /* 当前沿用第一个检测到的钢球作为跟踪目标。 */
                motor_track_ball_x(result->balls[0].cx);
            }
            else
            {
                /* K230 明确报告没有球时，立即停止。 */
                motor_stop();
            }
        }
        else
        {
            /*
             * F6 是持续转速命令。如果 K230 断线或不再发送任何帧，旧速度不会自动失效。
             * 因此超过 200 ms 没有新的视觉帧时，强制发送 FE 停止命令，避免无球空转。
             */
            if (vision_silence_ms < VISION_LOST_TIMEOUT_MS) {
                vision_silence_ms++;
            }
            if ((vision_silence_ms >= VISION_LOST_TIMEOUT_MS) &&
                (g_motor_state == MOTOR_STATE_RUNNING)) {
                uart0_send_string("Vision timeout: motor stop\r\n");
                motor_stop();
            }
        }

        /* 以约 1 ms 为单位更新电机短脉冲和视觉数据看门狗。 */
        motor_control_tick_1ms();
        motor_delay_us(1000);
    }
}





