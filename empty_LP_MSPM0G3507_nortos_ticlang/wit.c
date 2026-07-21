/*
 * wit.c - JY901S 陀螺仪驱动 (UART1/PA9)
 *
 * 使用 UART RX + RX_TIMEOUT 中断接收 11 字节帧，
 * 无需 DMA，直接在中断中解析。
 */
#include "wit.h"

WIT_Data_t wit_data;

/* JY901S 固定 11 字节帧 */
static uint8_t  g_wit_buf[11];
static uint8_t  g_wit_cnt;
static volatile bool g_wit_updated;
static volatile bool g_wit_fault;

#define WIT_IRQ_MASK (DL_UART_INTERRUPT_RX | DL_UART_INTERRUPT_RX_TIMEOUT_ERROR)
#define WIT_ERROR_MASK (DL_UART_INTERRUPT_OVERRUN_ERROR | \
                        DL_UART_INTERRUPT_PARITY_ERROR | \
                        DL_UART_INTERRUPT_FRAMING_ERROR | \
                        DL_UART_INTERRUPT_RX_TIMEOUT_ERROR | \
                        DL_UART_INTERRUPT_NOISE_ERROR)
#define WIT_ISR_BYTE_BUDGET 32

/* ========== 内部解析函数 ========== */

static bool wit_parse_packet(const uint8_t *pkt)
{
    uint8_t checksum = 0;
    for (int i = 0; i < 10; i++) checksum += pkt[i];
    if (checksum != pkt[10]) return false;

    switch (pkt[1]) {
    case 0x51:  /* 加速度 + 温度 */
        wit_data.ax = (int16_t)((pkt[3] << 8) | pkt[2]);
        wit_data.ay = (int16_t)((pkt[5] << 8) | pkt[4]);
        wit_data.az = (int16_t)((pkt[7] << 8) | pkt[6]);
        wit_data.temperature = (int16_t)((pkt[9] << 8) | pkt[8]) / 100.0f;
        break;
    case 0x52:  /* 角速度 */
        wit_data.gx = (int16_t)((pkt[3] << 8) | pkt[2]);
        wit_data.gy = (int16_t)((pkt[5] << 8) | pkt[4]);
        wit_data.gz = (int16_t)((pkt[7] << 8) | pkt[6]);
        break;
    case 0x53:  /* 姿态角 + 版本 */
        wit_data.roll  = (int16_t)((pkt[3] << 8) | pkt[2]) / 32768.0f * 180.0f;
        wit_data.pitch = (int16_t)((pkt[5] << 8) | pkt[4]) / 32768.0f * 180.0f;
        wit_data.yaw   = (int16_t)((pkt[7] << 8) | pkt[6]) / 32768.0f * 180.0f;
        wit_data.version = (int16_t)((pkt[9] << 8) | pkt[8]);
        g_wit_updated = true;
        break;
    default:
        break;
    }

    return true;
}

static void wit_resync_packet(void)
{
    uint8_t start = 1;

    while (start < sizeof(g_wit_buf) && g_wit_buf[start] != 0x55U) {
        start++;
    }

    if (start == sizeof(g_wit_buf)) {
        g_wit_cnt = 0;
        return;
    }

    g_wit_cnt = (uint8_t)(sizeof(g_wit_buf) - start);
    for (uint8_t i = 0; i < g_wit_cnt; i++) {
        g_wit_buf[i] = g_wit_buf[start + i];
    }
}

/* ========== 公开接口 ========== */

void WIT_Init(void)
{
    /* 开启 UART1 FIFO（当前 SysConfig 未配置，需要手动开） */
    DL_UART_enableFIFOs(UART_0_INST);

    /* 使能 RX 和 RX_TIMEOUT 中断 */
    DL_UART_clearInterruptStatus(UART_0_INST, WIT_IRQ_MASK | WIT_ERROR_MASK);
    DL_UART_enableInterrupt(UART_0_INST, WIT_IRQ_MASK);

    NVIC_SetPriority(UART_0_INST_INT_IRQN, 2);
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);

    g_wit_cnt     = 0;
    g_wit_updated = false;
    g_wit_fault   = false;
}

bool WIT_GetData(WIT_Data_t *dst)
{
    bool updated;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    updated = g_wit_updated;
    if (updated) {
        *dst = wit_data;
        g_wit_updated = false;
    }

    if (primask == 0U) __enable_irq();
    return updated;
}

bool WIT_HasFault(void)
{
    return g_wit_fault;
}

void WIT_Recover(void)
{
    uint8_t flush_count = 0;

    NVIC_DisableIRQ(UART_0_INST_INT_IRQN);
    DL_UART_disableInterrupt(UART_0_INST, WIT_IRQ_MASK);

    while (!DL_UART_isRXFIFOEmpty(UART_0_INST) && flush_count < 32) {
        (void)DL_UART_receiveData(UART_0_INST);
        flush_count++;
    }

    g_wit_cnt     = 0;
    g_wit_updated = false;
    g_wit_fault   = false;
    DL_UART_clearInterruptStatus(UART_0_INST, WIT_IRQ_MASK | WIT_ERROR_MASK);
    NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
    DL_UART_enableInterrupt(UART_0_INST, WIT_IRQ_MASK);
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
}

/* ========== UART1 中断处理 ========== */

void UART1_IRQHandler(void)
{
    uint8_t budget = WIT_ISR_BYTE_BUDGET;
    uint32_t errors = DL_UART_getRawInterruptStatus(UART_0_INST, WIT_ERROR_MASK);

    /* 错误位必须显式清除，否则噪声可能造成中断反复进入。 */
    if (errors != 0U) {
        DL_UART_clearInterruptStatus(UART_0_INST, errors);
        if (errors & (DL_UART_INTERRUPT_OVERRUN_ERROR |
                      DL_UART_INTERRUPT_PARITY_ERROR |
                      DL_UART_INTERRUPT_FRAMING_ERROR |
                      DL_UART_INTERRUPT_NOISE_ERROR)) {
            g_wit_cnt = 0;
        }
    }

    while (!DL_UART_isRXFIFOEmpty(UART_0_INST) && budget-- > 0U) {
        uint8_t byte = DL_UART_receiveData(UART_0_INST);

        /* 只在等待帧头时识别 0x55；帧负载中的 0x55 是合法数据。 */
        if (g_wit_cnt == 0U) {
            if (byte == 0x55U) g_wit_buf[g_wit_cnt++] = byte;
        } else {
            g_wit_buf[g_wit_cnt++] = byte;
            if (g_wit_cnt == sizeof(g_wit_buf)) {
                if (wit_parse_packet(g_wit_buf)) g_wit_cnt = 0;
                else wit_resync_packet();
            }
        }
    }

    DL_UART_clearInterruptStatus(UART_0_INST, WIT_IRQ_MASK | WIT_ERROR_MASK);

    /* 正常 FIFO 不会超过预算；持续有数据说明线路受到严重干扰。 */
    if (!DL_UART_isRXFIFOEmpty(UART_0_INST)) {
        DL_UART_disableInterrupt(UART_0_INST, WIT_IRQ_MASK);
        g_wit_fault = true;
    }
}
