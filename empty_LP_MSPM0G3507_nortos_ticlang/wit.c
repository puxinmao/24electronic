/*
 * wit.c - JY901S 陀螺仪驱动 (UART1/PA9)
 *
 * 使用 UART RX + RX_TIMEOUT 中断接收 11 字节帧，
 * 无需 DMA，直接在中断中解析。
 */
#include "wit.h"

WIT_Data_t wit_data;

/* DMA 缓冲区（中断中接收 33 字节，最多 3 帧） */
static uint8_t  g_wit_buf[33];
static uint8_t  g_wit_cnt;
static bool     g_wit_updated;

/* ========== 内部解析函数 ========== */

static void wit_parse_packet(const uint8_t *pkt)
{
    uint8_t checksum = 0;
    for (int i = 0; i < 10; i++) checksum += pkt[i];
    if (checksum != pkt[10]) return;  /* 校验失败 */

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
        break;
    default:
        break;
    }
    g_wit_updated = true;
}

/* ========== 公开接口 ========== */

void WIT_Init(void)
{
    /* 开启 UART1 FIFO（当前 SysConfig 未配置，需要手动开） */
    DL_UART_enableFIFOs(UART_0_INST);

    /* 使能 RX 和 RX_TIMEOUT 中断 */
    DL_UART_enableInterrupt(UART_0_INST,
        DL_UART_INTERRUPT_RX | DL_UART_INTERRUPT_RX_TIMEOUT_ERROR);

    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);

    g_wit_cnt     = 0;
    g_wit_updated = false;
}

bool WIT_GetData(WIT_Data_t *dst)
{
    if (g_wit_updated) {
        *dst = wit_data;
        g_wit_updated = false;
        return true;
    }
    return false;
}

/* ========== UART1 中断处理 ========== */

void UART1_IRQHandler(void)
{
    /* 读取 FIFO 中所有可用字节 */
    while (!DL_UART_isRXFIFOEmpty(UART_0_INST)) {
        uint8_t byte = DL_UART_receiveData(UART_0_INST);

        /* 帧同步：找 0x55 帧头 */
        if (byte == 0x55) {
            g_wit_cnt = 0;
            g_wit_buf[g_wit_cnt++] = byte;
        } else if (g_wit_cnt > 0 && g_wit_cnt < 11) {
            g_wit_buf[g_wit_cnt++] = byte;
            if (g_wit_cnt == 11) {
                /* 完整帧，解析 */
                wit_parse_packet(g_wit_buf);
                g_wit_cnt = 0;  /* 准备下一帧 */
            }
        } else {
            g_wit_cnt = 0;  /* 失步，丢弃 */
        }
    }
}
