#include "yb_protocol.h"

#include "stdlib.h"
#include "string.h"
#include "usart.h"

/*
 * K230 UART 协议（115200, 8N1）：
 *   $BALL,x,y,score\r\n  检测到小球，x: 0..639，y: 0..359，score: 0..100
 *   $BALL,-1,-1,0\r\n   未检测到小球
 *
 * 中断内只做有限长度的接收/解析和共享结果更新；绝不调用阻塞式 UART 打印，
 * 否则打印一行所需的时间会使后续 K230 字节丢失并破坏控制实时性。
 */
#define K230_IMAGE_WIDTH           (640)
#define K230_IMAGE_HEIGHT          (360)
#define K230_IMAGE_CENTER_X        (K230_IMAGE_WIDTH / 2)

static uint8_t RxBuffer[PTO_BUF_LEN_MAX];
static uint8_t RxIndex = 0U;
static uint8_t RxReceiving = 0U;
static volatile BallDetectResult g_ball_result = {0};

volatile BallDetectResult *Pto_Get_Ball_Result(void)
{
    return &g_ball_result;
}

void Pto_Clear_CMD_Flag(void)
{
    RxIndex = 0U;
    RxReceiving = 0U;
    memset(RxBuffer, 0, sizeof(RxBuffer));
}

static uint8_t parse_number(char **cursor, long *value, char delimiter)
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

/* UART RX 中断逐字节调用；只接受 '$' 开始且 LF 结束的一行。 */
void Pto_Data_Receive(uint8_t Rx_Temp)
{
    if (Rx_Temp == PTO_HEAD) {
        RxBuffer[0] = PTO_HEAD;
        RxIndex = 1U;
        RxReceiving = 1U;
        return;
    }

    if (RxReceiving == 0U) {
        return;
    }

    if (Rx_Temp == '\r') {
        return;
    }

    if (Rx_Temp == '\n') {
        RxBuffer[RxIndex] = '\0';
        Pto_Data_Parse(RxBuffer, RxIndex);
        RxIndex = 0U;
        RxReceiving = 0U;
        return;
    }

    if (RxIndex >= (PTO_BUF_LEN_MAX - 1U)) {
        Pto_Clear_CMD_Flag();
        return;
    }

    RxBuffer[RxIndex] = Rx_Temp;
    RxIndex++;
}

void Pto_Data_Parse(uint8_t *data_buf, uint8_t num)
{
    char *cursor;
    long x;
    long y;
    long score;

    if ((data_buf == 0) || (num < 11U) ||
        (strncmp((char *)data_buf, "$BALL,", 6U) != 0)) {
        return;
    }

    cursor = (char *)data_buf + 6;
    if ((parse_number(&cursor, &x, ',') == 0U) ||
        (parse_number(&cursor, &y, ',') == 0U) ||
        (parse_number(&cursor, &score, '\0') == 0U)) {
        return;
    }

    if ((x == -1L) && (y == -1L)) {
        g_ball_result.count = 0U;
        g_ball_result.new_data = 1U;
        return;
    }

    /* 拒绝异常值，避免损坏帧导致电机朝一侧持续运动。 */
    if ((x < 0L) || (x >= K230_IMAGE_WIDTH) ||
        (y < 0L) || (y >= K230_IMAGE_HEIGHT) ||
        (score < 0L) || (score > 100L)) {
        return;
    }

    /* 必须最后写 new_data，主循环据此只读取完整测量。 */
    g_ball_result.balls[0].cx = (int)(x - K230_IMAGE_CENTER_X);
    g_ball_result.balls[0].cy = (int)y;
    g_ball_result.balls[0].w = 0;
    g_ball_result.balls[0].score = (int)score;
    g_ball_result.count = 1U;
    g_ball_result.new_data = 1U;
}

void Pto_Loop(void)
{
    /* 协议已在 UART RX 中断中逐帧处理。 */
}
