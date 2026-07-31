#include "yb_protocol.h"

#include "stdlib.h"
#include "string.h"

#define K230_IMAGE_WIDTH       (640)
#define K230_IMAGE_HEIGHT      (360)
#define K230_IMAGE_CENTER_X    (K230_IMAGE_WIDTH / 2)

static uint8_t g_rx_buffer[PTO_BUF_LEN_MAX];
static uint8_t g_rx_index = 0U;
static uint8_t g_rx_receiving = 0U;
static volatile BallDetectResult g_ball_result = {0};

static void protocol_clear_receive(void)
{
    g_rx_index = 0U;
    g_rx_receiving = 0U;
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

static void protocol_parse_ball(uint8_t *data, uint8_t length)
{
    char *cursor;
    long x;
    long y;
    long score;

    if ((data == 0) || (length < 11U) ||
        (strncmp((char *)data, "$BALL,", 6U) != 0)) {
        return;
    }

    cursor = (char *)data + 6;
    if ((parse_number(&cursor, &x, ',') == 0U) ||
        (parse_number(&cursor, &y, ',') == 0U) ||
        (parse_number(&cursor, &score, '\0') == 0U)) {
        return;
    }

    if ((x == -1L) && (y == -1L)) {
        g_ball_result.detected = 0U;
        g_ball_result.new_data = 1U;
        return;
    }

    if ((x < 0L) || (x >= K230_IMAGE_WIDTH) ||
        (y < 0L) || (y >= K230_IMAGE_HEIGHT) ||
        (score < 0L) || (score > 100L)) {
        return;
    }

    g_ball_result.ball.cx = (int)(x - K230_IMAGE_CENTER_X);
    g_ball_result.ball.score = (int)score;
    g_ball_result.detected = 1U;
    g_ball_result.new_data = 1U;
}

void Pto_Data_Receive(uint8_t data)
{
    if (data == PTO_HEAD) {
        g_rx_buffer[0] = PTO_HEAD;
        g_rx_index = 1U;
        g_rx_receiving = 1U;
        return;
    }

    if (g_rx_receiving == 0U) {
        return;
    }
    if (data == '\r') {
        return;
    }
    if (data == '\n') {
        g_rx_buffer[g_rx_index] = '\0';
        protocol_parse_ball(g_rx_buffer, g_rx_index);
        protocol_clear_receive();
        return;
    }
    if (g_rx_index >= (PTO_BUF_LEN_MAX - 1U)) {
        protocol_clear_receive();
        return;
    }

    g_rx_buffer[g_rx_index++] = data;
}

volatile BallDetectResult *Pto_Get_Ball_Result(void)
{
    return &g_ball_result;
}
