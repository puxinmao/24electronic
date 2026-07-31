#include "ball_protocol.h"

#include <stdlib.h>
#include <string.h>

#include "control_config.h"
#include "ti_msp_dl_config.h"

#define BALL_LINE_BUFFER_SIZE (40U)

typedef struct {
    volatile uint8_t pending;
    volatile uint8_t detected;
    volatile int16_t cx_pixels;
    volatile uint8_t score;
} SharedBallResult;

static char g_line_buffer[BALL_LINE_BUFFER_SIZE];
static uint8_t g_line_length;
static uint8_t g_receiving;
static SharedBallResult g_result;

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

static void parse_ball_line(void)
{
    char *cursor;
    long x;
    long y;
    long score;

    if ((g_line_length < 11U) ||
        (strncmp(g_line_buffer, "$BALL,", 6U) != 0)) {
        return;
    }

    cursor = &g_line_buffer[6];
    if ((parse_field(&cursor, &x, ',') == 0U) ||
        (parse_field(&cursor, &y, ',') == 0U) ||
        (parse_field(&cursor, &score, '\0') == 0U)) {
        return;
    }

    if ((x == -1L) && (y == -1L)) {
        g_result.detected = 0U;
        g_result.pending = 1U;
        return;
    }
    if ((x < 0L) || (x >= K230_IMAGE_WIDTH) ||
        (y < 0L) || (y >= K230_IMAGE_HEIGHT) ||
        (score < 0L) || (score > 100L)) {
        return;
    }

    g_result.cx_pixels = (int16_t)(x - (K230_IMAGE_WIDTH / 2));
    g_result.score = (uint8_t)score;
    g_result.detected = 1U;
    g_result.pending = 1U;
}

void BallProtocol_ReceiveByte(uint8_t byte)
{
    if (byte == '$') {
        g_line_buffer[0] = '$';
        g_line_length = 1U;
        g_receiving = 1U;
        return;
    }
    if (g_receiving == 0U) {
        return;
    }
    if (byte == '\r') {
        return;
    }
    if (byte == '\n') {
        g_line_buffer[g_line_length] = '\0';
        parse_ball_line();
        g_line_length = 0U;
        g_receiving = 0U;
        return;
    }
    if (g_line_length >= (BALL_LINE_BUFFER_SIZE - 1U)) {
        g_line_length = 0U;
        g_receiving = 0U;
        return;
    }
    g_line_buffer[g_line_length++] = (char)byte;
}

BallFrameStatus BallProtocol_TakeLatest(BallSample *sample)
{
    BallFrameStatus status = BALL_FRAME_NONE;
    uint32_t interrupt_state;

    if (sample == 0) {
        return BALL_FRAME_NONE;
    }

    interrupt_state = __get_PRIMASK();
    __disable_irq();
    if (g_result.pending != 0U) {
        if (g_result.detected != 0U) {
            sample->cx_pixels = g_result.cx_pixels;
            sample->score = g_result.score;
            status = BALL_FRAME_VALID;
        } else {
            status = BALL_FRAME_NOT_DETECTED;
        }
        g_result.pending = 0U;
    }
    if (interrupt_state == 0U) {
        __enable_irq();
    }
    return status;
}
