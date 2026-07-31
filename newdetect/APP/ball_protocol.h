#ifndef BALL_PROTOCOL_H_
#define BALL_PROTOCOL_H_

#include <stdint.h>

typedef struct {
    int16_t cx_pixels;
    uint8_t score;
} BallSample;

typedef enum {
    BALL_FRAME_NONE = 0,
    BALL_FRAME_VALID,
    BALL_FRAME_NOT_DETECTED
} BallFrameStatus;

void BallProtocol_ReceiveByte(uint8_t byte);
BallFrameStatus BallProtocol_TakeLatest(BallSample *sample);

#endif
