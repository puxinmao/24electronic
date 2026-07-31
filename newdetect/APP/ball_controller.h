#ifndef BALL_CONTROLLER_H_
#define BALL_CONTROLLER_H_

#include <stdint.h>

#include "ball_protocol.h"
#include "zdt_motor.h"

typedef enum {
    BALL_CONTROL_STOP = 0,
    BALL_CONTROL_MOVE
} BallControlAction;

typedef struct {
    BallControlAction action;
    ZdtPipeDirection direction;
    uint16_t speed_rpm;
    int16_t position_centi_cm;
} BallControlCommand;

void BallController_Init(void);
void BallController_Reset(void);
BallControlCommand BallController_Update(const BallSample *sample,
                                         uint32_t now_ms);
int16_t BallController_MapPixels(int16_t cx_pixels);

#endif
