#ifndef _YB_PROTOCOL_H_
#define _YB_PROTOCOL_H_

#include "stdint.h"

#define PTO_BUF_LEN_MAX    (32)
#define PTO_HEAD           (0x24)

typedef struct {
    int cx;
    int score;
} BallInfo;

typedef struct {
    volatile uint8_t detected;
    volatile uint8_t new_data;
    volatile BallInfo ball;
} BallDetectResult;

void Pto_Data_Receive(uint8_t data);
volatile BallDetectResult *Pto_Get_Ball_Result(void);

#endif
