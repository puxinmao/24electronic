/*
 * encoder.h - 四轮霍尔编码器模块 (AB 相 4 倍频解码)
 */
#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

typedef enum {
    ENCODER_A_LEFT_FRONT = 0,
    ENCODER_B_LEFT_REAR,
    ENCODER_C_RIGHT_REAR,
    ENCODER_D_RIGHT_FRONT,
    ENCODER_COUNT
} EncoderWheel_t;

/* 初始化 GPIOA/GPIOB 上的八路编码器边沿中断。 */
void Encoder_Init(void);

/* 原子读取 A/B/C/D 四个车轮的累计 4 倍频脉冲数。 */
void Encoder_GetCounts(int32_t counts[ENCODER_COUNT]);

/* 清零四轮累计计数。 */
void Encoder_Reset(void);

#endif /* ENCODER_H */
