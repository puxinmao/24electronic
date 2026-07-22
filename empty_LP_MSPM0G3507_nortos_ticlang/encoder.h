/*
 * encoder.h - 霍尔编码器模块 (4x 解码)
 *
 * 引脚: LA=PB7, LB=PB6 (左电机编码器)
 *       RA=PB0, RB=PB16 (右电机编码器)
 * 中断: GROUP1_IRQHandler (GPIOB, 双边沿触发)
 */
#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

/* 初始化编码器中断 */
void Encoder_Init(void);

/* 获取累计位置 (脉冲数) */
int32_t Encoder_GetLeft(void);
int32_t Encoder_GetRight(void);

/* 原子读取左右轮累计位置，供双轮速度采样使用。 */
void Encoder_GetCounts(int32_t *left, int32_t *right);

/* 获取速度 (脉冲/s)，dt 为距离上次调用的秒数 */
int16_t Encoder_GetLeftSpeed(float dt);
int16_t Encoder_GetRightSpeed(float dt);

/* 清零计数 */
void Encoder_Reset(void);

#endif /* ENCODER_H */
