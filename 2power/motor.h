/*
 * motor.h - 电机驱动模块 (TB6612FNG + TIMA1 PWM)
 */
#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

/* PWM 最大占空比（由 SysConfig: period=2000） */
#define MOTOR_PWM_MAX  2000

/* 初始化电机 GPIO 和 PWM，默认停止+STBY=低 */
void Motor_Init(void);

/* 设置左电机转速: [-MOTOR_PWM_MAX, MOTOR_PWM_MAX]，正=正转，负=反转 */
void Motor_SetLeft(int16_t speed);

/* 设置右电机转速: [-MOTOR_PWM_MAX, MOTOR_PWM_MAX] */
void Motor_SetRight(int16_t speed);

/* 同时设置两电机 */
void Motor_SetBoth(int16_t left, int16_t right);

/* 刹车（制动） */
void Motor_Brake(void);

/* 滑行停止（待机） */
void Motor_Standby(void);

/* 使能驱动（解除待机） */
void Motor_Enable(void);

#endif /* MOTOR_H */
