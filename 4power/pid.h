/*
 * pid.h - PID 控制器模块
 */
#ifndef PID_H
#define PID_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    float Kp;           /* 比例系数 */
    float Ki;           /* 积分系数 */
    float Kd;           /* 微分系数 */
    float setpoint;     /* 目标值 */
    float integral;     /* 积分累计 */
    float prev_error;   /* 上一次误差 */
    float out_min;      /* 输出下限 */
    float out_max;      /* 输出上限 */
    float integral_limit; /* 积分限幅 */
} PID_t;

/* 初始化 PID 控制器 */
void PID_Init(PID_t *pid, float Kp, float Ki, float Kd,
              float out_min, float out_max, float integral_limit);

/* 设置目标值 */
void PID_SetSetpoint(PID_t *pid, float setpoint);

/* 计算 PID 输出 (dt = 调用间隔，单位秒) */
float PID_Compute(PID_t *pid, float measurement, float dt);

/* 用预算好的误差计算 PID 输出（适用于需要归一化误差的场合，如偏航角） */
float PID_ComputeError(PID_t *pid, float error, float dt);

/* 复位 PID（清除积分和上次误差） */
void PID_Reset(PID_t *pid);

#endif /* PID_H */
