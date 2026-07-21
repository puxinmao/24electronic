/*
 * control_straight.h - 偏航角直行模块
 *
 * 锁定当前偏航角为目标，Yaw PID 控制两轮差速保持方向。
 */
#ifndef CONTROL_STRAIGHT_H
#define CONTROL_STRAIGHT_H

#include <stdint.h>
#include <stdbool.h>

/* 配置 PID 参数和基础速度 */
void Straight_Config(float kp, float ki, float kd, int16_t base_speed);

/* 启动: 锁定 yaw 为目标, 使能电机 */
void Straight_Start(float current_yaw);

/* 每帧更新: 传入当前 yaw，输出电机控制。返回偏航误差 */
float Straight_Update(float current_yaw);

/* 停止: 刹车 + 待机 */
void Straight_Stop(void);

/* 获取当前目标偏航角 */
float Straight_GetTarget(void);

/* 是否正在运行 */
bool Straight_IsRunning(void);

#endif
