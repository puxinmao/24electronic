/*
 * control_line.h - 灰度循迹模块
 *
 * 读取灰度传感器位置，Line PID 控制两轮差速跟踪黑线。
 */
#ifndef CONTROL_LINE_H
#define CONTROL_LINE_H

#include <stdint.h>
#include <stdbool.h>

/* 配置 PID 参数和基础速度 */
void Line_Config(float kp, float ki, float kd, int16_t base_speed);

/* 启动: 复位 PID, 使能电机 */
void Line_Start(void);

/* 每帧更新: 传入灰度位图，输出电机控制 */
void Line_Update(uint8_t gray_map);

/* 停止 */
void Line_Stop(void);

/* 运行时调整基础速度（不重置 PID） */
void Line_SetBaseSpeed(int16_t speed);

/* 获取调试信息 */
float Line_GetPosition(uint8_t gray_map);
float Line_GetError(void);
float Line_GetCorrection(void);

#endif
