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

/* 配置独立的 PID 参数、速度和纠偏限幅。 */
void Line_ConfigWithCorrectionLimits(float kp, float ki, float kd,
                                     int16_t base_speed,
                                     float correction_limit,
                                     float edge_min_correction);

/* 启动: 复位 PID, 使能电机 */
void Line_Start(uint32_t now_ms);

/* 每个控制周期更新: 传入灰度位图和当前 SysTick 毫秒数 */
void Line_Update(uint8_t gray_map, uint32_t now_ms);

/* 停止 */
void Line_Stop(void);

/* 运行时调整基础速度（不重置 PID） */
void Line_SetBaseSpeed(int16_t speed);

/* 运行时缩放循迹纠偏量，范围 0.0 至 1.0。 */
void Line_SetCorrectionScale(float scale);

/* 获取调试信息 */
float Line_GetPosition(uint8_t gray_map);
float Line_GetError(void);
float Line_GetCorrection(void);

#endif
