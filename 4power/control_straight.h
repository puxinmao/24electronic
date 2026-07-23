/*
 * control_straight.h - 偏航角直行与原地转向模块
 *
 * 锁定当前偏航角为目标，Yaw PID 控制两轮差速保持方向。
 */
#ifndef CONTROL_STRAIGHT_H
#define CONTROL_STRAIGHT_H

#include <stdint.h>
#include <stdbool.h>

/* 配置 PID 参数和基础速度 */
void Straight_Config(float kp, float ki, float kd, int16_t base_speed);

/* 启动: 锁定 yaw 为目标；now_ms 使用系统 SysTick 毫秒计时。 */
void Straight_Start(float current_yaw, uint32_t now_ms);

/* 收到新 yaw 时更新；now_ms 用于计算真实 PID 间隔和启动渐变。 */
float Straight_Update(float current_yaw, uint32_t now_ms);

/* 停止: 刹车 + 待机 */
void Straight_Stop(void);

/* 获取当前目标偏航角 */
float Straight_GetTarget(void);

/* 是否正在运行 */
bool Straight_IsRunning(void);

typedef enum {
    TURN_IN_PLACE_RUNNING,
    TURN_IN_PLACE_BRAKING,
    TURN_IN_PLACE_DONE,
    TURN_IN_PLACE_TIMEOUT,
    TURN_IN_PLACE_STALLED
} TurnInPlaceResult_t;

/* 配置备用原地右转；帧数参数按陀螺仪姿态帧计数。 */
void TurnInPlace_Config(float angle_deg, int16_t fast_pwm, int16_t slow_pwm,
                        float slow_angle_deg, uint8_t brake_frames,
                        uint16_t timeout_frames, uint8_t stall_frames,
                        float stall_min_deg);
void TurnInPlace_Start(float current_yaw);
TurnInPlaceResult_t TurnInPlace_Update(float current_yaw);
void TurnInPlace_Stop(void);
float TurnInPlace_GetTarget(void);
float TurnInPlace_GetProgress(void);
bool TurnInPlace_IsRunning(void);
#endif
