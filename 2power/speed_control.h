/*
 * speed_control.h - 双轮编码器速度闭环模块
 *
 * 上层继续传入原有的反向 PWM 指令，模块将其换算为目标编码器速度，
 * 再由左右轮独立 PI 修正最终 PWM。
 */
#ifndef SPEED_CONTROL_H
#define SPEED_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    SPEED_CONTROL_FAULT_NONE = 0,
    SPEED_CONTROL_FAULT_LEFT_STALL,
    SPEED_CONTROL_FAULT_RIGHT_STALL
} SpeedControlFault_t;

/* 初始化速度闭环；编码器和 PWM 必须已经初始化。 */
void SpeedControl_Init(uint32_t now_ms);

/* 启动一次新的闭环过程，复位速度 PI 和编码器采样基准。 */
void SpeedControl_Start(uint32_t now_ms);

/*
 * 设置上层电机指令，含义与原 Motor_SetBoth 相同：
 * 正负号表示方向；绝对值越小驱动力/目标速度越大；0 表示停止。
 */
void SpeedControl_SetCommand(int16_t left, int16_t right);

/* 在主循环中持续调用；内部按照固定周期采样编码器并更新 PI。 */
bool SpeedControl_Update(uint32_t now_ms);

/* 停止闭环并刹车、待机；故障状态保留到显式清除。 */
void SpeedControl_Stop(void);

bool SpeedControl_IsRunning(void);
SpeedControlFault_t SpeedControl_GetFault(void);
void SpeedControl_ClearFault(void);

/* 调试数据，速度单位均为编码器 4 倍频脉冲/秒。 */
int32_t SpeedControl_GetLeftTarget(void);
int32_t SpeedControl_GetRightTarget(void);
int32_t SpeedControl_GetLeftMeasured(void);
int32_t SpeedControl_GetRightMeasured(void);
int16_t SpeedControl_GetLeftOutput(void);
int16_t SpeedControl_GetRightOutput(void);

#endif /* SPEED_CONTROL_H */
