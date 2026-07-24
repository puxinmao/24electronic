/*
 * speed_control.h - 四轮编码器独立速度闭环模块
 */
#ifndef SPEED_CONTROL_H
#define SPEED_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    SPEED_CONTROL_FAULT_NONE = 0,
    SPEED_CONTROL_FAULT_WHEEL_A_STALL,
    SPEED_CONTROL_FAULT_WHEEL_B_STALL,
    SPEED_CONTROL_FAULT_WHEEL_C_STALL,
    SPEED_CONTROL_FAULT_WHEEL_D_STALL
} SpeedControlFault_t;

void SpeedControl_Init(uint32_t now_ms);
void SpeedControl_Start(uint32_t now_ms);

/* 左指令驱动 A/B，右指令驱动 C/D；指令语义与已验证两轮工程一致。 */
void SpeedControl_SetCommand(int16_t left, int16_t right);

/* 主循环持续调用，达到固定采样周期时返回 true。 */
bool SpeedControl_Update(uint32_t now_ms);

/* 停止四轮闭环并刹车、待机；故障需显式清除。 */
void SpeedControl_Stop(void);

bool SpeedControl_IsRunning(void);
SpeedControlFault_t SpeedControl_GetFault(void);
void SpeedControl_ClearFault(void);

/* 读取指定 A/B/C/D 车轮的调试数据。 */
int32_t SpeedControl_GetTarget(uint32_t wheel);
int32_t SpeedControl_GetMeasured(uint32_t wheel);
int16_t SpeedControl_GetOutput(uint32_t wheel);

#endif /* SPEED_CONTROL_H */
