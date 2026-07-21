/*
 * wit.h - JY901S 陀螺仪模块 (UART1/PA9, 115200bps)
 *
 * 协议: 每帧 11 字节，帧头 0x55
 *   0x51 → 加速度 + 温度
 *   0x52 → 角速度
 *   0x53 → 姿态角 + 版本
 */
#ifndef WIT_H
#define WIT_H

#include "ti_msp_dl_config.h"
#include <stdint.h>

/* JY901S 解析后的数据 */
typedef struct {
    float pitch;        /* 俯仰角 (°)       */
    float roll;         /* 横滚角 (°)       */
    float yaw;          /* 偏航角 (°)       */
    float temperature;  /* 芯片温度 (°C)    */
    int16_t ax;         /* 加速度 X (mg)    */
    int16_t ay;         /* 加速度 Y (mg)    */
    int16_t az;         /* 加速度 Z (mg)    */
    int16_t gx;         /* 角速度 X (°/s)   */
    int16_t gy;         /* 角速度 Y (°/s)   */
    int16_t gz;         /* 角速度 Z (°/s)   */
    int16_t version;    /* 版本号           */
} WIT_Data_t;

extern WIT_Data_t wit_data;

/* 初始化 JY901S（开启 FIFO + 中断） */
void WIT_Init(void);

/* 获取最新数据（非阻塞，无新数据返回 false） */
bool WIT_GetData(WIT_Data_t *dst);

/* UART 噪声导致接收异常时返回 true */
bool WIT_HasFault(void);

/* 清空接收状态并重新开启 UART 中断 */
void WIT_Recover(void);

#endif
