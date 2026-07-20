/*
 * gray.h - 八路灰度传感器模块 (CD4051 多路选择器)
 *
 * 连接: OUT=PB12(数字量输入), AD0=PB17, AD1=PB15, AD2=PB8 (通道选择输出)
 * 通道: 0~7 通过 AD[2:0] 选择
 */
#ifndef GRAY_H
#define GRAY_H

#include <stdint.h>

/* 初始化灰度传感器 GPIO */
void Gray_Init(void);

/* 选择通道 0~7 */
void Gray_SelectChannel(uint8_t ch);

/* 读取当前通道的值 (0=黑/低, 1=白/高) */
uint8_t Gray_Read(void);

/* 依次读取 8 个通道，返回 8-bit 位图 (bit0 = CH0) */
uint8_t Gray_ReadAll(void);

#endif /* GRAY_H */
