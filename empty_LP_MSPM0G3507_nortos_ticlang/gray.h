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

/* 读取当前通道的数字电平；当前整车逻辑按 1=检测到黑线、0=未检测到处理。 */
uint8_t Gray_Read(void);

/* 依次读取 8 个通道，返回 8-bit 位图 (bit0 = CH0) */
uint8_t Gray_ReadAll(void);

/* 统计位图中看到黑色的路数 */
int Gray_BlackCount(uint8_t map);

#endif /* GRAY_H */
