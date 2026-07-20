/*
 * uart_debug.h - 串口调试模块
 *
 * UART_1: TX=PA10, RX=PA11, 115200-8N1 (调试用)
 * UART_0: RX=PA9, 仅接收 (外部数据输入)
 */
#ifndef UART_DEBUG_H
#define UART_DEBUG_H

#include <stdint.h>
#include <stdbool.h>

/* ========== 调试 UART (UART_1, TX=PA10) ========== */

/* 发送一个字节（阻塞） */
void DBG_SendChar(char c);

/* 发送字符串 */
void DBG_SendStr(const char *str);

/* 发送数字（十进制） */
void DBG_SendNum(int32_t val);

/* 发送 16 进制 */
void DBG_SendHex8(uint8_t val);
void DBG_SendHex32(uint32_t val);

/* 简易 printf，支持 %d %u %x %s %c %% */
void DBG_Printf(const char *fmt, ...);

/* ========== 外部数据 UART (UART_0, RX=PA9) ========== */

/* 初始化 PA9 接收（SysConfig 已完成，这里仅提提供接口） */
void UART_RX_Init(void);

/* 查询是否收到数据 */
bool UART_RX_Available(void);

/* 读取一个字节 */
uint8_t UART_RX_Read(void);

#endif /* UART_DEBUG_H */
