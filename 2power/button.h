/*
 * button.h - 按键扫描模块
 */
#ifndef BUTTON_H
#define BUTTON_H

#include <stdint.h>
#include <stdbool.h>
#include "ti_msp_dl_config.h"

/* 消抖 50ms + 等释放 (300ms 超时)，返回是否按下 */
bool Button_IsPressed(GPIO_Regs *port, uint32_t pin);

/* KEY2 hardware emergency stop (PA15, falling edge). */
void Button_EStopInit(void);
bool Button_EStopHandleIRQ(void);
bool Button_EStopIsPending(void);
bool Button_EStopTakeEvent(void);

#define KEY1_PRESSED  Button_IsPressed(GPIO_IO_KEY1_PORT, GPIO_IO_KEY1_PIN)
#define KEY2_PRESSED  Button_IsPressed(GPIO_IO_KEY2_PORT, GPIO_IO_KEY2_PIN)
#define KEY3_PRESSED  Button_IsPressed(GPIO_IO_KEY3_PORT, GPIO_IO_KEY3_PIN)

#endif
