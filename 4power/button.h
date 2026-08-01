/*
 * button.h - 按键扫描模块
 */
#ifndef BUTTON_H
#define BUTTON_H

#include <stdbool.h>
#include <stdint.h>
#include "ti_msp_dl_config.h"

/* 低电平按下，消抖 50 ms，并等待释放（最长 300 ms）。 */
bool Button_IsPressed(GPIO_Regs *port, uint32_t pin);

#define KEY1_PRESSED  Button_IsPressed(GPIO_IO_KEY1_PORT, GPIO_IO_KEY1_PIN)
#define KEY2_PRESSED  Button_IsPressed(GPIO_IO_KEY2_PORT, GPIO_IO_KEY2_PIN)
#define KEY3_PRESSED  Button_IsPressed(GPIO_IO_KEY3_PORT, GPIO_IO_KEY3_PIN)

#endif /* BUTTON_H */
