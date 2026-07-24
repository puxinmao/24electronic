/*
 * button.h - KEY1 按键扫描模块
 */
#ifndef BUTTON_H
#define BUTTON_H

#include <stdbool.h>
#include <stdint.h>
#include "ti_msp_dl_config.h"

/* 低电平按下，消抖 50 ms，并等待释放（最长 300 ms）。 */
bool Button_IsPressed(GPIO_Regs *port, uint32_t pin);

#define KEY1_PRESSED  Button_IsPressed(GPIO_IO_KEY1_PORT, GPIO_IO_KEY1_PIN) /* 仅 KEY1 绑定启动功能。 */

#endif /* BUTTON_H */
