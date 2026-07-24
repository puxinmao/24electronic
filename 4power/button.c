/*
 * button.c - KEY1 按键扫描实现
 */
#include "button.h"

/* 按键阻塞式消抖使用的毫秒延时，仅在检测到低电平后调用。 */
static void delay_ms(uint32_t ms)
{
    while (ms-- > 0U) delay_cycles(CPUCLK_FREQ / 1000U);
}

bool Button_IsPressed(GPIO_Regs *port, uint32_t pin)
{
    uint32_t release_timeout_ms = 300U;

    if (DL_GPIO_readPins(port, pin) != 0U) return false;
    delay_ms(50U);
    if (DL_GPIO_readPins(port, pin) != 0U) return false;

    while (DL_GPIO_readPins(port, pin) == 0U &&
           release_timeout_ms-- > 0U) {
        delay_ms(1U);
    }
    return true;
}
