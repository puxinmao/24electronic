/*
 * button.c - 按键扫描实现
 */
#include "button.h"
#include "ti_msp_dl_config.h"

static void delay_ms(uint32_t ms)
{
    while (ms--) { delay_cycles(CPUCLK_FREQ / 1000); }
}

bool Button_IsPressed(GPIO_Regs *port, uint32_t pin)
{
    if (DL_GPIO_readPins(port, pin) != 0) return false;  /* 没按 */

    delay_ms(50);                                          /* 消抖 */
    if (DL_GPIO_readPins(port, pin) != 0) return false;   /* 误触发 */

    /* 等释放，超时 300ms */
    uint32_t t = 300;
    while (DL_GPIO_readPins(port, pin) == 0 && t > 0) { delay_ms(1); t--; }
    return true;
}
