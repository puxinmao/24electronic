/*
 * button.c - 按键扫描实现
 */
#include "button.h"
#include "motor.h"
#include "ti_msp_dl_config.h"

static volatile bool g_estop_event;

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

void Button_EStopInit(void)
{
    g_estop_event = false;
    DL_GPIO_clearInterruptStatus(GPIO_IO_KEY2_PORT, GPIO_IO_KEY2_PIN);
    DL_GPIO_enableInterrupt(GPIO_IO_KEY2_PORT, GPIO_IO_KEY2_PIN);

    /* GPIO group must preempt the WIT UART receive interrupt. */
    NVIC_SetPriority(GPIO_IO_INT_IRQN, 0);
    NVIC_EnableIRQ(GPIO_IO_INT_IRQN);
}

bool Button_EStopHandleIRQ(void)
{
    uint32_t status = DL_GPIO_getEnabledInterruptStatus(
        GPIO_IO_KEY2_PORT, GPIO_IO_KEY2_PIN);

    if ((status & GPIO_IO_KEY2_PIN) == 0U) return false;

    DL_GPIO_clearInterruptStatus(GPIO_IO_KEY2_PORT, GPIO_IO_KEY2_PIN);

    /* SysConfig uses both edges; only the active-low press is an E-stop. */
    if (DL_GPIO_readPins(GPIO_IO_KEY2_PORT, GPIO_IO_KEY2_PIN) != 0U) {
        return false;
    }

    /* Stop first. Main handles state, display, and UART recovery later. */
    NVIC_DisableIRQ(UART_0_INST_INT_IRQN);
    Motor_Brake();
    Motor_Standby();
    g_estop_event = true;
    return true;
}

bool Button_EStopIsPending(void)
{
    return g_estop_event;
}

bool Button_EStopTakeEvent(void)
{
    uint32_t primask = __get_PRIMASK();
    bool event;

    __disable_irq();
    event = g_estop_event;
    g_estop_event = false;
    if (primask == 0U) __enable_irq();

    return event;
}
