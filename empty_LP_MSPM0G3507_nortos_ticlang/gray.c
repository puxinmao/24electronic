/*
 * gray.c - 灰度传感器实现
 */
#include "gray.h"
#include "ti_msp_dl_config.h"

/* 延时微秒（阻塞） */
static void delay_us(uint32_t us)
{
    while (us--) {
        delay_cycles(CPUCLK_FREQ / 1000000);
    }
}

void Gray_Init(void)
{
    /* SysConfig 已配置 OUT 为输入，AD0~2 为输出，初始低电平 */
}

void Gray_SelectChannel(uint8_t ch)
{
    if (ch & 1)
        DL_GPIO_setPins(GPIO_GRAY_PORT, GPIO_GRAY_AD0_PIN);
    else
        DL_GPIO_clearPins(GPIO_GRAY_PORT, GPIO_GRAY_AD0_PIN);

    if (ch & 2)
        DL_GPIO_setPins(GPIO_GRAY_PORT, GPIO_GRAY_AD1_PIN);
    else
        DL_GPIO_clearPins(GPIO_GRAY_PORT, GPIO_GRAY_AD1_PIN);

    if (ch & 4)
        DL_GPIO_setPins(GPIO_GRAY_PORT, GPIO_GRAY_AD2_PIN);
    else
        DL_GPIO_clearPins(GPIO_GRAY_PORT, GPIO_GRAY_AD2_PIN);

    delay_us(10); /* 等待 CD4051 切换 */
}

uint8_t Gray_Read(void)
{
    return (DL_GPIO_readPins(GPIO_GRAY_PORT, GPIO_GRAY_OUT_PIN)
                   != 0) ? 1 : 0;
}

uint8_t Gray_ReadAll(void)
{
    uint8_t result = 0;
    for (uint8_t ch = 0; ch < 8; ch++) {
        Gray_SelectChannel(ch);
        if (Gray_Read()) {
            result |= (1 << ch);
        }
    }
    return result;
}
