/*
 * encoder.c - 霍尔编码器 4x 解码
 *
 * 使用 AB 相查表法判断方向:
 *   正转: AB 序列 → 00→10→11→01→00...
 *   反转: AB 序列 → 00→01→11→10→00...
 *   每次边沿变化查表: +1(正转) / -1(反转) / 0(非法跳变)
 *
 * 中断: GROUP1_IRQHandler 同时处理 LA(PB7), LB(PB6), RA(PB0), RB(PB16)
 *       全部在 GPIOB 上，双边沿触发
 */
#include "encoder.h"
#include "button.h"
#include "ti_msp_dl_config.h"

/* 编码器状态 */
static volatile int32_t gEncLeft  = 0;
static volatile int32_t gEncRight = 0;

/* 速度计算：记录上一次位置和时间 */
static int32_t gEncLeftPrev  = 0;
static int32_t gEncRightPrev = 0;

/* ========== 查表 ========== */

/*
 * 4x 解码查表:
 *   idx = (prev_state << 2) | curr_state
 *   state = (B << 1) | A
 *   返回值: +1=正转一个脉冲, -1=反转一个脉冲, 0=无效
 */
static const int8_t enc4x_table[16] = {
    /* prev\curr   00      01      10      11  */
    /*  00    */    0,     -1,     +1,      0,
    /*  01    */   +1,      0,      0,     -1,
    /*  10    */   -1,      0,      0,     +1,
    /*  11    */    0,     +1,     -1,      0
};

/* 上一次 AB 状态 */
static uint8_t gLeftState  = 0;  /* bit1=LB, bit0=LA */
static uint8_t gRightState = 0;  /* bit1=RB, bit0=RA */

/* ========== 公开函数 ========== */

void Encoder_Init(void)
{
    /* 读取初始状态 */
    uint32_t in = DL_GPIO_readPins(GPIOB,
        DL_GPIO_PIN_0 | DL_GPIO_PIN_7 | DL_GPIO_PIN_6 | DL_GPIO_PIN_16);

    gLeftState  = 0;
    if (in & DL_GPIO_PIN_7)  gLeftState  |= 1;  /* LA */
    if (in & DL_GPIO_PIN_6)  gLeftState  |= 2;  /* LB */
    gRightState = 0;
    if (in & DL_GPIO_PIN_0)  gRightState |= 1;  /* RA */
    if (in & DL_GPIO_PIN_16) gRightState |= 2;  /* RB */

    gEncLeftPrev  = gEncLeft;
    gEncRightPrev = gEncRight;

    /* SysConfig 已配置 LA 中断，这里补上 RA + LB + RB */
    DL_GPIO_setLowerPinsPolarity(GPIOB,
        DL_GPIO_PIN_0_EDGE_RISE_FALL |
        DL_GPIO_PIN_6_EDGE_RISE_FALL |
        DL_GPIO_PIN_7_EDGE_RISE_FALL);
    DL_GPIO_setUpperPinsPolarity(GPIOB,
        DL_GPIO_PIN_16_EDGE_RISE_FALL);

    DL_GPIO_clearInterruptStatus(GPIOB,
        DL_GPIO_PIN_0 | DL_GPIO_PIN_7 | DL_GPIO_PIN_6 | DL_GPIO_PIN_16);
    DL_GPIO_enableInterrupt(GPIOB,
        DL_GPIO_PIN_0 | DL_GPIO_PIN_7 | DL_GPIO_PIN_6 | DL_GPIO_PIN_16);
    NVIC_EnableIRQ(GPIOB_INT_IRQn);
}

int32_t Encoder_GetLeft(void)  { return gEncLeft; }
int32_t Encoder_GetRight(void) { return gEncRight; }

int16_t Encoder_GetLeftSpeed(float dt)
{
    int32_t delta = gEncLeft - gEncLeftPrev;
    gEncLeftPrev = gEncLeft;
    if (dt <= 0.0f) return 0;
    return (int16_t)((float)delta / dt);
}

int16_t Encoder_GetRightSpeed(float dt)
{
    int32_t delta = gEncRight - gEncRightPrev;
    gEncRightPrev = gEncRight;
    if (dt <= 0.0f) return 0;
    return (int16_t)((float)delta / dt);
}

void Encoder_Reset(void)
{
    gEncLeft  = 0;
    gEncRight = 0;
    gEncLeftPrev  = 0;
    gEncRightPrev = 0;
}

/* ========== GPIOB 中断处理（GROUP1） ========== */

void GROUP1_IRQHandler(void)
{
    /* GPIOA and GPIOB share GROUP1. KEY2 emergency stop has priority. */
    (void)Button_EStopHandleIRQ();

    uint32_t status = DL_GPIO_getEnabledInterruptStatus(GPIOB,
        DL_GPIO_PIN_0 | DL_GPIO_PIN_7 | DL_GPIO_PIN_6 | DL_GPIO_PIN_16);

    /* 读取当前电平 */
    uint32_t pins = DL_GPIO_readPins(GPIOB,
        DL_GPIO_PIN_0 | DL_GPIO_PIN_7 | DL_GPIO_PIN_6 | DL_GPIO_PIN_16);

    /* 左编码器 */
    if (status & (DL_GPIO_PIN_7 | DL_GPIO_PIN_6)) {
        DL_GPIO_clearInterruptStatus(GPIOB, DL_GPIO_PIN_7 | DL_GPIO_PIN_6);

        uint8_t cur = 0;
        if (pins & DL_GPIO_PIN_7) cur |= 1;   /* LA = bit0 */
        if (pins & DL_GPIO_PIN_6) cur |= 2;   /* LB = bit1 */
        int8_t idx = (gLeftState << 2) | cur;
        gEncLeft += enc4x_table[idx];
        gLeftState = cur;
    }

    /* 右编码器 */
    if (status & (DL_GPIO_PIN_0 | DL_GPIO_PIN_16)) {
        DL_GPIO_clearInterruptStatus(GPIOB, DL_GPIO_PIN_0 | DL_GPIO_PIN_16);

        uint8_t cur = 0;
        if (pins & DL_GPIO_PIN_0)  cur |= 1;   /* RA = bit0 */
        if (pins & DL_GPIO_PIN_16) cur |= 2;   /* RB = bit1 */
        int8_t idx = (gRightState << 2) | cur;
        gEncRight += enc4x_table[idx];
        gRightState = cur;
    }
}
