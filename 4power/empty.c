/*
 * empty.c - 单路电机持续运行测试程序
 *
 * 按下 KEY1 后只启动 A 左前轮，另外三路保持关闭。
 * 启动后不会自动停车，结束测试时需要复位或断开电机电源。
 */
#include "ti_msp_dl_config.h"

#include "button.h"
#include "motor.h"
#include "oled.h"
#include "uart_debug.h"

#include <stdio.h>

typedef enum {
    MOTOR_TEST_IDLE = 0,
    MOTOR_TEST_RUNNING
} MotorTestState_t;

#define MOTOR_TEST_WHEEL MOTOR_WHEEL_A_LEFT_FRONT /* 当前测试 A 左前轮，需要测试其他轮时改为对应的 MOTOR_WHEEL_* 枚举。 */
#define MOTOR_TEST_PWM                           1500 /* 测试 PWM：数值越小驱动力越强，建议先从 1500 开始。 */

static MotorTestState_t sTestState = MOTOR_TEST_IDLE;
static char sDisplayBuffer[24];

static void motor_test_start(void)
{
    Motor_Brake();
    Motor_SetFour(0, 0, 0, 0);
    Motor_SetWheel(MOTOR_TEST_WHEEL, MOTOR_TEST_PWM);
    Motor_Enable();
    sTestState = MOTOR_TEST_RUNNING;
}

int main(void)
{
    SYSCFG_DL_init();
    Motor_Init();

    OLED_Init();
    OLED_Clear();
    OLED_ShowString(3, 0, (uint8_t *)"1-WHEEL TEST", 8);
    OLED_ShowString(3, 7, (uint8_t *)"KEY1 START", 8);
    DBG_SendStr("Single-wheel test ready: KEY1 starts motor A\r\n");

    while (1) {
        if (sTestState == MOTOR_TEST_IDLE && KEY1_PRESSED) {
            motor_test_start();
            sprintf(sDisplayBuffer, "A RUN P:%d", MOTOR_TEST_PWM);
            OLED_ShowString(3, 7, (uint8_t *)sDisplayBuffer, 8);
            DBG_Printf("Motor A running continuously, PWM=%d\r\n",
                       MOTOR_TEST_PWM);
        }
    }
}
