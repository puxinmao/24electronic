/*
 * motor.c - 电机驱动实现 (TB6612FNG)
 *
 * 引脚 (SysConfig自动生成):
 *  左电机: AIN_1=PA16, AIN_2=PA17, PWM=PB4(TIMA1_CCP0)
 *  右电机: BIN_1=PA24, BIN_2=PA25, PWM=PB1(TIMA1_CCP1)
 *  STBY:   PB9
 */
#include "motor.h"
#include "ti_msp_dl_config.h"

/* ========== 内部辅助 ========== */

static inline void left_forward(void)
{
    DL_GPIO_setPins(GPIO_MOTOR_AIN_1_PORT, GPIO_MOTOR_AIN_1_PIN);
    DL_GPIO_clearPins(GPIO_MOTOR_AIN_2_PORT, GPIO_MOTOR_AIN_2_PIN);
}

static inline void left_reverse(void)
{
    DL_GPIO_clearPins(GPIO_MOTOR_AIN_1_PORT, GPIO_MOTOR_AIN_1_PIN);
    DL_GPIO_setPins(GPIO_MOTOR_AIN_2_PORT, GPIO_MOTOR_AIN_2_PIN);
}

static inline void left_brake(void)
{
    DL_GPIO_setPins(GPIO_MOTOR_AIN_1_PORT, GPIO_MOTOR_AIN_1_PIN);
    DL_GPIO_setPins(GPIO_MOTOR_AIN_2_PORT, GPIO_MOTOR_AIN_2_PIN);
}

static inline void left_float(void)
{
    DL_GPIO_clearPins(GPIO_MOTOR_AIN_1_PORT, GPIO_MOTOR_AIN_1_PIN);
    DL_GPIO_clearPins(GPIO_MOTOR_AIN_2_PORT, GPIO_MOTOR_AIN_2_PIN);
}

static inline void right_forward(void)
{
    DL_GPIO_setPins(GPIO_MOTOR_BIN_1_PORT, GPIO_MOTOR_BIN_1_PIN);
    DL_GPIO_clearPins(GPIO_MOTOR_BIN_2_PORT, GPIO_MOTOR_BIN_2_PIN);
}

static inline void right_reverse(void)
{
    DL_GPIO_clearPins(GPIO_MOTOR_BIN_1_PORT, GPIO_MOTOR_BIN_1_PIN);
    DL_GPIO_setPins(GPIO_MOTOR_BIN_2_PORT, GPIO_MOTOR_BIN_2_PIN);
}

static inline void right_brake(void)
{
    DL_GPIO_setPins(GPIO_MOTOR_BIN_1_PORT, GPIO_MOTOR_BIN_1_PIN);
    DL_GPIO_setPins(GPIO_MOTOR_BIN_2_PORT, GPIO_MOTOR_BIN_2_PIN);
}

static inline void right_float(void)
{
    DL_GPIO_clearPins(GPIO_MOTOR_BIN_1_PORT, GPIO_MOTOR_BIN_1_PIN);
    DL_GPIO_clearPins(GPIO_MOTOR_BIN_2_PORT, GPIO_MOTOR_BIN_2_PIN);
}

/* ========== 公开接口 ========== */

void Motor_Init(void)
{
    /* SysConfig 已配置 GPIO 和 PWM，只需确保停止 */
    Motor_Brake();
    Motor_Standby();
}

void Motor_SetLeft(int16_t speed)
{
    uint16_t pwm_val;
    bool neg = (speed < 0);
    if (neg) speed = -speed;
    if (speed > MOTOR_PWM_MAX) speed = MOTOR_PWM_MAX;
    pwm_val = (uint16_t)speed;

    if (pwm_val == 0) {
        left_float();
    } else if (neg) {
        left_reverse();
    } else {
        left_forward();
    }

    DL_TimerA_setCaptureCompareValue(PWM_INST, pwm_val, DL_TIMER_CC_0_INDEX);
}

void Motor_SetRight(int16_t speed)
{
    uint16_t pwm_val;
    bool neg = (speed < 0);
    if (neg) speed = -speed;
    if (speed > MOTOR_PWM_MAX) speed = MOTOR_PWM_MAX;
    pwm_val = (uint16_t)speed;

    if (pwm_val == 0) {
        right_float();
    } else if (neg) {
        right_reverse();
    } else {
        right_forward();
    }

    DL_TimerA_setCaptureCompareValue(PWM_INST, pwm_val, DL_TIMER_CC_1_INDEX);
}

void Motor_SetBoth(int16_t left, int16_t right)
{
    Motor_SetLeft(left);
    Motor_SetRight(right);
}

void Motor_Brake(void)
{
    left_brake();
    right_brake();
    DL_TimerA_setCaptureCompareValue(PWM_INST, 0, DL_TIMER_CC_0_INDEX);
    DL_TimerA_setCaptureCompareValue(PWM_INST, 0, DL_TIMER_CC_1_INDEX);
}

void Motor_Standby(void)
{
    DL_GPIO_clearPins(GPIO_MOTOR_STBY_PORT, GPIO_MOTOR_STBY_PIN);
}

void Motor_Enable(void)
{
    DL_GPIO_setPins(GPIO_MOTOR_STBY_PORT, GPIO_MOTOR_STBY_PIN);
}
