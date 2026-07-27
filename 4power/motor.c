/*
 * motor.c - 四轮电机驱动实现 (双 TB6612FNG)
 *
 * 车轮编号: A=右后，B=后前，C=左前，D=左后
 */
#include "motor.h"
#include "ti_msp_dl_config.h"

#include <stdbool.h>

#define MOTOR_A_FORWARD_INVERTED  1 /* A 右后轮：置 1 时交换正/反转方向。 */
#define MOTOR_B_FORWARD_INVERTED  0 /* B 后前轮：置 1 时交换正/反转方向。 */
#define MOTOR_C_FORWARD_INVERTED  0 /* C 左前轮：置 1 时交换正/反转方向。 */
#define MOTOR_D_FORWARD_INVERTED  1 /* D 左后轮：置 1 时交换正/反转方向。 */

typedef struct {
    GPIO_Regs *in1_port;
    uint32_t in1_pin;
    GPIO_Regs *in2_port;
    uint32_t in2_pin;
    GPTIMER_Regs *pwm_timer;
    DL_TIMER_CC_INDEX pwm_channel;
    bool forward_inverted;
} MotorChannel_t;

static const MotorChannel_t sMotors[MOTOR_WHEEL_COUNT] = {
    { GPIO_MOTOR_AIN_1_PORT, GPIO_MOTOR_AIN_1_PIN,
      GPIO_MOTOR_AIN_2_PORT, GPIO_MOTOR_AIN_2_PIN,
      PWM_INST, GPIO_PWM_C0_IDX, MOTOR_A_FORWARD_INVERTED != 0 },
    { GPIO_MOTOR_BIN_1_PORT, GPIO_MOTOR_BIN_1_PIN,
      GPIO_MOTOR_BIN_2_PORT, GPIO_MOTOR_BIN_2_PIN,
      PWM_INST, GPIO_PWM_C1_IDX, MOTOR_B_FORWARD_INVERTED != 0 },
    { GPIO_MOTOR_CIN_1_PORT, GPIO_MOTOR_CIN_1_PIN,
      GPIO_MOTOR_CIN_2_PORT, GPIO_MOTOR_CIN_2_PIN,
      PWM_1_INST, GPIO_PWM_1_C0_IDX, MOTOR_C_FORWARD_INVERTED != 0 },
    { GPIO_MOTOR_DIN_1_PORT, GPIO_MOTOR_DIN_1_PIN,
      GPIO_MOTOR_DIN_2_PORT, GPIO_MOTOR_DIN_2_PIN,
      PWM_1_INST, GPIO_PWM_1_C1_IDX, MOTOR_D_FORWARD_INVERTED != 0 }
};

static void motor_set_direction(const MotorChannel_t *motor,
                                bool forward)
{
    if (motor->forward_inverted) forward = !forward;

    if (forward) {
        DL_GPIO_setPins(motor->in1_port, motor->in1_pin);
        DL_GPIO_clearPins(motor->in2_port, motor->in2_pin);
    } else {
        DL_GPIO_clearPins(motor->in1_port, motor->in1_pin);
        DL_GPIO_setPins(motor->in2_port, motor->in2_pin);
    }
}

static void motor_float(const MotorChannel_t *motor)
{
    DL_GPIO_clearPins(motor->in1_port, motor->in1_pin);
    DL_GPIO_clearPins(motor->in2_port, motor->in2_pin);
}

static void motor_brake(const MotorChannel_t *motor)
{
    DL_GPIO_setPins(motor->in1_port, motor->in1_pin);
    DL_GPIO_setPins(motor->in2_port, motor->in2_pin);
    DL_Timer_setCaptureCompareValue(motor->pwm_timer, 0,
                                    motor->pwm_channel);
}

void Motor_Init(void)
{
    Motor_Brake();
    Motor_Standby();
    DL_TimerA_startCounter(PWM_INST);
    DL_TimerG_startCounter(PWM_1_INST);
}

void Motor_SetWheel(MotorWheel_t wheel, int16_t speed)
{
    const MotorChannel_t *motor;
    int32_t magnitude;
    bool forward;

    if ((uint32_t)wheel >= MOTOR_WHEEL_COUNT) return;
    motor = &sMotors[wheel];
    magnitude = speed;
    forward = (magnitude >= 0);
    if (magnitude < 0) magnitude = -magnitude;
    if (magnitude > MOTOR_PWM_MAX) magnitude = MOTOR_PWM_MAX;

    if (magnitude == 0) {
        motor_float(motor);
    } else {
        motor_set_direction(motor, forward);
    }

    DL_Timer_setCaptureCompareValue(motor->pwm_timer, (uint32_t)magnitude,
                                    motor->pwm_channel);
}

void Motor_SetFour(int16_t a, int16_t b, int16_t c, int16_t d)
{
    Motor_SetWheel(MOTOR_WHEEL_A_RIGHT_REAR, a);
    Motor_SetWheel(MOTOR_WHEEL_B_RIGHT_FRONT, b);
    Motor_SetWheel(MOTOR_WHEEL_C_LEFT_FRONT, c);
    Motor_SetWheel(MOTOR_WHEEL_D_LEFT_REAR, d);
}

void Motor_SetLeft(int16_t speed)
{
    Motor_SetWheel(MOTOR_WHEEL_C_LEFT_FRONT, speed);
    Motor_SetWheel(MOTOR_WHEEL_D_LEFT_REAR, speed);
}

void Motor_SetRight(int16_t speed)
{
    Motor_SetWheel(MOTOR_WHEEL_A_RIGHT_REAR, speed);
    Motor_SetWheel(MOTOR_WHEEL_B_RIGHT_FRONT, speed);
}

void Motor_SetBoth(int16_t left, int16_t right)
{
    Motor_SetFour(right, right, left, left);
}

void Motor_Brake(void)
{
    uint32_t i;
    for (i = 0; i < MOTOR_WHEEL_COUNT; i++) motor_brake(&sMotors[i]);
}

void Motor_Standby(void)
{
    DL_GPIO_clearPins(GPIO_MOTOR_STBY_PORT, GPIO_MOTOR_STBY_PIN);
}

void Motor_Enable(void)
{
    DL_GPIO_setPins(GPIO_MOTOR_STBY_PORT, GPIO_MOTOR_STBY_PIN);
}
