/*
 * motor.h - 四轮电机驱动模块 (双 TB6612FNG + TIMA1/TIMG7 PWM)
 */
#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

#define MOTOR_PWM_MAX  2000 /* PWM 最大比较值，与 SysConfig 中两组 PWM 的 period=2000 一致。 */

typedef enum {
    MOTOR_WHEEL_A_RIGHT_REAR = 0,
    MOTOR_WHEEL_B_RIGHT_FRONT,
    MOTOR_WHEEL_C_LEFT_FRONT,
    MOTOR_WHEEL_D_LEFT_REAR,
    MOTOR_WHEEL_COUNT
} MotorWheel_t;

/* 初始化四路方向 GPIO 和两组 PWM，默认刹车并进入待机。 */
void Motor_Init(void);

/* 设置单轮指令，范围 [-MOTOR_PWM_MAX, MOTOR_PWM_MAX]，正值表示向前。 */
void Motor_SetWheel(MotorWheel_t wheel, int16_t speed);

/* 按 A/B/C/D（右后/右前/左前/左后）顺序设置四个车轮。 */
void Motor_SetFour(int16_t a, int16_t b, int16_t c, int16_t d);

/* 同侧两个车轮使用相同指令，保留已验证两轮代码的上层接口。 */
void Motor_SetLeft(int16_t speed);
void Motor_SetRight(int16_t speed);
void Motor_SetBoth(int16_t left, int16_t right);

/* 四轮短路制动。 */
void Motor_Brake(void);

/* 公共 STBY 拉低/拉高。 */
void Motor_Standby(void);
void Motor_Enable(void);

#endif /* MOTOR_H */
