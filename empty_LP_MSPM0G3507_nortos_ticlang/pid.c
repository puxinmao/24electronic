/*
 * pid.c - PID 控制器实现
 */
#include "pid.h"

void PID_Init(PID_t *pid, float Kp, float Ki, float Kd,
              float out_min, float out_max, float integral_limit)
{
    pid->Kp     = Kp;
    pid->Ki     = Ki;
    pid->Kd     = Kd;
    pid->setpoint  = 0.0f;
    pid->integral  = 0.0f;
    pid->prev_error = 0.0f;
    pid->out_min   = out_min;
    pid->out_max   = out_max;
    pid->integral_limit = integral_limit;
}

void PID_SetSetpoint(PID_t *pid, float setpoint)
{
    pid->setpoint = setpoint;
}

float PID_Compute(PID_t *pid, float measurement, float dt)
{
    float error = pid->setpoint - measurement;
    float output;

    /* P 项 */
    output = pid->Kp * error;

    /* I 项（带限幅） */
    pid->integral += error * dt;
    if (pid->integral >  pid->integral_limit) pid->integral =  pid->integral_limit;
    if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;
    output += pid->Ki * pid->integral;

    /* D 项（防微小 dt 除零） */
    if (dt > 1e-6f) {
        output += pid->Kd * (error - pid->prev_error) / dt;
    }
    pid->prev_error = error;

    /* 输出限幅 */
    if (output > pid->out_max) output = pid->out_max;
    if (output < pid->out_min) output = pid->out_min;

    return output;
}

void PID_Reset(PID_t *pid)
{
    pid->integral   = 0.0f;
    pid->prev_error = 0.0f;
}
