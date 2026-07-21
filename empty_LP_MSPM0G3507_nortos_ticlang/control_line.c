/*
 * control_line.c - 灰度循迹实现
 */
#include "control_line.h"
#include "pid.h"
#include "motor.h"
#include "ti_msp_dl_config.h"

#define LINE_CENTER 3.5f

static PID_t  sPid;
static int16_t sBaseSpeed = 500;
static float  sError = 0.0f;
static float  sCorr  = 0.0f;

/* ========== 公开接口 ========== */

void Line_Config(float kp, float ki, float kd, int16_t base_speed)
{
    PID_Init(&sPid, kp, ki, kd, -600, 600, 400);
    sBaseSpeed = base_speed;
}

void Line_Start(void)
{
    PID_Reset(&sPid);
    PID_SetSetpoint(&sPid, LINE_CENTER);
    Motor_Enable();
}

void Line_Update(uint8_t gray_map)
{
    float pos = Line_GetPosition(gray_map);
    sError = pos - LINE_CENTER;
    sCorr  = PID_Compute(&sPid, pos, 0.005f);

    int16_t left  = sBaseSpeed + (int16_t)sCorr;
    int16_t right = sBaseSpeed - (int16_t)sCorr;
    if (left  < 0) left  = 0;
    if (right < 0) right = 0;
    Motor_SetBoth(left, right);
}

void Line_Stop(void)
{
    Motor_Brake();
    Motor_Standby();
}

float Line_GetPosition(uint8_t gray_map)
{
    int16_t sw = 0, sn = 0;
    for (int i = 0; i < 8; i++) {
        if (gray_map & (1 << i)) { sw += i; sn++; }
    }
    return (sn == 0) ? LINE_CENTER : (float)sw / (float)sn;
}

float Line_GetError(void)      { return sError; }
float Line_GetCorrection(void) { return sCorr; }
