/*
 * control_line.c - 灰度循迹实现
 */
#include "control_line.h"
#include "pid.h"
#include "motor.h"
#include "ti_msp_dl_config.h"

#define LINE_CENTER 3.5f
#define LINE_CORR_LIMIT       750.0f
#define LINE_EDGE_ERROR         2.5f
#define LINE_EDGE_MIN_CORR     620.0f
#define LINE_CURVE_PWM_OFFSET  280
#define LINE_LOST_PWM_BASE     950
#define LINE_PWM_MIN            80
#define LINE_PWM_MAX          1800

static PID_t  sPid;
static int16_t sBaseSpeed = 500;
static float  sError = 0.0f;
static float  sCorr  = 0.0f;
static float  sLastValidError = 0.0f;

static int16_t line_clamp_pwm(int16_t pwm)
{
    if (pwm < LINE_PWM_MIN) return LINE_PWM_MIN;
    if (pwm > LINE_PWM_MAX) return LINE_PWM_MAX;
    return pwm;
}

/* ========== 公开接口 ========== */

void Line_Config(float kp, float ki, float kd, int16_t base_speed)
{
    PID_Init(&sPid, kp, ki, kd,
             -LINE_CORR_LIMIT, LINE_CORR_LIMIT, 400);
    sBaseSpeed = base_speed;
}

void Line_Start(void)
{
    PID_Reset(&sPid);
    PID_SetSetpoint(&sPid, LINE_CENTER);
    sError = 0.0f;
    sCorr = 0.0f;
    sLastValidError = 0.0f;
    Motor_Enable();
}

void Line_Update(uint8_t gray_map)
{
    int16_t drive_base = sBaseSpeed;

    if (gray_map == 0U) {
        /* Keep searching in the last known direction through a short gap. */
        sError = sLastValidError;
        if (sLastValidError > 0.1f) {
            sCorr = -LINE_CORR_LIMIT;
        } else if (sLastValidError < -0.1f) {
            sCorr = LINE_CORR_LIMIT;
        } else {
            sCorr = 0.0f;
        }
        if (drive_base < LINE_LOST_PWM_BASE) {
            drive_base = LINE_LOST_PWM_BASE;
        }
    } else {
        float pos = Line_GetPosition(gray_map);
        float abs_error;
        int16_t curve_offset;

        sError = pos - LINE_CENTER;
        sLastValidError = sError;
        sCorr = PID_Compute(&sPid, pos, 0.005f);

        abs_error = (sError < 0.0f) ? -sError : sError;
        if (sError >= LINE_EDGE_ERROR && sCorr > -LINE_EDGE_MIN_CORR) {
            sCorr = -LINE_EDGE_MIN_CORR;
        } else if (sError <= -LINE_EDGE_ERROR &&
                   sCorr < LINE_EDGE_MIN_CORR) {
            sCorr = LINE_EDGE_MIN_CORR;
        }

        /* This PWM is inverted: a larger compare value means less drive. */
        curve_offset = (int16_t)(abs_error * 80.0f);
        if (curve_offset > LINE_CURVE_PWM_OFFSET) {
            curve_offset = LINE_CURVE_PWM_OFFSET;
        }
        drive_base += curve_offset;
    }

    int16_t left  = line_clamp_pwm(drive_base + (int16_t)sCorr);
    int16_t right = line_clamp_pwm(drive_base - (int16_t)sCorr);
    Motor_SetBoth(left, right);
}

void Line_Stop(void)
{
    Motor_Brake();
    Motor_Standby();
}

void Line_SetBaseSpeed(int16_t speed)
{
    sBaseSpeed = speed;
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
