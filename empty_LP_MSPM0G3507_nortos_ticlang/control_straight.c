/*
 * control_straight.c - 偏航角直行实现
 */
#include "control_straight.h"
#include "pid.h"
#include "motor.h"
#include "ti_msp_dl_config.h"

static PID_t  sPid;
static float  sTargetYaw = 0.0f;
static int16_t sBaseSpeed = 700;
static bool   sRunning = false;
static bool   sFirstUpdate = false;
static uint8_t sRampUpdates = 0;

#define STRAIGHT_RAMP_UPDATES     20U
#define STRAIGHT_START_PWM_OFFSET 300
#define STRAIGHT_RIGHT_PWM_TRIM    40

/* ========== 内部辅助 ========== */

static float yaw_error_norm(float cur, float tgt)
{
    float e = cur - tgt;
    while (e >  180.0f) e -= 360.0f;
    while (e < -180.0f) e += 360.0f;
    return e;
}

/* ========== 公开接口 ========== */

void Straight_Config(float kp, float ki, float kd, int16_t base_speed)
{
    PID_Init(&sPid, kp, ki, kd, -600, 600, 400);
    sBaseSpeed = base_speed;
}

void Straight_Start(float current_yaw)
{
    sTargetYaw = current_yaw;
    PID_Reset(&sPid);
    PID_SetSetpoint(&sPid, sTargetYaw);
    Motor_Enable();
    sRunning = true;
    sFirstUpdate = true;
    sRampUpdates = 0;
}

float Straight_Update(float current_yaw)
{
    if (!sRunning) return 0.0f;

    float err  = yaw_error_norm(current_yaw, sTargetYaw);   /* cur-tgt，用于返回显示 */
    if (sFirstUpdate) {
        sPid.prev_error = -err;
        sFirstUpdate = false;
    }
    float corr = PID_ComputeError(&sPid, -err, 0.01f);       /* PID 使用归一化的 tgt-cur */

    int16_t base = sBaseSpeed;
    int16_t right_trim = 0;
    if (sRampUpdates < STRAIGHT_RAMP_UPDATES) {
        uint8_t remaining = STRAIGHT_RAMP_UPDATES - sRampUpdates;
        base += (int16_t)((STRAIGHT_START_PWM_OFFSET * remaining) /
                          STRAIGHT_RAMP_UPDATES);
        right_trim = (int16_t)((STRAIGHT_RIGHT_PWM_TRIM * remaining) /
                               STRAIGHT_RAMP_UPDATES);
        sRampUpdates++;
    }

    int16_t left  = base + (int16_t)corr;
    int16_t right = base - (int16_t)corr + right_trim;
    if (left  < 0) left  = 0;
    if (right < 0) right = 0;
    Motor_SetBoth(left, right);

    return err;
}

void Straight_Stop(void)
{
    sRunning = false;
    sFirstUpdate = false;
    Motor_Brake();
    Motor_Standby();
}

float Straight_GetTarget(void) { return sTargetYaw; }
bool  Straight_IsRunning(void) { return sRunning; }
