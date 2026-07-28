/*
 * control_straight.c - 偏航角直行实现
 */
#include "control_straight.h"
#include "pid.h"
#include "motor.h"
#include "speed_control.h"
#include "ti_msp_dl_config.h"

static PID_t  sPid;
static float  sTargetYaw = 0.0f;
static int16_t sBaseSpeed = 700;
static bool   sRunning = false;
static bool   sFirstUpdate = false;
static bool   sRampStarted = false;
static uint32_t sStartMs = 0;
static uint32_t sLastUpdateMs = 0;

/* 原地转向参数目前未接入按键流程，后续重新启用原地转向时再调整。 */
static float sTurnAngleDeg = 28.0f;
static int16_t sTurnFastPwm = 900;
static int16_t sTurnSlowPwm = 1200;
static float sTurnSlowAngleDeg = 10.0f;
static uint8_t sTurnBrakeFrames = 10;
static uint16_t sTurnTimeoutFrames = 300;
static uint8_t sTurnStallFrames = 40;
static float sTurnStallMinDeg = 0.5f;
static float sTurnTarget = 0.0f;
static float sTurnLastYaw = 0.0f;
static float sTurnProgress = 0.0f;
static float sTurnWatchProgress = 0.0f;
static uint16_t sTurnFrames = 0;
static uint8_t sTurnBrakeCount = 0;
static uint8_t sTurnWatchCount = 0;
static bool sTurnRunning = false;
static bool sTurnBraking = false;

/* ========== 直线控制现场调参区 ========== */
#define STRAIGHT_PID_DT_DEFAULT  0.005f /* 首次更新的默认 dt；后续使用 SysTick 实际间隔 */
#define STRAIGHT_PID_DT_MAX      0.050f /* 数据停顿后的 dt 上限，避免单次积分跨度过大 */
#define STRAIGHT_CORR_LIMIT       600.0f /* 左右轮最大差速修正；增大可加强纠偏，也更易摆动 */
#define STRAIGHT_INTEGRAL_LIMIT   400.0f /* 积分累计限幅；通常不需要现场调整 */

//==========启动时候的修正=========
#define STRAIGHT_RAMP_MS            200U /* 首次控制更新起的渐变时间；由 SysTick 计时 */
#define STRAIGHT_START_PWM_OFFSET   300  /* 启动时加到两轮 PWM；越大起步越慢，随后渐变为 0 */
#define STRAIGHT_RIGHT_PWM_TRIM      0 /* 启动时额外减弱右轮；只修正起步偏差，不能修正常态跑偏 */

/* ========== 内部辅助 ========== */

static float yaw_error_norm(float cur, float tgt)
{
    float e = cur - tgt;
    while (e >  180.0f) e -= 360.0f;
    while (e < -180.0f) e += 360.0f;
    return e;
}
static float yaw_norm(float yaw)
{
    while (yaw >  180.0f) yaw -= 360.0f;
    while (yaw < -180.0f) yaw += 360.0f;
    return yaw;
}


/* ========== 公开接口 ========== */

void Straight_Config(float kp, float ki, float kd, int16_t base_speed)
{
    PID_Init(&sPid, kp, ki, kd,
             -STRAIGHT_CORR_LIMIT, STRAIGHT_CORR_LIMIT,
             STRAIGHT_INTEGRAL_LIMIT);
    sBaseSpeed = base_speed;
}

static void straight_start(float target_yaw, uint32_t now_ms)
{
    sTargetYaw = target_yaw;
    PID_Reset(&sPid);
    PID_SetSetpoint(&sPid, sTargetYaw);
    SpeedControl_Start(now_ms);
    sRunning = true;
    sFirstUpdate = true;
    sRampStarted = false;
    sStartMs = now_ms;
    sLastUpdateMs = now_ms;
}

void Straight_Start(float current_yaw, uint32_t now_ms)
{
    straight_start(current_yaw, now_ms);
}

void Straight_StartToYaw(float target_yaw, uint32_t now_ms)
{
    straight_start(target_yaw, now_ms);
}

float Straight_Update(float current_yaw, uint32_t now_ms)
{
    if (!sRunning) return 0.0f;

    float err  = yaw_error_norm(current_yaw, sTargetYaw);   /* cur-tgt，用于返回显示 */
    float dt = STRAIGHT_PID_DT_DEFAULT;
    if (!sRampStarted) {
        sStartMs = now_ms;
        sRampStarted = true;
    }
    if (sFirstUpdate) {
        sPid.prev_error = -err;
        sFirstUpdate = false;
    } else {
        uint32_t elapsed_ms = now_ms - sLastUpdateMs;
        if (elapsed_ms > 0U) dt = (float)elapsed_ms * 0.001f;
        if (dt > STRAIGHT_PID_DT_MAX) dt = STRAIGHT_PID_DT_MAX;
    }
    sLastUpdateMs = now_ms;
    float corr = PID_ComputeError(&sPid, -err, dt); /* PID 使用归一化的 tgt-cur */

    int16_t base = sBaseSpeed;
    int16_t right_trim = 0;
    uint32_t ramp_elapsed_ms = now_ms - sStartMs;
    if (ramp_elapsed_ms < STRAIGHT_RAMP_MS) {
        uint32_t remaining_ms = STRAIGHT_RAMP_MS - ramp_elapsed_ms;
        base += (int16_t)((STRAIGHT_START_PWM_OFFSET * remaining_ms) /
                          STRAIGHT_RAMP_MS);
        right_trim = (int16_t)((STRAIGHT_RIGHT_PWM_TRIM * remaining_ms) /
                               STRAIGHT_RAMP_MS);
    }

    int16_t left  = base + (int16_t)corr;
    int16_t right = base - (int16_t)corr + right_trim;
    if (left  < 0) left  = 0;
    if (right < 0) right = 0;
    SpeedControl_SetCommand(left, right);

    return err;
}

void Straight_Stop(void)
{
    sRunning = false;
    sFirstUpdate = false;
    sRampStarted = false;
    SpeedControl_Stop();
}

float Straight_GetTarget(void) { return sTargetYaw; }
bool  Straight_IsRunning(void) { return sRunning; }
void TurnInPlace_Config(float angle_deg, int16_t fast_pwm, int16_t slow_pwm,
                        float slow_angle_deg, uint8_t brake_frames,
                        uint16_t timeout_frames, uint8_t stall_frames,
                        float stall_min_deg)
{
    sTurnAngleDeg = angle_deg;
    sTurnFastPwm = fast_pwm;
    sTurnSlowPwm = slow_pwm;
    sTurnSlowAngleDeg = slow_angle_deg;
    sTurnBrakeFrames = brake_frames;
    sTurnTimeoutFrames = timeout_frames;
    sTurnStallFrames = stall_frames;
    sTurnStallMinDeg = stall_min_deg;
}

void TurnInPlace_Start(float current_yaw)
{
    sTurnTarget = yaw_norm(current_yaw - sTurnAngleDeg);
    sTurnLastYaw = current_yaw;
    sTurnProgress = 0.0f;
    sTurnWatchProgress = 0.0f;
    sTurnFrames = 0;
    sTurnBrakeCount = 0;
    sTurnWatchCount = 0;
    sTurnBraking = false;
    sTurnRunning = true;
    Motor_Enable();
    Motor_SetBoth(sTurnFastPwm, -sTurnFastPwm);
}

void TurnInPlace_Stop(void)
{
    sTurnRunning = false;
    sTurnBraking = false;
    Motor_Brake();
    Motor_Standby();
}

TurnInPlaceResult_t TurnInPlace_Update(float current_yaw)
{
    bool stalled = false;

    if (!sTurnRunning) return TURN_IN_PLACE_DONE;
    if (++sTurnFrames >= sTurnTimeoutFrames) {
        TurnInPlace_Stop();
        return TURN_IN_PLACE_TIMEOUT;
    }

    if (sTurnBraking) {
        Motor_Brake();
        if (++sTurnBrakeCount >= sTurnBrakeFrames) {
            TurnInPlace_Stop();
            return TURN_IN_PLACE_DONE;
        }
        return TURN_IN_PLACE_BRAKING;
    }

    /* 实车右转时 yaw 递减；归一化后可正确跨越正负 180 度。 */
    float step = yaw_error_norm(sTurnLastYaw, current_yaw);
    sTurnLastYaw = current_yaw;
    if (step > -20.0f && step < 20.0f) sTurnProgress += step;

    if (++sTurnWatchCount >= sTurnStallFrames) {
        float window_progress = sTurnProgress - sTurnWatchProgress;
        sTurnWatchCount = 0;
        sTurnWatchProgress = sTurnProgress;
        stalled = window_progress < sTurnStallMinDeg;
    }

    if (stalled) {
        TurnInPlace_Stop();
        return TURN_IN_PLACE_STALLED;
    }
    if (sTurnProgress >= sTurnAngleDeg) {
        Motor_Brake();
        sTurnBraking = true;
        sTurnBrakeCount = 0;
        return TURN_IN_PLACE_BRAKING;
    }

    float remaining = sTurnAngleDeg - sTurnProgress;
    int16_t pwm = (remaining <= sTurnSlowAngleDeg) ?
                  sTurnSlowPwm : sTurnFastPwm;
    Motor_SetBoth(pwm, -pwm);
    return TURN_IN_PLACE_RUNNING;
}

float TurnInPlace_GetTarget(void) { return sTurnTarget; }
float TurnInPlace_GetProgress(void) { return sTurnProgress; }
bool TurnInPlace_IsRunning(void) { return sTurnRunning; }
