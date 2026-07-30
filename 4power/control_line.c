/*
 * control_line.c - 灰度循迹实现
 */
#include "control_line.h"
#include "pid.h"
#include "speed_control.h"
#include "ti_msp_dl_config.h"

/* ========== 巡线控制现场调参区 ========== */
#define LINE_CENTER              3.5f /* 8 路传感器的位置中心 0..7；安装无偏差时不要调整 */
#define LINE_PID_DT_DEFAULT    0.005f /* 首次更新的默认 dt；后续使用 SysTick 实际间隔 */
#define LINE_PID_DT_MAX        0.050f /* 主循环偶发阻塞后的 dt 上限，避免单次积分跨度过大 */
#define LINE_INTEGRAL_LIMIT     400.0f /* 积分累计限幅；当前 Ki=0，因此暂时不起作用 */
#define LINE_LOST_PWM_BASE       950   /* Line_Update(0) 的寻线速度；当前主流程未使用该分支 */

#define LINE_EDGE_ERROR           2.5f /* 误差达到此值视为压到边缘，强制使用较强转向 */

//=========更主要的===============
#define LINE_CORR_LIMIT         400.0f /* 最大左右差速修正；大误差时 PID 输出会被限制在此处 */
#define LINE_EDGE_MIN_CORR       150.0f /* 边缘状态的最小转向修正；增大可加强急弯纠偏 */
#define LINE_CURVE_PWM_PER_ERROR  80.0f /* 每 1.0 位置误差增加的减速 PWM；越大弯中越慢 */
#define LINE_CURVE_PWM_OFFSET    280   /* 弯道减速 PWM 的最大增量；越大急弯整体越慢 */

#define LINE_PWM_MIN              80   /* PWM 下限；PWM 反向，越小代表允许外侧轮驱动力越强 */
#define LINE_PWM_MAX            1800   /* PWM 上限；PWM 反向，越大代表允许内侧轮驱动力越弱 */

static PID_t  sPid;
static int16_t sBaseSpeed = 500;
static float  sError = 0.0f;
static float  sCorr  = 0.0f;
static float  sLastValidError = 0.0f;
static uint32_t sLastUpdateMs = 0;
static bool sFirstUpdate = false;

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
             -LINE_CORR_LIMIT, LINE_CORR_LIMIT,
             LINE_INTEGRAL_LIMIT);
    sBaseSpeed = base_speed;
}

void Line_Start(uint32_t now_ms)
{
    PID_Reset(&sPid);
    PID_SetSetpoint(&sPid, LINE_CENTER);
    sError = 0.0f;
    sCorr = 0.0f;
    sLastValidError = 0.0f;
    sLastUpdateMs = now_ms;
    sFirstUpdate = true;
    SpeedControl_Start(now_ms);
}

void Line_Update(uint8_t gray_map, uint32_t now_ms)
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
        float dt = LINE_PID_DT_DEFAULT;
        int16_t curve_offset;

        if (!sFirstUpdate) {
            uint32_t elapsed_ms = now_ms - sLastUpdateMs;
            if (elapsed_ms > 0U) dt = (float)elapsed_ms * 0.001f;
            if (dt > LINE_PID_DT_MAX) dt = LINE_PID_DT_MAX;
        }
        sFirstUpdate = false;
        sLastUpdateMs = now_ms;

        sError = pos - LINE_CENTER;
        sLastValidError = sError;
        sCorr = PID_Compute(&sPid, pos, dt);

        abs_error = (sError < 0.0f) ? -sError : sError;
        if (sError >= LINE_EDGE_ERROR && sCorr > -LINE_EDGE_MIN_CORR) {
            sCorr = -LINE_EDGE_MIN_CORR;
        } else if (sError <= -LINE_EDGE_ERROR &&
                   sCorr < LINE_EDGE_MIN_CORR) {
            sCorr = LINE_EDGE_MIN_CORR;
        }

        /* This PWM is inverted: a larger compare value means less drive. */
        curve_offset = (int16_t)(abs_error * LINE_CURVE_PWM_PER_ERROR);
        if (curve_offset > LINE_CURVE_PWM_OFFSET) {
            curve_offset = LINE_CURVE_PWM_OFFSET;
        }
        drive_base += curve_offset;
    }

    int16_t left  = line_clamp_pwm(drive_base + (int16_t)sCorr);
    int16_t right = line_clamp_pwm(drive_base - (int16_t)sCorr);
    SpeedControl_SetCommand(left, right);
}

void Line_Stop(void)
{
    SpeedControl_Stop();
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
