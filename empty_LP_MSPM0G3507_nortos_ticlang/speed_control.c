/*
 * speed_control.c - 双轮编码器速度闭环实现
 */
#include "speed_control.h"

#include "encoder.h"
#include "motor.h"

/* ========== 速度闭环现场调参区 ========== */
#define SPEED_CLOSED_LOOP_ENABLE          1 /* 1=开启速度闭环；0=保留原始开环 PWM，便于对照测试 */
#define SPEED_CONTROL_PERIOD_MS          20U /* 速度 PI 周期；20 ms 对应 50 Hz */
#define SPEED_PI_DT_MAX               0.100f /* 阻塞后的积分 dt 上限，避免积分突然增大 */

/* PWM=0 对应最大理论输出，PWM=2000 对应最小理论输出。 */
#define SPEED_FULL_OUTPUT_PPS        5680.0f /* 满输出对应的估计编码器脉冲/秒；应先按实测校准 */
#define SPEED_LEFT_TARGET_SCALE         1.0f /* 左轮目标比例；两侧编码器规格相同时保持 1 */
#define SPEED_RIGHT_TARGET_SCALE        1.0f /* 右轮目标比例；两侧编码器规格相同时保持 1 */

#define SPEED_KP                        0.08f /* 比例：速度误差每 1 脉冲/秒产生的 PWM 修正 */
#define SPEED_KI                        0.15f /* 积分：消除负载和电池变化造成的持续速度误差 */
#define SPEED_INTEGRAL_LIMIT         1800.0f /* 积分误差限幅，单位为 脉冲/秒*秒 */
#define SPEED_CORRECTION_LIMIT        350.0f /* PI 对原始 PWM 的最大修正量 */

#define SPEED_FILTER_ALPHA              0.40f /* 新速度样本权重；越小越平滑但响应越慢 */
#define SPEED_OUTPUT_PWM_MIN               80 /* 闭环运行时最强输出限制；越小驱动力越强 */
#define SPEED_OUTPUT_PWM_MAX             1950 /* 闭环运行时最弱输出限制；越大驱动力越弱 */

#define SPEED_STALL_PROTECTION_ENABLE       1 /* 1=启用单轮无脉冲停车保护；0=关闭 */
#define SPEED_STALL_TARGET_MIN_PPS      600.0f /* 目标高于此值才检查，避免急弯内侧低速轮误报 */
#define SPEED_STALL_TIMEOUT_MS            500U /* 单轮连续无脉冲达到此时间后停车 */

typedef struct {
    int16_t command;
    int16_t output;
    float target_pps;
    float measured_pps;
    float integral;
    float correction;
    uint32_t last_pulse_ms;
} SpeedWheel_t;

static SpeedWheel_t sLeft;
static SpeedWheel_t sRight;
static int32_t sLastLeftCount = 0;
static int32_t sLastRightCount = 0;
static uint32_t sLastUpdateMs = 0;
static bool sRunning = false;
static bool sFirstSample = true;
static SpeedControlFault_t sFault = SPEED_CONTROL_FAULT_NONE;

static int16_t speed_abs_command(int16_t command)
{
    int32_t magnitude = command;
    if (magnitude < 0) magnitude = -magnitude;
    if (magnitude > MOTOR_PWM_MAX) magnitude = MOTOR_PWM_MAX;
    return (int16_t)magnitude;
}

static int8_t speed_command_sign(int16_t command)
{
    if (command > 0) return 1;
    if (command < 0) return -1;
    return 0;
}

static float speed_target_from_command(int16_t command, float scale)
{
    int16_t pwm = speed_abs_command(command);
    if (pwm == 0 || pwm >= MOTOR_PWM_MAX) return 0.0f;

    return ((float)(MOTOR_PWM_MAX - pwm) / (float)MOTOR_PWM_MAX) *
           SPEED_FULL_OUTPUT_PPS * scale;
}

static int16_t speed_output_from_correction(const SpeedWheel_t *wheel)
{
    int8_t sign = speed_command_sign(wheel->command);
    if (sign == 0 || wheel->target_pps <= 0.0f) return 0;

#if SPEED_CLOSED_LOOP_ENABLE
    float pwm = (float)speed_abs_command(wheel->command) - wheel->correction;
    if (pwm < SPEED_OUTPUT_PWM_MIN) pwm = SPEED_OUTPUT_PWM_MIN;
    if (pwm > SPEED_OUTPUT_PWM_MAX) pwm = SPEED_OUTPUT_PWM_MAX;
    return (int16_t)(sign * (int16_t)pwm);
#else
    return wheel->command;
#endif
}

static void speed_reset_wheel(SpeedWheel_t *wheel, uint32_t now_ms)
{
    wheel->command = 0;
    wheel->output = 0;
    wheel->target_pps = 0.0f;
    wheel->measured_pps = 0.0f;
    wheel->integral = 0.0f;
    wheel->correction = 0.0f;
    wheel->last_pulse_ms = now_ms;
}

static void speed_set_wheel_command(SpeedWheel_t *wheel, int16_t command,
                                    float target_scale)
{
    int8_t old_sign = speed_command_sign(wheel->command);
    int8_t new_sign = speed_command_sign(command);

    if (new_sign != old_sign) {
        wheel->integral = 0.0f;
        wheel->correction = 0.0f;
    }

    wheel->command = command;
    wheel->target_pps = speed_target_from_command(command, target_scale);
    if (wheel->target_pps <= 0.0f) {
        wheel->integral = 0.0f;
        wheel->correction = 0.0f;
    }
    wheel->output = speed_output_from_correction(wheel);
}

static void speed_update_wheel(SpeedWheel_t *wheel, int32_t delta,
                               float sample_dt, float pi_dt,
                               uint32_t now_ms,
                               SpeedControlFault_t stall_fault)
{
    float pulse_count = (delta < 0) ? -(float)delta : (float)delta;
    float sample_pps = pulse_count / sample_dt;

    if (sFirstSample) {
        wheel->measured_pps = sample_pps;
    } else {
        wheel->measured_pps += SPEED_FILTER_ALPHA *
                               (sample_pps - wheel->measured_pps);
    }

#if SPEED_STALL_PROTECTION_ENABLE
    if (delta != 0 || wheel->target_pps < SPEED_STALL_TARGET_MIN_PPS) {
        wheel->last_pulse_ms = now_ms;
    } else if ((now_ms - wheel->last_pulse_ms) >= SPEED_STALL_TIMEOUT_MS &&
               sFault == SPEED_CONTROL_FAULT_NONE) {
        sFault = stall_fault;
    }
#else
    wheel->last_pulse_ms = now_ms;
    (void)stall_fault;
#endif

#if SPEED_CLOSED_LOOP_ENABLE
    if (wheel->target_pps > 0.0f) {
        float error = wheel->target_pps - wheel->measured_pps;
        wheel->integral += error * pi_dt;
        if (wheel->integral > SPEED_INTEGRAL_LIMIT) {
            wheel->integral = SPEED_INTEGRAL_LIMIT;
        }
        if (wheel->integral < -SPEED_INTEGRAL_LIMIT) {
            wheel->integral = -SPEED_INTEGRAL_LIMIT;
        }

        wheel->correction = SPEED_KP * error + SPEED_KI * wheel->integral;
        if (wheel->correction > SPEED_CORRECTION_LIMIT) {
            wheel->correction = SPEED_CORRECTION_LIMIT;
        }
        if (wheel->correction < -SPEED_CORRECTION_LIMIT) {
            wheel->correction = -SPEED_CORRECTION_LIMIT;
        }
    } else {
        wheel->integral = 0.0f;
        wheel->correction = 0.0f;
    }
#else
    wheel->integral = 0.0f;
    wheel->correction = 0.0f;
#endif

    wheel->output = speed_output_from_correction(wheel);
}

void SpeedControl_Init(uint32_t now_ms)
{
    speed_reset_wheel(&sLeft, now_ms);
    speed_reset_wheel(&sRight, now_ms);
    Encoder_GetCounts(&sLastLeftCount, &sLastRightCount);
    sLastUpdateMs = now_ms;
    sRunning = false;
    sFirstSample = true;
    sFault = SPEED_CONTROL_FAULT_NONE;
}

void SpeedControl_Start(uint32_t now_ms)
{
    speed_reset_wheel(&sLeft, now_ms);
    speed_reset_wheel(&sRight, now_ms);
    Encoder_GetCounts(&sLastLeftCount, &sLastRightCount);
    sLastUpdateMs = now_ms;
    sFirstSample = true;
    sFault = SPEED_CONTROL_FAULT_NONE;
    sRunning = true;
    Motor_Enable();
}

void SpeedControl_SetCommand(int16_t left, int16_t right)
{
    if (!sRunning || sFault != SPEED_CONTROL_FAULT_NONE) return;

    speed_set_wheel_command(&sLeft, left, SPEED_LEFT_TARGET_SCALE);
    speed_set_wheel_command(&sRight, right, SPEED_RIGHT_TARGET_SCALE);
    Motor_SetBoth(sLeft.output, sRight.output);
}

bool SpeedControl_Update(uint32_t now_ms)
{
    uint32_t elapsed_ms;
    int32_t left_count;
    int32_t right_count;
    float sample_dt;
    float pi_dt;

    if (!sRunning || sFault != SPEED_CONTROL_FAULT_NONE) return false;

    elapsed_ms = now_ms - sLastUpdateMs;
    if (elapsed_ms < SPEED_CONTROL_PERIOD_MS) return false;

    Encoder_GetCounts(&left_count, &right_count);
    sample_dt = (float)elapsed_ms * 0.001f;
    pi_dt = sample_dt;
    if (pi_dt > SPEED_PI_DT_MAX) pi_dt = SPEED_PI_DT_MAX;

    speed_update_wheel(&sLeft, left_count - sLastLeftCount,
                       sample_dt, pi_dt, now_ms,
                       SPEED_CONTROL_FAULT_LEFT_STALL);
    speed_update_wheel(&sRight, right_count - sLastRightCount,
                       sample_dt, pi_dt, now_ms,
                       SPEED_CONTROL_FAULT_RIGHT_STALL);

    sLastLeftCount = left_count;
    sLastRightCount = right_count;
    sLastUpdateMs = now_ms;
    sFirstSample = false;

    if (sFault != SPEED_CONTROL_FAULT_NONE) {
        sRunning = false;
        Motor_Brake();
        Motor_Standby();
        return true;
    }

    Motor_SetBoth(sLeft.output, sRight.output);
    return true;
}

void SpeedControl_Stop(void)
{
    sRunning = false;
    sLeft.command = 0;
    sRight.command = 0;
    sLeft.output = 0;
    sRight.output = 0;
    sLeft.target_pps = 0.0f;
    sRight.target_pps = 0.0f;
    sLeft.integral = 0.0f;
    sRight.integral = 0.0f;
    sLeft.correction = 0.0f;
    sRight.correction = 0.0f;
    Motor_Brake();
    Motor_Standby();
}

bool SpeedControl_IsRunning(void) { return sRunning; }
SpeedControlFault_t SpeedControl_GetFault(void) { return sFault; }
void SpeedControl_ClearFault(void) { sFault = SPEED_CONTROL_FAULT_NONE; }

int32_t SpeedControl_GetLeftTarget(void) { return (int32_t)sLeft.target_pps; }
int32_t SpeedControl_GetRightTarget(void) { return (int32_t)sRight.target_pps; }
int32_t SpeedControl_GetLeftMeasured(void) { return (int32_t)sLeft.measured_pps; }
int32_t SpeedControl_GetRightMeasured(void) { return (int32_t)sRight.measured_pps; }
int16_t SpeedControl_GetLeftOutput(void) { return sLeft.output; }
int16_t SpeedControl_GetRightOutput(void) { return sRight.output; }
