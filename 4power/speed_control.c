/*
 * speed_control.c - 四轮编码器独立速度闭环实现
 */
#include "speed_control.h"

#include "encoder.h"
#include "motor.h"

#define SPEED_CLOSED_LOOP_ENABLE          1 /* 1=启用四轮速度闭环；0=直接输出原始指令。 */
#define SPEED_CONTROL_PERIOD_MS          20U /* 速度 PI 采样周期，20 ms 对应 50 Hz。 */
#define SPEED_PI_DT_MAX               0.100f /* 阻塞后参与积分的最大 dt，避免积分突变。 */
#define SPEED_FULL_OUTPUT_PPS        5680.0f /* 最强输出时估计的 4 倍频脉冲/秒，需按实车校准。 */

#define SPEED_A_TARGET_SCALE             1.0f /* A 左前轮目标速度比例。 */
#define SPEED_B_TARGET_SCALE             1.0f /* B 左后轮目标速度比例。 */
#define SPEED_C_TARGET_SCALE             1.0f /* C 右后轮目标速度比例。 */
#define SPEED_D_TARGET_SCALE             1.0f /* D 右前轮目标速度比例。 */

#define SPEED_KP                         0.08f /* 速度 PI 比例系数。 */
#define SPEED_KI                         0.15f /* 速度 PI 积分系数。 */
#define SPEED_INTEGRAL_LIMIT          1800.0f /* 积分项限幅，单位为脉冲/秒。 */
#define SPEED_CORRECTION_LIMIT         350.0f /* PI 对原始 PWM 的最大修正量。 */
#define SPEED_FILTER_ALPHA               0.40f /* 新样本滤波权重，越小越平滑、响应越慢。 */
#define SPEED_OUTPUT_PWM_MIN                80 /* 闭环允许的最强 PWM 比较值。 */
#define SPEED_OUTPUT_PWM_MAX              1950 /* 闭环允许的最弱 PWM 比较值。 */

#define SPEED_STALL_PROTECTION_ENABLE        1 /* 1=任一车轮无脉冲时停车；0=关闭保护。 */
#define SPEED_STALL_TARGET_MIN_PPS       600.0f /* 高于该目标速度时才进行堵转检测。 */
#define SPEED_STALL_TIMEOUT_MS             500U /* 单轮连续无脉冲达到该时间即判定堵转。 */

typedef struct {
    int16_t command;
    int16_t output;
    float target_pps;
    float measured_pps;
    float integral;
    float correction;
    uint32_t last_pulse_ms;
} SpeedWheel_t;

static SpeedWheel_t sWheels[MOTOR_WHEEL_COUNT];
static int32_t sLastCounts[ENCODER_COUNT];
static uint32_t sLastUpdateMs;
static bool sRunning;
static bool sFirstSample;
static SpeedControlFault_t sFault;

static const float sTargetScales[MOTOR_WHEEL_COUNT] = {
    SPEED_A_TARGET_SCALE, SPEED_B_TARGET_SCALE,
    SPEED_C_TARGET_SCALE, SPEED_D_TARGET_SCALE
};

static const SpeedControlFault_t sStallFaults[MOTOR_WHEEL_COUNT] = {
    SPEED_CONTROL_FAULT_WHEEL_A_STALL,
    SPEED_CONTROL_FAULT_WHEEL_B_STALL,
    SPEED_CONTROL_FAULT_WHEEL_C_STALL,
    SPEED_CONTROL_FAULT_WHEEL_D_STALL
};

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
    float pwm;

    if (sign == 0 || wheel->target_pps <= 0.0f) return 0;
#if SPEED_CLOSED_LOOP_ENABLE
    pwm = (float)speed_abs_command(wheel->command) - wheel->correction;
    if (pwm < SPEED_OUTPUT_PWM_MIN) pwm = SPEED_OUTPUT_PWM_MIN;
    if (pwm > SPEED_OUTPUT_PWM_MAX) pwm = SPEED_OUTPUT_PWM_MAX;
    return (int16_t)(sign * (int16_t)pwm);
#else
    (void)pwm;
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

static void speed_set_wheel_command(uint32_t index, int16_t command)
{
    SpeedWheel_t *wheel = &sWheels[index];
    int8_t old_sign = speed_command_sign(wheel->command);
    int8_t new_sign = speed_command_sign(command);

    if (new_sign != old_sign) {
        wheel->integral = 0.0f;
        wheel->correction = 0.0f;
    }
    wheel->command = command;
    wheel->target_pps = speed_target_from_command(command,
                                                   sTargetScales[index]);
    if (wheel->target_pps <= 0.0f) {
        wheel->integral = 0.0f;
        wheel->correction = 0.0f;
    }
    wheel->output = speed_output_from_correction(wheel);
}

static void speed_update_wheel(uint32_t index, int32_t delta,
                               float sample_dt, float pi_dt,
                               uint32_t now_ms)
{
    SpeedWheel_t *wheel = &sWheels[index];
    float pulses = (delta < 0) ? -(float)delta : (float)delta;
    float sample_pps = pulses / sample_dt;

    if (sFirstSample) wheel->measured_pps = sample_pps;
    else wheel->measured_pps += SPEED_FILTER_ALPHA *
                                (sample_pps - wheel->measured_pps);

#if SPEED_STALL_PROTECTION_ENABLE
    if (delta != 0 || wheel->target_pps < SPEED_STALL_TARGET_MIN_PPS) {
        wheel->last_pulse_ms = now_ms;
    } else if ((now_ms - wheel->last_pulse_ms) >= SPEED_STALL_TIMEOUT_MS &&
               sFault == SPEED_CONTROL_FAULT_NONE) {
        sFault = sStallFaults[index];
    }
#else
    wheel->last_pulse_ms = now_ms;
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

static void speed_apply_outputs(void)
{
    Motor_SetFour(sWheels[0].output, sWheels[1].output,
                  sWheels[2].output, sWheels[3].output);
}

static void speed_reset_all(uint32_t now_ms)
{
    uint32_t i;
    for (i = 0; i < MOTOR_WHEEL_COUNT; i++) {
        speed_reset_wheel(&sWheels[i], now_ms);
    }
    Encoder_GetCounts(sLastCounts);
    sLastUpdateMs = now_ms;
    sFirstSample = true;
}

void SpeedControl_Init(uint32_t now_ms)
{
    speed_reset_all(now_ms);
    sRunning = false;
    sFault = SPEED_CONTROL_FAULT_NONE;
}

void SpeedControl_Start(uint32_t now_ms)
{
    speed_reset_all(now_ms);
    sFault = SPEED_CONTROL_FAULT_NONE;
    sRunning = true;
    Motor_Enable();
}

void SpeedControl_SetCommand(int16_t left, int16_t right)
{
    if (!sRunning || sFault != SPEED_CONTROL_FAULT_NONE) return;
    speed_set_wheel_command(MOTOR_WHEEL_A_LEFT_FRONT, left);
    speed_set_wheel_command(MOTOR_WHEEL_B_LEFT_REAR, left);
    speed_set_wheel_command(MOTOR_WHEEL_C_RIGHT_REAR, right);
    speed_set_wheel_command(MOTOR_WHEEL_D_RIGHT_FRONT, right);
    speed_apply_outputs();
}

bool SpeedControl_Update(uint32_t now_ms)
{
    int32_t counts[ENCODER_COUNT];
    uint32_t elapsed_ms;
    uint32_t i;
    float sample_dt;
    float pi_dt;

    if (!sRunning || sFault != SPEED_CONTROL_FAULT_NONE) return false;
    elapsed_ms = now_ms - sLastUpdateMs;
    if (elapsed_ms < SPEED_CONTROL_PERIOD_MS) return false;

    Encoder_GetCounts(counts);
    sample_dt = (float)elapsed_ms * 0.001f;
    pi_dt = sample_dt;
    if (pi_dt > SPEED_PI_DT_MAX) pi_dt = SPEED_PI_DT_MAX;
    for (i = 0; i < MOTOR_WHEEL_COUNT; i++) {
        speed_update_wheel(i, counts[i] - sLastCounts[i],
                           sample_dt, pi_dt, now_ms);
        sLastCounts[i] = counts[i];
    }
    sLastUpdateMs = now_ms;
    sFirstSample = false;

    if (sFault != SPEED_CONTROL_FAULT_NONE) {
        sRunning = false;
        Motor_Brake();
        Motor_Standby();
        return true;
    }
    speed_apply_outputs();
    return true;
}

void SpeedControl_Stop(void)
{
    uint32_t i;
    sRunning = false;
    for (i = 0; i < MOTOR_WHEEL_COUNT; i++) {
        speed_reset_wheel(&sWheels[i], sLastUpdateMs);
    }
    Motor_Brake();
    Motor_Standby();
}

bool SpeedControl_IsRunning(void) { return sRunning; }
SpeedControlFault_t SpeedControl_GetFault(void) { return sFault; }
void SpeedControl_ClearFault(void) { sFault = SPEED_CONTROL_FAULT_NONE; }

int32_t SpeedControl_GetTarget(uint32_t wheel)
{
    return (wheel < MOTOR_WHEEL_COUNT) ?
           (int32_t)sWheels[wheel].target_pps : 0;
}

int32_t SpeedControl_GetMeasured(uint32_t wheel)
{
    return (wheel < MOTOR_WHEEL_COUNT) ?
           (int32_t)sWheels[wheel].measured_pps : 0;
}

int16_t SpeedControl_GetOutput(uint32_t wheel)
{
    return (wheel < MOTOR_WHEEL_COUNT) ? sWheels[wheel].output : 0;
}
