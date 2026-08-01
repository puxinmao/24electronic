/*
 * empty.c - 四轮速度闭环与 KEY1 循迹主程序。
 *
 * KEY1/KEY2 均按黑线循迹；KEY1 可按丢线/累计一圈停车，
 * KEY2 以陀螺仪角速度识别进入半圆后停车。
 */
#include "ti_msp_dl_config.h"

#include "button.h"
#include "control_line.h"
#include "encoder.h"
#include "gray.h"
#include "motor.h"
#include "oled.h"
#include "speed_control.h"
#include "wit.h"

#define SENSOR_DISPLAY_PERIOD_MS  100U  /* OLED 与传感器状态刷新周期。 */
#define SYSTICK_PERIOD_MS         1U   /* 系统 SysTick 的计时单位。 */
 
#define LINE_CONTROL_PERIOD_MS      5U      /* 循迹 PID 更新周期。 */
#define LINE_LOST_STOP_MS           300U   /* 连续丢失黑线后停车的确认时间。 */
#define LINE_KP                     250.0f /* 循迹 PID 比例系数。 */
#define LINE_KI                      0.0f   /* 循迹 PID 积分系数。 */
#define LINE_KD                      3.0f   /* 循迹 PID 微分系数。 */
#define LINE_BASE_SPEED             1400   /* 循迹基础 PWM，数值越小速度越快。 */
#define YAW_STOP_DEGREES             360.0f  /* 累计转过一圈后停车。 */
#define KEY1_START_PWM                1650  /* KEY1 渐变起点；PWM 反向，值越大起步越慢。 */
#define KEY1_START_PWM_STEP              1  /* 每个循迹周期提升的速度参数；值越大加速越快。 */
#define KEY1_DECEL_PWM       MOTOR_PWM_MAX /* KEY1 减速终点；达到后两侧电机指令为零。 */
#define KEY1_DECEL_PWM_STEP             1  /* 每个循迹周期的减速步进；值越大减速越快。 */

#define KEY2_STOP_TIME_MS              6500U /* KEY2 起步后的定时停车时间。 */
#define KEY2_START_PWM                1650  /* KEY2 渐变起点；PWM 反向，值越大起步越慢。 */
#define KEY2_START_PWM_STEP              1  /* 每个循迹周期提升的速度参数；值越大加速越快。 */
#define KEY2_DECEL_START_MS           5000U /* KEY2 起步后开始缓慢减速的时间。 */
#define KEY2_DECEL_PWM                1650  /* KEY2 减速目标；PWM 反向，值越大速度越慢。 */
#define KEY2_DECEL_PWM_STEP              1  /* 每个循迹周期的减速步进；值越大减速越快。 */

#define KEY3_LINE_KP                 800.0f /* KEY3 独立循迹 PID 比例系数。 */
#define KEY3_LINE_KI                   0.0f /* KEY3 独立循迹 PID 积分系数。 */
#define KEY3_LINE_KD                   3.0f /* KEY3 独立循迹 PID 微分系数。 */
#define KEY3_LINE_PWM                  800  /* KEY3 独立循迹速度；PWM 反向，数值越小速度越快，且不参与减速。 */
#define KEY3_CORRECTION_LIMIT        700.0f /* KEY3 独立循迹最大左右纠偏量。 */
#define KEY3_EDGE_MIN_CORRECTION     300.0f /* KEY3 压到传感器边缘时的最小纠偏量。 */
#define KEY3_DECEL_START_MS          10000U /* KEY3 起步后开始减速并允许识别停车线的时间。 */
#define KEY3_DECEL_PWM                1800  /* KEY3 减速后的循迹速度；PWM 反向，值越大速度越慢。 */
#define KEY3_DECEL_PWM_STEP              4  /* KEY3 每个循迹周期的减速步进。 */
#define KEY3_STOP_BLACK_COUNT            4U /* 至少 4 个黑线传感器检测到黑线时停车。 */
#define KEY3_YAW_STOP_DEGREES         360.0f /* KEY3 陀螺仪累计转满一圈的兜底停车阈值。 */

static volatile uint32_t sSystemTickMs;

void SysTick_Handler(void)
{
    sSystemTickMs++;
}

static uint32_t get_system_tick_ms(void)
{
    return sSystemTickMs;
}

static float yaw_delta_degrees(float current_yaw, float previous_yaw)
{
    float delta = current_yaw - previous_yaw;

    /* JY901S yaw wraps at -180/180 degrees. */
    while (delta > 180.0f) delta -= 360.0f;
    while (delta < -180.0f) delta += 360.0f;
    return delta;
}

static void format_elapsed_time(char line[17], uint32_t elapsed_ms)
{
    uint32_t centiseconds = elapsed_ms / 10U;
    uint32_t minutes = centiseconds / 6000U;
    uint32_t seconds = (centiseconds / 100U) % 60U;
    uint32_t hundredths = centiseconds % 100U;

    line[0] = 'T';
    line[1] = 'I';
    line[2] = 'M';
    line[3] = 'E';
    line[4] = ':';
    line[5] = (char)('0' + (minutes / 10U) % 10U);
    line[6] = (char)('0' + minutes % 10U);
    line[7] = ':';
    line[8] = (char)('0' + seconds / 10U);
    line[9] = (char)('0' + seconds % 10U);
    line[10] = '.';
    line[11] = (char)('0' + hundredths / 10U);
    line[12] = (char)('0' + hundredths % 10U);
    line[13] = ' ';
    line[14] = ' ';
    line[15] = ' ';
    line[16] = '\0';
}

static void format_run_status(char line[17], bool line_tracking,
                               bool line_lost, uint32_t lost_ms,
                               bool key2_tracking)
{
    uint8_t i;

    for (i = 0U; i < 16U; i++) {
        line[i] = ' ';
    }
    line[16] = '\0';

    if (line_lost) {
        uint32_t remaining_ms = (lost_ms >= LINE_LOST_STOP_MS) ? 0U :
                                LINE_LOST_STOP_MS - lost_ms;
        line[0] = 'L';
        line[1] = 'O';
        line[2] = 'S';
        line[3] = 'T';
        line[4] = ':';
        line[5] = (char)('0' + (remaining_ms / 1000U) % 10U);
        line[6] = '.';
        line[7] = (char)('0' + (remaining_ms / 100U) % 10U);
        line[8] = 'S';
    } else if (line_tracking || key2_tracking) {
        line[0] = 'L';
        line[1] = 'I';
        line[2] = 'N';
        line[3] = 'E';
        line[4] = ':';
        line[5] = 'O';
        line[6] = 'K';
    } else {
        line[0] = 'L';
        line[1] = 'I';
        line[2] = 'N';
        line[3] = 'E';
        line[4] = ':';
        line[5] = 'S';
        line[6] = 'T';
        line[7] = 'O';
        line[8] = 'P';
    }
}

static void format_gray(char line[20], uint8_t gray_map)
{
    uint8_t bit;

    line[0] = 'G';
    line[1] = 'R';
    line[2] = 'A';
    line[3] = 'Y';
    line[4] = ':';
    for (bit = 0U; bit < 8U; bit++) {
        line[5U + bit] = ((gray_map & (1U << (7U - bit))) != 0U) ? '1' : '0';
    }
    line[13] = ' ';
    line[14] = 'N';
    line[15] = ':';
    line[16] = (char)('0' + Gray_BlackCount(gray_map));
    line[17] = ' ';
    line[18] = ' ';
    line[19] = '\0';
}

static void show_status(bool line_tracking, bool key2_tracking,
                        bool key3_tracking, bool imu_seen,
                        SpeedControlFault_t speed_fault)
{
    if (speed_fault == SPEED_CONTROL_FAULT_WHEEL_A_STALL) {
        OLED_ShowString(0, 0, (uint8_t *)"SPD FAULT: A", 8);
    } else if (speed_fault == SPEED_CONTROL_FAULT_WHEEL_B_STALL) {
        OLED_ShowString(0, 0, (uint8_t *)"SPD FAULT: B", 8);
    } else if (speed_fault == SPEED_CONTROL_FAULT_WHEEL_C_STALL) {
        OLED_ShowString(0, 0, (uint8_t *)"SPD FAULT: C", 8);
    } else if (speed_fault == SPEED_CONTROL_FAULT_WHEEL_D_STALL) {
        OLED_ShowString(0, 0, (uint8_t *)"SPD FAULT: D", 8);
    } else if (line_tracking) {
        OLED_ShowString(0, 0, (uint8_t *)"KEY1: LINE TRACK", 8);
    } else if (key2_tracking) {
        OLED_ShowString(0, 0, (uint8_t *)"KEY2: LINE TRACK", 8);
    } else if (key3_tracking) {
        OLED_ShowString(0, 0, (uint8_t *)"KEY3: LINE TRACK", 8);
    } else if (!imu_seen) {
        OLED_ShowString(0, 0, (uint8_t *)"KEY2: IMU WAIT ", 8);
    } else {
        OLED_ShowString(0, 0, (uint8_t *)"KEY1: READY     ", 8);
    }
}

int main(void)
{
    WIT_Data_t imu_data;
    uint8_t gray_map;
    bool line_tracking = false;
    bool key2_tracking = false;
    bool key3_tracking = false;
    bool key3_stop_detected = false;
    bool line_lost = false;
    bool imu_seen = false;
    bool yaw_sample_valid = false;
    bool key1_decelerating = false;
    uint32_t line_lost_start_ms = 0U;
    uint32_t run_start_ms = 0U;
    uint32_t frozen_elapsed_ms = 0U;
    uint32_t last_line_update_ms = 0U;
    uint32_t last_display_update_ms = 0U;
    int16_t key1_base_speed = KEY1_START_PWM;
    int16_t key2_base_speed = KEY2_START_PWM;
    int16_t key3_base_speed = KEY3_LINE_PWM;
    float last_yaw = 0.0f;
    float latest_yaw = 0.0f;
    float accumulated_yaw_degrees = 0.0f;
    char time_line[17];
    char gray_line[20];
    char line_line[17];

    SYSCFG_DL_init();
    SysTick_Config(CPUCLK_FREQ / (1000U / SYSTICK_PERIOD_MS));
    Motor_Init();
    Motor_Standby();
    Encoder_Init();
    SpeedControl_Init(get_system_tick_ms());
    Gray_Init();
    WIT_Init();

    OLED_Init();
    show_status(false, false, false, false, SPEED_CONTROL_FAULT_NONE);

    while (1) {
        uint32_t now = get_system_tick_ms();

        if (WIT_GetData(&imu_data)) {
            latest_yaw = imu_data.yaw;
            imu_seen = true;

            if (line_tracking || key3_tracking) {
                if (!yaw_sample_valid) {
                    last_yaw = imu_data.yaw;
                    yaw_sample_valid = true;
                } else {
                    float delta = yaw_delta_degrees(imu_data.yaw, last_yaw);

                    last_yaw = imu_data.yaw;
                    accumulated_yaw_degrees += delta;
                }
            }
        }

        if (!line_tracking && !key2_tracking && !key3_tracking && KEY1_PRESSED) {
            now = get_system_tick_ms();
            Line_Config(LINE_KP, LINE_KI, LINE_KD, LINE_BASE_SPEED);
            Line_Start(now);
            last_line_update_ms = now;
            line_lost = false;
            run_start_ms = now;
            frozen_elapsed_ms = 0U;
            yaw_sample_valid = imu_seen;
            last_yaw = latest_yaw;
            accumulated_yaw_degrees = 0.0f;
            key1_decelerating = false;
            key1_base_speed = KEY1_START_PWM;
            line_tracking = true;
            format_elapsed_time(time_line, 0U);
            OLED_ShowString(0, 2, (uint8_t *)time_line, 8);
            show_status(true, false, false, imu_seen, SPEED_CONTROL_FAULT_NONE);
        }

        if (!line_tracking && !key2_tracking && !key3_tracking && KEY2_PRESSED) {
            if (imu_seen) {
                now = get_system_tick_ms();
                Line_Config(LINE_KP, LINE_KI, LINE_KD, LINE_BASE_SPEED);
                Line_Start(now);
                last_line_update_ms = now;
                run_start_ms = now;
                frozen_elapsed_ms = 0U;
                key2_base_speed = KEY2_START_PWM;
                key2_tracking = true;
                format_elapsed_time(time_line, 0U);
                OLED_ShowString(0, 2, (uint8_t *)time_line, 8);
                show_status(false, true, false, imu_seen, SPEED_CONTROL_FAULT_NONE);
            } else {
                show_status(false, false, false, false, SPEED_CONTROL_FAULT_NONE);
            }
        }

        if (!line_tracking && !key2_tracking && !key3_tracking && KEY3_PRESSED) {
            now = get_system_tick_ms();
            Line_ConfigWithCorrectionLimits(KEY3_LINE_KP, KEY3_LINE_KI,
                                            KEY3_LINE_KD, KEY3_LINE_PWM,
                                            KEY3_CORRECTION_LIMIT,
                                            KEY3_EDGE_MIN_CORRECTION);
            Line_Start(now);
            last_line_update_ms = now;
            line_lost = false;
            run_start_ms = now;
            frozen_elapsed_ms = 0U;
            key3_stop_detected = false;
            yaw_sample_valid = imu_seen;
            last_yaw = latest_yaw;
            accumulated_yaw_degrees = 0.0f;
            key3_base_speed = KEY3_LINE_PWM;
            key3_tracking = true;
            format_elapsed_time(time_line, 0U);
            OLED_ShowString(0, 2, (uint8_t *)time_line, 8);
            show_status(false, false, true, imu_seen, SPEED_CONTROL_FAULT_NONE);
        }

        if ((line_tracking || key2_tracking || key3_tracking) &&
            (now - last_line_update_ms) >= LINE_CONTROL_PERIOD_MS) {
            last_line_update_ms = now;
            gray_map = Gray_ReadAll();

            if (line_tracking) {
                if (key1_decelerating) {
                    if (key1_base_speed < KEY1_DECEL_PWM) {
                        key1_base_speed += KEY1_DECEL_PWM_STEP;
                        if (key1_base_speed > KEY1_DECEL_PWM) {
                            key1_base_speed = KEY1_DECEL_PWM;
                        }
                    }
                    Line_SetCorrectionScale(
                        (float)(KEY1_DECEL_PWM - key1_base_speed) /
                        (float)(KEY1_DECEL_PWM - LINE_BASE_SPEED));
                } else if (key1_base_speed > LINE_BASE_SPEED) {
                    key1_base_speed -= KEY1_START_PWM_STEP;
                    if (key1_base_speed < LINE_BASE_SPEED) {
                        key1_base_speed = LINE_BASE_SPEED;
                    }
                    Line_SetCorrectionScale(1.0f);
                }
                Line_SetBaseSpeed(key1_base_speed);
            } else if (key3_tracking) {
                if ((now - run_start_ms) >= KEY3_DECEL_START_MS &&
                    key3_base_speed < KEY3_DECEL_PWM) {
                    key3_base_speed += KEY3_DECEL_PWM_STEP;
                    if (key3_base_speed > KEY3_DECEL_PWM) {
                        key3_base_speed = KEY3_DECEL_PWM;
                    }
                }
                Line_SetCorrectionScale(1.0f);
                Line_SetBaseSpeed(key3_base_speed);
            } else if (key2_tracking) {
                if ((now - run_start_ms) >= KEY2_DECEL_START_MS) {
                    if (key2_base_speed < KEY2_DECEL_PWM) {
                        key2_base_speed += KEY2_DECEL_PWM_STEP;
                        if (key2_base_speed > KEY2_DECEL_PWM) {
                            key2_base_speed = KEY2_DECEL_PWM;
                        }
                    }
                } else if (key2_base_speed > LINE_BASE_SPEED) {
                    key2_base_speed -= KEY2_START_PWM_STEP;
                    if (key2_base_speed < LINE_BASE_SPEED) {
                        key2_base_speed = LINE_BASE_SPEED;
                    }
                }
                Line_SetBaseSpeed(key2_base_speed);
            }

            if (key3_tracking &&
                (now - run_start_ms) >= KEY3_DECEL_START_MS &&
                Gray_BlackCount(gray_map) >= KEY3_STOP_BLACK_COUNT) {
                key3_stop_detected = true;
            }

            if ((line_tracking || key3_tracking) && gray_map == 0U) {
                if (!line_lost) {
                    line_lost = true;
                    line_lost_start_ms = now;
                }
            } else if (line_tracking || key3_tracking) {
                line_lost = false;
            }

            Line_Update(gray_map, now);
        }

        if (line_tracking && !key1_decelerating && line_lost &&
            (now - line_lost_start_ms) >= LINE_LOST_STOP_MS) {
            Line_Stop();
            frozen_elapsed_ms = now - run_start_ms;
            line_tracking = false;
            line_lost = false;
            show_status(false, false, false, imu_seen, SPEED_CONTROL_FAULT_NONE);
        }

        if (line_tracking && yaw_sample_valid &&
            (accumulated_yaw_degrees >= YAW_STOP_DEGREES ||
             accumulated_yaw_degrees <= -YAW_STOP_DEGREES)) {
            key1_decelerating = true;
        }

        if (line_tracking && key1_decelerating &&
            key1_base_speed >= KEY1_DECEL_PWM) {
            Line_Stop();
            frozen_elapsed_ms = now - run_start_ms;
            line_tracking = false;
            line_lost = false;
            show_status(false, false, false, imu_seen, SPEED_CONTROL_FAULT_NONE);
        }

        if (key3_tracking && key3_stop_detected) {
            Line_Stop();
            frozen_elapsed_ms = now - run_start_ms;
            key3_tracking = false;
            line_lost = false;
            key3_stop_detected = false;
            show_status(false, false, false, imu_seen, SPEED_CONTROL_FAULT_NONE);
        }

        if (key3_tracking && yaw_sample_valid &&
            (accumulated_yaw_degrees >= KEY3_YAW_STOP_DEGREES ||
             accumulated_yaw_degrees <= -KEY3_YAW_STOP_DEGREES)) {
            Line_Stop();
            frozen_elapsed_ms = now - run_start_ms;
            key3_tracking = false;
            line_lost = false;
            show_status(false, false, false, imu_seen, SPEED_CONTROL_FAULT_NONE);
        }

        if (key2_tracking &&
            (now - run_start_ms) >= KEY2_STOP_TIME_MS) {
            Line_Stop();
            frozen_elapsed_ms = now - run_start_ms;
            key2_tracking = false;
            show_status(false, false, false, imu_seen, SPEED_CONTROL_FAULT_NONE);
        }

        if ((line_tracking || key2_tracking || key3_tracking) &&
            SpeedControl_Update(now) &&
            SpeedControl_GetFault() != SPEED_CONTROL_FAULT_NONE) {
            frozen_elapsed_ms = now - run_start_ms;
            line_tracking = false;
            key2_tracking = false;
            key3_tracking = false;
            line_lost = false;
            show_status(false, false, false, imu_seen, SpeedControl_GetFault());
        }

        if ((now - last_display_update_ms) >= SENSOR_DISPLAY_PERIOD_MS) {
            uint32_t elapsed_ms = (line_tracking || key2_tracking || key3_tracking) ?
                                  now - run_start_ms : frozen_elapsed_ms;
            uint32_t lost_ms = line_lost ? now - line_lost_start_ms : 0U;

            last_display_update_ms = now;
            gray_map = Gray_ReadAll();

            format_elapsed_time(time_line, elapsed_ms);
            format_gray(gray_line, gray_map);
            format_run_status(line_line, line_tracking, line_lost, lost_ms,
                              key2_tracking);

            OLED_ShowString(0, 2, (uint8_t *)time_line, 8);
            OLED_ShowString(0, 4, (uint8_t *)gray_line, 8);
            OLED_ShowString(0, 6, (uint8_t *)line_line, 8);
        }
    }
}
