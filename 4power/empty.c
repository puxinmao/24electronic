/*
 * empty.c - 四轮速度闭环与 KEY1 循迹主程序。
 *
 * 按下 KEY1 后开始循迹，连续丢失黑线 200 ms 后停车。
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

#define SENSOR_DISPLAY_PERIOD_MS  50U  /* OLED 与传感器状态刷新周期。 */
#define SYSTICK_PERIOD_MS         1U   /* 系统 SysTick 的计时单位。 */

#define LINE_CONTROL_PERIOD_MS      5U      /* 循迹 PID 更新周期。 */
#define LINE_LOST_STOP_MS           200U    /* 连续丢失黑线后停车的确认时间。 */
#define LINE_KP                     600.0f /* 循迹 PID 比例系数。 */
#define LINE_KI                      0.0f   /* 循迹 PID 积分系数。 */
#define LINE_KD                      8.0f   /* 循迹 PID 微分系数。 */
#define LINE_BASE_SPEED             1000    /* 循迹基础 PWM，数值越小速度越快。 */

#define START_STOP_LINE_MAP          0x7EU  /* 起点/终点横向基准线，OLED 显示为 01111110。 */
#define START_LINE_LEAVE_CONFIRM_MS    80U  /* 连续离开基准线该时间后，允许检测一圈终点。 */
#define START_LINE_DETECT_CONFIRM_MS   80U  /* 连续识别到终点基准线该时间后停车。 */
#define MIN_LAP_TIME_MS              2500U  /* 最短单圈运行时间，避免起步时误判停车。 */

static volatile uint32_t sSystemTickMs;

void SysTick_Handler(void)
{
    sSystemTickMs++;
}

static uint32_t get_system_tick_ms(void)
{
    return sSystemTickMs;
}

static bool is_start_stop_line(uint8_t gray_map)
{
    return gray_map == START_STOP_LINE_MAP;
}

static float relative_yaw(float raw_yaw, float zero_yaw)
{
    float yaw = raw_yaw - zero_yaw;

    while (yaw > 180.0f) yaw -= 360.0f;
    while (yaw < -180.0f) yaw += 360.0f;
    return yaw;
}

static void format_time(char line[17], uint32_t elapsed_ms)
{
    uint32_t seconds = (elapsed_ms / 1000U) % 10000U;
    uint32_t tenths = (elapsed_ms / 100U) % 10U;

    line[0] = 'T';
    line[1] = 'I';
    line[2] = 'M';
    line[3] = 'E';
    line[4] = ':';
    line[5] = (char)('0' + (seconds / 1000U) % 10U);
    line[6] = (char)('0' + (seconds / 100U) % 10U);
    line[7] = (char)('0' + (seconds / 10U) % 10U);
    line[8] = (char)('0' + seconds % 10U);
    line[9] = '.';
    line[10] = (char)('0' + tenths);
    line[11] = 's';
    line[12] = ' ';
    line[13] = ' ';
    line[14] = ' ';
    line[15] = ' ';
    line[16] = '\0';
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

static void format_yaw(char line[17], float yaw, bool valid)
{
    int32_t tenths;
    uint32_t magnitude;

    line[0] = 'Y';
    line[1] = 'A';
    line[2] = 'W';
    line[3] = ':';

    if (!valid) {
        line[4] = '-';
        line[5] = '-';
        line[6] = '-';
        line[7] = '-';
        line[8] = '.';
        line[9] = '-';
    } else {
        tenths = (yaw >= 0.0f) ? (int32_t)(yaw * 10.0f + 0.5f)
                               : (int32_t)(yaw * 10.0f - 0.5f);
        line[4] = (tenths < 0) ? '-' : '+';
        magnitude = (uint32_t)((tenths < 0) ? -tenths : tenths);
        line[5] = (char)('0' + (magnitude / 1000U) % 10U);
        line[6] = (char)('0' + (magnitude / 100U) % 10U);
        line[7] = (char)('0' + (magnitude / 10U) % 10U);
        line[8] = '.';
        line[9] = (char)('0' + magnitude % 10U);
    }

    line[10] = ' ';
    line[11] = 'D';
    line[12] = 'E';
    line[13] = 'G';
    line[14] = ' ';
    line[15] = ' ';
    line[16] = '\0';
}

static void show_status(bool line_tracking, SpeedControlFault_t speed_fault)
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
    } else {
        OLED_ShowString(0, 0, (uint8_t *)"KEY1: READY     ", 8);
    }
}

int main(void)
{
    WIT_Data_t imu_data;
    float yaw = 0.0f;
    float yaw_zero = 0.0f;
    uint8_t gray_map;
    bool imu_seen = false;
    bool line_tracking = false;
    bool line_lost = false;
    bool stopwatch_running = false;
    bool start_line_left = false;
    bool start_line_leave_pending = false;
    bool start_line_detecting = false;
    uint32_t line_lost_start_ms = 0U;
    uint32_t last_line_update_ms = 0U;
    uint32_t last_display_update_ms = 0U;
    uint32_t lap_start_ms = 0U;
    uint32_t stopwatch_start_ms = 0U;
    uint32_t stopwatch_elapsed_ms = 0U;
    uint32_t start_line_leave_start_ms = 0U;
    uint32_t start_line_detect_start_ms = 0U;
    char time_line[17];
    char gray_line[20];
    char yaw_line[17];

    SYSCFG_DL_init();
    SysTick_Config(CPUCLK_FREQ / (1000U / SYSTICK_PERIOD_MS));
    Motor_Init();
    Motor_Standby();
    Encoder_Init();
    SpeedControl_Init(get_system_tick_ms());
    Gray_Init();
    WIT_Init();

    OLED_Init();
    show_status(false, SPEED_CONTROL_FAULT_NONE);

    while (1) {
        uint32_t now = get_system_tick_ms();

        if (WIT_GetData(&imu_data)) {
            if (!imu_seen) {
                yaw_zero = imu_data.yaw;
                imu_seen = true;
            }
            yaw = relative_yaw(imu_data.yaw, yaw_zero);
        }

        if (!line_tracking && KEY1_PRESSED) {
            now = get_system_tick_ms();
            Line_Config(LINE_KP, LINE_KI, LINE_KD, LINE_BASE_SPEED);
            Line_Start(now);
            last_line_update_ms = now;
            line_lost = false;
            line_tracking = true;
            lap_start_ms = now;
            stopwatch_start_ms = now;
            stopwatch_elapsed_ms = 0U;
            stopwatch_running = true;
            start_line_left = false;
            start_line_leave_pending = false;
            start_line_detecting = false;
            show_status(true, SPEED_CONTROL_FAULT_NONE);
        }

        if (line_tracking &&
            (now - last_line_update_ms) >= LINE_CONTROL_PERIOD_MS) {
            last_line_update_ms = now;
            gray_map = Gray_ReadAll();

            if (gray_map == 0U) {
                if (!line_lost) {
                    line_lost = true;
                    line_lost_start_ms = now;
                }
            } else {
                line_lost = false;
            }

            Line_Update(gray_map, now);

            /* 起步后必须先稳定离开基准线，避免把起点直接当作终点。 */
            if (!start_line_left) {
                if (!is_start_stop_line(gray_map)) {
                    if (!start_line_leave_pending) {
                        start_line_leave_pending = true;
                        start_line_leave_start_ms = now;
                    } else if ((now - start_line_leave_start_ms) >=
                               START_LINE_LEAVE_CONFIRM_MS) {
                        start_line_left = true;
                        start_line_leave_pending = false;
                    }
                } else {
                    start_line_leave_pending = false;
                }
            } else if ((now - lap_start_ms) >= MIN_LAP_TIME_MS) {
                /* 完成一圈后稳定识别横向基准线，停止在起点位置。 */
                if (is_start_stop_line(gray_map)) {
                    if (!start_line_detecting) {
                        start_line_detecting = true;
                        start_line_detect_start_ms = now;
                    } else if ((now - start_line_detect_start_ms) >=
                               START_LINE_DETECT_CONFIRM_MS) {
                        Line_Stop();
                        line_tracking = false;
                        line_lost = false;
                        stopwatch_elapsed_ms = now - stopwatch_start_ms;
                        stopwatch_running = false;
                        start_line_detecting = false;
                        show_status(false, SPEED_CONTROL_FAULT_NONE);
                    }
                } else {
                    start_line_detecting = false;
                }
            }
        }

        if (line_tracking && line_lost &&
            (now - line_lost_start_ms) >= LINE_LOST_STOP_MS) {
            Line_Stop();
            line_tracking = false;
            line_lost = false;
            stopwatch_elapsed_ms = now - stopwatch_start_ms;
            stopwatch_running = false;
            show_status(false, SPEED_CONTROL_FAULT_NONE);
        }

        if (line_tracking &&
            SpeedControl_Update(now) &&
            SpeedControl_GetFault() != SPEED_CONTROL_FAULT_NONE) {
            line_tracking = false;
            line_lost = false;
            stopwatch_elapsed_ms = now - stopwatch_start_ms;
            stopwatch_running = false;
            show_status(false, SpeedControl_GetFault());
        }

        if ((now - last_display_update_ms) >= SENSOR_DISPLAY_PERIOD_MS) {
            last_display_update_ms = now;
            gray_map = Gray_ReadAll();

            if (stopwatch_running) {
                stopwatch_elapsed_ms = now - stopwatch_start_ms;
            }

            format_time(time_line, stopwatch_elapsed_ms);
            format_gray(gray_line, gray_map);

            OLED_ShowString(0, 2, (uint8_t *)time_line, 8);
            OLED_ShowString(0, 4, (uint8_t *)gray_line, 8);
            format_yaw(yaw_line, yaw, imu_seen);
            OLED_ShowString(0, 6, (uint8_t *)yaw_line, 8);
        }
    }
}
