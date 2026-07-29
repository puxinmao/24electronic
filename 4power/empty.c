/*
 * empty.c - 四轮速度闭环与 KEY1 循迹主程序。
 *
 * 按下 KEY1 后开始循迹，连续丢失黑线 1 s 或累计偏航一圈后停车。
 * 计时和丢线判定均使用 MSPM0 的 SysTick；JY901S 仅用于一圈停车判定。
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
#define LINE_LOST_STOP_MS           300U   /* 连续丢失黑线后停车的确认时间。 */
#define LINE_KP                     600.0f /* 循迹 PID 比例系数。 */
#define LINE_KI                      0.0f   /* 循迹 PID 积分系数。 */
#define LINE_KD                      8.0f   /* 循迹 PID 微分系数。 */
#define LINE_BASE_SPEED             1000    /* 循迹基础 PWM，数值越小速度越快。 */
#define YAW_STOP_DEGREES             360.0f  /* 累计转过一圈后停车。 */

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

static void format_line_status(char line[17], bool line_tracking,
                               bool line_lost, uint32_t lost_ms)
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
    } else if (line_tracking) {
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
    uint8_t gray_map;
    bool line_tracking = false;
    bool line_lost = false;
    bool imu_seen = false;
    bool yaw_sample_valid = false;
    uint32_t line_lost_start_ms = 0U;
    uint32_t run_start_ms = 0U;
    uint32_t frozen_elapsed_ms = 0U;
    uint32_t last_line_update_ms = 0U;
    uint32_t last_display_update_ms = 0U;
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
    show_status(false, SPEED_CONTROL_FAULT_NONE);

    while (1) {
        uint32_t now = get_system_tick_ms();

        if (WIT_GetData(&imu_data)) {
            latest_yaw = imu_data.yaw;
            imu_seen = true;

            if (line_tracking) {
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

        if (!line_tracking && KEY1_PRESSED) {
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
            line_tracking = true;
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
        }

        if (line_tracking && line_lost &&
            (now - line_lost_start_ms) >= LINE_LOST_STOP_MS) {
            Line_Stop();
            frozen_elapsed_ms = now - run_start_ms;
            line_tracking = false;
            line_lost = false;
            show_status(false, SPEED_CONTROL_FAULT_NONE);
        }

        if (line_tracking && yaw_sample_valid &&
            (accumulated_yaw_degrees >= YAW_STOP_DEGREES ||
             accumulated_yaw_degrees <= -YAW_STOP_DEGREES)) {
            Line_Stop();
            frozen_elapsed_ms = now - run_start_ms;
            line_tracking = false;
            line_lost = false;
            show_status(false, SPEED_CONTROL_FAULT_NONE);
        }

        if (line_tracking &&
            SpeedControl_Update(now) &&
            SpeedControl_GetFault() != SPEED_CONTROL_FAULT_NONE) {
            frozen_elapsed_ms = now - run_start_ms;
            line_tracking = false;
            line_lost = false;
            show_status(false, SpeedControl_GetFault());
        }

        if ((now - last_display_update_ms) >= SENSOR_DISPLAY_PERIOD_MS) {
            uint32_t elapsed_ms = line_tracking ? now - run_start_ms :
                                                  frozen_elapsed_ms;
            uint32_t lost_ms = line_lost ? now - line_lost_start_ms : 0U;

            last_display_update_ms = now;
            gray_map = Gray_ReadAll();

            format_elapsed_time(time_line, elapsed_ms);
            format_gray(gray_line, gray_map);
            format_line_status(line_line, line_tracking, line_lost, lost_ms);

            OLED_ShowString(0, 2, (uint8_t *)time_line, 8);
            OLED_ShowString(0, 4, (uint8_t *)gray_line, 8);
            OLED_ShowString(0, 6, (uint8_t *)line_line, 8);
        }
    }
}
