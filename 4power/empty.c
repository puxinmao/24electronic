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
#define LINE_KP                     1200.0f /* 循迹 PID 比例系数。 */
#define LINE_KI                      0.0f   /* 循迹 PID 积分系数。 */
#define LINE_KD                      8.0f   /* 循迹 PID 微分系数。 */
#define LINE_BASE_SPEED             1300    /* 循迹基础 PWM，数值越小速度越快。 */

static volatile uint32_t sSystemTickMs;

void SysTick_Handler(void)
{
    sSystemTickMs++;
}

static uint32_t get_system_tick_ms(void)
{
    return sSystemTickMs;
}

static float relative_yaw(float raw_yaw, float zero_yaw)
{
    float yaw = raw_yaw - zero_yaw;

    while (yaw > 180.0f) yaw -= 360.0f;
    while (yaw < -180.0f) yaw += 360.0f;
    return yaw;
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

static void format_imu_status(char line[17], bool imu_seen)
{
    uint8_t i;

    for (i = 0U; i < 16U; i++) {
        line[i] = ' ';
    }
    line[16] = '\0';

    if (WIT_HasFault()) {
        line[0] = 'I';
        line[1] = 'M';
        line[2] = 'U';
        line[3] = ':';
        line[4] = 'F';
        line[5] = 'A';
        line[6] = 'U';
        line[7] = 'L';
        line[8] = 'T';
    } else if (!imu_seen) {
        line[0] = 'I';
        line[1] = 'M';
        line[2] = 'U';
        line[3] = ':';
        line[4] = 'W';
        line[5] = 'A';
        line[6] = 'I';
        line[7] = 'T';
    } else {
        line[0] = 'I';
        line[1] = 'M';
        line[2] = 'U';
        line[3] = ':';
        line[4] = 'O';
        line[5] = 'K';
    }

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
    uint32_t line_lost_start_ms = 0U;
    uint32_t last_line_update_ms = 0U;
    uint32_t last_display_update_ms = 0U;
    char yaw_line[17];
    char gray_line[20];
    char imu_line[17];

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
                /* 上电后第一帧陀螺仪偏航角作为小车初始 0 度方向。 */
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
            line_tracking = false;
            line_lost = false;
            show_status(false, SPEED_CONTROL_FAULT_NONE);
        }

        if (line_tracking &&
            SpeedControl_Update(now) &&
            SpeedControl_GetFault() != SPEED_CONTROL_FAULT_NONE) {
            line_tracking = false;
            line_lost = false;
            show_status(false, SpeedControl_GetFault());
        }

        if ((now - last_display_update_ms) >= SENSOR_DISPLAY_PERIOD_MS) {
            last_display_update_ms = now;
            gray_map = Gray_ReadAll();

            format_yaw(yaw_line, yaw, imu_seen);
            format_gray(gray_line, gray_map);

            OLED_ShowString(0, 2, (uint8_t *)yaw_line, 8);
            OLED_ShowString(0, 4, (uint8_t *)gray_line, 8);
            format_imu_status(imu_line, imu_seen);
            OLED_ShowString(0, 6, (uint8_t *)imu_line, 8);
        }
    }
}
