/*
 * empty.c - Four-wheel speed-loop test and line tracking.
 *
 * KEY1 locks the current gyro yaw to drive straight until a black line is
 * detected, then switches to line tracking. KEY2 starts line tracking
 * directly. Both line-tracking paths stop after the gray sensors have
 * continuously lost the black line for 200 ms.
 */
#include "ti_msp_dl_config.h"

#include "button.h"
#include "control_line.h"
#include "control_straight.h"
#include "encoder.h"
#include "gray.h"
#include "motor.h"
#include "oled.h"
#include "speed_control.h"
#include "wit.h"

#define SENSOR_DISPLAY_PERIOD_MS  50U
#define SYSTICK_PERIOD_MS         1U

#define STRAIGHT_CONTROL_PERIOD_MS 5U
#define STRAIGHT_KP                 80.0f
#define STRAIGHT_KI                  0.0f
#define STRAIGHT_KD                  2.0f
#define STRAIGHT_BASE_SPEED           700
#define RETURN_STRAIGHT_DURATION_MS  5000U

#define LINE_CONTROL_PERIOD_MS      5U
#define LINE_LOST_STOP_MS           200U
#define LINE_KP                     1200.0f
#define LINE_KI                      0.0f
#define LINE_KD                      8.0f
#define LINE_BASE_SPEED             1000

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

static float normalize_yaw(float yaw)
{
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

static void show_status(bool straight_running, bool line_tracking,
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
    } else if (straight_running) {
        OLED_ShowString(0, 0, (uint8_t *)"KEY1: GYRO RUN", 8);
    } else if (line_tracking) {
        OLED_ShowString(0, 0, (uint8_t *)"KEY2: LINE TRACK", 8);
    } else {
        OLED_ShowString(0, 0, (uint8_t *)"K1 GYRO K2 LINE", 8);
    }
}

int main(void)
{
    WIT_Data_t imu_data;
    float yaw = 0.0f;
    float yaw_zero = 0.0f;
    float key1_yaw = 0.0f;
    uint8_t gray_map;
    bool imu_seen = false;
    bool straight_running = false;
    bool line_tracking = false;
    bool line_lost = false;
    bool key1_route_active = false;
    bool return_leg_started = false;
    uint32_t last_straight_update_ms = 0U;
    uint32_t return_straight_start_ms = 0U;
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
    show_status(false, false, SPEED_CONTROL_FAULT_NONE);

    while (1) {
        uint32_t now = get_system_tick_ms();

        if (WIT_GetData(&imu_data)) {
            if (!imu_seen) {
                /* The vehicle's heading when its first IMU frame arrives is 0. */
                yaw_zero = imu_data.yaw;
                imu_seen = true;
            }
            yaw = relative_yaw(imu_data.yaw, yaw_zero);
        }

        if (!straight_running && !line_tracking && imu_seen && KEY1_PRESSED) {
            now = get_system_tick_ms();
            key1_yaw = yaw;
            Straight_Config(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD,
                            STRAIGHT_BASE_SPEED);
            Straight_Start(yaw, now);
            Straight_Update(yaw, now);
            last_straight_update_ms = now;
            straight_running = true;
            key1_route_active = true;
            return_leg_started = false;
            show_status(true, false, SPEED_CONTROL_FAULT_NONE);
        }

        if (!straight_running && !line_tracking && KEY2_PRESSED) {
            now = get_system_tick_ms();
            Line_Config(LINE_KP, LINE_KI, LINE_KD, LINE_BASE_SPEED);
            Line_Start(now);
            last_line_update_ms = now;
            line_lost = false;
            line_tracking = true;
            key1_route_active = false;
            return_leg_started = false;
            show_status(false, true, SPEED_CONTROL_FAULT_NONE);
        }

        if (straight_running &&
            (now - last_straight_update_ms) >= STRAIGHT_CONTROL_PERIOD_MS) {
            last_straight_update_ms = now;
            Straight_Update(yaw, now);
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

        if (straight_running && (gray_map = Gray_ReadAll()) != 0U) {
            /* A black line starts the same tracking path used by KEY2. */
            Straight_Stop();
            Line_Config(LINE_KP, LINE_KI, LINE_KD, LINE_BASE_SPEED);
            Line_Start(now);
            Line_Update(gray_map, now);
            straight_running = false;
            line_lost = false;
            last_line_update_ms = now;
            line_tracking = true;
            show_status(false, true, SPEED_CONTROL_FAULT_NONE);
        }

        if (straight_running && return_leg_started &&
            (now - return_straight_start_ms) >= RETURN_STRAIGHT_DURATION_MS) {
            Straight_Stop();
            straight_running = false;
            key1_route_active = false;
            return_leg_started = false;
            show_status(false, false, SPEED_CONTROL_FAULT_NONE);
        }

        if (line_tracking && line_lost &&
            (now - line_lost_start_ms) >= LINE_LOST_STOP_MS) {
            if (key1_route_active && !return_leg_started) {
                now = get_system_tick_ms();
                Straight_Config(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD,
                                STRAIGHT_BASE_SPEED);
                Straight_StartToYaw(normalize_yaw(key1_yaw + 180.0f), now);
                Straight_Update(yaw, now);
                last_straight_update_ms = now;
                return_straight_start_ms = now;
                straight_running = true;
                line_tracking = false;
                line_lost = false;
                return_leg_started = true;
                show_status(true, false, SPEED_CONTROL_FAULT_NONE);
            } else {
                Line_Stop();
                line_tracking = false;
                line_lost = false;
                key1_route_active = false;
                return_leg_started = false;
                show_status(false, false, SPEED_CONTROL_FAULT_NONE);
            }
        }

        if ((straight_running || line_tracking) &&
            SpeedControl_Update(now) &&
            SpeedControl_GetFault() != SPEED_CONTROL_FAULT_NONE) {
            straight_running = false;
            line_tracking = false;
            line_lost = false;
            key1_route_active = false;
            return_leg_started = false;
            show_status(false, false, SpeedControl_GetFault());
        }

        if ((now - last_display_update_ms) >= SENSOR_DISPLAY_PERIOD_MS) {
            last_display_update_ms = now;
            gray_map = Gray_ReadAll();

            format_yaw(yaw_line, yaw, imu_seen);
            format_gray(gray_line, gray_map);
            format_imu_status(imu_line, imu_seen);

            OLED_ShowString(0, 2, (uint8_t *)yaw_line, 8);
            OLED_ShowString(0, 4, (uint8_t *)gray_line, 8);
            OLED_ShowString(0, 6, (uint8_t *)imu_line, 8);
        }
    }
}
