/*
 * empty.c - 小车主程序
 *
 * 模块依赖: pid, motor, encoder, uart_debug, gray, oled, wit
 *
 * 按键功能:
 *   KEY1 → 偏航角直行 (锁定当前 Yaw 用 PID 保持方向)
 *   KEY2 → 停止
 *   KEY3 → 灰度循迹 (CD4051 八路位置 → PID 修正)
 */
#include "ti_msp_dl_config.h"

#include "pid.h"
#include "motor.h"
#include "encoder.h"
#include "uart_debug.h"
#include "gray.h"
#include "oled.h"
#include "wit.h"

#include <stdio.h>

/* ========== 延时 ========== */
static void delay_ms(uint32_t ms)
{
    while (ms--) {
        delay_cycles(CPUCLK_FREQ / 1000);
    }
}

/* ========== 运行模式 ========== */
typedef enum {
    MODE_IDLE = 0,
    MODE_YAW  = 1,   /* KEY1: 偏航角直行 */
    MODE_LINE = 2,   /* KEY3: 灰度循迹   */
} RunMode_t;

static RunMode_t g_mode       = MODE_IDLE;
static bool      g_running    = false;

/* ========== 偏航角 PID ========== */
static PID_t  gPidYaw;
static float  g_target_yaw = 0.0f;
#define YAW_BASE_SPEED  1200

/* ========== 灰度循迹 PID ========== */
static PID_t  gPidLine;
#define LINE_BASE_SPEED 500
#define LINE_CENTER     3.5f   /* 8 路传感器中心位置 (0~7 的中间) */

/* ========== 显示缓冲 ========== */
static char g_oled_buf[32];

/* ========== 偏航角误差归一化（±180°） ========== */
static float yaw_error_norm(float current, float target)
{
    float err = current - target;
    while (err >  180.0f) err -= 360.0f;
    while (err < -180.0f) err += 360.0f;
    return err;
}

/* ========== 灰度传感器 → 线位置（加权平均） ========== */
static float gray_get_position(uint8_t map)
{
    int16_t sum_w = 0, sum_n = 0;
    for (int i = 0; i < 8; i++) {
        if (map & (1 << i)) {
            sum_w += i;
            sum_n++;
        }
    }
    if (sum_n == 0) return LINE_CENTER;  /* 没看到线，保持居中 */
    return (float)sum_w / (float)sum_n;
}

/* ========== main ========== */
int main(void)
{
    /* ---------- 系统初始化 ---------- */
    SYSCFG_DL_init();
    DBG_SendStr("System OK\r\n");

    Motor_Init();
    Motor_Standby();
    DL_TimerA_startCounter(PWM_INST);

    Encoder_Init();
    Gray_Init();

    /* ---------- PID ---------- */
    PID_Init(&gPidYaw,  25.0f, 0.2f,  8.0f, -600, 600, 400);
    PID_Init(&gPidLine, 160.0f, 0.0f, 30.0f, -600, 600, 400);

    /* ---------- OLED + JY901S ---------- */
    OLED_Init();
    OLED_Clear();
    OLED_ShowString(3, 0, (uint8_t *)"T3 Car Ready", 8);
    WIT_Init();
    OLED_ShowString(3, 2, (uint8_t *)"WIT init...", 8);

    DBG_SendStr("Ready\r\n");

    /* ================================================================
     *   主循环
     * ================================================================ */
    while (1) {
        WIT_Data_t wit;
        bool wit_new = WIT_GetData(&wit);
        uint8_t gray_map = Gray_ReadAll();

        /* --- OLED 角度显示 --- */
        if (wit_new) {
            sprintf(g_oled_buf, "P:%-5.1f", wit.pitch);
            OLED_ShowString(3, 0, (uint8_t *)g_oled_buf, 8);
            sprintf(g_oled_buf, "R:%-5.1f", wit.roll);
            OLED_ShowString(67, 0, (uint8_t *)g_oled_buf, 8);
            sprintf(g_oled_buf, "Y:%-6.1f", wit.yaw);
            OLED_ShowString(3, 2, (uint8_t *)g_oled_buf, 8);
        }

        /* --- 模式: 偏航角直行 (KEY1) --- */
        if (g_running && g_mode == MODE_YAW && wit_new) {
            float yaw_err = yaw_error_norm(wit.yaw, g_target_yaw);
            float corr = PID_Compute(&gPidYaw, wit.yaw, 0.01f);

            int16_t left  = YAW_BASE_SPEED + (int16_t)corr;
            int16_t right = YAW_BASE_SPEED - (int16_t)corr;
            if (left  < 0) left  = 0;
            if (right < 0) right = 0;
            Motor_SetBoth(left, right);

            sprintf(g_oled_buf, "E:%-5.1f", yaw_err);
            OLED_ShowString(3, 6, (uint8_t *)g_oled_buf, 8);
            sprintf(g_oled_buf, "C:%-5d", (int)corr);
            OLED_ShowString(67, 6, (uint8_t *)g_oled_buf, 8);

            DBG_Printf("Y:%.1f T:%.1f E:%.1f C:%d\r\n",
                wit.yaw, g_target_yaw, yaw_err, (int)corr);
        }

        /* --- 模式: 灰度循迹 (KEY3) --- */
        if (g_running && g_mode == MODE_LINE) {
            float pos   = gray_get_position(gray_map);
            float error = pos - LINE_CENTER;
            float corr  = PID_Compute(&gPidLine, pos, 0.005f);

            int16_t left  = LINE_BASE_SPEED + (int16_t)corr;
            int16_t right = LINE_BASE_SPEED - (int16_t)corr;
            if (left  < 0) left  = 0;
            if (right < 0) right = 0;
            Motor_SetBoth(left, right);

            /* OLED: 显示位置 & 误差 & 校正 */
            sprintf(g_oled_buf, "P:%.1f", pos);
            OLED_ShowString(3, 5, (uint8_t *)g_oled_buf, 8);
            sprintf(g_oled_buf, "E:%-5.1f", error);
            OLED_ShowString(3, 6, (uint8_t *)g_oled_buf, 8);
            sprintf(g_oled_buf, "C:%-5d", (int)corr);
            OLED_ShowString(67, 6, (uint8_t *)g_oled_buf, 8);

            DBG_Printf("G:0x%02X  P:%.1f  E:%.1f  C:%d\r\n",
                gray_map, pos, error, (int)corr);
        }

        /* --- KEY1 → 偏航角直行 --- */
        if (DL_GPIO_readPins(GPIO_IO_KEY1_PORT, GPIO_IO_KEY1_PIN) == 0) {
            delay_ms(50);
            if (DL_GPIO_readPins(GPIO_IO_KEY1_PORT, GPIO_IO_KEY1_PIN) == 0) {
                g_target_yaw = wit_data.yaw;
                PID_Reset(&gPidYaw);
                PID_SetSetpoint(&gPidYaw, g_target_yaw);

                g_mode    = MODE_YAW;
                g_running = true;
                Encoder_Reset();
                Motor_Enable();

                DBG_Printf("GO YAW  T:%.1f\r\n", g_target_yaw);
                OLED_ShowString(3, 4, (uint8_t *)"            ", 8); /* 清 P行 */
                sprintf(g_oled_buf, "YAW  T:%.0f", g_target_yaw);
                OLED_ShowString(3, 7, (uint8_t *)g_oled_buf, 8);

                while (DL_GPIO_readPins(GPIO_IO_KEY1_PORT,
                       GPIO_IO_KEY1_PIN) == 0) {}
            }
        }

        /* --- KEY2 → 停止 --- */
        if (DL_GPIO_readPins(GPIO_IO_KEY2_PORT, GPIO_IO_KEY2_PIN) == 0) {
            delay_ms(50);
            if (DL_GPIO_readPins(GPIO_IO_KEY2_PORT, GPIO_IO_KEY2_PIN) == 0) {
                g_running = false;
                g_mode    = MODE_IDLE;
                Motor_Brake();
                Motor_Standby();

                OLED_ShowString(3, 4, (uint8_t *)"            ", 8);
                OLED_ShowString(3, 5, (uint8_t *)"            ", 8);
                OLED_ShowString(3, 6, (uint8_t *)"            ", 8);
                OLED_ShowString(3, 7, (uint8_t *)"STOP      ", 8);

                DBG_SendStr("STOP L:");
                DBG_SendNum(Encoder_GetLeft());
                DBG_SendStr(" R:");
                DBG_SendNum(Encoder_GetRight());
                DBG_SendStr("\r\n");

                while (DL_GPIO_readPins(GPIO_IO_KEY2_PORT,
                       GPIO_IO_KEY2_PIN) == 0) {}
            }
        }

        /* --- KEY3 → 灰度循迹 --- */
        if (DL_GPIO_readPins(GPIO_IO_KEY3_PORT, GPIO_IO_KEY3_PIN) == 0) {
            delay_ms(50);
            if (DL_GPIO_readPins(GPIO_IO_KEY3_PORT, GPIO_IO_KEY3_PIN) == 0) {
                /* 以当前中线位置为目标 */
                float init_pos = gray_get_position(gray_map);
                PID_Reset(&gPidLine);
                PID_SetSetpoint(&gPidLine, LINE_CENTER);

                g_mode    = MODE_LINE;
                g_running = true;
                Encoder_Reset();
                Motor_Enable();

                DBG_Printf("GO LINE  G:0x%02X  P:%.1f\r\n", gray_map, init_pos);
                OLED_ShowString(3, 4, (uint8_t *)"            ", 8);
                sprintf(g_oled_buf, "LINE 0x%02X", gray_map);
                OLED_ShowString(3, 7, (uint8_t *)g_oled_buf, 8);

                while (DL_GPIO_readPins(GPIO_IO_KEY3_PORT,
                       GPIO_IO_KEY3_PIN) == 0) {}
            }
        }
    }
}
