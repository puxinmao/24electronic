/*
 * empty.c - 小车主程序 (开环版本)
 *
 * 按键:
 *   KEY1 → 偏航角直行 + 遇黑停止
 *   KEY2 → 停止
 *   KEY3 → 先直行 → 遇黑线 → 循迹
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

#define KEY_WAIT_TIMEOUT 300       /* 按键释放超时 (ms) */

/* ========== 延时 ========== */
static void delay_ms(uint32_t ms)
{
    while (ms--) { delay_cycles(CPUCLK_FREQ / 1000); }
}

/* 等待引脚释放，超时 300ms 强制退出 */
static void wait_key_release(GPIO_Regs *port, uint32_t pin)
{
    uint32_t t = KEY_WAIT_TIMEOUT;
    while (DL_GPIO_readPins(port, pin) == 0 && t > 0) {
        delay_ms(1);
        t--;
    }
}

/* ========== 运行模式 ========== */
typedef enum {
    MODE_IDLE = 0,
    MODE_YAW  = 1,   /* KEY1: 偏航直行 + 黑停 */
    MODE_LINE = 2,   /* KEY3: 直行→循迹       */
} RunMode_t;

static RunMode_t g_mode       = MODE_IDLE;
static bool      g_running    = false;
static uint32_t  g_print_cnt  = 0;
static uint8_t   g_line_phase = 0;   /* KEY3: 0=直行  1=循迹 */

/* ========== 偏航角 PID ========== */
static PID_t  gPidYaw;
static float  g_target_yaw = 0.0f;
#define YAW_BASE_SPEED  700
#define YAW_CORR_MAX    600

/* ========== 循迹 PID ========== */
static PID_t  gPidLine;
#define LINE_BASE_SPEED 700
#define LINE_CORR_MAX   600
#define LINE_CENTER     3.5f

/* ========== 显示缓冲 ========== */
static char g_buf[32];

/* ================================================================= */
static float yaw_error_norm(float cur, float tgt)
{
    float e = cur - tgt;
    while (e >  180.0f) e -= 360.0f;
    while (e < -180.0f) e += 360.0f;
    return e;
}

static float gray_get_position(uint8_t map)
{
    int16_t sw = 0, sn = 0;
    for (int i = 0; i < 8; i++) {
        if (map & (1 << i)) { sw += i; sn++; }
    }
    return (sn == 0) ? LINE_CENTER : (float)sw / (float)sn;
}

static int gray_black_count(uint8_t map)
{
    int cnt = 0;
    for (int i = 0; i < 8; i++) {
        if (map & (1 << i)) cnt++;
    }
    return cnt;
}

/* ================================================================= */
int main(void)
{
    SYSCFG_DL_init();
    DBG_SendStr("System OK\r\n");

    Motor_Init();
    Motor_Standby();
    DL_TimerA_startCounter(PWM_INST);
    Encoder_Init();
    Gray_Init();

    /* ---------- PID ---------- */
    PID_Init(&gPidYaw,  50.0f, 0.2f,  8.0f, -YAW_CORR_MAX,  YAW_CORR_MAX,  400);
    PID_Init(&gPidLine, 240.0f, 0.0f, 30.0f, -LINE_CORR_MAX, LINE_CORR_MAX, 400);

    /* ---------- OLED + WIT ---------- */
    OLED_Init();
    OLED_Clear();
    OLED_ShowString(3, 0, (uint8_t *)"T3 Car Ready", 8);
    WIT_Init();
    OLED_ShowString(3, 2, (uint8_t *)"WIT init...", 8);
    DBG_SendStr("Ready\r\n");

    /* ================================================================ */
    while (1) {
        WIT_Data_t wit;
        bool wit_new = WIT_GetData(&wit);
        uint8_t gray_map = Gray_ReadAll();
        g_print_cnt++;

        /* --- OLED 角度 --- */
        if (wit_new) {
            sprintf(g_buf, "P:%-5.1f", wit.pitch);
            OLED_ShowString(3, 0, (uint8_t *)g_buf, 8);
            sprintf(g_buf, "R:%-5.1f", wit.roll);
            OLED_ShowString(67, 0, (uint8_t *)g_buf, 8);
            sprintf(g_buf, "Y:%-6.1f", wit.yaw);
            OLED_ShowString(3, 2, (uint8_t *)g_buf, 8);
        }

        /* ============================================================
         *   MODE_YAW: 偏航直行 + 遇黑停止 (KEY1)
         * ============================================================ */
        if (g_running && g_mode == MODE_YAW && wit_new) {
            float yaw_err = yaw_error_norm(wit.yaw, g_target_yaw);
            float corr = PID_Compute(&gPidYaw, wit.yaw, 0.01f);

            int16_t left  = YAW_BASE_SPEED + (int16_t)corr;
            int16_t right = YAW_BASE_SPEED - (int16_t)corr;
            if (left  < 0) left  = 0;
            if (right < 0) right = 0;
            Motor_SetBoth(left, right);

            /* 黑线停止 */
            if (gray_black_count(gray_map) >= 2) {
                g_running = false;
                g_mode    = MODE_IDLE;
                Motor_Brake();
                Motor_Standby();
                OLED_ShowString(3, 4, (uint8_t *)"            ", 8);
                OLED_ShowString(3, 5, (uint8_t *)"            ", 8);
                OLED_ShowString(3, 6, (uint8_t *)"            ", 8);
                OLED_ShowString(3, 7, (uint8_t *)"STOP BLK  ", 8);
                DBG_Printf("STOP BLACK G:0x%02X\r\n", gray_map);
            }

            if (g_print_cnt % 10 == 0) {
                sprintf(g_buf, "E:%-5.1f", yaw_err);
                OLED_ShowString(3, 6, (uint8_t *)g_buf, 8);
                sprintf(g_buf, "C:%-5d", (int)corr);
                OLED_ShowString(67, 6, (uint8_t *)g_buf, 8);
                DBG_Printf("Y:%.1f T:%.1f E:%.1f C:%d\r\n",
                    wit.yaw, g_target_yaw, yaw_err, (int)corr);
            }
        }

        /* ============================================================
         *   MODE_LINE: 先直行 → 遇黑线 → 循迹 (KEY3)
         * ============================================================ */
        if (g_running && g_mode == MODE_LINE) {

            if (g_line_phase == 0 && wit_new) {
                /* ---- 阶段 0: 偏航直行，等黑线 ---- */
                float yaw_err = yaw_error_norm(wit.yaw, g_target_yaw);
                float corr = PID_Compute(&gPidYaw, wit.yaw, 0.01f);

                int16_t left  = LINE_BASE_SPEED + (int16_t)corr;
                int16_t right = LINE_BASE_SPEED - (int16_t)corr;
                if (left  < 0) left  = 0;
                if (right < 0) right = 0;
                Motor_SetBoth(left, right);

                if (gray_black_count(gray_map) >= 2) {
                    g_line_phase = 1;
                    PID_Reset(&gPidLine);
                    PID_SetSetpoint(&gPidLine, LINE_CENTER);
                    OLED_ShowString(3, 7, (uint8_t *)"LINE TRACK", 8);
                    DBG_Printf("-> LINE TRACK G:0x%02X\r\n", gray_map);
                }

                if (g_print_cnt % 10 == 0) {
                    OLED_ShowString(3, 4, (uint8_t *)"STRAIGHT  ", 8);
                    sprintf(g_buf, "E:%-5.1f", yaw_err);
                    OLED_ShowString(3, 6, (uint8_t *)g_buf, 8);
                }

            } else {
                /* ---- 阶段 1: 灰度循迹 ---- */
                float pos   = gray_get_position(gray_map);
                float error = pos - LINE_CENTER;
                float corr  = PID_Compute(&gPidLine, pos, 0.005f);

                int16_t left  = LINE_BASE_SPEED + (int16_t)corr;
                int16_t right = LINE_BASE_SPEED - (int16_t)corr;
                if (left  < 0) left  = 0;
                if (right < 0) right = 0;
                Motor_SetBoth(left, right);

                if (g_print_cnt % 10 == 0) {
                    sprintf(g_buf, "P:%.1f", pos);
                    OLED_ShowString(3, 4, (uint8_t *)g_buf, 8);
                    sprintf(g_buf, "E:%-5.1f", error);
                    OLED_ShowString(3, 6, (uint8_t *)g_buf, 8);
                    sprintf(g_buf, "C:%-5d", (int)corr);
                    OLED_ShowString(67, 6, (uint8_t *)g_buf, 8);
                    DBG_Printf("G:0x%02X P:%.1f E:%.1f C:%d\r\n",
                        gray_map, pos, error, (int)corr);
                }
            }
        }

        /* ============================================================
         *   KEY1 → 偏航直行 + 黑停
         * ============================================================ */
        if (DL_GPIO_readPins(GPIO_IO_KEY1_PORT, GPIO_IO_KEY1_PIN) == 0) {
            delay_ms(50);
            if (DL_GPIO_readPins(GPIO_IO_KEY1_PORT, GPIO_IO_KEY1_PIN) == 0) {
                g_target_yaw = wit_data.yaw;
                PID_Reset(&gPidYaw);
                PID_SetSetpoint(&gPidYaw, g_target_yaw);

                g_mode    = MODE_YAW;
                g_running = true;
                Motor_Enable();

                OLED_ShowString(3, 4, (uint8_t *)"            ", 8);
                OLED_ShowString(3, 5, (uint8_t *)"            ", 8);
                sprintf(g_buf, "YAW T:%.0f", g_target_yaw);
                OLED_ShowString(3, 7, (uint8_t *)g_buf, 8);
                DBG_Printf("GO YAW T:%.1f\r\n", g_target_yaw);

                wait_key_release(GPIO_IO_KEY1_PORT, GPIO_IO_KEY1_PIN);
            }
        }

        /* ============================================================
         *   KEY2 → 停止
         * ============================================================ */
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

                wait_key_release(GPIO_IO_KEY2_PORT, GPIO_IO_KEY2_PIN);
            }
        }

        /* ============================================================
         *   KEY3 → 直行→循迹
         * ============================================================ */
        if (DL_GPIO_readPins(GPIO_IO_KEY3_PORT, GPIO_IO_KEY3_PIN) == 0) {
            delay_ms(50);
            if (DL_GPIO_readPins(GPIO_IO_KEY3_PORT, GPIO_IO_KEY3_PIN) == 0) {
                g_target_yaw = wit_data.yaw;
                PID_Reset(&gPidYaw);
                PID_SetSetpoint(&gPidYaw, g_target_yaw);
                g_line_phase = 0;

                g_mode    = MODE_LINE;
                g_running = true;
                Motor_Enable();

                OLED_ShowString(3, 4, (uint8_t *)"            ", 8);
                OLED_ShowString(3, 5, (uint8_t *)"            ", 8);
                sprintf(g_buf, "YAW T:%.0f", g_target_yaw);
                OLED_ShowString(3, 7, (uint8_t *)g_buf, 8);
                DBG_Printf("GO LINE T:%.1f\r\n", g_target_yaw);

                wait_key_release(GPIO_IO_KEY3_PORT, GPIO_IO_KEY3_PIN);
            }
        }
    }
}
