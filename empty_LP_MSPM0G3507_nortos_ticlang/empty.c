/*
 * empty.c - 小车主程序
 *
 * 模块依赖: pid, motor, encoder, uart_debug, gray, oled, wit
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

/* ========== 偏航角 PID + 运行状态 ========== */
static PID_t  gPidYaw;
static bool   g_running    = false;
static float  g_target_yaw = 0.0f;
#define YAW_BASE_SPEED  1200    /* 基础速度 (0~2000) */

/* ========== 显示缓冲 ========== */
static char g_oled_buf[32];

/* ========== 偏航角误差归一化（处理 ±180° 边界穿越） ========== */
static float yaw_error_norm(float current, float target)
{
    float err = current - target;
    while (err >  180.0f) err -= 360.0f;
    while (err < -180.0f) err += 360.0f;
    return err;
}

/* ========== main ========== */
int main(void)
{
    /* ---------- 系统初始化 ---------- */
    SYSCFG_DL_init();
    DBG_SendStr("System OK\r\n");

    /* ---------- 电机 ---------- */
    Motor_Init();
    Motor_Standby();
    DL_TimerA_startCounter(PWM_INST);

    /* ---------- 编码器 ---------- */
    Encoder_Init();

    /* ---------- 灰度传感器 ---------- */
    Gray_Init();

    /* ---------- 偏航角 PID ---------- */
    /* Kp=25: 偏1°→校正25; Ki=0.2: 缓慢消静差; Kd=8: 抑制过冲 */
    PID_Init(&gPidYaw, 25.0f, 0.2f, 8.0f, -600, 600, 400);

    /* ---------- OLED ---------- */
    OLED_Init();
    OLED_Clear();
    OLED_ShowString(3, 0, (uint8_t *)"T3 Car Ready", 8);

    /* ---------- JY901S ---------- */
    WIT_Init();
    OLED_ShowString(3, 2, (uint8_t *)"WIT init...", 8);

    DBG_SendStr("Ready\r\n");

    /* ================================================================
     *   主循环
     * ================================================================ */
    while (1) {
        WIT_Data_t wit;
        bool wit_new = WIT_GetData(&wit);

        /* --- OLED 更新角度显示 --- */
        if (wit_new) {
            sprintf(g_oled_buf, "P:%-5.1f", wit.pitch);
            OLED_ShowString(3, 0, (uint8_t *)g_oled_buf, 8);
            sprintf(g_oled_buf, "R:%-5.1f", wit.roll);
            OLED_ShowString(67, 0, (uint8_t *)g_oled_buf, 8);
            sprintf(g_oled_buf, "Y:%-6.1f", wit.yaw);
            OLED_ShowString(3, 2, (uint8_t *)g_oled_buf, 8);
        }

        /* --- 直行 PID 控制 --- */
        if (g_running && wit_new) {
            float yaw_err = yaw_error_norm(wit.yaw, g_target_yaw);
            float corr = PID_Compute(&gPidYaw, wit.yaw, 0.01f);

            int16_t left  = YAW_BASE_SPEED + (int16_t)corr;
            int16_t right = YAW_BASE_SPEED - (int16_t)corr;
            if (left  < 0) left  = 0;
            if (right < 0) right = 0;
            Motor_SetBoth(left, right);

            /* OLED 第 6/7 行：显示偏航误差和校正 */
            sprintf(g_oled_buf, "E:%-5.1f", yaw_err);
            OLED_ShowString(3, 6, (uint8_t *)g_oled_buf, 8);
            sprintf(g_oled_buf, "C:%-5d", (int)corr);
            OLED_ShowString(67, 6, (uint8_t *)g_oled_buf, 8);

            /* 串口 */
            DBG_Printf("Y:%.1f T:%.1f E:%.1f C:%d\r\n",
                wit.yaw, g_target_yaw, yaw_err, (int)corr);
        }

        /* --- KEY1 → 锁定偏航角，直行 --- */
        if (DL_GPIO_readPins(GPIO_IO_KEY1_PORT, GPIO_IO_KEY1_PIN) == 0) {
            delay_ms(50);
            if (DL_GPIO_readPins(GPIO_IO_KEY1_PORT, GPIO_IO_KEY1_PIN) == 0) {
                /* 记录当前偏航角作为目标 */
                g_target_yaw = wit_data.yaw;
                PID_Reset(&gPidYaw);
                PID_SetSetpoint(&gPidYaw, g_target_yaw);

                Encoder_Reset();
                Motor_Enable();
                g_running = true;

                DBG_Printf("GO  YawTarget:%.1f\r\n", g_target_yaw);
                sprintf(g_oled_buf, "GO  T:%.0f", g_target_yaw);
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
                Motor_Brake();
                Motor_Standby();

                OLED_ShowString(3, 6, (uint8_t *)"            ", 8); /* 清 E/C 行 */
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
    }
}
