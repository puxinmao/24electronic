/*
 * empty.c - 小车主程序
 *
 * 控制模块: control_straight / control_line
 *
 * KEY1 → 偏航直行 + 黑停
 * KEY2 → 停止
 * KEY3 → 直行 → 遇黑 → 循迹
 */
#include "ti_msp_dl_config.h"

#include "control_straight.h"
#include "control_line.h"
#include "motor.h"
#include "encoder.h"
#include "uart_debug.h"
#include "gray.h"
#include "oled.h"
#include "wit.h"

#include <stdio.h>

/* ========== 状态 ========== */
typedef enum { S_IDLE, S_YAW, S_LINE_STRAIGHT, S_LINE_TRACK } State_t;
static State_t  g_state = S_IDLE;
static uint32_t g_cnt   = 0;
static char     g_buf[32];

#define OLED_CLR(l) OLED_ShowString(3, l, (uint8_t *)"            ", 8)

/* ========== 小工具 ========== */
static void delay_ms(uint32_t ms) {
    while (ms--) delay_cycles(CPUCLK_FREQ / 1000);
}

/* ========== main ========== */
int main(void)
{
    SYSCFG_DL_init();
    DBG_SendStr("System OK\r\n");

    Motor_Init(); 
    Motor_Standby();
    DL_TimerA_startCounter(PWM_INST);
    Encoder_Init();
    Gray_Init();

    Straight_Config(50.0f, 0.2f, 8.0f,  700);
    Line_Config(240.0f, 0.0f, 30.0f, 700);

    OLED_Init(); OLED_Clear();
    OLED_ShowString(3, 0, (uint8_t *)"T3 Car Ready", 8);
    WIT_Init();
    OLED_ShowString(3, 2, (uint8_t *)"WIT init...", 8);
    DBG_SendStr("Ready\r\n");

    while (1) {
        WIT_Data_t wit;
        bool wit_new = WIT_GetData(&wit);
        uint8_t gray_map = Gray_ReadAll();
        g_cnt++;

        /* ---- OLED 角度 ---- */
        if (wit_new) {
            sprintf(g_buf, "P:%-5.1f", wit.pitch);
            OLED_ShowString(3, 0, (uint8_t *)g_buf, 8);
            sprintf(g_buf, "R:%-5.1f", wit.roll);
            OLED_ShowString(67, 0, (uint8_t *)g_buf, 8);
            sprintf(g_buf, "Y:%-6.1f", wit.yaw);
            OLED_ShowString(3, 2, (uint8_t *)g_buf, 8);
        }

        /* ========== S_YAW: 偏航直行 + 黑停 ========== */
        if (g_state == S_YAW && wit_new) {
            float err = Straight_Update(wit.yaw);

            if (Gray_BlackCount(gray_map) >= 4) {
                Straight_Stop();
                g_state = S_IDLE;
                OLED_CLR(4); OLED_CLR(5); OLED_CLR(6);
                OLED_ShowString(3, 7, (uint8_t *)"STOP BLK  ", 8);
                DBG_Printf("STOP BLACK G:0x%02X\r\n", gray_map);
            }
            if (g_cnt % 10 == 0) {
                sprintf(g_buf, "E:%-5.1f", err);
                OLED_ShowString(3, 6, (uint8_t *)g_buf, 8);
                DBG_Printf("Y:%.1f T:%.1f E:%.1f\r\n",
                    wit.yaw, Straight_GetTarget(), err);
            }
        }

        /* ========== S_LINE_STRAIGHT: 直行等黑 ========== */
        if (g_state == S_LINE_STRAIGHT) {
            if (wit_new) {
                float err = Straight_Update(wit.yaw);
                if (g_cnt % 10 == 0) {
                    OLED_ShowString(3, 4, (uint8_t *)"STRAIGHT  ", 8);
                    sprintf(g_buf, "E:%-5.1f", err);
                    OLED_ShowString(3, 6, (uint8_t *)g_buf, 8);
                }
            }

            /* 黑线检测：每圈都查，不依赖 wit_new */
            if (Gray_BlackCount(gray_map) >= 2) {
                Line_Start();
                g_state = S_LINE_TRACK;
                OLED_ShowString(3, 7, (uint8_t *)"LINE TRACK", 8);
                DBG_Printf("-> LINE TRACK G:0x%02X\r\n", gray_map);
            }
        }

        /* ========== S_LINE_TRACK: 循迹 ========== */
        if (g_state == S_LINE_TRACK) {
            Line_Update(gray_map);

            /* 丢线检测：连续 50 帧无线（灯全灭）自动停止 */
            static uint16_t lost_cnt = 0;
            if (gray_map == 0) {
                if (++lost_cnt >= 50) {
                    Line_Stop();
                    g_state = S_IDLE;
                    lost_cnt = 0;
                    OLED_CLR(4); OLED_CLR(5); OLED_CLR(6);
                    OLED_ShowString(3, 7, (uint8_t *)"LOST LINE ", 8);
                    DBG_SendStr("STOP LOST LINE\r\n");
                }
            } else {
                lost_cnt = 0;
            }

            /* 降低 OLED/UART 刷新频率，避免阻塞导致循环计时抖动 */
            if (g_cnt % 100 == 0) {
                sprintf(g_buf, "P:%.1f", Line_GetPosition(gray_map));
                OLED_ShowString(3, 4, (uint8_t *)g_buf, 8);
                sprintf(g_buf, "E:%-5.1f", Line_GetError());
                OLED_ShowString(3, 6, (uint8_t *)g_buf, 8);
                sprintf(g_buf, "C:%-5.0f", Line_GetCorrection());
                OLED_ShowString(67, 6, (uint8_t *)g_buf, 8);
                DBG_Printf("G:0x%02X E:%.1f C:%.0f\r\n",
                    gray_map, Line_GetError(), Line_GetCorrection());
            }
        }

        /* ========== 按键（行内检查，不封模块） ========== */

        /* KEY1 → 偏航直行 + 黑停 */
        if (DL_GPIO_readPins(GPIO_IO_KEY1_PORT, GPIO_IO_KEY1_PIN) == 0) {
            delay_ms(50);
            if (DL_GPIO_readPins(GPIO_IO_KEY1_PORT, GPIO_IO_KEY1_PIN) == 0) {
                Straight_Start(wit_data.yaw);
                g_state = S_YAW;
                OLED_CLR(4); OLED_CLR(5); OLED_CLR(6);
                sprintf(g_buf, "YAW T:%.0f", Straight_GetTarget());
                OLED_ShowString(3, 7, (uint8_t *)g_buf, 8);
                DBG_Printf("GO YAW T:%.1f\r\n", Straight_GetTarget());
                uint32_t t = 300;
                while (DL_GPIO_readPins(GPIO_IO_KEY1_PORT, GPIO_IO_KEY1_PIN) == 0
                       && t > 0) { delay_ms(1); t--; }
            }
        }

        /* KEY2 → 停止（急停键：第一次检测到按下立即执行，不等50ms二次确认） */
        if (DL_GPIO_readPins(GPIO_IO_KEY2_PORT, GPIO_IO_KEY2_PIN) == 0) {
            if (g_state == S_LINE_TRACK) Line_Stop();
            else Straight_Stop();
            g_state = S_IDLE;
            OLED_CLR(4); OLED_CLR(5); OLED_CLR(6);
            OLED_ShowString(3, 7, (uint8_t *)"STOP      ", 8);
            DBG_SendStr("STOP L:");
            DBG_SendNum(Encoder_GetLeft());
            DBG_SendStr(" R:");
            DBG_SendNum(Encoder_GetRight());
            DBG_SendStr("\r\n");
            /* 等待释放，超时 300ms */
            uint32_t t = 300;
            while (DL_GPIO_readPins(GPIO_IO_KEY2_PORT, GPIO_IO_KEY2_PIN) == 0
                   && t > 0) { delay_ms(1); t--; }
        }

        /* KEY3 → 直行→循迹 */
        if (DL_GPIO_readPins(GPIO_IO_KEY3_PORT, GPIO_IO_KEY3_PIN) == 0) {
            delay_ms(50);
            if (DL_GPIO_readPins(GPIO_IO_KEY3_PORT, GPIO_IO_KEY3_PIN) == 0) {
                Straight_Start(wit_data.yaw);
                g_state = S_LINE_STRAIGHT;
                OLED_CLR(4); OLED_CLR(5); OLED_CLR(6);
                sprintf(g_buf, "YAW T:%.0f", Straight_GetTarget());
                OLED_ShowString(3, 7, (uint8_t *)g_buf, 8);
                DBG_Printf("GO LINE T:%.1f\r\n", Straight_GetTarget());
                uint32_t t = 300;
                while (DL_GPIO_readPins(GPIO_IO_KEY3_PORT, GPIO_IO_KEY3_PIN) == 0
                       && t > 0) { delay_ms(1); t--; }
            }
        }
    }
}
