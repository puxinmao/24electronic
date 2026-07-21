/*
 * empty.c - 小车主程序
 *
 * 控制模块: control_straight / control_line
 *
 * KEY1 → 偏航直行 + 黑停
 * KEY2 → 停止
 * KEY3 → 直行 → 遇黑 → 循迹
 * KEY4 → 原地转向 38° → 直行 → 遇黑 → 循迹
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
typedef enum { S_IDLE, S_YAW, S_TURN_IN_PLACE, S_LINE_STRAIGHT, S_LINE_TRACK } State_t;
static State_t  g_state = S_IDLE;
static uint32_t g_cnt   = 0;
static char     g_buf[32];
static uint16_t g_track_entry = 0;  /* 入场减速帧计数 */
static float    g_turn_target = 0.0f;
static float    g_turn_last_yaw = 0.0f;
static float    g_turn_progress = 0.0f;
static uint8_t  g_turn_brake_frames = 0;
static uint16_t g_turn_frames = 0;
static bool     g_turn_braking = false;
static bool     g_key4_armed = true;

#define LINE_SPEED_NORMAL  700   /* 正常循迹速度 */
#define LINE_SPEED_ENTRY   1000  /* 入场减速速度（反向PWM，值大=慢） */
#define LINE_ENTRY_FRAMES  300   /* 减速持续帧数（约80ms） */
#define LINE_DETECT_MIN      1   /* 任意一路检测到黑线即进入循迹 */
#define KEY1_STOP_BLACK_MIN  2   /* KEY1 至少两路检测到黑线时停止 */

#define TURN_ANGLE_DEG       38.0f
#define TURN_PWM_FAST         900
#define TURN_PWM_SLOW        1200
#define TURN_SLOW_ANGLE      10.0f
#define TURN_BRAKE_FRAMES       10  /* 刹车后等待约 100ms 再开始直行 */
#define TURN_TIMEOUT_FRAMES    300  /* 约 3 秒（JY901S 姿态角 100Hz） */

#define OLED_CLR(l) OLED_ShowString(3, l, (uint8_t *)"            ", 8)

/* ========== 小工具 ========== */
static void delay_ms(uint32_t ms) {
    while (ms--) delay_cycles(CPUCLK_FREQ / 1000);
}

static float yaw_error_norm(float target, float current)
{
    float error = target - current;
    while (error >  180.0f) error -= 360.0f;
    while (error < -180.0f) error += 360.0f;
    return error;
}

static float yaw_norm(float yaw)
{
    while (yaw >  180.0f) yaw -= 360.0f;
    while (yaw < -180.0f) yaw += 360.0f;
    return yaw;
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
        if (g_state == S_YAW) {
            if (Gray_BlackCount(gray_map) >= KEY1_STOP_BLACK_MIN) {
                Straight_Stop();
                g_state = S_IDLE;
                OLED_CLR(4); OLED_CLR(5); OLED_CLR(6);
                OLED_ShowString(3, 7, (uint8_t *)"STOP BLK  ", 8);
                DBG_Printf("STOP BLACK G:0x%02X\r\n", gray_map);
            }
            if (g_state == S_YAW && wit_new) {
                float err = Straight_Update(wit.yaw);
                if (g_cnt % 10 == 0) {
                    sprintf(g_buf, "E:%-5.1f", err);
                    OLED_ShowString(3, 6, (uint8_t *)g_buf, 8);
                    DBG_Printf("Y:%.1f T:%.1f E:%.1f\r\n",
                        wit.yaw, Straight_GetTarget(), err);
                }
            }
        }

        /* ========== S_TURN_IN_PLACE: KEY4 原地转向 ========== */
        if (g_state == S_TURN_IN_PLACE && wit_new) {
            if (++g_turn_frames >= TURN_TIMEOUT_FRAMES) {
                Straight_Stop();
                g_state = S_IDLE;
                g_turn_braking = false;
                OLED_ShowString(3, 7, (uint8_t *)"TURN TIMEOUT", 8);
                DBG_SendStr("STOP TURN TIMEOUT\r\n");
            } else if (g_turn_braking) {
                /* 到角后只刹车等待，不反向纠偏，避免车体左右摇摆。 */
                Motor_Brake();
                if (++g_turn_brake_frames >= TURN_BRAKE_FRAMES) {
                    Straight_Start(wit.yaw);
                    g_state = S_LINE_STRAIGHT;
                    g_turn_braking = false;
                    OLED_ShowString(3, 4, (uint8_t *)"STRAIGHT  ", 8);
                    OLED_ShowString(3, 7, (uint8_t *)"TURN DONE ", 8);
                    DBG_Printf("TURN DONE Y:%.1f -> STRAIGHT\r\n", wit.yaw);
                }
            } else {
                /* 实车右转时 yaw 递减；逐帧累计，跨过 +/-180° 也能正确计角。 */
                float step = yaw_error_norm(g_turn_last_yaw, wit.yaw);
                g_turn_last_yaw = wit.yaw;
                if (step > 0.0f && step < 20.0f) g_turn_progress += step;

                if (g_turn_progress >= TURN_ANGLE_DEG) {
                    Motor_Brake();
                    g_turn_braking = true;
                    g_turn_brake_frames = 0;
                    DBG_Printf("TURN ANGLE %.1f, BRAKE\r\n", g_turn_progress);
                } else {
                    float remaining = TURN_ANGLE_DEG - g_turn_progress;
                    int16_t pwm = (remaining <= TURN_SLOW_ANGLE) ?
                                  TURN_PWM_SLOW : TURN_PWM_FAST;
                    Motor_SetBoth(pwm, -pwm);
                }

                if (g_cnt % 10 == 0) {
                    sprintf(g_buf, "TA:%-5.1f", g_turn_progress);
                    OLED_ShowString(3, 6, (uint8_t *)g_buf, 8);
                    DBG_Printf("TURN Y:%.1f A:%.1f T:%.1f\r\n",
                        wit.yaw, g_turn_progress, g_turn_target);
                }
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
            if (Gray_BlackCount(gray_map) >= LINE_DETECT_MIN) {
                Line_SetBaseSpeed(LINE_SPEED_ENTRY); /* 先以慢速入场 */
                Line_Start();
                g_state = S_LINE_TRACK;
                g_track_entry = LINE_ENTRY_FRAMES;
                OLED_ShowString(3, 7, (uint8_t *)"LINE TRACK", 8);
                DBG_Printf("-> LINE TRACK G:0x%02X\r\n", gray_map);
            }
        }

        /* ========== S_LINE_TRACK: 循迹 ========== */
        if (g_state == S_LINE_TRACK) {
            /* 入场减速：倒计到0后恢复正常速度 */
            if (g_track_entry > 0) {
                if (--g_track_entry == 0) {
                    Line_SetBaseSpeed(LINE_SPEED_NORMAL);
                    DBG_SendStr("LINE normal speed\r\n");
                }
            }

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
            float snap_yaw = wit_data.yaw;  /* 第一时间记录，不受消抖延时影响 */
            delay_ms(50);
            if (DL_GPIO_readPins(GPIO_IO_KEY1_PORT, GPIO_IO_KEY1_PIN) == 0) {
                Straight_Start(snap_yaw);
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

        /* KEY2 → 停止（第一次检测到按下立即执行） */
        if (DL_GPIO_readPins(GPIO_IO_KEY2_PORT, GPIO_IO_KEY2_PIN) == 0) {
            if (g_state == S_LINE_TRACK) Line_Stop();
            else Straight_Stop();
            g_state = S_IDLE;
            g_turn_braking = false;
            OLED_CLR(4); OLED_CLR(5); OLED_CLR(6);
            OLED_ShowString(3, 7, (uint8_t *)"STOP      ", 8);
            DBG_SendStr("STOP L:");
            DBG_SendNum(Encoder_GetLeft());
            DBG_SendStr(" R:");
            DBG_SendNum(Encoder_GetRight());
            DBG_SendStr("\r\n");
            uint32_t t = 300;
            while (DL_GPIO_readPins(GPIO_IO_KEY2_PORT, GPIO_IO_KEY2_PIN) == 0
                   && t > 0) { delay_ms(1); t--; }
        }

        /* KEY3 → 直行→循迹 */
        if (DL_GPIO_readPins(GPIO_IO_KEY3_PORT, GPIO_IO_KEY3_PIN) == 0) {
            float snap_yaw = wit_data.yaw;  /* 第一时间记录 */
            delay_ms(50);
            if (DL_GPIO_readPins(GPIO_IO_KEY3_PORT, GPIO_IO_KEY3_PIN) == 0) {
                Straight_Start(snap_yaw);
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

        /* KEY4 → 原地转向 38°，完成后复用 KEY3 的直行→循迹流程 */
        if (g_key4_armed &&
            DL_GPIO_readPins(GPIO_IO_KEY4_PORT, GPIO_IO_KEY4_PIN) == 0) {
            float snap_yaw = wit_data.yaw;  /* 第一时间记录 */
            delay_ms(50);
            if (DL_GPIO_readPins(GPIO_IO_KEY4_PORT, GPIO_IO_KEY4_PIN) == 0) {
                g_key4_armed = false;
                g_turn_target = yaw_norm(snap_yaw - TURN_ANGLE_DEG);
                g_turn_last_yaw = snap_yaw;
                g_turn_progress = 0.0f;
                g_turn_brake_frames = 0;
                g_turn_frames = 0;
                g_turn_braking = false;
                Motor_Enable();
                Motor_SetBoth(TURN_PWM_FAST, -TURN_PWM_FAST);
                g_state = S_TURN_IN_PLACE;
                OLED_CLR(4); OLED_CLR(5); OLED_CLR(6);
                sprintf(g_buf, "TGT:%.0f", g_turn_target);
                OLED_ShowString(3, 7, (uint8_t *)g_buf, 8);
                DBG_Printf("GO TURN Y:%.1f T:%.1f\r\n", snap_yaw, g_turn_target);
            }
        }
        if (DL_GPIO_readPins(GPIO_IO_KEY4_PORT, GPIO_IO_KEY4_PIN) != 0) {
            g_key4_armed = true;
        }
    }
}
