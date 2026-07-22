/*
 * empty.c - 小车主程序
 *
 * 控制模块: control_straight / control_line
 *
 * KEY1 → 偏航直行 + 黑停
 * KEY2 → 停止
 * KEY3 → 直行 → 遇黑 → 循迹 → 终点直行一段时间 → 停止
 * KEY4 → 未使用（原地转向算法已提取到 control_straight）
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
#include "button.h"

#include <stdio.h>

/* ========== 状态 ========== */
typedef enum {
    S_IDLE,
    S_YAW,
    S_LINE_STRAIGHT,
    S_LINE_TRACK,
    S_LINE_FINISH_RUN
} State_t;
static State_t  g_state = S_IDLE;
static uint32_t g_cnt   = 0;
static char     g_buf[32];
static uint16_t g_track_entry = 0;  /* 入场减速帧计数 */
static bool     g_wit_ready = false;
static uint16_t g_wit_silent_loops = 0;
static bool     g_line_lost_timing = false;
static uint32_t g_line_lost_start_ms = 0;
static uint32_t g_line_finish_start_ms = 0;
static volatile uint32_t g_ms_ticks = 0;
static float    g_latest_yaw = 0.0f;

#define STRAIGHT_SPEED_NORMAL   700  /* 直线基础 PWM；数值越大，实际速度越慢 */
#define LINE_SPEED_NORMAL       700  /* 正常循迹基础 PWM；数值越大，实际速度越慢 */
#define LINE_SPEED_ENTRY       1000  /* 刚进入循迹的基础 PWM；数值越大，入场越慢 */

#define STRAIGHT_KP            50.0f /* 直线比例：纠偏弱时增大，持续蛇形时减小 */
#define STRAIGHT_KI             0.2f /* 直线积分：处理长期小偏差；调试初期保持不动 */
#define STRAIGHT_KD             8.0f /* 直线微分：抑制过冲；细碎抖动时减小 */

#define LINE_KP              1200.0f /* 巡线比例：主要影响中心附近，达到输出限幅后再增大无效 */
#define LINE_KI                 0.0f /* 巡线积分：数字灰度巡线建议保持为 0 */
#define LINE_KD                 8.0f /* 巡线微分：抑制慢摆，也会放大探头位置跳变 */

#define LINE_ENTRY_FRAMES       300  /* 入场慢速持续的主循环次数；越大，慢速保持越久 */
#define LINE_DETECT_MIN           1  /* 直行转循迹的门槛：检测到几路黑线就切换 */
#define KEY1_STOP_BLACK_MIN       1  /* KEY1 直行停止门槛：同时检测到几路黑线就停止 */

#define WIT_LOSS_LOOP_LIMIT      2000 /* 连续无新陀螺仪数据的主循环次数；超限停车 */

#define LINE_END_CONFIRM_MS      50U /* 连续全灭确认时间；越大越不易误判终点，越小越快确认 */
#define LINE_FINISH_PWM          700  /* 终点后偏航直行 PWM；数值越大，实际运行速度越慢 */
#define LINE_FINISH_RUN_MS     500U /* 终点后直行毫秒数；1000 为 1 秒，越大运行越久 */

#define OLED_CLR(l) OLED_ShowString(3, l, (uint8_t *)"            ", 8)

/* ========== 小工具 ========== */
static void delay_ms(uint32_t ms) {
    while (ms--) delay_cycles(CPUCLK_FREQ / 1000);
}

static uint32_t elapsed_ms(uint32_t start_ms)
{
    return g_ms_ticks - start_ms;
}

void SysTick_Handler(void)
{
    g_ms_ticks++;
}


static bool state_uses_wit(State_t state)
{
    return state == S_YAW || state == S_LINE_STRAIGHT || state == S_LINE_FINISH_RUN;
}

static void stop_vehicle(void)
{
    Straight_Stop();
    Straight_Config(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD,
                    STRAIGHT_SPEED_NORMAL);
    Line_Stop();
    Line_SetBaseSpeed(LINE_SPEED_NORMAL);
    g_state = S_IDLE;
    g_track_entry = 0;
    g_line_lost_timing = false;
    g_line_lost_start_ms = 0;
    g_line_finish_start_ms = 0;
    TurnInPlace_Stop();
    g_wit_silent_loops = 0;
}

static bool wit_can_start(void)
{
    if (WIT_HasFault()) {
        WIT_Recover();
        g_wit_ready = false;
    }

    if (g_wit_ready && !Button_EStopIsPending()) return true;

    OLED_ShowString(3, 7, (uint8_t *)"WIT WAIT  ", 8);
    DBG_SendStr("START BLOCKED: WIT NOT READY\r\n");
    return false;
}

/* ========== main ========== */
int main(void)
{
    SYSCFG_DL_init();
    (void)DL_SYSTICK_config(CPUCLK_FREQ / 1000U);
    DBG_SendStr("System OK\r\n");

    Motor_Init(); 
    Motor_Standby();
    DL_TimerA_startCounter(PWM_INST);
    Encoder_Init();
    Button_EStopInit();
    Gray_Init();

    Straight_Config(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD,
                    STRAIGHT_SPEED_NORMAL);
    Line_Config(LINE_KP, LINE_KI, LINE_KD, LINE_SPEED_NORMAL);

    OLED_Init(); OLED_Clear();
    OLED_ShowString(3, 0, (uint8_t *)"T3 Car Ready", 8);
    WIT_Init();
    OLED_ShowString(3, 2, (uint8_t *)"WIT init...", 8);
    DBG_SendStr("Ready\r\n");

    while (1) {
        if (Button_EStopTakeEvent()) {
            stop_vehicle();
            WIT_Recover();
            g_wit_ready = false;
            OLED_CLR(4); OLED_CLR(5); OLED_CLR(6);
            OLED_ShowString(3, 7, (uint8_t *)"STOP      ", 8);
            DBG_SendStr("EMERGENCY STOP\r\n");
            continue;
        }

        if (WIT_HasFault()) {
            stop_vehicle();
            WIT_Recover();
            g_wit_ready = false;
            OLED_ShowString(3, 7, (uint8_t *)"WIT ERROR ", 8);
            DBG_SendStr("STOP WIT UART FAULT\r\n");
            continue;
        }

        WIT_Data_t wit;
        bool wit_new = WIT_GetData(&wit);
        uint8_t gray_map = Gray_ReadAll();
        g_cnt++;

        if (wit_new) {
            g_latest_yaw = wit.yaw;
            g_wit_ready = true;
            g_wit_silent_loops = 0;
        } else if (g_wit_ready &&
                   g_wit_silent_loops < WIT_LOSS_LOOP_LIMIT) {
            g_wit_silent_loops++;
        }

        if (g_state == S_IDLE &&
            g_wit_silent_loops >= WIT_LOSS_LOOP_LIMIT) {
            g_wit_ready = false;
        }

        if (state_uses_wit(g_state) &&
            g_wit_silent_loops >= WIT_LOSS_LOOP_LIMIT) {
            stop_vehicle();
            WIT_Recover();
            g_wit_ready = false;
            OLED_CLR(4); OLED_CLR(5); OLED_CLR(6);
            OLED_ShowString(3, 7, (uint8_t *)"WIT LOST  ", 8);
            DBG_SendStr("STOP WIT DATA LOST\r\n");
            continue;
        }

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
                float err = Straight_Update(wit.yaw, g_ms_ticks);
                if (g_cnt % 10 == 0) {
                    sprintf(g_buf, "E:%-5.1f", err);
                    OLED_ShowString(3, 6, (uint8_t *)g_buf, 8);
                    DBG_Printf("Y:%.1f T:%.1f E:%.1f\r\n",
                        wit.yaw, Straight_GetTarget(), err);
                }
            }
        }


        /* ========== S_LINE_STRAIGHT: 直行等黑 ========== */
        if (g_state == S_LINE_STRAIGHT) {
            if (wit_new) {
                float err = Straight_Update(wit.yaw, g_ms_ticks);
                if (g_cnt % 10 == 0) {
                    OLED_ShowString(3, 4, (uint8_t *)"STRAIGHT  ", 8);
                    sprintf(g_buf, "E:%-5.1f", err);
                    OLED_ShowString(3, 6, (uint8_t *)g_buf, 8);
                }
            }

            /* 黑线检测：每圈都查，不依赖 wit_new */
            if (Gray_BlackCount(gray_map) >= LINE_DETECT_MIN) {
                Straight_Stop();
                Line_SetBaseSpeed(LINE_SPEED_ENTRY); /* 先以慢速入场 */
                Line_Start();
                g_state = S_LINE_TRACK;
                g_track_entry = LINE_ENTRY_FRAMES;
                g_line_lost_timing = false;
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

            /* 全灭期间改为两轮同速向前，避免沿最后转向量画大圈。 */
            if (gray_map == 0U) {
                Motor_SetBoth(LINE_FINISH_PWM, LINE_FINISH_PWM);
                if (!g_line_lost_timing) {
                    g_line_lost_timing = true;
                    g_line_lost_start_ms = g_ms_ticks;
                }
                if (elapsed_ms(g_line_lost_start_ms) >= LINE_END_CONFIRM_MS) {
                    g_line_lost_timing = false;
                    Straight_Config(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD,
                                    LINE_FINISH_PWM);
                    Straight_Start(g_latest_yaw, g_ms_ticks);
                    g_line_finish_start_ms = g_ms_ticks;
                    g_state = S_LINE_FINISH_RUN;
                    OLED_CLR(4); OLED_CLR(5); OLED_CLR(6);
                    OLED_ShowString(3, 7, (uint8_t *)"FINISH RUN", 8);
                    DBG_Printf("LINE END -> RUN %luMS\r\n",
                               (unsigned long)LINE_FINISH_RUN_MS);
                }
            } else {
                g_line_lost_timing = false;
                Line_Update(gray_map);
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

        /* ========== S_LINE_FINISH_RUN: 按毫秒计时，不依赖陀螺仪帧率 ========== */
        if (g_state == S_LINE_FINISH_RUN) {
            if (wit_new) (void)Straight_Update(wit.yaw, g_ms_ticks);
            if (elapsed_ms(g_line_finish_start_ms) >= LINE_FINISH_RUN_MS) {
                Straight_Stop();
                Straight_Config(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD,
                                STRAIGHT_SPEED_NORMAL);
                g_state = S_IDLE;
                g_line_finish_start_ms = 0;
                OLED_CLR(4); OLED_CLR(5); OLED_CLR(6);
                OLED_ShowString(3, 7, (uint8_t *)"FINISH STOP", 8);
                DBG_SendStr("STOP LINE FINISH\r\n");
            }
        }

        /* ========== 按键（行内检查，不封模块） ========== */

        /* KEY1 → 偏航直行 + 黑停 */
        if (DL_GPIO_readPins(GPIO_IO_KEY1_PORT, GPIO_IO_KEY1_PIN) == 0) {
            float snap_yaw = g_latest_yaw;
            delay_ms(50);
            if (DL_GPIO_readPins(GPIO_IO_KEY1_PORT, GPIO_IO_KEY1_PIN) == 0 &&
                DL_GPIO_readPins(GPIO_IO_KEY2_PORT, GPIO_IO_KEY2_PIN) != 0 &&
                wit_can_start()) {
                Straight_Start(snap_yaw, g_ms_ticks);
                g_wit_silent_loops = 0;
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

        /* KEY3 → 直行→循迹 */
        if (DL_GPIO_readPins(GPIO_IO_KEY3_PORT, GPIO_IO_KEY3_PIN) == 0) {
            float snap_yaw = g_latest_yaw;
            delay_ms(50);
            if (DL_GPIO_readPins(GPIO_IO_KEY3_PORT, GPIO_IO_KEY3_PIN) == 0 &&
                DL_GPIO_readPins(GPIO_IO_KEY2_PORT, GPIO_IO_KEY2_PIN) != 0 &&
                wit_can_start()) {
                Straight_Start(snap_yaw, g_ms_ticks);
                g_wit_silent_loops = 0;
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
