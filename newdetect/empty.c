#include <stdint.h>

#include "app_uart.h"
#include "ball_controller.h"
#include "ball_protocol.h"
#include "control_config.h"
#include "ti_msp_dl_config.h"
#include "zdt_motor.h"

#define MOTOR_STARTUP_DELAY_MS (100U)

static volatile uint32_t g_milliseconds;

void SysTick_Handler(void)
{
    g_milliseconds++;
}

static uint32_t milliseconds(void)
{
    return g_milliseconds;
}

static void wait_milliseconds(uint32_t duration_ms)
{
    uint32_t start_ms = milliseconds();

    while ((milliseconds() - start_ms) < duration_ms) {
        __WFI();
    }
}

int main(void)
{
    BallSample sample;
    BallFrameStatus frame_status;
    BallControlCommand control_command;
    uint32_t now_ms;
    uint32_t last_vision_ms = 0U;
    uint8_t vision_active = 0U;

    SYSCFG_DL_init();
    if (SysTick_Config(CPUCLK_FREQ / 1000U) != 0U) {
        while (1) {
            __WFI();
        }
    }

    AppUart_InitK230();
    ZdtMotor_Init();
    BallController_Init();
    wait_milliseconds(MOTOR_STARTUP_DELAY_MS);
    ZdtMotor_Enable();
    if ((ZDT_STARTUP_TEST_ENABLED != 0U) &&
        (ZdtMotor_IsReady() != 0U)) {
        ZdtMotor_SetSpeed(ZDT_PIPE_RAISE, ZDT_MOTOR_SPEED_RPM);
        wait_milliseconds(ZDT_STARTUP_TEST_DURATION_MS);
        ZdtMotor_Stop();
        wait_milliseconds(ZDT_STARTUP_TEST_DURATION_MS);
        ZdtMotor_SetSpeed(ZDT_PIPE_LOWER, ZDT_MOTOR_SPEED_RPM);
        wait_milliseconds(ZDT_STARTUP_TEST_DURATION_MS);
        ZdtMotor_Stop();
    }

    while (1) {
        now_ms = milliseconds();
        frame_status = BallProtocol_TakeLatest(&sample);

        if (frame_status == BALL_FRAME_VALID) {
            last_vision_ms = now_ms;
            vision_active = 1U;
            control_command = BallController_Update(&sample, now_ms);
            if (control_command.action == BALL_CONTROL_MOVE) {
                ZdtMotor_SetSpeed(control_command.direction,
                                  control_command.speed_rpm);
                wait_milliseconds(control_command.duration_ms);
                ZdtMotor_Stop();
            } else {
                ZdtMotor_Stop();
            }
        } else if (frame_status == BALL_FRAME_NOT_DETECTED) {
            last_vision_ms = now_ms;
            vision_active = 0U;
            BallController_Reset();
            ZdtMotor_Stop();
        }

        if ((vision_active != 0U) &&
            ((now_ms - last_vision_ms) >= VISION_LOST_TIMEOUT_MS)) {
            vision_active = 0U;
            BallController_Reset();
            ZdtMotor_Stop();
        }

        ZdtMotor_Poll();
        __WFI();
    }
}
