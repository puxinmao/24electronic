#include <stdint.h>

#include "app_uart.h"
#include "ball_controller.h"
#include "ball_protocol.h"
#include "control_config.h"
#include "ti_msp_dl_config.h"
#include "zdt_motor.h"

#define MOTOR_STARTUP_DELAY_MS (100U)
/* PB20 回中按键去抖时间，单位：ms。 */
#define BTN_CENTER_DEBOUNCE_MS (20U)

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
    uint32_t last_pid_debug_ms = 0U;
    uint32_t last_motor_attempt_ms = 0U;
    uint32_t motor_motion_update_ms = 0U;
    uint8_t vision_active = 0U;
    uint8_t command_available = 0U;
    uint8_t motor_running = 0U;
    ZdtPipeDirection motor_direction = ZDT_PIPE_RAISE;
    uint16_t motor_speed_rpm = 0U;
    int32_t motor_tilt_mdeg = 0;
    int32_t target_tilt_mdeg;
    int32_t tilt_error_mdeg;
    int32_t motion_delta_mdeg;
    uint32_t motion_elapsed_ms;
    ZdtPipeDirection requested_direction;
    uint16_t requested_speed_rpm;
    BallControllerTelemetry telemetry;
    ZdtCanStatus can_status;
    uint32_t btn_center_since_ms = 0U;
    uint8_t btn_center_timing = 0U;
    uint8_t btn_center_active = 0U;

    SYSCFG_DL_init();
    if (SysTick_Config(CPUCLK_FREQ / 1000U) != 0U) {
        while (1) {
            __WFI();
        }
    }

    AppUart_InitK230();
    ZdtMotor_Init();
    BallController_Init();
    control_command = (BallControlCommand){
        BALL_CONTROL_STOP, ZDT_PIPE_RAISE, 0U, 0U, 0U, 0U, 0U, 0, 0
    };
    wait_milliseconds(MOTOR_STARTUP_DELAY_MS);
    (void)ZdtMotor_Enable();
    if ((ZDT_STARTUP_TEST_ENABLED != 0U) &&
        (ZdtMotor_IsReady() != 0U)) {
        (void)ZdtMotor_SetSpeed(ZDT_PIPE_RAISE, ZDT_MOTOR_MIN_SPEED_RPM);
        wait_milliseconds(ZDT_STARTUP_TEST_DURATION_MS);
        (void)ZdtMotor_Stop();
        wait_milliseconds(ZDT_STARTUP_TEST_DURATION_MS);
        (void)ZdtMotor_SetSpeed(ZDT_PIPE_LOWER, ZDT_MOTOR_MIN_SPEED_RPM);
        wait_milliseconds(ZDT_STARTUP_TEST_DURATION_MS);
        (void)ZdtMotor_Stop();
    }

    while (1) {
        now_ms = milliseconds();

        /* PB20 回中按键：低电平有效（内部上拉），去抖后激活回中模式。 */
        if (DL_GPIO_readPins(KEY1_PORT, KEY1_PIN_1_PIN) == 0U) {
            if (btn_center_timing == 0U) {
                btn_center_timing = 1U;
                btn_center_since_ms = now_ms;
            } else if ((now_ms - btn_center_since_ms) >= BTN_CENTER_DEBOUNCE_MS) {
                if (btn_center_active == 0U) {
                    BallController_Reset();
                }
                btn_center_active = 1U;
            }
        } else {
            btn_center_timing = 0U;
            btn_center_active = 0U;
        }

        frame_status = BallProtocol_TakeLatest(&sample);

        if (frame_status == BALL_FRAME_VALID) {
            last_vision_ms = now_ms;
            vision_active = 1U;
            control_command = BallController_Update(&sample, now_ms);
            command_available = 1U;
        } else if (frame_status == BALL_FRAME_NOT_DETECTED) {
            /* 单帧漏检时保留上一轮 PID 状态，连续漏检超时后才统一停机。 */
        }

        if ((vision_active != 0U) &&
            ((now_ms - last_vision_ms) >= VISION_LOST_TIMEOUT_MS)) {
            vision_active = 0U;
            command_available = 0U;
            BallController_Reset();
            control_command = (BallControlCommand){
                BALL_CONTROL_STOP, ZDT_PIPE_RAISE, 0U, 0U, 0U, 0U, 0U, 0, 0
            };
        }

        motion_elapsed_ms = now_ms - motor_motion_update_ms;
        motor_motion_update_ms = now_ms;
        if ((motor_running != 0U) && (motion_elapsed_ms <= 100U)) {
            motion_delta_mdeg = (int32_t)motion_elapsed_ms *
                (int32_t)motor_speed_rpm * PIPE_TILT_MDEG_PER_RPM_MS;
            motor_tilt_mdeg +=
                (motor_direction == ZDT_PIPE_RAISE) ?
                motion_delta_mdeg : -motion_delta_mdeg;
            if (motor_tilt_mdeg > PIPE_TILT_ABSOLUTE_LIMIT_MDEG) {
                motor_tilt_mdeg = PIPE_TILT_ABSOLUTE_LIMIT_MDEG;
            } else if (motor_tilt_mdeg < -PIPE_TILT_ABSOLUTE_LIMIT_MDEG) {
                motor_tilt_mdeg = -PIPE_TILT_ABSOLUTE_LIMIT_MDEG;
            }
        }

        target_tilt_mdeg = control_command.target_tilt_milli_degree;
        if ((vision_active == 0U) || (command_available == 0U) ||
            (btn_center_active != 0U)) {
            target_tilt_mdeg = 0;
        }
        if (target_tilt_mdeg > PIPE_TILT_ABSOLUTE_LIMIT_MDEG) {
            target_tilt_mdeg = PIPE_TILT_ABSOLUTE_LIMIT_MDEG;
        } else if (target_tilt_mdeg < -PIPE_TILT_ABSOLUTE_LIMIT_MDEG) {
            target_tilt_mdeg = -PIPE_TILT_ABSOLUTE_LIMIT_MDEG;
        }
        tilt_error_mdeg = target_tilt_mdeg - motor_tilt_mdeg;

        if ((tilt_error_mdeg <= PIPE_TILT_POSITION_TOLERANCE_MDEG) &&
            (tilt_error_mdeg >= -PIPE_TILT_POSITION_TOLERANCE_MDEG)) {
            if ((motor_running != 0U) &&
                ((now_ms - last_motor_attempt_ms) >=
                 MOTOR_CAN_COMMAND_RETRY_MS)) {
                last_motor_attempt_ms = now_ms;
                if (ZdtMotor_Stop() != 0U) {
                    motor_running = 0U;
                    motor_speed_rpm = 0U;
                    motor_tilt_mdeg = target_tilt_mdeg;
                }
            }
        } else {
            requested_direction = (tilt_error_mdeg > 0) ?
                ZDT_PIPE_RAISE : ZDT_PIPE_LOWER;
            requested_speed_rpm =
                ((tilt_error_mdeg >= PIPE_TILT_FAST_MOVE_ERROR_MDEG) ||
                 (tilt_error_mdeg <= -PIPE_TILT_FAST_MOVE_ERROR_MDEG)) ?
                ZDT_MOTOR_MAX_SPEED_RPM : ZDT_MOTOR_MIN_SPEED_RPM;
            if (((motor_running == 0U) ||
                 (motor_direction != requested_direction) ||
                 (motor_speed_rpm != requested_speed_rpm)) &&
                ((now_ms - last_motor_attempt_ms) >=
                 MOTOR_CAN_COMMAND_RETRY_MS)) {
                last_motor_attempt_ms = now_ms;
                if (ZdtMotor_SetSpeed(requested_direction,
                                      requested_speed_rpm) != 0U) {
                    motor_running = 1U;
                    motor_direction = requested_direction;
                    motor_speed_rpm = requested_speed_rpm;
                    motor_motion_update_ms = now_ms;
                }
            }
        }

        if ((PID_DEBUG_OUTPUT_ENABLED != 0U) &&
            ((now_ms - last_pid_debug_ms) >= PID_DEBUG_PERIOD_MS)) {
            last_pid_debug_ms = now_ms;
            BallController_GetTelemetry(&telemetry);
            telemetry.direction = (motor_running != 0U) ?
                (int8_t)motor_direction : 0;
            telemetry.speed_rpm = (motor_running != 0U) ?
                motor_speed_rpm : 0U;
            telemetry.burst_duration_ms = 0U;
            ZdtMotor_GetCanStatus(&can_status);
            AppUart_SendPidTelemetry(&telemetry,
                                     g_zdt_can_tx_success_count,
                                     g_zdt_can_tx_failure_count,
                                     &can_status);
        }

        ZdtMotor_Poll();
        __WFI();
    }
}
