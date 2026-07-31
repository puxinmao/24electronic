#include "app_uart.h"

#include "ball_protocol.h"
#include "ti_msp_dl_config.h"

#define PID_DEBUG_LINE_SIZE (208U)

static uint16_t append_text(char *line, uint16_t index, const char *text)
{
    while (*text != '\0') {
        line[index++] = *text++;
    }
    return index;
}

static uint16_t append_unsigned(char *line, uint16_t index, uint32_t value)
{
    char digits[10];
    uint8_t count = 0U;

    do {
        digits[count++] = (char)('0' + (value % 10U));
        value /= 10U;
    } while (value != 0U);

    while (count != 0U) {
        line[index++] = digits[--count];
    }
    return index;
}

static uint16_t append_centi_value(char *line, uint16_t index, int16_t value)
{
    int32_t absolute_value = value;

    if (absolute_value < 0) {
        line[index++] = '-';
        absolute_value = -absolute_value;
    } else {
        line[index++] = '+';
    }
    index = append_unsigned(line, index, (uint32_t)(absolute_value / 100));
    line[index++] = '.';
    line[index++] = (char)('0' + ((absolute_value / 10) % 10));
    line[index++] = (char)('0' + (absolute_value % 10));
    return index;
}

static uint16_t append_signed(char *line, uint16_t index, int16_t value)
{
    int32_t absolute_value = value;

    if (absolute_value < 0) {
        line[index++] = '-';
        absolute_value = -absolute_value;
    } else {
        line[index++] = '+';
    }
    return append_unsigned(line, index, (uint32_t)absolute_value);
}

static void transmit_line(const char *line, uint16_t length)
{
    uint16_t index;

    for (index = 0U; index < length; index++) {
        DL_UART_Main_transmitDataBlocking(UART_0_INST,
                                          (uint8_t)line[index]);
    }
}

void AppUart_InitK230(void)
{
    NVIC_ClearPendingIRQ(UART_2_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_2_INST_INT_IRQN);
}

void AppUart_SendPidTelemetry(const BallControllerTelemetry *telemetry,
                              uint32_t can_tx_success,
                              uint32_t can_tx_failure,
                              const ZdtCanStatus *can_status)
{
    char line[PID_DEBUG_LINE_SIZE];
    uint16_t length = 0U;

    if ((telemetry == 0) || (can_status == 0)) {
        return;
    }

    length = append_text(line, length, "PID p=");
    length = append_centi_value(line, length,
                                telemetry->position_centi_cm);
    length = append_text(line, length, " pp=");
    length = append_centi_value(line, length,
                                telemetry->predicted_position_centi_cm);
    length = append_text(line, length, " v=");
    length = append_centi_value(line, length,
                                telemetry->filtered_velocity_centi_cm_s);
    length = append_text(line, length, " e=");
    length = append_centi_value(line, length,
                                telemetry->position_error_centi_cm);
    length = append_text(line, length, " vd=");
    length = append_centi_value(line, length,
                                telemetry->desired_velocity_centi_cm_s);
    length = append_text(line, length, " ev=");
    length = append_centi_value(line, length,
                                telemetry->velocity_error_centi_cm_s);
    length = append_text(line, length, " u=");
    length = append_signed(line, length, telemetry->pulse_rate);
    length = append_text(line, length, " dir=");
    length = append_text(line, length,
                         (telemetry->direction > 0) ? "R" :
                         ((telemetry->direction < 0) ? "L" : "S"));
    length = append_text(line, length, " rpm=");
    length = append_unsigned(line, length, telemetry->speed_rpm);
    length = append_text(line, length, " mode=");
    length = append_text(line, length,
                         (telemetry->control_mode == 1U) ? "F" :
                         ((telemetry->control_mode == 2U) ? "R" :
                          ((telemetry->control_mode == 3U) ? "D" :
                           ((telemetry->control_mode == 4U) ? "B" : "S"))));
    length = append_text(line, length, " ms=");
    length = append_unsigned(line, length, telemetry->burst_duration_ms);
    length = append_text(line, length, " hold=");
    length = append_unsigned(line, length, telemetry->holding_center);
    length = append_text(line, length, " vis=");
    length = append_unsigned(line, length, telemetry->vision_active);
    length = append_text(line, length, " can=");
    length = append_unsigned(line, length, can_tx_success);
    length = append_text(line, length, "/");
    length = append_unsigned(line, length, can_tx_failure);
    length = append_text(line, length, " tec=");
    length = append_unsigned(line, length, can_status->tx_error_count);
    length = append_text(line, length, " rec=");
    length = append_unsigned(line, length, can_status->rx_error_count);
    length = append_text(line, length, " lec=");
    length = append_unsigned(line, length, can_status->last_error_code);
    length = append_text(line, length, " bo=");
    length = append_unsigned(line, length, can_status->bus_off);
    length = append_text(line, length, " ep=");
    length = append_unsigned(line, length, can_status->error_passive);
    length = append_text(line, length, "\r\n");
    transmit_line(line, length);
}

void UART_2_INST_IRQHandler(void)
{
    switch (DL_UART_getPendingInterrupt(UART_2_INST)) {
        case DL_UART_IIDX_RX:
            while (DL_UART_Main_isRXFIFOEmpty(UART_2_INST) == false) {
                BallProtocol_ReceiveByte(
                    (uint8_t)DL_UART_Main_receiveData(UART_2_INST));
            }
            break;
        default:
            break;
    }
}
