#include "app_uart.h"

#include "ball_protocol.h"
#include "ti_msp_dl_config.h"

void AppUart_InitK230(void)
{
    NVIC_ClearPendingIRQ(UART_2_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_2_INST_INT_IRQN);
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
