#include "usart.h"
#include "yb_protocol.h"

void uart2_init(void)
{
    NVIC_ClearPendingIRQ(UART_2_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_2_INST_INT_IRQN);
}

void UART_2_INST_IRQHandler(void)
{
    uint8_t received_data;

    switch (DL_UART_getPendingInterrupt(UART_2_INST)) {
        case DL_UART_IIDX_RX:
            received_data = DL_UART_Main_receiveData(UART_2_INST);
            Pto_Data_Receive(received_data);
            break;
        default:
            break;
    }
}
