#include "usart.h"
#include "yb_protocol.h"

#define UART1_RX_BUFFER_SIZE    (32)

/* UART3 的 CAN-TTL 回包缓存；主循环用于等待张大头命令确认帧。 */
static volatile uint8_t s_uart1_rx_buffer[UART1_RX_BUFFER_SIZE];
static volatile uint8_t s_uart1_rx_head = 0;
static volatile uint8_t s_uart1_rx_tail = 0;

void uart0_init(void)
{
    NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
}

void uart0_send_char(unsigned char data)
{
    while (DL_UART_isBusy(UART_0_INST) == true) { }
    DL_UART_Main_transmitData(UART_0_INST, data);
}

void uart0_send_string(char *str)
{
    while ((str != 0) && (*str != 0))
    {
        uart0_send_char((unsigned char)*str++);
    }
}

#if !defined(__MICROLIB)
#if (__ARMCLIB_VERSION <= 6000000)
struct __FILE
{
    int handle;
};
#endif

FILE __stdout;

void _sys_exit(int x)
{
    (void)x;
}
#endif

int fputc(int ch, register FILE *stream)
{
    (void)stream;
    uart0_send_char((unsigned char)ch);
    return ch;
}

void UART_0_INST_IRQHandler(void)
{
    uint8_t received_data;

    switch (DL_UART_getPendingInterrupt(UART_0_INST))
    {
        case DL_UART_IIDX_RX:
            received_data = DL_UART_Main_receiveData(UART_0_INST);
            uart0_send_char(received_data);
            break;
        default:
            break;
    }
}

/* -------------------------------------------------------------------------- */
/* UART3: PB2=TX, PB3=RX，连接张大头 CAN-TTL 模块                              */
/* -------------------------------------------------------------------------- */
void uart1_init(void)
{
    s_uart1_rx_head = 0;
    s_uart1_rx_tail = 0;
    NVIC_ClearPendingIRQ(UART_1_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_1_INST_INT_IRQN);
}

void uart1_send_char(uint8_t data)
{
    while (DL_UART_isBusy(UART_1_INST) == true) { }
    DL_UART_Main_transmitData(UART_1_INST, data);
}

void uart1_send_bytes(const uint8_t *data, uint8_t length)
{
    uint8_t i;

    if (data == 0) {
        return;
    }

    for (i = 0; i < length; i++)
    {
        uart1_send_char(data[i]);
    }
}

void uart1_clear_rx(void)
{
    s_uart1_rx_tail = s_uart1_rx_head;
}

uint8_t uart1_read_byte(uint8_t *data)
{
    if ((data == 0) || (s_uart1_rx_tail == s_uart1_rx_head)) {
        return 0;
    }

    *data = s_uart1_rx_buffer[s_uart1_rx_tail];
    s_uart1_rx_tail = (uint8_t)((s_uart1_rx_tail + 1U) % UART1_RX_BUFFER_SIZE);
    return 1;
}

void UART_1_INST_IRQHandler(void)
{
    uint8_t received_data;
    uint8_t next_head;

    switch (DL_UART_getPendingInterrupt(UART_1_INST))
    {
        case DL_UART_IIDX_RX:
            received_data = DL_UART_Main_receiveData(UART_1_INST);
            next_head = (uint8_t)((s_uart1_rx_head + 1U) % UART1_RX_BUFFER_SIZE);
            if (next_head != s_uart1_rx_tail) {
                s_uart1_rx_buffer[s_uart1_rx_head] = received_data;
                s_uart1_rx_head = next_head;
            }
            break;
        default:
            break;
    }
}

/* -------------------------------------------------------------------------- */
/* UART1: PA8=TX, PA9=RX，接收 K230 检测数据                                  */
/* -------------------------------------------------------------------------- */
void uart2_init(void)
{
    NVIC_ClearPendingIRQ(UART_2_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_2_INST_INT_IRQN);
}

void uart2_send_char(unsigned char data)
{
    while (DL_UART_isBusy(UART_2_INST) == true) { }
    DL_UART_Main_transmitData(UART_2_INST, data);
}

void uart2_send_string(char *str)
{
    while ((str != 0) && (*str != 0))
    {
        uart2_send_char((unsigned char)*str++);
    }
}

void UART_2_INST_IRQHandler(void)
{
    uint8_t received_data;

    switch (DL_UART_getPendingInterrupt(UART_2_INST))
    {
        case DL_UART_IIDX_RX:
            received_data = DL_UART_Main_receiveData(UART_2_INST);
            Pto_Data_Receive(received_data);
            break;
        default:
            break;
    }
}
