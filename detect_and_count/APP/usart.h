#ifndef __USART_H__
#define __USART_H__

#include "ti_msp_dl_config.h"
#include "stdio.h"
#include "stdint.h"

void uart0_init(void);
void uart0_send_char(unsigned char data);
void uart0_send_string(char *str);

/* UART3（PB2=TX, PB3=RX）：连接张大头 CAN-TTL 模块 */
void uart1_init(void);
void uart1_send_char(uint8_t data);
void uart1_send_bytes(const uint8_t *data, uint8_t length);
void uart1_clear_rx(void);
uint8_t uart1_read_byte(uint8_t *data);

/* UART1（PA8=TX, PA9=RX）：连接 K230 */
void uart2_init(void);
void uart2_send_char(unsigned char data);
void uart2_send_string(char *str);

#endif
