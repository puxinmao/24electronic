#ifndef	__USART_H__
#define __USART_H__

#include "ti_msp_dl_config.h"
#include "stdio.h"


void uart0_init(void);
void uart0_send_char(unsigned char data);
void uart0_send_string(char* str);


void uart2_init(void);
void uart2_send_char(unsigned char data);
void uart2_send_string(char* str);

#endif
