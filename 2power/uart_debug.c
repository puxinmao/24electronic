/*
 * uart_debug.c - 串口调试实现
 */
#include "uart_debug.h"
#include "ti_msp_dl_config.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* ========== 调试 UART (UART_1, TX=PA10, RX=PA11) ========== */

void DBG_SendChar(char c)
{
    DL_UART_Main_transmitDataBlocking(UART_1_INST, (uint8_t)c);
}

void DBG_SendStr(const char *str)
{
    while (*str) {
        DL_UART_Main_transmitDataBlocking(UART_1_INST, (uint8_t)*str++);
    }
}

void DBG_SendNum(int32_t val)
{
    char buf[12];
    int i = 0;

    if (val < 0) {
        DBG_SendChar('-');
        val = -val;
    }

    if (val == 0) {
        DBG_SendChar('0');
        return;
    }

    char tmp[12];
    int j = 0;
    while (val) {
        tmp[j++] = '0' + (val % 10);
        val /= 10;
    }
    while (j) buf[i++] = tmp[--j];
    buf[i] = '\0';
    DBG_SendStr(buf);
}

void DBG_SendHex8(uint8_t val)
{
    static const char hex[] = "0123456789ABCDEF";
    DBG_SendChar(hex[val >> 4]);
    DBG_SendChar(hex[val & 0x0F]);
}

void DBG_SendHex32(uint32_t val)
{
    DBG_SendHex8((val >> 24) & 0xFF);
    DBG_SendHex8((val >> 16) & 0xFF);
    DBG_SendHex8((val >> 8)  & 0xFF);
    DBG_SendHex8( val        & 0xFF);
}

void DBG_Printf(const char *fmt, ...)
{
    char buf[128];
    va_list args;
    va_start(args, fmt);
    /* 使用编译器自带的 vsnprintf */
    int len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if (len > 0) {
        DBG_SendStr(buf);
    }
}

/* ========== 外部数据 UART (UART_0, RX=PA9) ========== */

void UART_RX_Init(void)
{
    /* SysConfig 已在 SYSCFG_DL_UART_0_init() 中完成初始化 */
}

bool UART_RX_Available(void)
{
    return !DL_UART_Main_isRXFIFOEmpty(UART_0_INST);
}

uint8_t UART_RX_Read(void)
{
    return DL_UART_Main_receiveData(UART_0_INST);
}
