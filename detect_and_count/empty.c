#include "ti_msp_dl_config.h"
#include "stdio.h"
#include "usart.h"
#include "yb_protocol.h"


int main(void)
{
    SYSCFG_DL_init();
    uart0_init();
    uart2_init();

    /* 初始化完成提示（通过调试串口 UART0 输出） */
    uart0_send_string("MSPM0G3507 Ball Detect Receiver\r\n");
    uart0_send_string("UART2(PA8/PA9) waiting for K230...\r\n");

    /* 主循环保持空 —— 所有数据处理在 UART2 ISR -> Pto_Data_Parse 中完成 */
    /* 参考所有 MSPM0-K230 官方例程的 empty main loop 模式 */
    while (1)
    {
    }
}
