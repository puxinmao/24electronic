#include "usart.h"
#include "yb_protocol.h"


void uart0_init(void)
{
	//清除串口中断标志
	//Clear the serial port interrupt flag
	NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
	//使能串口中断
	//Enable serial port interrupt
	NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
}

//串口发送一个字节
//The serial port sends a byte
void uart0_send_char(unsigned char data)
{
	//当串口0忙的时候等待
	//Wait when serial port 0 is busy
	while( DL_UART_isBusy(UART_0_INST) == true );
	//发送
	//send
	DL_UART_Main_transmitData(UART_0_INST, data);
}

//串口发送字符串 Send string via serial port
void uart0_send_string(char* str)
{
    //当前字符串地址不在结尾 并且 字符串首地址不为空
		// The current string address is not at the end and the string first address is not empty
    while(*str!=0&&str!=0)
    {
        //发送字符串首地址中的字符，并且在发送完成之后首地址自增
		// Send the characters in the first address of the string, and increment the first address after sending.
        uart0_send_char(*str++);
    }
}

#if !defined(__MICROLIB)
//不使用微库的话就需要添加下面的函数
//If you don't use the micro library, you need to add the following function
#if (__ARMCLIB_VERSION <= 6000000)
//如果编译器是AC5  就定义下面这个结构体
//If the compiler is AC5, define the following structure
struct __FILE
{
	int handle;
};
#endif

FILE __stdout;

//定义_sys_exit()以避免使用半主机模式
//Define _sys_exit() to avoid using semihosting mode
void _sys_exit(int x)
{
	x = x;
}
#endif


//printf函数重定义
//printf function redefinition
int fputc(int ch, register FILE *stream)
{
	//当串口0忙的时候等待，不忙的时候再发送传进来的字符
	//Wait when serial port 0 is busy, and send the incoming characters when it is not busy
	while( DL_UART_isBusy(UART_0_INST) == true );
	
	DL_UART_Main_transmitData(UART_0_INST, ch);
	
	return ch;
}

//串口的中断服务函数
//Serial port interrupt service function
void UART_0_INST_IRQHandler(void)
{
	uint8_t receivedData = 0;
	
	//如果产生了串口中断
	//If a serial port interrupt occurs
	switch( DL_UART_getPendingInterrupt(UART_0_INST) )
	{
		case DL_UART_IIDX_RX://如果是接收中断	If it is a receive interrupt
			
			// 接收发送过来的数据保存	Receive and save the data sent
			receivedData = DL_UART_Main_receiveData(UART_0_INST);
		
			// 将保存的数据再发送出去 Send the saved data again
            uart0_send_char(receivedData);
		
			break;
		
		default://其他的串口中断	Other serial port interrupts
			break;
	}
}


///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////



void uart2_init(void)
{
	//清除串口中断标志
	//Clear the serial port interrupt flag
	NVIC_ClearPendingIRQ(UART_2_INST_INT_IRQN);
	//使能串口中断
	//Enable serial port interrupt
	NVIC_EnableIRQ(UART_2_INST_INT_IRQN);
}

//串口发送一个字节
//The serial port sends a byte
void uart2_send_char(unsigned char data)
{
	//当串口0忙的时候等待
	//Wait when serial port 0 is busy
	while( DL_UART_isBusy(UART_2_INST) == true );
	//发送
	//send
	DL_UART_Main_transmitData(UART_2_INST, data);
}

//串口发送字符串 Send string via serial port
void uart2_send_string(char* str)
{
    //当前字符串地址不在结尾 并且 字符串首地址不为空
		// The current string address is not at the end and the string first address is not empty
    while(*str!=0&&str!=0)
    {
        //发送字符串首地址中的字符，并且在发送完成之后首地址自增
		// Send the characters in the first address of the string, and increment the first address after sending.
        uart2_send_char(*str++);
    }
}


//串口的中断服务函数
//Serial port interrupt service function
void UART_2_INST_IRQHandler(void)
{
	uint8_t received = 0;
	
	//如果产生了串口中断
	//If a serial port interrupt occurs
	switch( DL_UART_getPendingInterrupt(UART_2_INST) )
	{
		case DL_UART_IIDX_RX://如果是接收中断	If it is a receive interrupt
			
			// 接收发送过来的数据保存	Receive and save the data sent
			received = DL_UART_Main_receiveData(UART_2_INST);
            // uart0_send_char(received);
            Pto_Data_Receive(received);
			
			break;
		
		default://其他的串口中断	Other serial port interrupts
			break;
	}
}




