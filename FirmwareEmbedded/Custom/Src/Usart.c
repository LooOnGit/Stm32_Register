
#include"D:\Dev_STM32\FirmwareEmbedded\Custom\Inc\clock.h"
#include"D:\Dev_STM32\FirmwareEmbedded\Custom\Inc\Usart.h"
#include"stdint.h"
#include"string.h"
#include<stdarg.h>
#include<stdio.h>


char recv_data[32];
void UART_Init()
{
    clock_enable_AHB1(GPIOB_peripheral);
    clock_enable_APB2(USART1_peripheral);
	uint32_t *MODERB = (uint32_t*)(0x40020400);
	*MODERB &= ~(0b1111<<12); // sure to clear all bit to 0
	*MODERB |= (0b10<<12) | (0b10<<14); //PB6, PB7 set alternate function mode
	uint32_t *AFRL = (uint32_t*)(0x40020420); //set AF7
	*AFRL &= ~(0xff<<24);
	*AFRL |= (7<<28) | (7<<24);
	// UART:
	// baudrate: 9600
	// frame: + Data length: 8 bit
	//		  + parity: (none/even/odd)  none
    
	//__HAL_RCC_USART1_CLK_ENABLE();
	uint32_t *BRR = (uint32_t*)(0x40011008);
	uint32_t *CR1 = (uint32_t*)(0x4001100c);
	*BRR |= (162<<4) | (12<<0); //set Baudrate in 9600 at 25Mhz 
	*CR1 &= ~(1<<10); // disable parity
	*CR1 &= ~(1<<12); // set data length in 8 bit data
	*CR1 |= (1<<2)|(1<<3)|(1<<13 ); // enable TX,RT, UART

/*#if 0
	*CR1 |= (1<<5); //RXNE interrupt enable

	uint32_t *NVIC_ISER1 = (uint32_t*)(0xE000E104);
	*NVIC_ISER1 |= (1<<5);
#else
when RXNE is set, send a signal to DMA2, DMA2 transfer to RAM
	uint32_t *CR3 = (uint32_t*)(0x40011000 + 0x14); // enable DMA receiver
	*CR3 |= (1<<6);
#endif */
}
static void UART_send_1byte(char data)
{
	uint32_t *SR = (uint32_t*)(0x40011000);
	uint32_t *DR = (uint32_t*)(0x40011004);
	while(((*SR>>7) & 1) == 0); // wait DR emty
	*DR = data;
	while(((*SR>>6) & 1) == 0); // wait transmitter is complete
	*SR &= ~(1<<6); //clear TC flag
}


void UART_send_string(char *msg)
{
int data_length = strlen(msg);
	for(int i=0;i<data_length;i++)
	{
	UART_send_1byte(msg[i]);
	}
}

char UART_read_1byte()
{
	
	uint32_t *SR = (uint32_t*)(0x40011000);
	uint32_t *DR = (uint32_t*)(0x40011004);
	while(((*SR>>5) & 1) == 0); // wait DR emty
	char recv_data = *DR;
	return recv_data;
}
int rx_index = 0;
void USART1_IRQHandler()
{
	uint32_t *DR = (uint32_t*)(0x40011004);
	recv_data[rx_index++] = *DR;
}
void uart_rx_handler()
{
	uint32_t *DR = (uint32_t*)(0x40011004);
	recv_data[rx_index++] = *DR;
}
void dma2_stream5_handler()
{
	uint32_t *DR = (uint32_t*)(0x40011004);
	recv_data[rx_index++] = *DR;
	uint32_t *DMA_HIFCR = (uint32_t*)(0x40026400 + 0x0C); // clear interrupt flag
	*DMA_HIFCR |= (1<<11);
}
void dma2_uart1_rx_init()
{
    clock_enable_AHB1(DMA2_peripheral);

	uint32_t *DMA_S5CR = (uint32_t*)(0x40026400 + 0x10 + 0x18*5); // set DMA2 stream 5 channel 4
	uint32_t *DMA_S5NDTR = (uint32_t*)(0x40026400 + 0x14 + 0x18*5); //stream 5 number of data register
	uint32_t *DMA_S5PAR = (uint32_t*)(0x40026400 + 0x18 + 0x18*5); //stream 5 peripheral address register
	uint32_t *DMA_S5M0AR = (uint32_t*)(0x40026400 + 0x1C + 0x18*5);//stream 5 memory 0 address register


	/* * recv: 7 byte
	 * from: UART DR: 0x40011004
	 * to rec_data:  0x20000428
	 */
	*DMA_S5NDTR = 7;
	*DMA_S5PAR = 0x40011004;
	*DMA_S5M0AR = (uint32_t)recv_data;
	*DMA_S5CR |= (0b100<<25); // use DMA2 stream 5 channel 4
	*DMA_S5CR |= (1<<10); //enable memory increment mode
	*DMA_S5CR |= (1<<0); //enable stream
	*DMA_S5CR |= (1<<8); // circle mode
	*DMA_S5CR |= (1<<4); //enable interrupt

	uint32_t *NVIC_ISER2 = (uint32_t*)(0xE000E108); //enable NVIC DMA2 stream 5
	*NVIC_ISER2 |= (1<<4);
}


void usart_printf(char* format,...)
{
	char buf[1024] = {0};
	char buf_len = 0;
	va_list ap;
	va_start(ap, format);
	vsprintf(buf, format, ap);
	va_end(ap);
	buf_len = strlen(buf);
	for (int  i = 0; i < buf_len; i++)
	{
		UART_send_1byte(buf[i]);
	}

}