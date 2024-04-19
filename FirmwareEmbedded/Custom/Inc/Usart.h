

#ifndef INC_UART_H_
#define INC_UART_H_

void UART_Init();
void UART_send_string(char *msg);
char UART_read_1byte();
void USART1_IRQHandler();
void dma2_uart1_rx_init();
void usart_printf(char* format,...);


#endif /* INC_UART_H_ */
