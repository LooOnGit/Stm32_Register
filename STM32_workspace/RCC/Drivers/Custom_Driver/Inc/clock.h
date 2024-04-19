/*
 * clock.h
 *
 *  Created on: Mar 4, 2024
 *      Author: admin
 */

#ifndef CUSTOM_DRIVER_INC_CLOCK_H_
#define CUSTOM_DRIVER_INC_CLOCK_H_

typedef enum
{
	GPIOA_peripheral = 0,
	GPIOB_peripheral,
	GPIOC_peripheral,
	GPIOD_peripheral,
	GPIOE_peripheral,
	GPIOH_peripheral = 7,
	CRC_peripheral = 12,
	DMA1_peripheral = 21,
	DMA2_peripheral
}AHB1_peripheral_t;

typedef enum
{
	OTGFS_peripheral = 7
}AHB2_peripheral_t;


typedef enum
{
	TIM2_peripheral = 0,
	TIM3_peripheral,
	TIM4_peripheral,
	TIM5_peripheral,
	WWDG_peripheral = 11,
	SPI2_peripheral = 14,
	SPI3_peripheral,
	UART_peripheral = 17,
	I2C1_peripheral = 21,
	I2C2_peripheral,
	I2C3_peripheral,
	PWR_peripheral = 28
}APB1_peripheral_t;

typedef enum
{
	TIM1_peripheral=0,
	USART1_peripheral = 4,
	USART6_peripheral,
	ADC1_peripheral = 8,
	SDIO_peripheral = 11,
	SPI1_peripheral,
	SPI4_peripheral,
	SYSCFS_peripheral,
	TIM9_peripheral = 16,
	TIM10_peripheral,
	TIM11_peripheral,
	SPI5_peripheral = 20
}APB2_peripheral_t;


void clock_init();
void clock_enable_AHB1(AHB1_peripheral_t peripheral);
void clock_enable_AHB2(AHB2_peripheral_t peripheral);
void clock_enable_APB1(APB1_peripheral_t peripheral);
void clock_enable_APB2(APB2_peripheral_t peripheral);
#endif /* CUSTOM_DRIVER_INC_CLOCK_H_ */
