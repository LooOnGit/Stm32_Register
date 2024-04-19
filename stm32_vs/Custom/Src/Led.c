

#include "D:\Dev_STM32\stm32_vs\Custom\Inc\led.h"
#include "D:\Dev_STM32\stm32_vs\Custom\Inc\clock.h"
#include"stdint.h"
void led_init()
{
	clock_enable_AHB1(GPIOD_peripheral);
	uint32_t* GPIOD_MODER = (uint32_t*)(0x40020c00);
	* GPIOD_MODER |= (0b01 << 24) | (0b01 << 26) | (0b01 << 28) | (0b01 << 30);
	* GPIOD_MODER &=~(0b1111 << 12);
}

void led_control(led_color led,led_stage stage)
{
	uint32_t * GPIOD_ODR = (uint32_t*)(0x40020c14);
	if(stage  == 1)
	{
		*GPIOD_ODR |= 1<< led;
	}
	if(stage  == 0)
	{
		*GPIOD_ODR &= ~(1<< led);
	}

}
