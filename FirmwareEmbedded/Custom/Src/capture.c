#include <stdint.h>
#include "D:\Dev_STM32\FirmwareEmbedded\Custom\Inc\clock.h"


#define TIM5_ADDR_BASE 0x40000C00 
#define GPIOA_ADDR_BASE 0x40020000
void capture_init()
{
    //set up gpio
    clock_enable_AHB1(GPIOA_peripheral);
    uint32_t* GPIOA_MODER = (uint32_t*)(GPIOA_ADDR_BASE + 0x00);
    uint32_t* GPIOA_AFRL = (uint32_t*)(GPIOA_ADDR_BASE + 0x20);
    *GPIOA_MODER &= ~(0b11 << 0);
    *GPIOA_MODER |= (0b10 << 0);
    *GPIOA_AFRL |= (2<<0);

    clock_enable_APB1(TIM5_peripheral);
    uint16_t* TIMx_CR1 = (uint16_t*)(TIM5_ADDR_BASE + 0x00);
    uint32_t* TIM5_ARR = (uint32_t*)(TIM5_ADDR_BASE + 0x2C);
    uint32_t* TIM5_PSC = (uint32_t*)(TIM5_ADDR_BASE + 0x28);
    *TIM5_ARR = 0xffffffff;
    *TIM5_PSC = 16 -1;

    /* timer5 channel 1 - capture input*/
    uint16_t* TIM5_CCMR1 = (uint16_t*)(TIM5_ADDR_BASE + 0x18);
    uint16_t* TIM5_CCER = (uint16_t*)(TIM5_ADDR_BASE + 0x20);
    uint16_t* TIMx_SMCR = (uint16_t*)(TIM5_ADDR_BASE + 0x08);

    *TIM5_CCMR1 |= (0b01 << 0); //set channel 1 in capture input mode
    *TIM5_CCER &= ~((1<< 3) | (1<< 1)); //set rising as trigger to 
    *TIM5_CCER |= (1<< 0); //enable channel 1

    *TIM5_CCMR1 |= (0b10 << 8); //set channel 2 in capture input mode(map wit TI1)
    *TIM5_CCER &= ~(1<< 7); *TIM5_CCER |= (1 << 5); //set falling as trigger to
    *TIM5_CCER |= (1<< 4); //enable channel 2

    *TIMx_SMCR |= (0b100 << 0) | (0b101 << 4); //reset cnt when rising
    *TIMx_CR1 |= (1<<0);
}


uint32_t capture_frequence()
{
    uint32_t* TIM5_CCR1 = (uint32_t*)(TIM5_ADDR_BASE + 0x34);
    return *TIM5_CCR1;
}
uint32_t capture_pulse_width()
{
    uint32_t* TIM5_CCR2 = (uint32_t*)(TIM5_ADDR_BASE + 0x38);
    return *TIM5_CCR2;
}
