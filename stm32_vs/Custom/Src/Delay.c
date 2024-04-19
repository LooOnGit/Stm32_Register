
#include "D:\Dev_STM32\stm32_vs\Custom\Inc\clock.h"
#include "stdint.h"
#include "D:\Dev_STM32\stm32_vs\Custom\Inc\delay.h"
#define TIMER 1
#define SYS_TICK 2
#if DELAY_SRC != TIMER1 && DELAY_SRC != SYS_TICK
#error DELAY_SRC must be TIMER1 or SYS_TICK
#endif


uint32_t tim1_cnt = 0;




#if (DELAY_SRC  == TIMER1)

void TIM1_UP_TIM10_IRQHandler()
{
	uint16_t* SR = (uint16_t*)(0x40010010);
	*SR &= ~(1<<0);
	tim1_cnt++;
}
#else
void SysTick_Handler()
{
    tim1_cnt++;
}
#endif


void delay(uint32_t sec)
{
    uint32_t current_cnt = tim1_cnt; // dung de luu gia tri dem duoc cua bien tim1_cnt
    while( (uint32_t)(tim1_cnt - current_cnt) < sec );
}



void delay_init()
{
#if (DELAY_SRC == TIMER1)
   //set 1 sec for timer
   //rcc --> 16MHz -> psc(16) --> 1000000Hz
   //ARR = 1000

   clock_enable_APB2(TIM1_peripheral);
   uint16_t* ARR = (uint16_t*)(0x4001002c);
   uint16_t* PSC = (uint16_t*)(0x40010028);
   uint16_t* DIER = (uint16_t*)(0x4001000c);
   uint16_t* CR1 = (uint16_t*)(0x40010000);
   *ARR = 1000;
   *PSC = 100 - 1;


   *DIER |= 1<<0;//enable interrupt
   *CR1 |= 1<<0;

   //Now, counter begin 0 up to 1000, next auto reload ve 0

   uint32_t *ISER0 = (uint32_t*)(0xe000e100);
   *ISER0 |= (1<<25);

#else
   uint32_t *CSR = (uint32_t*)(0xe000e010);
   uint32_t *RVR = (uint32_t*)(0xe000e014);
   *RVR = 16000;
   *CSR |= (1<<0) | (1<<1);
#endif
}
