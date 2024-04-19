#include <stdint.h>
#include "D:\Dev_STM32\stm32_vs\Custom\Inc\led.h"
#include "D:\Dev_STM32\stm32_vs\Custom\Inc\delay.h"
#include "D:\Dev_STM32\stm32_vs\Custom\Inc\clock.h"

void SystemInit ();
void setup()
{
    led_init();
    delay_init();
    clock_init();
}

int main(){
    setup();
    while(1){
        led_control(GREEN, ON);
        delay(1000);
        led_control(GREEN, OFF);
        delay(1000);
    }
    return 0;
}

void SystemInit (){
    
}