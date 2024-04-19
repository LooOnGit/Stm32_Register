#include <stdint.h>
#include "D:\Dev_STM32\FirmwareEmbedded\Custom\Inc\clock.h"
#include "D:\Dev_STM32\FirmwareEmbedded\Custom\Inc\ADC.h"
#define ADC_ADDRESS_BASE 0x40012000
#define ADC_IN 16
#define ADC_IN1 1


void adc_init()
{
    clock_enable_AHB1(GPIOA_peripheral);
    uint32_t *GPIOA_MODER = (uint32_t*)(0x40012000);
    *GPIOA_MODER |= (0b11 <<2);
    clock_enable_APB2(ADC1_peripheral);
    uint32_t * ADC_CR2 = (uint32_t*)(0x40012008);
    uint32_t * ADC_JSQR = (uint32_t*)(0x40012038);
    uint32_t * ADC_CCR = (uint32_t*)(ADC_ADDRESS_BASE + 0x300 +0x04);
    uint32_t * ADC_SMPR1 = (uint32_t*)(0x4001200c);

    * ADC_SMPR1 |= (0b111 << 18); //using 12 bit resolution (request 15 cycles for convert)
    * ADC_JSQR &= ~(0b11 << 20);  
     // Injected sequence length = 00: 1 convertion
    //set source for JSQ4 is temp sensor
    
    * ADC_JSQR |= (ADC_IN << 15);  
    * ADC_JSQR |= (ADC_IN1 << 10);
    
    * ADC_CR2 |= 1<<0; //Enable ADC
    * ADC_CCR |= 1<< 23;  //enabel temp sensor
}

float adc_get_temp_ss()
{
    float temp = 0;
    float vin =0;
    uint16_t raw_data =0;
    uint32_t * ADC_SR = (uint32_t*)(ADC_ADDRESS_BASE + 0x00);
    uint32_t * ADC_CR2 = (uint32_t*)(ADC_ADDRESS_BASE+ 0x08);
    uint32_t * ADC_JDR1 = (uint32_t*)(ADC_ADDRESS_BASE+ 0x3c);

    //trigger ADC convert
    * ADC_CR2 |= 1<<22;

    //wait until end of convert 
    while (((*ADC_SR >> 2) & 1)==0);
    *ADC_SR &= ~(1<<2);
    raw_data = *ADC_JDR1;

    vin = (raw_data * 3000) / 4095.0;
    temp =((vin - 760 ) / 2.5 ) + 25;
    return temp;
}

float adc_get_vin_pal()
{
    
    float vin =0;
    uint16_t raw_data =0;
        uint32_t * ADC_SR = (uint32_t*)(ADC_ADDRESS_BASE + 0x00);
        uint32_t * ADC_CR2 = (uint32_t*)(ADC_ADDRESS_BASE+ 0x08);
        uint32_t * ADC_JDR1 = (uint32_t*)(ADC_ADDRESS_BASE+ 0x3c);

    //trigger ADC convert
    * ADC_CR2 |= 1<<22;

    //wait until end of convert 
    while (((*ADC_SR >> 2) & 1)==0);
    *ADC_SR &= ~(1<<2);
    raw_data = *ADC_JDR1;

    vin = (raw_data * 3000) / 4095.0;
    return vin;
}
