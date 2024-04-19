typedef enum
{
    GPIOA_peripheral = 0,
    GPIOB_peripheral = 1,
    GPIOC_peripheral = 2,
    GPIOD_peripheral = 3,
    GPIOE_peripheral = 4,
    GPIOH_peripheral = 7 ,
    CRC_peripheral = 12,
    DMA1_peripheral = 21,
    DMA2_peripheral =22
}AHB1_peripheral_t;
void clock_enable_AHB1(int peripheral);


typedef enum
{
    OTGFS_peripheral = 7,
    
}AHB2_peripheral_t;
void clock_enable_AHB2(int peripheral);


typedef enum
{
    TIM2_peripheral = 0,
    TIM3_peripheral = 1,
    TIM4_peripheral = 2,
    TIM5_peripheral = 3,
    WWDG_peripheral = 11,
    SPI2_peripheral = 14,
    SPI3_peripheral = 15,
    USART2_peripheral = 17,
    I2C1_peripheral = 21,
    I2C2_peripheral = 22,
    I2C3_peripheral =23,
    PWR_peripheral=28
}APB1_peripheral_t;
void clock_enable_APB1(int peripheral);

typedef enum
{
    TIM1_peripheral = 0,
    USART1_peripheral =4 ,
    USART6_peripheral =5,
    ADC1_peripheral = 8 ,
    SDIO_peripheral =11 ,
    SPI1_peripheral = 12,
    SPI4_peripheral = 13,
    SYSCFG_peripheral =15,
    TIM9_peripheral =16,
    TIM10_peripheral =17,
    TIM11_peripheral =18 ,
    SPI5_peripheral = 20
}APB2_peripheral_t;
void clock_enable_APB2(int peripheral);

void clock_init();



