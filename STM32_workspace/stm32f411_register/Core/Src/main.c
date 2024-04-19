/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include <stdarg.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
char recv_data[32];
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
char new_fw_data[5748];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define GPIOA_BASE_ARR 0x40020000
#define GPIOD_BASE_ARR 0x40020C00
void button_init(){
	__HAL_RCC_GPIOA_CLK_ENABLE();
	//set PA0 in floating input
	uint32_t* GPIOA_MODER = (uint32_t*)(GPIOA_BASE_ARR + 0x00);
	*GPIOA_MODER &=~(0b11 << 0); //set PA0 in INPUT MODE
	uint32_t* GPIOA_PUPDR = (uint32_t*)(GPIOA_BASE_ARR + 0x0C);
	*GPIOA_PUPDR &=~(0b11 << 0); //set no pull-up, pull-down (floating)
}

char read_button_state(){
	uint32_t* GPIOA_IDR = (uint32_t*)(GPIOA_BASE_ARR + 0x10);
	return (*GPIOA_IDR>>0 & 1);
}

void leds_init(){
	__HAL_RCC_GPIOD_CLK_ENABLE();
	uint32_t* GPIOD_MODER = (uint32_t*)(GPIOD_BASE_ARR + 0x00);
	*GPIOD_MODER |= (0b01 << 24);//set PD12 in OUTPUTv
	*GPIOD_MODER |= (0b01 << 26);//set PD13 in OUTPUT
	*GPIOD_MODER |= (0b01 << 28);//set PD14 in OUTPUT
	*GPIOD_MODER |= (0b01 << 30);//set PD15 in OUTPUT
	*GPIOD_MODER &= ~(0b1111 << 12);//set PD12, PD13, PD14, PD15 in push-pull
}

void led_control(char led_state){
	uint32_t* GPIOD_ODR = (uint32_t*)(GPIOD_BASE_ARR + 0x14);
	if(led_state==1){
		*GPIOD_ODR |= 1<<12;
	}else{
		*GPIOD_ODR &=~(1<<12);
	}
}

void exti0_init()
{
	__HAL_RCC_GPIOA_CLK_ENABLE();
	uint32_t* EXTI_IMR = (uint32_t*)(0x40013c00 + 0x00);
	uint32_t* EXTI_RTSR = (uint32_t*)(0x40013c00 + 0x08);

	*EXTI_RTSR |= (1 << 0);//set Rising exti0
	*EXTI_IMR |= (1 << 0);//enable exti0

	//Setup NVIC
	uint32_t* NVIC_ISER0 = (uint32_t*)(0xe000e100);//ISER2 thì ở địa chỉ 0xE00E108
	*NVIC_ISER0 |= (1<<6); //enable interrupt for event in position 6 vector table (EXTI0)
}

void EXTI0_IRQHandler()
{
	__asm("nop");
	uint32_t* EXTI_PR = (uint32_t*)(0x40013c00 + 0x14);
	*EXTI_PR |= (1<<0); //clear interrupt event
}

void custom_exti0_handler()
{
	__asm("nop");
		uint32_t* EXTI_PR = (uint32_t*)(0x40013c00 + 0x14);
		*EXTI_PR |= (1<<0); //clear interrupt event
}

void UART1_init()
{
	__HAL_RCC_GPIOB_CLK_ENABLE();
	//set PB6 as UART1_TX(AF07) and PB7 as UART1_RX(AF07)
	uint32_t* MODER = (uint32_t*)(0x40020400);
	uint32_t* AFRL = (uint32_t*)(0x40020420);
	*MODER &=~(0b1111 << 12); //clear 4 bit 0000
	*MODER |= (0b10 << 12)|(0b10 << 14);//set alternate function
	*AFRL &= ~(0xff << 24);
	*AFRL |= (7 << 24) | (7 << 28);//set AF07 for PB6 and PB7

	//UART
	//baun rate: 9600
	// +frame:
	// +date len: 8byte
	// +parity (none/add/even): none
	__HAL_RCC_USART1_CLK_ENABLE();
	uint32_t* BRR = (uint32_t*)(0x40011008);
	uint32_t* CR1 = (uint32_t*)(0x4001100c);
	*BRR = (104 << 4) | (3 << 0);//set baunrate in 9600
	*CR1 &= ~(1 << 10); //disable parity
	*CR1 &= ~(1 << 12);//set data lenght 8 bits data
	*CR1 |= (1 << 13) | (1 << 2) | (1 << 3);//enable transmiter, reciver, uart

#if 0
	//enable RXNE interrupt -> ưhen RXNE is set, UART1 generate interrupt event send to NVIC
	*CR1 |= (1<<5);

	//NVIC accept interrupt event, which is send from UART1
	uint32_t* ISER1 = (uint32_t*)(0xE000E104); //37 thuoc iser1 vì iser0 tới 32
	*ISER1 |= 1 << (37 - 32);//37-32 = 5 vì tới 32 đã là của iser0
#else
	//when RXNE is set, send signal to DMA2, DMA2 move data to RAM
//	uint32_t* CR3 = (uint32_t*)(0x40011014);
//	*CR3 |= (1 << 6);

#endif

}

void UART1_Send_1byte(char data){

	//trinh tu read SR to Write DR
	uint32_t* SR = (uint32_t*)(0x40011000);
	uint32_t* DR = (uint32_t*)(0x40011004);
	while(((*SR >> 7) & 1) == 0);//Wait DR empty
	*DR = data;						//write data to DR to UART1 transfer data via TX(PB6)
	while(((*SR >> 6) & 1) == 0);//wait transmitter of UART1 complete transmit
	*SR &=~(1<<6);				//clear TC flag
}


void UART1_Send_String(char* msg)
{
	int msg_len = strlen(msg);
	for(int i = 0; i< msg_len; i++){
		UART1_Send_1byte(msg[i]);
	}
}

char UART1_Recv_1Byte()
{
	uint32_t* SR = (uint32_t*)(0x40011000);
	uint32_t* DR = (uint32_t*)(0x40011004);
	while(((*SR >> 5) & 1) == 0);   //Wait RXNE flag to read recv data
	char recv_data = *DR;			//read recv data
	return recv_data;
}


void uart_printf(char *format, ...){
	va_list aptr;
	va_start(aptr, format);
	char buffer[128] = {0};
	vsprintf(buffer, format, aptr);
	UART1_Send_String(buffer);
	va_end(aptr);
}

int rx_index=0;

void USART1_rx_hand(){
	uint32_t* DR = (uint32_t*)(0x40011004);
	recv_data[rx_index++] = *DR;
	if(strstr(recv_data, "LED_ON") != NULL){
		led_control(1);
		rx_index = 0;
		memset(recv_data, 0, sizeof(rx_index));
	}
	else if(strstr(recv_data, "LED_OFF") != NULL){
		led_control(0);
		rx_index = 0;
		memset(recv_data, 0, sizeof(rx_index));
	}

}
#define DMA2_ADDRESS 0x40026400
void dma2_uart1rx_init(){
	__HAL_RCC_USART1_CLK_ENABLE();
	//user DMA2 stream5 channel 4 --> UART1_Rx (DMA mapping table)
	uint32_t* DMA_S5CR = (uint32_t*)(DMA2_ADDRESS + 0x10 + 0x18 * 5);
	uint32_t* DMA_S5NDTR = (uint32_t*)(DMA2_ADDRESS + 0x14 + 0x18 * 5);
	uint32_t* DMA_S5PAR = (uint32_t*)(DMA2_ADDRESS + 0x18 + 0x18 * 5);
	uint32_t* DMA_S5M0AR = (uint32_t*)(DMA2_ADDRESS + 0x1C + 0x18 * 5);
	//recv: 7bytes
	//from: UART_DR   (0x40011004)
	//to: recv_data (0x20000428
	*DMA_S5NDTR = 7;
	*DMA_S5PAR = 0x40011004;
	*DMA_S5M0AR = recv_data;

	*DMA_S5CR |= (0b100 << 25); //select channel 4 for stream 5
	*DMA_S5CR |= (0b1 << 10); //enable Memory increment mode //mỗi lần nhận dữ liệu thì tăng lên tránh ghi đè
	*DMA_S5CR |= (0b1 << 8); // enable Circular mode // khi nhận đủ 7 byte thì nó nhận tiếp và bỏ lại vị trí 1 như ring buffer
	*DMA_S5CR |= (0b1 << 4);//enable transfer complete interrupt
	*DMA_S5CR |= (0b1 << 0);//enable DMA2 stream 5

	uint32_t* ISER2 = (uint32_t*)(0xE000E108);//NVIC register summary
	*ISER2 |= 1 << (68 - 64); // vector table
}

void DMA2_Stream5_IRQHandler(){
	__asm("nop");
	//clean interrupt flash - > transfer complete interrupt
	uint32_t* HIFCR = (uint32_t*)(DMA2_ADDRESS + 0x0C);
	*HIFCR |= 1 << 11; // DMA
	if(strstr(recv_data, "LED_ON") != NULL){
		led_control(1);
	}
	else if(strstr(recv_data, "LED_OFF") != NULL){
		led_control(0);
	}
	memset(recv_data, 0, 7);
}

#define FLASH_ADDR_BASE 0x40023C00
__attribute__ ((section (".function_in_ram"))) void Flash_Erase_Sector(char sector){ // du an thu te thif phai co tra ve ma loi
	uint32_t* FLASH_SR = (uint32_t*)(FLASH_ADDR_BASE + 0x0C);
	uint32_t* FLASH_CR = (uint32_t*)(FLASH_ADDR_BASE + 0x10);
	uint32_t* FLASH_KEYR = (uint32_t*)(FLASH_ADDR_BASE + 0x04);
	//Check that no Flash memory operation is going. wait BSY
	while(((*FLASH_SR >> 16) &1 ) == 1);
	if(((*FLASH_CR >> 31) & 1) == 1){
		//unlock CR
		*FLASH_KEYR =  0x45670123;
		*FLASH_KEYR = 0xCDEF89AB;
	}
	*FLASH_CR |= (1 << 1) | (sector << 3);
	*FLASH_CR |= (1<<16); //start erase operation
	while(((*FLASH_SR >> 16) & 1) == 1);//wait BSY is clean
	*FLASH_CR &= ~(1 << 1);
}

__attribute__ ((section (".function_in_ram"))) void Flash_Program(uint8_t* addr, uint8_t value){
	uint32_t* FLASH_SR = (uint32_t*)(FLASH_ADDR_BASE + 0x0C);
	uint32_t* FLASH_CR = (uint32_t*)(FLASH_ADDR_BASE + 0x10);
	uint32_t* FLASH_KEYR = (uint32_t*)(FLASH_ADDR_BASE + 0x04);
	if(((*FLASH_CR >> 31) & 1) == 1){
		//unlock CR
		*FLASH_KEYR =  0x45670123;
		*FLASH_KEYR = 0xCDEF89AB;
	}
	//Check that no Flash memory operation is going. wait BSY
	while(((*FLASH_SR >> 16) &1 ) == 1);
	//set the PG bit the FLASH_CR register
	*FLASH_CR |= (1 << 0);
	*addr = value;
	while(((*FLASH_SR >> 16) & 1) == 1);
	*FLASH_CR &= ~(1<<0);
}

__attribute__ ((section (".function_in_ram"))) void update_firmware()
{
	/* disable system tick */
	uint32_t* SYST_CR = (uint32_t*)(0xe000e010);
	*SYST_CR &= ~1;
	//erase flash in section 0*/
	Flash_Erase_Sector(0);
	//from section 0 with data in new_fw_data array (RAM)
	char *flash_addr = (char*) 0x08000000;
	for(int i = 0; i < sizeof(new_fw_data); i++)
	{
		Flash_Program(flash_addr + i, new_fw_data[i]);
	}
	//reset system
	uint32_t* AIRCR = (uint32_t*)0xE000ed0c;
	*AIRCR = (0x5fa << 16) | (1 << 2);

}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */
  leds_init();
//  exti0_init();
  UART1_init();
//  dma2_uart1rx_init();
//  char button_state = 0;
  //copy vector table to RAM (start from 0x2000 0000)
//  uint8_t* ram = (uint8_t*)0x20000000;
//  uint8_t* vttb = (uint8_t*)0x00000000;//vector table
//  for(int i = 0; i < 0x198;i++){
//	  ram[i]= vttb[i];
//  }
//  memcpy(0x20000000, 0x00000000, 0x198);
//  //talk with arm, when interrupt event goto RAM to find vector table
//  uint32_t* VTOR = (uint32_t*)0xe000ed08;
//  *VTOR = 0x20000000;
//
//  //register function handler address into 0x58
//  uint32_t* function_address = (uint32_t*)0x20000058;
//  *function_address = (uint32_t)(custom_exti0_handler) | 1;
//
//  function_address = (uint32_t*)0x200000D4;
//  *function_address = (uint32_t)(USART1_rx_hand) | 1;
//
//  function_address = 0x20000150;//interrupt and event in vector table
//  *function_address = (uint32_t)(DMA2_Stream5_IRQHandler) | 1;
  /* USER CODE END 2 */

  uart_printf("please send %d byte to STM32\r\n", sizeof(new_fw_data));
  for(int i = 0; i < sizeof(new_fw_data); i++){
	  new_fw_data[i] = UART1_Recv_1Byte();
  }
  uart_printf("start update firware, Dont power off device\r\n");
  update_firmware();


  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
	  HAL_Delay(1000);
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
	  HAL_Delay(1000);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
