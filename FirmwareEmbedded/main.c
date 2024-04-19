/*
 * main.c
 *
 *  Created on: Oct 2, 2023
 *      Author: ACER
 */


#include <stdint.h>
#include ".\Custom\Inc\led.h"
#include "D:\Dev_STM32\FirmwareEmbedded\Custom\Inc\delay.h"
#include "D:\Dev_STM32\FirmwareEmbedded\Custom\Inc\clock.h"
#include "D:\Dev_STM32\FirmwareEmbedded\Custom\Inc\capture.h"
#include "D:\Dev_STM32\FirmwareEmbedded\Custom\Inc\Usart.h"
#include "D:\Dev_STM32\FirmwareEmbedded\Custom\Inc\ADC.h"
#include "D:\Dev_STM32\FirmwareEmbedded\FreeRTOS-LTS\FreeRTOS\FreeRTOS-Kernel\include\FreeRTOS.h"
#include "D:\Dev_STM32\FirmwareEmbedded\FreeRTOS-LTS\FreeRTOS\FreeRTOS-Kernel\include\task.h"
#include"D:\Dev_STM32\FirmwareEmbedded\FreeRTOS-LTS\FreeRTOS\FreeRTOS-Kernel\include\FreeRTOS.h"
#include<D:\Dev_STM32\FirmwareEmbedded\FreeRTOS-LTS\FreeRTOS\FreeRTOS-Kernel\include\task.h>
#include<D:\Dev_STM32\FirmwareEmbedded\FreeRTOS-LTS\FreeRTOS\FreeRTOS-Kernel\portable\GCC\ARM_CM4F\portmacro.h>
#include<D:\Dev_STM32\FirmwareEmbedded\FreeRTOS-LTS\FreeRTOS\FreeRTOS-Kernel\include\queue.h>
#include<D:\Dev_STM32\FirmwareEmbedded\FreeRTOS-LTS\FreeRTOS\FreeRTOS-Kernel\include\event_groups.h>
#include<D:\Dev_STM32\FirmwareEmbedded\FreeRTOS-LTS\FreeRTOS\FreeRTOS-Kernel\include\semphr.h>




QueueHandle_t temp_queue;
EventGroupHandle_t temp_event;
SemaphoreHandle_t uart_lock;
void vApplicationMallocFailedHook(){
}

void vApplicationTickHook(){

}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char * pcTaskName){


}

void vApplicationIdleHook(){

}

BaseType_t xTimerCreateTimerTask() {
    return pdPASS; 
}



void func_1()
{
	while(1)
	{
	led_control(RED, ON);
	vTaskDelay(1000);
	led_control(RED, OFF);
	vTaskDelay(1000);
	}

}

void func_2()
{
	while(1)
	{
	led_control(GREEN, ON);
	vTaskDelay(2000);
	led_control(GREEN, OFF);
	vTaskDelay(2000);
	}
}
float commom_memory;
void func_3(void * param){

	for(;;){
		// wait until temp_event is set
		xEventGroupSync(temp_event, 0, 1, 0xffffffff);

		float temp = 0;

		xSemaphoreTake(uart_lock, 0xffffffff);
		xQueueReceive(temp_queue, &temp, 10000);
		usart_printf("\033[0;36m [Taks 3]\033[0m Temperature: ", temp);
		while(uxQueueMessagesWaiting(temp_queue) > 0)
		{
			xQueueReceive(temp_queue, &temp, 0);
			usart_printf("%.2f, ", temp);
		}

		usart_printf("\b\b]\r\n");
		xSemaphoreGive(uart_lock);
		vTaskDelay(2000);
	}
}
void func_4(void * param){

	 int measua_cnt = 0;
	 while (1)
	 {
		float temp = adc_get_temp_ss();
		xQueueGenericSend(temp_queue, &temp,10000, queueSEND_TO_BACK);
		vTaskDelay(100);
		if(++ measua_cnt >= 10){
			measua_cnt = 0;
			// set event
			xEventGroupSetBits(temp_event,1);
		}
	 }

}

void SystemInit()
{}
	
void setup()
{
    adc_init();
	led_init();
	clock_init();
	UART_Init();
}

void func_5(void * param){
	while (1)
	{
		xSemaphoreTake(uart_lock, 0xffffffff);
		usart_printf("\033[0;31m [Taks 5]\033[0m Tempaerature read By ADC \r\n");
		xSemaphoreGive(uart_lock);
		vTaskDelay(2000);
	}

}
int main()
{
	setup();
	TaskHandle_t task_1 = NULL;
	TaskHandle_t task_2 = NULL;
	TaskHandle_t task_3 = NULL;
	TaskHandle_t task_4 = NULL;
	TaskHandle_t task_5 = NULL;
	xTaskCreate(func_1, "task 1",512, NULL, 0, &task_1);
	xTaskCreate(func_2, "task 2",512, NULL, 0, &task_2);
	xTaskCreate(func_3, "task 3",512, NULL, 0, &task_3);
	xTaskCreate(func_4, "task 4",512, NULL, 0, &task_4);
	xTaskCreate(func_5, "task 5",512, NULL, 0, &task_5);
	temp_queue = xQueueCreate(20, sizeof(float));
	temp_event = xEventGroupCreate();
	uart_lock = xSemaphoreCreateMutex();

	vTaskStartScheduler();
	while (1)
	{

	}

	return 0;
}