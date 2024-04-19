#FREERTOS_DIR = FreeRTOS-LTS/FreeRTOS/FreeRTOS-Kernel
# FREERTOS_DIR=FreeRTOS-LTS/FreeRTOS/FreeRTOS-Kernel
HEADER=-u _printf_float -specs=nano.specs -specs=nosys.specs -mfpu=fpv4-sp-d16 -mthumb -mfloat-abi=hard -mcpu=cortex-m4 -std=gnu11 -ICustom_Driver/Inc/ -IFreeRTOS-LTS/FreeRTOS/FreeRTOS-Kernel/include/ -I. -IFreeRTOS-LTS/FreeRTOS/FreeRTOS-Kernel/portable/GCC/ARM_CM4F
#HEADER=-u _printf_float -specs=nano.specs -specs=nosys.specs -mfpu=fpv4-sp-d16 -mthumb -mfloat-abi=hard -mcpu=cortex-m4


All:
	# arm-none-eabi-gcc -c $(FREERTOS_DIR)/croutine.c $(HEADER) -o build/croutine.o
	# arm-none-eabi-gcc -c $(FREERTOS_DIR)/event_groups.c $(HEADER) -o build/event_groups.o
	# arm-none-eabi-gcc -c $(FREERTOS_DIR)/list.c $(HEADER) -o build/list.o
	# arm-none-eabi-gcc -c $(FREERTOS_DIR)/queue.c $(HEADER) -o build/queue.o
	# arm-none-eabi-gcc -c $(FREERTOS_DIR)/stream_buffer.c $(HEADER) -o build/stream_buffer.o
	# arm-none-eabi-gcc -c $(FREERTOS_DIR)/tasks.c $(HEADER) -o build/tasks.o
	# arm-none-eabi-gcc -c $(FREERTOS_DIR)/timers.c $(HEADER) -o build/timers.o
	# arm-none-eabi-gcc -c $(FREERTOS_DIR)/portable/GCC/ARM_CM4F/port.c $(HEADER) -o build/port.o
	# arm-none-eabi-gcc -c $(FREERTOS_DIR)/portable/MemMang/heap_4.c $(HEADER) -std=gnu11 -o build/heap.o
	arm-none-eabi-gcc -c main.c $(HEADER) -o build/main.o
	arm-none-eabi-gcc -c Custom/Src/Led.c $(HEADER) -o build/Led.o
	arm-none-eabi-gcc -c Custom/Src/clock.c $(HEADER) -o build/clock.o
	arm-none-eabi-gcc -c Custom/Src/Delay.c $(HEADER) -o build/Delay.o
	arm-none-eabi-gcc -c Custom/Src/capture.c $(HEADER) -o build/capture.o
	arm-none-eabi-gcc -c Custom/Src/ADC.c $(HEADER) -o build/ADC.o
	arm-none-eabi-gcc -c Custom/Src/Usart.c $(HEADER) -o build/Usart.o
	# arm-none-eabi-gcc -c -x assembler-with-cpp startup_stm32f411vetx.s -mcpu=cortex-m4 -std=gnu11 -o build/startup.o
	arm-none-eabi-gcc D:\Dev_STM32\stm32_vs\build\ADC.o D:\Dev_STM32\stm32_vs\build\capture.o D:\Dev_STM32\stm32_vs\build\clock.o D:\Dev_STM32\stm32_vs\build\Delay.o D:\Dev_STM32\stm32_vs\build\Led.o D:\Dev_STM32\stm32_vs\build\main.o D:\Dev_STM32\stm32_vs\build\startup.o $(HEADER) -T"STM32F411VETX_FLASH.ld" -Wl,-Map="file.map" -Wl,--gc-sections -static -o build/blink_led.elf
	arm-none-eabi-objcopy -O ihex build/blink_led.elf build/blink_led.hex
	arm-none-eabi-objcopy -O binary build/blink_led.elf build/blink_led.bin
clean:
	rmdir /q /s build
	mkdir build


Flash:
	ST-LINK_CLI -c SWD -P build/blink_led.hex -V -Run