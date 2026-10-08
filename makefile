CC=arm-none-eabi-gcc
CFLAGS=-mcpu=cortex-m33 -mfpu=auto -mfloat-abi=hard -mthumb -nostdlib -W -Werror -Wall -Wextra -ffunction-sections -g
CPPFLAGS= -DSTM32H533xx -Ivendor/CMSIS/Device/ST/STM32H5/Include \
	 -Ivendor/CMSIS/Core/Include -Ivendor -I. \
	 -Ivendor/FreeRTOS-Kernel/include \
	 -Ivendor/FreeRTOS-Kernel/portable/GCC/ARM_CM33_NTZ/non_secure

LINKER_FILE=linker_script.ld 
LDFLAGS=-T $(LINKER_FILE)

BINARY = v68vcu.elf
OBJS = 	startup.o \
		modules/logging.o \
		modules/control.o \
		vendor/printf/printf.o \
		vendor/CMSIS/Device/ST/STM32H5/Source/Templates/system_stm32h5xx.o \
		blinky_test.o

all: $(BINARY)
FREERTOS = vendor/FreeRTOS-Kernel
FREERTOS_PORT = $(FREERTOS)/portable/GCC/ARM_CM33_NTZ/non_secure

$(BINARY): main.o startup.o gpio.o can.o flash.o logging.o printf.o control.o dynamics.o system_stm32h5xx.o \
           tasks.o list.o queue.o timers.o heap_1.o port.o portasm.o
	$(CC) $(CFLAGS) $(CPPFLAGS) $(LDFLAGS) $^ -o $(BINARY) -lgcc

tasks.o: $(FREERTOS)/tasks.c
	$(CC) $(CFLAGS) $(CPPFLAGS) $(FREERTOS)/tasks.c -c

list.o: $(FREERTOS)/list.c
	$(CC) $(CFLAGS) $(CPPFLAGS) $(FREERTOS)/list.c -c

queue.o: $(FREERTOS)/queue.c
	$(CC) $(CFLAGS) $(CPPFLAGS) $(FREERTOS)/queue.c -c

timers.o: $(FREERTOS)/timers.c
	$(CC) $(CFLAGS) $(CPPFLAGS) $(FREERTOS)/timers.c -c

heap_1.o: $(FREERTOS)/portable/MemMang/heap_1.c
	$(CC) $(CFLAGS) $(CPPFLAGS) $(FREERTOS)/portable/MemMang/heap_1.c -c

port.o: $(FREERTOS_PORT)/port.c
	$(CC) $(CFLAGS) $(CPPFLAGS) $(FREERTOS_PORT)/port.c -c

portasm.o: $(FREERTOS_PORT)/portasm.c
	$(CC) $(CFLAGS) $(CPPFLAGS) $(FREERTOS_PORT)/portasm.c -c

startup.o: startup.c
	$(CC) $(CFLAGS) $(CPPFLAGS) $(LDFLAGS) startup.c -c

main.o: main.c
	$(CC) $(CFLAGS) $(CPPFLAGS) $(LDFLAGS) main.c -c

logging.o: modules/logging.c
	$(CC) $(CFLAGS) $(CPPFLAGS) $(LDFLAGS) modules/logging.c -c

gpio.o: modules/gpio.c
	$(CC) $(CFLAGS) $(CPPFLAGS) $(LDFLAGS) modules/gpio.c -c

can.o: modules/can.c
	$(CC) $(CFLAGS) $(CPPFLAGS) $(LDFLAGS) modules/can.c -c

flash.o: modules/flash.c
	$(CC) $(CFLAGS) $(CPPFLAGS) $(LDFLAGS) modules/flash.c -c

printf.o: vendor/printf/printf.c
	$(CC) $(CFLAGS) $(CPPFLAGS) $(LDFLAGS) vendor/printf/printf.c -c

control.o: modules/control.c
	$(CC) $(CFLAGS) $(CPPFLAGS) -O3 $(LDFLAGS) modules/control.c -c 

dynamics.o: modules/dynamics.c
	$(CC) $(CFLAGS) $(CPPFLAGS) -O3 $(LDFLAGS) modules/dynamics.c -c 

system_stm32h5xx.o: vendor/CMSIS/Device/ST/STM32H5/Source/Templates/system_stm32h5xx.c
	$(CC) $(CFLAGS) $(CPPFLAGS) $(LDFLAGS) vendor/CMSIS/Device/ST/STM32H5/Source/Templates/system_stm32h5xx.c -c

.PHONY: clean
clean:
	rm -f *.o

.PHONY: cleanall
cleanall:
	rm -f *.o *.elf

.PHONY: lint
lint:
	cpplint --linelength=100 main.c startup.c modules/*

.PHONY: check
check:
	cppcheck main.c --check-level=exhaustive --force --enable=all $(CPPFLAGS)

.PHONY: program
program:
	openocd -f interface/stlink.cfg -f target/stm32h5x.cfg -c "program $(BINARY) verify reset exit"

.PHONY: debug
debug:
	openocd -f interface/stlink.cfg -f target/stm32h5x.cfg  

.PHONY: reset
reset:
	openocd -f interface/stlink.cfg -f target/stm32h5x.cfg -c "reset"
