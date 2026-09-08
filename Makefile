CC=arm-none-eabi-gcc
CFLAGS=-mcpu=cortex-m4 -mthumb -nostdlib -mfpu=fpv4-sp-d16 -mfloat-abi=hard -O2 -DSTM32F401xE

CPPFLAGS=-DSTM32F401xE \
 -I/home/jstifter/code/embedded/stm32/libraries/CMSIS/Device/ST/STM32F4/Include \
 -I/home/jstifter/code/embedded/stm32/libraries/CMSIS/CMSIS/Core/Include \
 -I/home/jstifter/code/embedded/stm32/projects/flight_controller/src/drivers \
 -I/home/jstifter/code/embedded/stm32/projects/flight_controller/src/include \
 -I/home/jstifter/code/embedded/stm32/projects/flight_controller/src/applications \
 -I/home/jstifter/code/embedded/stm32/libraries/FreeRTOS/include \
 -I/home/jstifter/code/embedded/stm32/libraries/FreeRTOS/portable/GCC/ARM_CM4F


LINKER_FILE=stm32f401re.ld
LDFLAGS=-T $(LINKER_FILE) -L/home/jstifter/code/embedded/stm32/libraries/CMSIS/Device/ST/STM32F4/Include -lc -lgcc
STARTUP_FILE=/home/jstifter/code/embedded/stm32/libraries/CMSIS/Device/ST/STM32F4/Source/Templates/gcc/startup_stm32f401xe.s

# Allow user to specify which test to run
TEST ?= default



# how to chose which test we are running if we so chose
# Select the source file based on TEST
ifeq ($(TEST), blink_led)
	SOURCE_FILE = src/tests/blink_led.c
else ifeq ($(TEST), gpio_pin_in_out)
	SOURCE_FILE = src/tests/gpio_pin_in_out.c
else ifeq ($(TEST), uart_test)
	SOURCE_FILE = src/tests/uart_tester.c
else ifeq ($(TEST), i2c_test)
	SOURCE_FILE = src/tests/i2c_tester.c
else ifeq ($(TEST), spi_test)
	SOURCE_FILE = src/tests/spi_tester.c
else ifeq ($(TEST), pwm_test)
	SOURCE_FILE = src/tests/pwm_tester.c
else ifeq ($(TEST), mpu_6050_test)
	SOURCE_FILE = src/tests/mpu_6050_test.c
else ifeq ($(TEST), motor1_test)
	SOURCE_FILE = src/tests/motor1_test.c
else
	SOURCE_FILE = main.c
endif

# If we say DEBUG then compile with the debug flag for gdb
ifeq ($(DEBUG),1)
	CFLAGS += -Og -g3
else
	CFLAGS += -O2
endif




# Define common object files
OBJECTS=$(STARTUP_FILE) \
 /home/jstifter/code/embedded/stm32/libraries/CMSIS/Device/ST/STM32F4/Source/Templates/system_stm32f4xx.c \
 src/drivers/gpio_driver.c \
 src/drivers/mcu_driver.c  \
 src/drivers/uart_driver.c \
 src/drivers/i2c_driver.c  \
 src/drivers/spi_driver.c  \
 src/drivers/pwm_driver.c  \
 src/applications/mpu_6050.c \
 /home/jstifter/code/embedded/stm32/libraries/FreeRTOS/tasks.c \
 /home/jstifter/code/embedded/stm32/libraries/FreeRTOS/queue.c \
 /home/jstifter/code/embedded/stm32/libraries/FreeRTOS/timers.c \
 /home/jstifter/code/embedded/stm32/libraries/FreeRTOS/list.c \
 /home/jstifter/code/embedded/stm32/libraries/FreeRTOS/croutine.c \
 /home/jstifter/code/embedded/stm32/libraries/FreeRTOS/portable/GCC/ARM_CM4F/port.c \
 /home/jstifter/code/embedded/stm32/libraries/FreeRTOS/portable/MemMang/heap_4.c

# Default target
all: code.elf

# Final binary
code.elf: $(SOURCE_FILE) $(OBJECTS)
	$(CC) $(CFLAGS) $(CPPFLAGS) $(LDFLAGS) $^ -o $@

# Clean target
.PHONY: clean
clean:
	rm -f code.elf

# Programmer settings
PROGRAMMER=openocd
PROGRAMMER_FLAGS=-f interface/stlink.cfg -f target/stm32f4x.cfg

# Flash target
flash: code.elf
	$(PROGRAMMER) $(PROGRAMMER_FLAGS) -c "program code.elf verify reset exit"

# Test run target
test: code.elf
	$(PROGRAMMER) $(PROGRAMMER_FLAGS) -c "program code.elf verify reset exit"
