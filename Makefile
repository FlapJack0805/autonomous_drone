CC=arm-none-eabi-gcc
CFLAGS=-mcpu=cortex-m4 -mthumb -nostdlib
CPPFLAGS=-DSTM32F401xE \
 -I/home/jstifter/code/embedded/stm32/libraries/CMSIS/Device/ST/STM32F4/Include \
 -I/home/jstifter/code/embedded/stm32/libraries/CMSIS/CMSIS/Core/Include \
 -I/home/jstifter/code/embedded/stm32/projects/autonomous_drone/src/drivers
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
else
	SOURCE_FILE = main.c
endif



# Define common object files
OBJECTS=$(STARTUP_FILE) \
 /home/jstifter/code/embedded/stm32/libraries/CMSIS/Device/ST/STM32F4/Source/Templates/system_stm32f4xx.c \
 src/drivers/gpio_driver.c \
 src/drivers/mcu_driver.c  \
 src/drivers/uart_driver.c

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
