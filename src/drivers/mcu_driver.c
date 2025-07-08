#include "mcu_driver.h"
#include "stm32f4xx.h"
#include "system_stm32f4xx.h"
#include "gpio_driver.h"

//required to have defined for some reason
void _init(void) {}
void _fini(void) {}

void mcu_init()
{
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // Enable GPIOA clock
	volatile uint32_t dummy;
	dummy = RCC->AHB1ENR;
	dummy = RCC->AHB1ENR;
	io_init();
}

