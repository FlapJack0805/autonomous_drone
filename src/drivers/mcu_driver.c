#include "mcu_driver.h"
#include "stm32f4xx.h"
#include "system_stm32f4xx.h"
#include "gpio_driver.h"

//required to have defined for some reason
void _init(void) {}
void _fini(void) {}

void mcu_init()
{
	io_init();
	interrupt_init();
}

