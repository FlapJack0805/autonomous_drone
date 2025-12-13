#include "mcu_driver.h"
#include "stm32f4xx.h"
#include "system_stm32f4xx.h"
#include "gpio_driver.h"
#include "uart_driver.h"
#include "i2c_driver.h"
#include "spi_driver.h"
#include "pwm_driver.h"

//required to have defined for some reason
void _init(void) {}
void _fini(void) {}

void mcu_init()
{
	io_init();
	interrupt_init();
	uart_init();
	i2c_init();
	spi_init();
	pwm_init();
}


