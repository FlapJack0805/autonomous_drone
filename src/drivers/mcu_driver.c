#include "mcu_driver.h"

//required to have defined for some reason
void _init(void) {}
void _fini(void) {}

void mcu_init()
{
	io_init();
	//interrupt_init();
	uart_init();
	i2c_init();
	spi_init();
	pwm_init();
	//imu_init();
}


