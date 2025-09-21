#include "mcu_driver.h"
#include "gpio_driver.h"
#include "spi_driver.h" 
#include <stdint.h>
#include "stm32f4xx.h"
#include "system_stm32f4xx.h"




int main(void)
{
    mcu_init();
    gpio_set_spi(SPI1);
}
