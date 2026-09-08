#ifndef MCU_DRIVER_H
#define MCU_DRIVER_H

#include "stm32f4xx.h"
#include "system_stm32f4xx.h"
#include "gpio_driver.h"
#include "uart_driver.h"
#include "i2c_driver.h"
#include "spi_driver.h"
#include "pwm_driver.h"
#include "mpu_6050.h"

void mcu_init(void);

#endif
