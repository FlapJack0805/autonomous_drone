#ifndef UART_H
#define UART_H

#include <stdint.h>
#include "stm32f4xx.h"
#include "system_stm32f4xx.h"
#include "gpio_driver.h"

void uart_init(void);
void _putchar(char c, USART_TypeDef *USART);



#endif

