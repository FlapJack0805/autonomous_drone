#ifndef UART_H
#define UART_H

#include <stdint.h>

void uart_init(void);
void uart_write(uint8_t len, uint8_t *bytes);



#endif

