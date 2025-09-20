#include "mcu_driver.h"
#include "gpio_driver.h"
#include "uart_driver.h" 
#include <stdint.h>
#include "stm32f4xx.h"
#include "system_stm32f4xx.h"

void delay(volatile uint32_t time)
{
    for (volatile uint32_t i = 0; i < time; i++);
}

int main()
{
    mcu_init();
    gpio_set_uart(USART1); //USART1: PA9 (TX),  PA10 (RX)

    /*
    while (true)
    {
        io_set_output((io_e)PA5, IO_OUTPUT_HIGH);  // LED ON
        USART1->DR = 0x55;                 // you should see a single frame on PA10
        while (!(USART1->SR & USART_SR_TC));

        delay(50000);
        io_set_output((io_e)PA5, IO_OUTPUT_LOW);  // LED ON
        delay(50000);
    }
    */

    while (true)
    {
        io_set_output((io_e)PA5, IO_OUTPUT_HIGH);  // LED ON
        _putchar('H', USART1);
        _putchar('E', USART1);
        _putchar('L', USART1);
        _putchar('L', USART1);
        _putchar('O', USART1);
        _putchar(',', USART1);
        _putchar(' ', USART1);
        _putchar('W', USART1);
        _putchar('O', USART1);
        _putchar('R', USART1);
        _putchar('L', USART1);
        _putchar('D', USART1);
        _putchar('!', USART1);
        _putchar(' ', USART1);

        delay(50000);
        io_set_output((io_e)PA5, IO_OUTPUT_LOW);  // LED ON
        delay(50000);
    }
}
