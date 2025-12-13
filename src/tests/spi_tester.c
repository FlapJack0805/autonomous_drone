#include "mcu_driver.h"
#include "gpio_driver.h"
#include "spi_driver.h" 
#include <stdint.h>
#include "stm32f4xx.h"
#include "system_stm32f4xx.h"


void delay(volatile uint32_t time)
{
    for (volatile uint32_t i = 0; i < time; i++);
}


int main(void)
{
    mcu_init();
    gpio_set_spi(SPI2); //SPI2: PB13(SCK),  PB14(MISO), PB15 (MOSI)

    const uint8_t word_buf[5] = {0x54, 0x55, 0x68, 0xFF, 0x00};
    const uint16_t word_buf_len = 5;
    uint8_t *recieve_buf;
    const uint16_t recieve_buf_len = 0;

    while (true)
    {
        spi_transfer(word_buf, word_buf_len, recieve_buf, recieve_buf_len);
        
        io_set_output((io_e)PA5, IO_OUTPUT_HIGH);  // LED ON
        delay(500000);
        io_set_output((io_e)PA5, IO_OUTPUT_LOW);  // LED ON
        delay(500000);
    }
}
