#include "mcu_driver.h"
#include "gpio_driver.h"
#include "i2c_driver.h" 
#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx.h"
#include "system_stm32f4xx.h"
#include "FreeRTOSIncludes.h"

void vBlinkLed(void* args)
{
    (void)args;

    while (1)
    {
        io_set_output((io_e)PA5, IO_OUTPUT_HIGH);
        vTaskDelay(pdMS_TO_TICKS(500));

        io_set_output((io_e)PA5, IO_OUTPUT_LOW);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void vSendI2C(void* args)
{
    (void)args;

    const uint8_t address = 0x60;
    const uint8_t word_buf[5] = {0x54, 0x55, 0x68, 0xFF, 0x00};
    const uint16_t word_buf_len = 5;
    uint8_t read_buf[3];
    const uint16_t read_buf_len = 3;

    while (1)
    {
        i2c_transfer(address, word_buf, word_buf_len, read_buf, read_buf_len);

        if (read_buf[2] == 3)
        {
            io_set_output((io_e)PA5, IO_OUTPUT_HIGH);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));  
    }
}

int main(void)
{
    mcu_init();


    /*
    BaseType_t blink_ok = xTaskCreate(
        vBlinkLed,          // Task function
        "Blink",            // Task name
        256,                // Stack depth in words, not bytes
        NULL,               // Task parameter
        1,                  // Priority
        NULL                // Optional task handle
    );
    */

    BaseType_t i2c_ok = xTaskCreate(
        vSendI2C,
        "I2C",
        256,
        NULL,
        1,
        NULL
    );


    if (/*blink_ok != pdPASS ||*/ i2c_ok != pdPASS)
    {
        while (1)
        {
            // Task creation failed
        }
    }


    vTaskStartScheduler();

    // Should never get here unless there is not enough heap
    while (1)
    {
    }
}
