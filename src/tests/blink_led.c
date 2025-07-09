#include "mcu_driver.h"
#include "gpio_driver.h"
#include <stdint.h>


void delay(uint32_t num) 
{
    for (volatile uint64_t i = 0; i < num; i++) 
	{
        // Simple delay loop to make the LED blink visible
	}
}


void blink_led(void) 
{
    while (1) {
        io_set_output((io_e)PA3, IO_OUTPUT_HIGH);  // LED ON
        io_set_output((io_e)PA5, IO_OUTPUT_HIGH);  // LED ON
        delay(100000);
        io_set_output((io_e)PA3, IO_OUTPUT_LOW);   // LED OFF
        io_set_output((io_e)PA5, IO_OUTPUT_LOW);   // LED OFF
        delay(100000);
    }
}


int main(void) {
    mcu_init();
    blink_led();  // Start blinking the onboard LED

    return 0; // Typically won't reach here in an embedded environment
}
