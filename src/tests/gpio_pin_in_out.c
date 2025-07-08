#include "mcu_driver.h"
#include "gpio_driver.h"
#include <stdint.h>



int main(void) {
    mcu_init();

    struct io_config input_config = 
    {
        .mode = IO_MODE_INPUT,
        .otype = IO_OTYPE_PUSH_PULL,
        .ospeed = IO_OSPEED_LOW,
        .resistor = IO_RESISTOR_PULL_DOWN,
        .output = IO_OUTPUT_LOW
    };

    io_configure((io_e)PA2, &input_config);

    while (1)
    {
        if (io_get_input((io_e)PA2) == IO_INPUT_HIGH)
        {
            io_set_output(GREEN_LED, IO_OUTPUT_HIGH);
        }

        else 
        {
            io_set_output(GREEN_LED, IO_OUTPUT_LOW);
        }
    }

    return 0; // Typically won't reach here in an embedded environment
}
