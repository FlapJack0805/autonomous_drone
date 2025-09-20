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
        .resistor = IO_RESISTOR_PULL_UP,
        .output = IO_OUTPUT_LOW
    };

    struct io_config output_config = 
    {
        .mode = IO_MODE_OUTPUT,
        .otype = IO_OTYPE_PUSH_PULL,
        .ospeed = IO_OSPEED_LOW,
        .resistor = IO_RESISTOR_NONE,
        .output = IO_OUTPUT_LOW
    };

    io_configure((io_e)PC13, &input_config);
    io_configure((io_e)PA10, &output_config);

    while (1)
    {
        if (io_get_input((io_e)PC13) == IO_INPUT_HIGH)
        {
            io_set_output((io_e)PA10, IO_OUTPUT_HIGH);
        }

        else 
        {
            io_set_output((io_e)PA10, IO_OUTPUT_LOW);
        }
    }

    return 0; // Typically won't reach here in an embedded environment
}
