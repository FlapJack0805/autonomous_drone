#include "mcu_driver.h"
#include "pwm_driver.h"
#include "gpio_driver.h"



void delay(long val)
{
    for (volatile long i = 0; i < val; ++i);
}


int main(void)
{
    mcu_init();

    int pwm = 1000;
    while (1)
    {

        esc_set_pulse_us_period(front_left_motor, pwm);
        io_set_output((io_e)PA5, IO_OUTPUT_HIGH);  // LED ON
        delay(50000);
        io_set_output((io_e)PA5, IO_OUTPUT_LOW);  // LED ON
        delay(50000);

        ++pwm;

        if (pwm >= 2000)
        {
            pwm = 1000;
        }
    }
}
