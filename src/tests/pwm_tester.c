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

    float pwm = 0;
    while (1)
    {
        pwm_set_duty_cycle(front_left_motor, pwm);
        pwm_set_duty_cycle(front_right_motor, pwm);
        pwm_set_duty_cycle(back_left_motor, pwm);
        pwm_set_duty_cycle(back_right_motor, pwm);

        ++pwm;

        if (pwm > 100)
        {
            pwm = 0;
        }

        io_set_output((io_e)PA5, IO_OUTPUT_HIGH);  // LED ON
        delay(50000);
        io_set_output((io_e)PA5, IO_OUTPUT_LOW);  // LED ON
        delay(50000);

    }
}
