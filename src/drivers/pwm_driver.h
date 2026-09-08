#ifndef PWM_H
#define PWM_H

#include <stdint.h>


typedef enum
{
    front_left_motor,
    front_right_motor, 
    back_left_motor,
    back_right_motor
} pwm_e;

void pwm_init(void);
void esc_set_pulse_us_period(pwm_e pwm, uint16_t pulse_us);

#endif
