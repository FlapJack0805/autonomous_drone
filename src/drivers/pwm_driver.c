#include "pwm_driver.h"
#include "stm32f4xx.h"
#include "system_stm32f4xx.h"
#include <stdbool.h>

#define NUM_MOTORS 4
#define TIMER_FREQUENCY 16000000u
#define PWM_FREQUENCY 50u
#define PRESCALER 15u
#define PWM_PERIOD_TICKS (TIMER_FREQUENCY / (PWM_FREQUENCY * (PRESCALER + 1)))
#define PWM_ARR (PWM_PERIOD_TICKS - 1)

#define ESC_MIN_US      1000u
#define ESC_MID_US      1500u
#define ESC_MAX_US      2000u
#define ESC_PERIOD_US   20000u

static void pwm_config(void)
{
    // channel 1
    TIM2->CCMR1 &= ~TIM_CCMR1_OC1M;
    TIM2->CCMR1 |= (6u << TIM_CCMR1_OC1M_Pos);   // PWM mode 1
    TIM2->CCMR1 |= TIM_CCMR1_OC1PE;
    TIM2->CCER  |= TIM_CCER_CC1E;
    TIM2->CCR1 = 1000;

    // channel 2
    TIM2->CCMR1 &= ~TIM_CCMR1_OC2M;
    TIM2->CCMR1 |= (6u << TIM_CCMR1_OC2M_Pos);
    TIM2->CCMR1 |= TIM_CCMR1_OC2PE;
    TIM2->CCER  |= TIM_CCER_CC2E;
    TIM2->CCR2 = 1000;

    // channel 3
    TIM2->CCMR2 &= ~TIM_CCMR2_OC3M;
    TIM2->CCMR2 |= (6u << TIM_CCMR2_OC3M_Pos);
    TIM2->CCMR2 |= TIM_CCMR2_OC3PE;
    TIM2->CCER  |= TIM_CCER_CC3E;
    TIM2->CCR3 = 1000;

    // channel 4 
    TIM2->CCMR2 &= ~TIM_CCMR2_OC4M;
    TIM2->CCMR2 |= (6u << TIM_CCMR2_OC4M_Pos);
    TIM2->CCMR2 |= TIM_CCMR2_OC4PE;
    TIM2->CCER  |= TIM_CCER_CC4E;
    TIM2->CCR4 = 1000;

    // Set AFR for PA0 and PA1 (front motors)
    GPIOA->AFR[0] &= ~(GPIO_AFRL_AFRL0 | GPIO_AFRL_AFRL1 );
    GPIOA->AFR[0] |= (1 << GPIO_AFRL_AFSEL0_Pos) | (1 << GPIO_AFRL_AFSEL1_Pos);

    // Set AFR for PB10 and PB11 (back motors)
    GPIOB->AFR[1] &= ~(GPIO_AFRH_AFRH2 | GPIO_AFRH_AFRH3);
    GPIOB->AFR[1] |= (1 << GPIO_AFRH_AFSEL10_Pos) | (1 << GPIO_AFRH_AFSEL11_Pos);
    
    // Enable output for this channel
    TIM2->CCER |= 1 | (1 << 4) | (1 << 8) | (1 << 12);  // Each channel has 4 bits in CCER
}


void pwm_init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->PSC = PRESCALER;
    TIM2->ARR = PWM_ARR;
    pwm_config();
    TIM2->CR1 |=  TIM_CR1_CEN;
    TIM2->EGR = TIM_EGR_UG;
}


void esc_set_pulse_us_period(pwm_e motor, uint16_t pulse_us)
{
    if (pulse_us < ESC_MIN_US) pulse_us = ESC_MIN_US;
    if (pulse_us > ESC_MAX_US) pulse_us = ESC_MAX_US;

    switch (motor)
    {
        case front_left_motor:  TIM2->CCR1 = pulse_us; break;
        case front_right_motor: TIM2->CCR2 = pulse_us; break;
        case back_left_motor:   TIM2->CCR3 = pulse_us; break;
        case back_right_motor:  TIM2->CCR4 = pulse_us; break;
    }
} 
