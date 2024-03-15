#ifndef _HAL_TIMER_H_
#define _HAL_TIMER_H_

/*
 *  STM32F4xx timer driver
 *  Martin Kopka 2024 
*/

#include "stm32f4xx.h"
#include "hal/gpio.h"

//---- ENUMERATIONS ----------------------------------------------------------------------------------------------------------------------------------------------

// counter count direction
typedef enum {

    TIMER_DIR_UP = 0,
    TIMER_DIR_DOWN = 1

} timer_dir_t;

//---- FUNCTIONS -------------------------------------------------------------------------------------------------------------------------------------------------

// initializes the timer to count up or down at specified frequency
void timer_init_counter(TIM_TypeDef *timer, uint32_t frequency_hz, timer_dir_t direction, uint32_t reload_value);

// initializes the timer to generate edge aligned PWM
void timer_init_pwm(TIM_TypeDef *timer, uint8_t channel, GPIO_TypeDef *port, uint8_t gpio, uint32_t frequency_hz, uint32_t reload_value);

// sets the counter enable bit
static inline void timer_start_count(TIM_TypeDef *timer) {set_bits(timer->CR1, TIM_CR1_CEN);}

// stops the counter enable bit
static inline void timer_stop_count(TIM_TypeDef *timer) {clear_bits(timer->CR1, TIM_CR1_CEN);}

// clears the counter count value
static inline void timer_reset_count(TIM_TypeDef *timer) {timer->CNT = 0;}

// returns the counter count value
static inline uint32_t timer_get_count(TIM_TypeDef *timer) {return (timer->CNT);}

// sets the PWM channel duty cycle (duty = 100 * duty / (reload_val - 1) [%])
static inline void timer_set_pwm_duty(TIM_TypeDef *timer, uint8_t channel, uint32_t duty) {

    if      (channel == 1) timer->CCR1 = duty;
    else if (channel == 2) timer->CCR2 = duty;
    else if (channel == 3) timer->CCR3 = duty;
    else                   timer->CCR4 = duty;
}

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

// returns the duty cycle of a PWM channel (duty = 100 * duty / (reload_val - 1) [%])
static inline uint32_t timer_get_pwm_duty(TIM_TypeDef *timer, uint8_t channel) {

    if      (channel == 1) return (timer->CCR1);
    else if (channel == 2) return (timer->CCR2);
    else if (channel == 3) return (timer->CCR3);
    else                   return (timer->CCR4);
}

//----------------------------------------------------------------------------------------------------------------------------------------------------------------

#endif /* _HAL_TIMER_H_ */