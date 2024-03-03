#include "hal/timer.h"
#include "hal/rcc.h"

//---- PRIVATE FUNCTIONS -----------------------------------------------------------------------------------------------------------------------------------------

static inline rcc_peripheral_clock_en_t get_timer_clock(TIM_TypeDef *timer) {

    if      (timer == TIM1)  return (RCC_PERIPH_APB2_TIM1);
    else if (timer == TIM2)  return (RCC_PERIPH_APB1_TIM2);
    else if (timer == TIM3)  return (RCC_PERIPH_APB1_TIM3);
    else if (timer == TIM4)  return (RCC_PERIPH_APB1_TIM4);
    else if (timer == TIM5)  return (RCC_PERIPH_APB1_TIM5);
    else if (timer == TIM9)  return (RCC_PERIPH_APB2_TIM9);
    else if (timer == TIM10) return (RCC_PERIPH_APB2_TIM10);
    else                     return (RCC_PERIPH_APB2_TIM11);
}

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

static inline gpio_alternate_function_t get_timer_alternate_function(TIM_TypeDef *timer) {

    if      (timer == TIM1)  return (GPIO_ALTERNATE_FUNCTION_TIM1_TIM2);
    else if (timer == TIM2)  return (GPIO_ALTERNATE_FUNCTION_TIM1_TIM2);
    else if (timer == TIM3)  return (GPIO_ALTERNATE_FUNCTION_TIM3_TIM4_TIM5);
    else if (timer == TIM4)  return (GPIO_ALTERNATE_FUNCTION_TIM3_TIM4_TIM5);
    else if (timer == TIM5)  return (GPIO_ALTERNATE_FUNCTION_TIM3_TIM4_TIM5);
    else if (timer == TIM9)  return (GPIO_ALTERNATE_FUNCTION_TIM9_TIM10_TIM11);
    else if (timer == TIM10) return (GPIO_ALTERNATE_FUNCTION_TIM9_TIM10_TIM11);
    else                     return (GPIO_ALTERNATE_FUNCTION_TIM9_TIM10_TIM11);
}

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

static inline uint32_t get_timer_clock_source_frequency(TIM_TypeDef *timer) {

    if      (timer == TIM1)  return (PCLK2_timer_frequency_hz);
    else if (timer == TIM2)  return (PCLK1_timer_frequency_hz);
    else if (timer == TIM3)  return (PCLK1_timer_frequency_hz);
    else if (timer == TIM4)  return (PCLK1_timer_frequency_hz);
    else if (timer == TIM5)  return (PCLK1_timer_frequency_hz);
    else if (timer == TIM9)  return (PCLK2_timer_frequency_hz);
    else if (timer == TIM10) return (PCLK2_timer_frequency_hz);
    else                     return (PCLK2_timer_frequency_hz);
}

//---- FUNCTIONS -------------------------------------------------------------------------------------------------------------------------------------------------

// initializes the timer to count up or down at specified frequency
void timer_init_counter(TIM_TypeDef *timer, uint32_t frequency_hz, timer_dir_t direction, uint32_t reload_value) {

    rcc_enable_peripheral_clock(get_timer_clock(timer));

    // set prescale value and reset counter
    timer->PSC = get_timer_clock_source_frequency(timer) / frequency_hz - 1;
    timer->ARR = reload_value;      // auto-reload value
    timer->CNT = 0;                 // reset the counter value

    set_bits(timer->CR1, TIM_CR1_ARPE);                 // auto-reload enable
    write_masked(timer->CR1, 0 << 5, TIM_CR1_CMS);      // edge aligned mode

    if (direction == TIMER_DIR_UP) clear_bits(timer->CR1, TIM_CR1_DIR);
    else                           set_bits(timer->CR1, TIM_CR1_DIR);
}

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

// initializes the timer to generate edge aligned PWM
void timer_init_pwm(TIM_TypeDef *timer, uint8_t channel, GPIO_TypeDef *port, uint8_t gpio, uint32_t frequency_hz, uint32_t reload_value) {

    rcc_enable_peripheral_clock(gpio_get_clock(port));
    rcc_enable_peripheral_clock(get_timer_clock(timer));
    gpio_set_mode(port, gpio, GPIO_MODE_ALTERNATE_FUNCTION);
    gpio_set_alternate_function(port, gpio , get_timer_alternate_function(timer));

    // set capture/compare channel as output (CCxS)
    if (channel <= 2 ) write_masked(timer->CCMR1, 0 << (8 * (channel - 1)), 0x3 << (8 * (channel - 1)));
    else               write_masked(timer->CCMR2, 0 << (8 * (channel - 3)), 0x3 << (8 * (channel - 3)));

    // set capture/compate output polarity as active (CCxP)
    clear_bits(timer->CCER, (1 << ((channel - 1) * 4 + 1)));

    // set output compare mode as PWM (OCxM)
    if (channel <= 2 ) write_masked(timer->CCMR1, 6 << (8 * (channel - 1) + 4), 0x7 << (8 * (channel - 1) + 4));
    else               write_masked(timer->CCMR2, 6 << (8 * (channel - 3) + 4), 0x7 << (8 * (channel - 3) + 4));

    timer->PSC = get_timer_clock_source_frequency(timer) / ((reload_value + 1) * frequency_hz) - 1;
    timer->ARR = reload_value;      // auto-reload value

    // set compare value to 0 -> 0% duty
    if      (channel == 1) timer->CCR1 = 0;
    else if (channel == 2) timer->CCR2 = 0;
    else if (channel == 3) timer->CCR3 = 0;
    else                   timer->CCR4 = 0;

    // set output compare preload enable (OCxPE)
    if (channel <= 2 ) set_bits(timer->CCMR1, 1 << (8 * (channel - 1) + 3));
    else               set_bits(timer->CCMR2, 1 << (8 * (channel - 3) + 3));

    set_bits(timer->CR1, TIM_CR1_ARPE);                 // auto-reload enable
    set_bits(timer->CCER, 1 << ((channel - 1) * 4));    // capture/compare output enable
    write_masked(timer->CR1, 0 << 5, TIM_CR1_CMS);      // edge aligned mode
    clear_bits(timer->CR1, TIM_CR1_DIR);                // count up
    set_bits(timer->CR1, TIM_CR1_CEN);                  // counter enable
}

//----------------------------------------------------------------------------------------------------------------------------------------------------------------
