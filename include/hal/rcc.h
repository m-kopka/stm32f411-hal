#ifndef _HAL_RCC_H_
#define _HAL_RCC_H_

/*
 *  STM32F4xx Reset and clock control driver
 *  Martin Kopka 2024 
*/

#include "stm32f4xx.h"

//---- ENUMERATIONS ----------------------------------------------------------------------------------------------------------------------------------------------

// peripheral clock enables,
typedef enum {

    RCC_PERIPH_AHB1_DMA2    = 0 * 32 + 22,
    RCC_PERIPH_AHB1_DMA1    = 0 * 32 + 21,
    RCC_PERIPH_AHB1_CRC     = 0 * 32 + 12,
    RCC_PERIPH_AHB1_GPIOH   = 0 * 32 + 7,
    RCC_PERIPH_AHB1_GPIOE   = 0 * 32 + 4,
    RCC_PERIPH_AHB1_GPIOD   = 0 * 32 + 3,
    RCC_PERIPH_AHB1_GPIOC   = 0 * 32 + 2,
    RCC_PERIPH_AHB1_GPIOB   = 0 * 32 + 1,
    RCC_PERIPH_AHB1_GPIOA   = 0 * 32 + 0,
    RCC_PERIPH_AHB2_OTGFS   = 1 * 32 + 7,
    RCC_PERIPH_APB1_PWR     = 2 * 32 + 28,
    RCC_PERIPH_APB1_I2C3    = 2 * 32 + 23,
    RCC_PERIPH_APB1_I2C2    = 2 * 32 + 22,
    RCC_PERIPH_APB1_I2C1    = 2 * 32 + 21,
    RCC_PERIPH_APB1_USART2  = 2 * 32 + 17,
    RCC_PERIPH_APB1_SPI3    = 2 * 32 + 15,
    RCC_PERIPH_APB1_SPI2    = 2 * 32 + 14,
    RCC_PERIPH_APB1_WWDG    = 2 * 32 + 11,
    RCC_PERIPH_APB1_TIM5    = 2 * 32 + 3,
    RCC_PERIPH_APB1_TIM4    = 2 * 32 + 2,
    RCC_PERIPH_APB1_TIM3    = 2 * 32 + 1,
    RCC_PERIPH_APB1_TIM2    = 2 * 32 + 0,
    RCC_PERIPH_APB2_SPI5    = 3 * 32 + 20,
    RCC_PERIPH_APB2_TIM11   = 3 * 32 + 18,
    RCC_PERIPH_APB2_TIM10   = 3 * 32 + 17,
    RCC_PERIPH_APB2_TIM9    = 3 * 32 + 16,
    RCC_PERIPH_APB2_SYSCFG  = 3 * 32 + 14,
    RCC_PERIPH_APB2_SPI4    = 3 * 32 + 13,
    RCC_PERIPH_APB2_SPI1    = 3 * 32 + 12,
    RCC_PERIPH_APB2_SDIO    = 3 * 32 + 11,
    RCC_PERIPH_APB2_ADC1    = 3 * 32 + 8,
    RCC_PERIPH_APB2_USART6  = 3 * 32 + 5,
    RCC_PERIPH_APB2_USART1  = 3 * 32 + 4,
    RCC_PERIPH_APB2_TIM1    = 3 * 32 + 0

} rcc_peripheral_clock_en_t;

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

// AHB prescaler options
typedef enum {

    RCC_SYS_CLOCK_DIV1   = 0,
    RCC_SYS_CLOCK_DIV2   = 8,
    RCC_SYS_CLOCK_DIV4   = 9,
    RCC_SYS_CLOCK_DIV8   = 10,
    RCC_SYS_CLOCK_DIV16  = 11,
    RCC_SYS_CLOCK_DIV64  = 12,
    RCC_SYS_CLOCK_DIV128 = 13,
    RCC_SYS_CLOCK_DIV256 = 14,
    RCC_SYS_CLOCK_DIV512 = 15

} rcc_system_clock_div_t;

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

// APB1 and APB2 prescaler options
typedef enum {

    RCC_PERIPH_CLOCK_DIV1  = 0,
    RCC_PERIPH_CLOCK_DIV2  = 4,
    RCC_PERIPH_CLOCK_DIV4  = 5,
    RCC_PERIPH_CLOCK_DIV8  = 6,
    RCC_PERIPH_CLOCK_DIV16 = 7

} rcc_peripheral_clock_div_t;

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

// PLL source options
typedef enum {

    RCC_PLL_SOURCE_HSI = 0,     // HSI clock selected as PLL clock entry
    RCC_PLL_SOURCE_HSE = 1      // HSE clock selected as PLL clock entry

} rcc_pll_src_t;

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

// system clock switch options
typedef enum {

    RCC_SYSTEM_CLOCK_SOURCE_HSI = 0,    // HSI oscillator selected as system clock
    RCC_SYSTEM_CLOCK_SOURCE_HSE = 1,    // HSE oscillator selected as system clock
    RCC_SYSTEM_CLOCK_SOURCE_PLL = 2     // PLL selected as system clock

} rcc_system_clock_src_t;

//---- DATA ------------------------------------------------------------------------------------------------------------------------------------------------------

// CAUTION: READ ONLY
extern uint32_t HSI_frequency_hz;           // frequency of the High Speed Internal RC oscillator [Hz]
extern uint32_t HSE_frequency_hz;           // frequency of the High Speed External oscillator [Hz]
extern uint32_t PLL_frequency_hz;           // output frequency of PLL [Hz]
extern uint32_t HLCK_frequency_hz;          // AHB1 and core frequency [Hz]
extern uint32_t PCLK1_frequency_hz;         // APB1 frequency [Hz]
extern uint32_t PCLK1_timer_frequency_hz;   // frequency of APB1 timers [Hz]
extern uint32_t PCLK2_frequency_hz;         // APB2 frequency [Hz]
extern uint32_t PCLK2_timer_frequency_hz;   // frequency of APB2 timers [Hz]

//---- FUNCTIONS -------------------------------------------------------------------------------------------------------------------------------------------------

// updates the clock frequency variables based on the current settings
void rcc_update_clocks(void);

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

// enables the high speed external oscillator
static inline void rcc_enable_hse(uint32_t hse_frequency_hz) {

    set_bits(RCC->CR, RCC_CR_HSEON);
    while(bit_is_clear(RCC->CR, RCC_CR_HSERDY));

    HSE_frequency_hz = hse_frequency_hz;
}

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

// disables the low speed external oscillator
static inline void rcc_disable_hse(void) {

    clear_bits(RCC->CR, RCC_CR_HSEON);
    while(bit_is_set(RCC->CR, RCC_CR_HSERDY));

    HSE_frequency_hz = 0;
}

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

// sets AHB, APB1 and APB2 bus clock prescalers
static inline void rcc_set_bus_prescalers(rcc_system_clock_div_t AHB_div, rcc_peripheral_clock_div_t APB1_div, rcc_peripheral_clock_div_t APB2_div) {

    write_masked(RCC->CFGR, AHB_div  << 4, RCC_CFGR_HPRE);      // AHB prescaler
    write_masked(RCC->CFGR, APB1_div << 10, RCC_CFGR_PPRE1);    // APB Low speed prescaler
	write_masked(RCC->CFGR, APB2_div << 13, RCC_CFGR_PPRE2);    // APB High speed prescaler

    extern rcc_system_clock_div_t HPRE_val;
    extern rcc_peripheral_clock_div_t PPRE1_val, PPRE2_val;
    HPRE_val = AHB_div;
    PPRE1_val = APB1_div;
    PPRE2_val = APB2_div;
    rcc_update_clocks();
}

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

// sets the system clock source
static inline void rcc_set_system_clock_source(rcc_system_clock_src_t src) {

    write_masked(RCC->CFGR, src << 0, RCC_CFGR_SW);
    while ((RCC->CFGR & RCC_CFGR_SWS) != (src << 2));

    extern rcc_system_clock_src_t system_clock_source;
    system_clock_source = src;
    rcc_update_clocks();
}

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

// enables clock to the specified peripheral
static inline void rcc_enable_peripheral_clock(rcc_peripheral_clock_en_t clock) {

    if      (clock < 1 * 32) set_bits(RCC->AHB1ENR, (1 << (clock - 0 * 32)));
    else if (clock < 2 * 32) set_bits(RCC->AHB2ENR, (1 << (clock - 1 * 32)));
    else if (clock < 3 * 32) set_bits(RCC->APB1ENR, (1 << (clock - 2 * 32)));
    else if (clock < 4 * 32) set_bits(RCC->APB2ENR, (1 << (clock - 3 * 32)));
}

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

// configures the PLL block and enables it
void rcc_pll_init(uint32_t output_freq_hz, rcc_pll_src_t src);

//----------------------------------------------------------------------------------------------------------------------------------------------------------------

#endif /* _HAL_RCC_H_ */