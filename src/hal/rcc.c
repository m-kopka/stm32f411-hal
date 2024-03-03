#include "hal/rcc.h"

//---- DATA ------------------------------------------------------------------------------------------------------------------------------------------------------

rcc_system_clock_div_t HPRE_val            = RCC_SYS_CLOCK_DIV1;                // value of the AHB1 prescaler
rcc_peripheral_clock_div_t PPRE1_val       = RCC_PERIPH_CLOCK_DIV1;             // value of the APB1 prescaler
rcc_peripheral_clock_div_t PPRE2_val       = RCC_PERIPH_CLOCK_DIV1;             // value of the APB2 prescaler
rcc_system_clock_src_t system_clock_source = RCC_SYSTEM_CLOCK_SOURCE_HSI;       // system clock source (HSI, HSE or PLL)
uint32_t HSI_frequency_hz         = HSI_VALUE;      // High Speed Internal oscillator frequency
uint32_t HSE_frequency_hz         = 0;              // High Speed External oscillator frequency
uint32_t PLL_frequency_hz         = 0;              // PLL output frequency
uint32_t HLCK_frequency_hz        = HSI_VALUE;      // System clock frequency
uint32_t PCLK1_frequency_hz       = HSI_VALUE;      // APB1 bus frequency
uint32_t PCLK1_timer_frequency_hz = HSI_VALUE;      // frequency of timer clocks on APB1 bus
uint32_t PCLK2_frequency_hz       = HSI_VALUE;      // APB2 bus frequency
uint32_t PCLK2_timer_frequency_hz = HSI_VALUE;      // frequency of timer clock on APB2 bus

//---- FUNCTIONS -------------------------------------------------------------------------------------------------------------------------------------------------

// configures the PLL block and enables it
void rcc_pll_init(uint32_t output_freq_hz, rcc_pll_src_t src) {

    if (src == RCC_PLL_SOURCE_HSE && HSE_frequency_hz == 0) return;     // HSE selected as source but is not enabled

    uint32_t input_freq_hz = (src == RCC_PLL_SOURCE_HSE) ? HSE_frequency_hz : HSI_frequency_hz;

    uint32_t pll_m = input_freq_hz / 1000000;           // input prescaler for 1MHz input frequency
    uint32_t pll_n = output_freq_hz * 2 / 1000000;      // multiplier for output frequency x 2
    uint32_t pll_p = 0;                                 // postdivider / 2

    // disable PLL and wait for ready flag
    clear_bits(RCC->CR, RCC_CR_PLLON);
    while (bit_is_set(RCC->CR, RCC_CR_PLLRDY));

    // configure PLL
    RCC->PLLCFGR = (pll_m << 0) | (pll_n << 6) | (pll_p << 16) | (src << 22);

    // enable PLL and wait for ready flag
    set_bits(RCC->CR, RCC_CR_PLLON);
    while (bit_is_clear(RCC->CR, RCC_CR_PLLRDY));

    PLL_frequency_hz = input_freq_hz / pll_m * pll_n / 2;
}

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

// updates the clock frequency variables based on the current settings
void rcc_update_clocks(void) {

    uint32_t HPRE_div, PPRE1_div, PPRE2_div;

    // get AHB1 division factor
    if (HPRE_val == RCC_SYS_CLOCK_DIV1)   HPRE_div = 1;
    if (HPRE_val == RCC_SYS_CLOCK_DIV2)   HPRE_div = 2;
    if (HPRE_val == RCC_SYS_CLOCK_DIV4)   HPRE_div = 4;
    if (HPRE_val == RCC_SYS_CLOCK_DIV8)   HPRE_div = 8;
    if (HPRE_val == RCC_SYS_CLOCK_DIV16)  HPRE_div = 16;
    if (HPRE_val == RCC_SYS_CLOCK_DIV64)  HPRE_div = 64;
    if (HPRE_val == RCC_SYS_CLOCK_DIV128) HPRE_div = 128;
    if (HPRE_val == RCC_SYS_CLOCK_DIV256) HPRE_div = 256;
    if (HPRE_val == RCC_SYS_CLOCK_DIV512) HPRE_div = 512;

    // get APB1 division factor
    if (PPRE1_val == RCC_PERIPH_CLOCK_DIV1)  PPRE1_div = 1;
    if (PPRE1_val == RCC_PERIPH_CLOCK_DIV2)  PPRE1_div = 2;
    if (PPRE1_val == RCC_PERIPH_CLOCK_DIV4)  PPRE1_div = 4;
    if (PPRE1_val == RCC_PERIPH_CLOCK_DIV8)  PPRE1_div = 8;
    if (PPRE1_val == RCC_PERIPH_CLOCK_DIV16) PPRE1_div = 16;

    // get APB2 division factor
    if (PPRE2_val == RCC_PERIPH_CLOCK_DIV1)  PPRE2_div = 1;
    if (PPRE2_val == RCC_PERIPH_CLOCK_DIV2)  PPRE2_div = 2;
    if (PPRE2_val == RCC_PERIPH_CLOCK_DIV4)  PPRE2_div = 4;
    if (PPRE2_val == RCC_PERIPH_CLOCK_DIV8)  PPRE2_div = 8;
    if (PPRE2_val == RCC_PERIPH_CLOCK_DIV16) PPRE2_div = 16;

    // get system clock frequency based on the switch position
    if (system_clock_source == RCC_SYSTEM_CLOCK_SOURCE_HSI) HLCK_frequency_hz = HSI_frequency_hz / HPRE_div;
    if (system_clock_source == RCC_SYSTEM_CLOCK_SOURCE_HSE) HLCK_frequency_hz = HSE_frequency_hz / HPRE_div;
    if (system_clock_source == RCC_SYSTEM_CLOCK_SOURCE_PLL) HLCK_frequency_hz = PLL_frequency_hz / HPRE_div;

    // get frequency of APB1 and APB2
    PCLK1_frequency_hz = HLCK_frequency_hz / PPRE1_div;
    PCLK2_frequency_hz = HLCK_frequency_hz / PPRE2_div;

    PCLK1_timer_frequency_hz = (PPRE1_div == 1) ? PCLK1_frequency_hz : PCLK1_frequency_hz * 2;      // frequency of APB1 timers is PCLK1*2 unless the APB1 divider is 1
    PCLK2_timer_frequency_hz = PCLK2_frequency_hz;
}

//----------------------------------------------------------------------------------------------------------------------------------------------------------------
