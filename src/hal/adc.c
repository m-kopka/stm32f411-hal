#include "hal/adc.h"
#include "hal/rcc.h"
#include "hal/gpio.h"

//---- FUNCTIONS -------------------------------------------------------------------------------------------------------------------------------------------------

// initilizes ADC1 to single conversion mode and 12bit resolution
void adc_init(void) {

    rcc_enable_peripheral_clock(RCC_PERIPH_APB2_ADC1);

    write_masked(ADC->CCR, 3 << 16, 0x3 << 16);     // ADC clock = PCLK2 / 8
    set_bits(ADC1->CR2, ADC_CR2_ADON);              // enable ADC
}

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

// starts a conversion on the specified ADC channel, waits for the result, then returns it
uint16_t adc_read(uint8_t channel) {

    write_masked(ADC1->SQR3, channel, 0x1f);        // select channel
    set_bits(ADC1->CR2, ADC_CR2_SWSTART);           // start conversion
    while (bit_is_clear(ADC1->SR, ADC_SR_EOC));     // wait for the conversion to finish

    return (ADC1->DR);
}

//----------------------------------------------------------------------------------------------------------------------------------------------------------------
