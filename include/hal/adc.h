#ifndef _HAL_ADC_H_
#define _HAL_ADC_H_

/*
 *  STM32F4xx ADC driver
 *  Martin Kopka 2024 
*/

#include "stm32f4xx.h"

//---- FUNCTIONS -------------------------------------------------------------------------------------------------------------------------------------------------

// initilizes ADC1 to single conversion mode and 12bit resolution
void adc_init(void);

// starts a conversion on the specified ADC channel, waits for the result, then returns it
uint16_t adc_read(uint8_t channel);

//----------------------------------------------------------------------------------------------------------------------------------------------------------------

#endif /* _HAL_ADC_H_ */