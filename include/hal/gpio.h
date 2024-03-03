#ifndef _HAL_GPIO_H_
#define _HAL_GPIO_H_

/*
 *  STM32F4xx GPIO driver
 *  Martin Kopka 2024 
*/

#include "stm32f4xx.h"
#include "hal/rcc.h"

#define HIGH    1
#define LOW     0

//---- ENUMERATIONS ----------------------------------------------------------------------------------------------------------------------------------------------

// GPIO mode options
typedef enum {

    GPIO_MODE_INPUT              = 0,
    GPIO_MODE_OUTPUT             = 1,
    GPIO_MODE_ALTERNATE_FUNCTION = 2,
    GPIO_MODE_ANALOG             = 3

} gpio_mode_t;

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

// GPIO output type options
typedef enum {

    GPIO_OUTPUT_TYPE_PUSH_PULL  = 0,
    GPIO_OUTPUT_TYPE_OPEN_DRAIN = 1

} gpio_output_type_t;

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

// GPIO output speed options
typedef enum {

    GPIO_OUTPUT_SPEED_LOW    = 0,
    GPIO_OUTPUT_SPEED_MEDIUM = 1,
    GPIO_OUTPUT_SPEED_FAST   = 2,
    GPIO_OUTPUT_SPEED_HIGH   = 3

} gpio_output_speed_t;

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

// gpio pull-up/pull-down options
typedef enum {

    GPIO_PULL_NONE = 0,
    GPIO_PULL_UP   = 1,
    GPIO_PULL_DOWN = 2,

} gpio_pull_t;

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

// gpio alternate function options
typedef enum {

    GPIO_ALTERNATE_FUNCTION_SYS_AF = 0,
    GPIO_ALTERNATE_FUNCTION_TIM1_TIM2 = 1,
    GPIO_ALTERNATE_FUNCTION_TIM3_TIM4_TIM5 = 2,
    GPIO_ALTERNATE_FUNCTION_TIM9_TIM10_TIM11 = 3,
    GPIO_ALTERNATE_FUNCTION_I2C1_I2C2_I2C3 = 4,
    GPIO_ALTERNATE_FUNCTION_SPI1_SPI2_SPI3 = 5,
    GPIO_ALTERNATE_FUNCTION_SPI2_SPI3_SPI4_SPI5 = 6,
    GPIO_ALTERNATE_FUNCTION_SPI3_USART1_USART2 = 7,
    GPIO_ALTERNATE_FUNCTION_USART6 = 8,
    GPIO_ALTERNATE_FUNCTION_I2C2_I2C3 = 9,
    GPIO_ALTERNATE_FUNCTION_OTG1_FS = 10,
    GPIO_ALTERNATE_FUNCTION_SDIO = 12

} gpio_alternate_function_t;

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

typedef enum {

    GPIO_IRQ_RISING_EDGE,
    GPIO_IRQ_FALLING_EDGE,
    GPIO_IRQ_BOTH_EDGES

} gpio_irq_type_t;

//---- FUNCTIONS -------------------------------------------------------------------------------------------------------------------------------------------------

// configures GPIO mode
static inline void gpio_set_mode(GPIO_TypeDef *port, uint8_t gpio, gpio_mode_t mode) {

    write_masked(port->MODER, mode << (2 * gpio), 0x3 << (2 * gpio));
}

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

// configures GPIO output as push-pull or open-drain option
static inline void gpio_set_output_type(GPIO_TypeDef *port, uint8_t gpio, gpio_output_type_t type) {

    write_masked(port->OTYPER, type << gpio, 0x1 << gpio);
}

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

// configures GPIO output speed option
static inline void gpio_set_output_speed(GPIO_TypeDef *port, uint8_t gpio, gpio_output_speed_t speed) {

    write_masked(port->OSPEEDR, speed << (2 * gpio), 0x3 << (2 * gpio));
}

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

// configures GPIO pull-up/pull-down option
static inline void gpio_set_pull(GPIO_TypeDef *port, uint8_t gpio, gpio_output_speed_t speed) {

    write_masked(port->OSPEEDR, speed << (2 * gpio), 0x3 << (2 * gpio));
}

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

// configures GPIO alternate function option
static inline void gpio_set_alternate_function(GPIO_TypeDef *port, uint8_t gpio, gpio_alternate_function_t function) {

    if (gpio < 8) write_masked(port->AFR[0], function << (4 *  gpio     ), 0xf << (4 *  gpio     ));
    else          write_masked(port->AFR[1], function << (4 * (gpio - 8)), 0xf << (4 * (gpio - 8)));
}

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

// sets GPIO output to HIGH or LOW
static inline void gpio_write(GPIO_TypeDef *port, uint8_t gpio, bool state) {

    if (state) set_bits(port->BSRRL, (1 << gpio));
    else       set_bits(port->BSRRH, (1 << gpio));
}

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

// toggles GPIO output
static inline void gpio_toggle(GPIO_TypeDef *port, uint8_t gpio) {

    xor_bits(port->ODR, (1 << gpio));
}

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

// reads GPIO input
static inline bool gpio_get(GPIO_TypeDef* port, uint8_t gpio) {

    return (!!((port)->IDR & (1 << gpio)));
}

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

// configures the GPIO interrupt
void gpio_init_interrupt(GPIO_TypeDef* port, uint8_t gpio, gpio_irq_type_t type);

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

// returns the clock enable for specified GPIO port
static inline rcc_peripheral_clock_en_t gpio_get_clock(GPIO_TypeDef* port) {

    if      (port == GPIOA) return (RCC_PERIPH_AHB1_GPIOA);
    else if (port == GPIOB) return (RCC_PERIPH_AHB1_GPIOB);
    else if (port == GPIOC) return (RCC_PERIPH_AHB1_GPIOC);
    else if (port == GPIOD) return (RCC_PERIPH_AHB1_GPIOD);
    else if (port == GPIOE) return (RCC_PERIPH_AHB1_GPIOE);
    else                    return (RCC_PERIPH_AHB1_GPIOH);
}

//----------------------------------------------------------------------------------------------------------------------------------------------------------------

#endif /* _HAL_GPIO_H_ */