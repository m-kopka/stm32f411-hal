#include "hal/gpio.h"

//---- PRIVATE FUNCTIONS -----------------------------------------------------------------------------------------------------------------------------------------

static inline uint8_t __get_gpio_EXTIx(GPIO_TypeDef* port) {

    if      (port == GPIOA) return 0;
    else if (port == GPIOB) return 1;
    else if (port == GPIOC) return 2;
    else if (port == GPIOD) return 3;
    else if (port == GPIOE) return 4;
    else                    return 7;
}

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

static inline IRQn_Type __get_gpio_irq(uint8_t gpio) {

         if (gpio == 0) return (EXTI0_IRQn);
    else if (gpio == 1) return (EXTI1_IRQn);
    else if (gpio == 2) return (EXTI2_IRQn);
    else if (gpio == 3) return (EXTI3_IRQn);
    else if (gpio == 4) return (EXTI4_IRQn);
    else if (gpio >= 5 && gpio <= 9) return (EXTI9_5_IRQn);
    else return (EXTI15_10_IRQn);
}

//---- FUNCTIONS -------------------------------------------------------------------------------------------------------------------------------------------------

// configures the GPIO interrupt
void gpio_init_interrupt(GPIO_TypeDef* port, uint8_t gpio, gpio_irq_type_t type) {

    rcc_enable_peripheral_clock(RCC_PERIPH_APB2_SYSCFG);
    gpio_set_mode(port, gpio, GPIO_MODE_INPUT);

    // connect GPIO to its respective EXTI line
    write_masked(SYSCFG->EXTICR[gpio / 4], __get_gpio_EXTIx(port) << ((gpio % 4) * 4), 0xF << ((gpio % 4) * 4));

    // Setup the button's EXTI line as an interrupt.
    set_bits(EXTI->IMR, 1 << gpio);

    // enable the rising edge trigger
    if (type == GPIO_IRQ_BOTH_EDGES || type == GPIO_IRQ_RISING_EDGE)  set_bits(EXTI->RTSR, (1 << gpio));
    else clear_bits(EXTI->RTSR, (1 << gpio));

    // enable the falling edge trigger
    if (type == GPIO_IRQ_BOTH_EDGES || type == GPIO_IRQ_FALLING_EDGE) set_bits(EXTI->FTSR, (1 << gpio));     // enable the falling edge trigger
    else clear_bits(EXTI->FTSR, (1 << gpio));

    // enable the NVIC interrupt at minimum priority.
    NVIC_SetPriority(__get_gpio_irq(gpio), 0x03);
    NVIC_EnableIRQ(__get_gpio_irq(gpio));
}

//----------------------------------------------------------------------------------------------------------------------------------------------------------------
