#ifndef _HAL_SPI_H_
#define _HAL_SPI_H_

/*
 *  STM32F4xx SPI driver
 *  Martin Kopka 2024 
*/

#include "stm32f4xx.h"
#include "hal/gpio.h"

//---- ENUMERATIONS ----------------------------------------------------------------------------------------------------------------------------------------------

// SPI baud rate divisor values (BR bits in the SPI_CR1 register)
typedef enum {

    SPI_DIV_2   = 0x0 << 3,
    SPI_DIV_4   = 0x1 << 3,
    SPI_DIV_8   = 0x2 << 3,
    SPI_DIV_16  = 0x3 << 3,
    SPI_DIV_32  = 0x4 << 3,
    SPI_DIV_64  = 0x5 << 3,
    SPI_DIV_128 = 0x6 << 3,
    SPI_DIV_256 = 0x7 << 3

} spi_div_t;

//---- FUNCTIONS -------------------------------------------------------------------------------------------------------------------------------------------------

static inline void spi_write(SPI_TypeDef *spi, uint16_t data, GPIO_TypeDef *ss_port, uint8_t ss_gpio) {

    gpio_write(ss_port, ss_gpio, LOW);       // pull the !SS line low

    spi->DR = data;

    /*  during discontinuous communications, there is a 2 APB clock period delay between the
    write operation to SPI_DR and the BSY bit setting. As a consequence, it is mandatory to
    wait first until TXE=1 and then until BSY=0 after writing the last data.  */
    while (bit_is_clear(spi->SR, SPI_SR_TXE));
    while (bit_is_set(spi->SR, SPI_SR_BSY));

    gpio_write(ss_port, ss_gpio, HIGH);       // pull the !SS line high
}

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

static inline uint16_t spi_read(SPI_TypeDef *spi) {

    return (spi->DR);
}

//----------------------------------------------------------------------------------------------------------------------------------------------------------------

#endif /* _HAL_SPI_H_ */