#ifndef _HAL_UART_H_
#define _HAL_UART_H_

/*
 *  STM32F411 UART hardware abstraction layer
 *  Martin Kopka 2024
 *
 *  Implements software FIFOs (circular buffers) for:
 *  • data to be transmitted by the hardware
 *  • received data not yet read by the software
*/ 

#include "stm32f4xx.h"
#include "hal/gpio.h"

//---- FUNCTIONS -------------------------------------------------------------------------------------------------------------------------------------------------

// initializes the UART hardware, the callee must provide a TX and RX buffers and their size. The driver then uses the buffers as TX and RX fifos
void uart_init(USART_TypeDef *uart, uint32_t baudrate, GPIO_TypeDef *tx_port, uint8_t tx_gpio, GPIO_TypeDef *rx_port, uint8_t rx_gpio, char *tx_buffer, uint32_t tx_buffer_size, char *rx_buffer, uint32_t rx_buffer_size);

// returns true, if RX buffer contains new data
bool uart_has_data(USART_TypeDef *uart);

// transmits one byte via UART
void uart_putc(USART_TypeDef *uart, char c);

// converts a number to string and sends it via UART
void uart_puti(USART_TypeDef *uart, int num);

// transmits a null-terminated string via UART
void uart_puts(USART_TypeDef *uart, const char *str);

// reads one byte from the RX buffer; returns -1 if there is no new data
int16_t uart_getc(USART_TypeDef *uart);

//----------------------------------------------------------------------------------------------------------------------------------------------------------------

#endif /* _HAL_UART_H_ */