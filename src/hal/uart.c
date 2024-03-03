#include "hal/uart.h"
#include "hal/rcc.h"
#include "utils/string.h"

//---- PRIVATE DATA ----------------------------------------------------------------------------------------------------------------------------------------------

static uart_fifo_t *tx_fifo[3] = {0};       // UART transmit FIFO buffer for UART1, UART2 and UART6
static uart_fifo_t *rx_fifo[3] = {0};       // UART receive FIFO buffer for UART1, UART2 and UART6

//---- PRIVATE FUNCTIONS -----------------------------------------------------------------------------------------------------------------------------------------

static force_inline void fifo_push(uart_fifo_t *fifo, char data) {

    fifo->data[fifo->head++] = data;

    if (fifo->head == UART_FIFO_SIZE) fifo->head = 0;
    if (fifo->head == fifo->tail) fifo->is_full = true;
}

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

static force_inline char fifo_pop(uart_fifo_t *fifo) {

    char data = fifo->data[fifo->tail++];
    if (fifo->tail == UART_FIFO_SIZE) fifo->tail = 0;
    fifo->is_full = false;

    return data;
}

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

static force_inline bool fifo_has_data(uart_fifo_t *fifo) {

    return ((fifo->head != fifo->tail) || fifo->is_full);
}

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

static force_inline bool fifo_is_full(uart_fifo_t *fifo) {

    return (fifo->is_full);
}

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

static force_inline void fifo_flush(uart_fifo_t *fifo) {

    fifo->tail = fifo->head;
    fifo->is_full = false;
}

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

static force_inline rcc_peripheral_clock_en_t get_uart_clock(USART_TypeDef *uart) {

    if      (uart == USART1) return (RCC_PERIPH_APB2_USART1);
    else if (uart == USART2) return (RCC_PERIPH_APB1_USART2);
    else                     return (RCC_PERIPH_APB2_USART6);
}

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

static force_inline gpio_alternate_function_t get_alternate_function(USART_TypeDef *uart) {

    if      (uart == USART1) return (GPIO_ALTERNATE_FUNCTION_SPI3_USART1_USART2);
    else if (uart == USART2) return (GPIO_ALTERNATE_FUNCTION_SPI3_USART1_USART2);
    else                     return (GPIO_ALTERNATE_FUNCTION_USART6);
}

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

static force_inline IRQn_Type get_uart_irq(USART_TypeDef *uart) {

    if      (uart == USART1) return (USART1_IRQn);
    else if (uart == USART2) return (USART2_IRQn);
    else                     return (USART6_IRQn);
}

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

static force_inline rcc_peripheral_clock_en_t get_gpio_clock(GPIO_TypeDef *gpio_port) {

    if      (gpio_port == GPIOA) return (RCC_PERIPH_AHB1_GPIOA);
    else if (gpio_port == GPIOB) return (RCC_PERIPH_AHB1_GPIOB);
    else if (gpio_port == GPIOC) return (RCC_PERIPH_AHB1_GPIOC);
    else if (gpio_port == GPIOD) return (RCC_PERIPH_AHB1_GPIOD);
    else if (gpio_port == GPIOE) return (RCC_PERIPH_AHB1_GPIOE);
    else                         return (RCC_PERIPH_AHB1_GPIOH);
}

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

static force_inline uint8_t get_fifo(USART_TypeDef *uart) {

    if      (uart == USART1) return (0);
    else if (uart == USART2) return (1);
    else                     return (2);
}

//---- FUNCTIONS -------------------------------------------------------------------------------------------------------------------------------------------------

// initializes the UART hardware
void uart_init(USART_TypeDef *uart, uint32_t baudrate, GPIO_TypeDef *tx_port, uint8_t tx_gpio, GPIO_TypeDef *rx_port, uint8_t rx_gpio, uart_fifo_t *tx_buffer, uart_fifo_t *rx_buffer) {

    if (tx_port == 0 && rx_port == 0) return;
    if (tx_buffer == 0 && rx_buffer == 0) return;
    
    rcc_enable_peripheral_clock(get_uart_clock(uart));
    rcc_enable_peripheral_clock(get_gpio_clock(tx_port));
    rcc_enable_peripheral_clock(get_gpio_clock(rx_port));

    if (tx_port != 0) {

        gpio_set_mode(tx_port, tx_gpio, GPIO_MODE_ALTERNATE_FUNCTION);
        gpio_set_alternate_function(tx_port, tx_gpio, get_alternate_function(uart));
    }         
    
    if (rx_port != 0) {

        gpio_set_mode(rx_port, rx_gpio, GPIO_MODE_ALTERNATE_FUNCTION);
        gpio_set_alternate_function(rx_port, rx_gpio, get_alternate_function(uart));
    }

    // USART2 is on APB1, USART1 and USART6 are on APB2
    uint32_t bus_frequency = (uart == USART2) ? PCLK1_frequency_hz : PCLK2_frequency_hz;

    // configure the baud rate divisor
    // baud divisor is PCLK * (1/16) / baud
    // by using the formula PCLK / baud, we get the value shifted left by 4 bits (16x larger)
    // the top bits become the integer part of BRR and the bottom 4 bits become the fractional part
	uart->BRR = bus_frequency / baudrate;

    // oversampling 16, 8bit word, disable parity
    uart->CR1 = 0;
    if (tx_buffer != 0) set_bits(uart->CR1, USART_CR1_TE);                          // transmit enable if a tx_fifo was provided by the user
    if (rx_buffer != 0) set_bits(uart->CR1, USART_CR1_RE | USART_CR1_RXNEIE);       // receive enable and RX data register not empty interrupt if a tx_fifo was provided by the user
    uart->CR2 = 0;
    uart->CR3 = 0;
    set_bits(uart->CR1, USART_CR1_UE);      // UART enable

    tx_fifo[get_fifo(uart)] = tx_buffer;
    rx_fifo[get_fifo(uart)] = rx_buffer;
    if (tx_buffer != 0) fifo_flush(tx_fifo[get_fifo(uart)]);
    if (rx_buffer != 0) fifo_flush(rx_fifo[get_fifo(uart)]);

    NVIC_EnableIRQ(get_uart_irq(uart));         // enable the UART IRQ in NVIC
    NVIC_SetPriority(get_uart_irq(uart), 0);
}

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

// returns true, if the RX buffer contains new data
volatile bool uart_has_data(USART_TypeDef *uart) {

    if (rx_fifo[get_fifo(uart)] == 0) false;
    return (fifo_has_data(rx_fifo[get_fifo(uart)]));
}

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

// transmits one byte via UART; waits if the TX fifo is full
void uart_putc(USART_TypeDef *uart, char c) {

    if (tx_fifo[get_fifo(uart)] == 0) return;
    while(fifo_is_full(tx_fifo[get_fifo(uart)]));

    __disable_irq();
    fifo_push(tx_fifo[get_fifo(uart)], c);
    set_bits(uart->CR1, USART_CR1_TXEIE);     // enable TX data register empty interrupt
    __enable_irq();
}

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

// converts a number to string and sends it via UART
void uart_puti(USART_TypeDef *uart, int num) {

    char temp_buff[16];
    itoa(num, temp_buff, 16, 10);
    uart_puts(uart, temp_buff);
}

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

// transmits a null-terminated string via UART; waits if the TX fifo is full
void uart_puts(USART_TypeDef *uart, const char *str) {

    while (*str != '\0') uart_putc(uart, *str++);
}

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

// reads one byte from the RX buffer; returns -1 if there is no new data
int16_t uart_getc(USART_TypeDef *uart) {

    if (rx_fifo[get_fifo(uart)] == 0) return -1;
    if (!fifo_has_data(rx_fifo[get_fifo(uart)])) return -1;
    return fifo_pop(rx_fifo[get_fifo(uart)]);
}

//---- IRQ HANDLERS ----------------------------------------------------------------------------------------------------------------------------------------------

static force_inline void uart_handler(USART_TypeDef *uart) {

    // interrupt was triggered by TX buffer empty
    if (bit_is_set(uart->SR, USART_SR_TXE)) {
        
        if (fifo_has_data(tx_fifo[get_fifo(uart)])) uart->DR = fifo_pop(tx_fifo[get_fifo(uart)]);
        else clear_bits(uart->CR1, USART_CR1_TXEIE);      // done transmiting disable TX data register empty interrupt
    }

    // interrupt was triggered by RX buffer not empty
    if (bit_is_set(uart->SR, USART_SR_RXNE)) {

        if (!fifo_is_full(rx_fifo[get_fifo(uart)])) fifo_push(rx_fifo[get_fifo(uart)], uart->DR);
    }

    NVIC_ClearPendingIRQ(get_uart_irq(uart));
}

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

void USART1_Handler(void) {

    uart_handler(USART1);
}

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

void USART2_Handler(void) {

    uart_handler(USART2);
}

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

void USART6_Handler(void) {

    uart_handler(USART6);
}

//----------------------------------------------------------------------------------------------------------------------------------------------------------------