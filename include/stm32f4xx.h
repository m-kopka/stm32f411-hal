#ifndef _STM32F4XX_H_
#define _STM32F4XX_H_

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_registers.h"

//----------------------------------------------------------------------------------------------------------------------------------------------------------------

// set bits in register
#define set_bits(address, mask) ((address) |= ((uint32_t)mask))

// clear bits in register
#define clear_bits(address, mask) ((address) &= ~((uint32_t)mask))

// xor bits in register
#define xor_bits(address, mask) ((address) ^= ((uint32_t)mask))

/** write bits to a group of adjacent bits in register
 * @param address register to manipulate
 * @param value new value
 * @param mask bit mask
*/
#define write_masked(address, value, mask) ((address) = ((address) & (~(mask))) | ((value)))

#define bit_is_set(address, mask) ((address) & (mask))

#define bit_is_clear(address, mask) (!((address) & (mask)))

#define _BIT(bit) (1 << (bit))

#define force_inline inline __attribute__((always_inline))

//----------------------------------------------------------------------------------------------------------------------------------------------------------------

#define SRAM_START  0x20000000U
#define SRAM_SIZE   (128U * 1024U) // 128KB
#define SRAM_END    ((SRAM_START) + (SRAM_SIZE))
#define STACK_TOP   SRAM_END

//----------------------------------------------------------------------------------------------------------------------------------------------------------------

#endif /* _STM32F4XX_H_ */