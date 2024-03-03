
#include "stm32f4xx.h"
#include "startup/vector_table.h"

extern uint32_t _la_data;       // start address of .data segment in FLASH
extern uint32_t _sdata;         // start address of .data segment in SRAM
extern uint32_t _edata;         //   end address of .data segment in SRAM
extern uint32_t _sbss;          // start address of .bss segment in SRAM 
extern uint32_t _ebss;          //   end address of .bss segment in SRAM

#define DATA_SIZE (((uint32_t)&_edata) - ((uint32_t)&_sdata))       // .data segment size
#define BSS_SIZE  (((uint32_t)&_ebss) - ((uint32_t)&_sbss))         // .bss segment size


extern int main();

// entry point
void Reset_Handler() {

    // copy .data segment from FLASH to SRAM
    uint8_t *p_src = (uint8_t*)&_la_data;
    uint8_t *p_dst = (uint8_t*)&_sdata;

    for (uint32_t i = 0; i < DATA_SIZE; i++) {

        *p_dst++ = *p_src++;
    }

    // initialize .bss segment to 0
    p_dst = (uint8_t*)&_sbss;

    for (uint32_t i = 0; i < BSS_SIZE; i++) {

        *p_dst++ = 0x00;
    }

    main();     // branch to main
}

void Default_Handler() {

    return;
}
