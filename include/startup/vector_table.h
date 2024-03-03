#ifndef _VECTOR_TABLE_H_
#define _VECTOR_TABLE_H_

#include "stm32f4xx.h"
#include "startup/irq_handlers.h"

uint32_t *vector_table[] __attribute__((section(".vector_table"))) = {

    (uint32_t*)STACK_TOP,                   // stack init value
    (uint32_t*)Reset_Handler,               // Reset
    (uint32_t*)NMI_Handler,                 // Non maskable interrupt, Clock Security System
    (uint32_t*)HardFault_Handler,           // All class of fault
    (uint32_t*)MemManage_Handler,           // Memory management
    (uint32_t*)BusFault_Handler,            // Pre-fetch fault, memory access fault
    (uint32_t*)UsageFault_Handler,          // Undefined instruction or illegal state
    0,                                      // (reserved)
    0,                                      // (reserved)
    0,                                      // (reserved)
    0,                                      // (reserved)
    (uint32_t*)SVCall_Handler,              // System Service call via SWI instruction
    (uint32_t*)Debug_Monitor_Handler,       // Debug Monitor
    0,                                      // (reserved)
    (uint32_t*)PendSV_Handler,               // Pendable request for system service
    (uint32_t*)SysTick_Handler,              // System tick timer
    (uint32_t*)WWDG_Handler,                 // [0]  Window Watchdog interrupt
    (uint32_t*)PVD_Handler,                  // [1]  EXTI Line 16 interrupt / PVD through EXTI line detection interrupt
    (uint32_t*)TAMP_STAMP_Handler,           // [2]  EXTI Line 21 interrupt / Tamper and TimeStamp interrupts through the EXTI line
    (uint32_t*)RTC_WKUP_Handler,             // [3]  EXTI Line 22 interrupt / RTC Wakeup interrupt through the EXTI line
    (uint32_t*)FLASH_Handler,                // [4]  Flash global interrupt
    (uint32_t*)RCC_Handler,                  // [5]  RCC global interrupt
    (uint32_t*)EXTI0_Handler,                // [6]  EXTI Line0 interrupt
    (uint32_t*)EXTI1_Handler,                // [7]  EXTI Line1 interrupt
    (uint32_t*)EXTI2_Handler,                // [8]  EXTI Line2 interrupt
    (uint32_t*)EXTI3_Handler,                // [9]  EXTI Line3 interrupt
    (uint32_t*)EXTI4_Handler,                // [10] EXTI Line4 interrupt
    (uint32_t*)DMA1_Stream0_Handler,         // [11] DMA1 Stream0 global interrupt
    (uint32_t*)DMA1_Stream1_Handler,         // [12] DMA1 Stream1 global interrupt
    (uint32_t*)DMA1_Stream2_Handler,         // [13] DMA1 Stream2 global interrupt
    (uint32_t*)DMA1_Stream3_Handler,         // [14] DMA1 Stream3 global interrupt
    (uint32_t*)DMA1_Stream4_Handler,         // [15] DMA1 Stream4 global interrupt
    (uint32_t*)DMA1_Stream5_Handler,         // [16] DMA1 Stream5 global interrupt
    (uint32_t*)DMA1_Stream6_Handler,         // [17] DMA1 Stream6 global interrupt
    (uint32_t*)ADC_Handler,                  // [18] ADC1 global interrupts
    (uint32_t*)CAN1_TX_Handler,              // [19] CAN1 TX interrupt
    (uint32_t*)CAN1_RX0_Handler,             // [20] CAN1 RX0 interrupt
    (uint32_t*)CAN1_RX1_Handler,             // [21] CAN1 RX1 interrupt
    (uint32_t*)CAN1_SCE_Handler,             // [22] CAN1 SCE interrupt
    (uint32_t*)EXTI9_5_Handler,              // [23] EXTI Line[9:5] interrupts
    (uint32_t*)TIM1_BRK_TIM9_Handler,        // [24] TIM1 Break interrupt and TIM9 global interrupt
    (uint32_t*)TIM1_UP_TIM10_Handler,        // [25] TIM1 Update interrupt and TIM10 global interrupt
    (uint32_t*)TIM1_TRG_COM_TIM11_Handler,   // [26] TIM1 Trigger and Commutation interrupts and TIM11 global interrupt
    (uint32_t*)TIM1_CC_Handler,              // [27] TIM1 Capture Compare interrupt
    (uint32_t*)TIM2_Handler,                 // [28] TIM2 global interrupt
    (uint32_t*)TIM3_Handler,                 // [29] TIM3 global interrupt
    (uint32_t*)TIM4_Handler,                 // [30] TIM4 global interrupt
    (uint32_t*)I2C1_EV_Handler,              // [31] I2C1 event interrupt
    (uint32_t*)I2C1_ER_Handler,              // [32] I2C1 error interrupt
    (uint32_t*)I2C2_EV_Handler,              // [33] I2C2 event interrupt
    (uint32_t*)I2C2_ER_Handler,              // [34] I2C2 event interrupt
    (uint32_t*)SPI1_Handler,                 // [35] SPI1 global interrupt
    (uint32_t*)SPI2_Handler,                 // [36] SPI2 global interrupt
    (uint32_t*)USART1_Handler,               // [37] USART1 global interrupt
    (uint32_t*)USART2_Handler,               // [38] USART2 global interrupt
    0,                                       // [39] 
    (uint32_t*)EXTI15_10_Handler,            // [40] EXTI Line[15:10] interrupts
    (uint32_t*)RTC_Alarm_Handler,            // [41] EXTI Line 17 interrupt / RTC Alarms (A and B) through EXTI line interrupt
    (uint32_t*)OTG_FS_WKUP_Handler,          // [42] EXTI Line 18 interrupt / USB On-The-Go FS Wakeup through EXTI line interrupt
    0,                                       // [43] 
    0,                                       // [44] 
    0,                                       // [45] 
    0,                                       // [46] 
    (uint32_t*)DMA1_Stream7_Handler,         // [47] DMA1 Stream7 global interrupt
    0,                                       // [48] 
    (uint32_t*)SDIO_Handler,                 // [49] SDIO global interrupt
    (uint32_t*)TIM5_Handler,                 // [50] TIM5 global interrupt
    (uint32_t*)SPI3_Handler,                 // [51] SPI3 global interrupt
    0,                                       // [52] 
    0,                                       // [53] 
    0,                                       // [54] 
    0,                                       // [55] 
    (uint32_t*)DMA2_Stream0_Handler,         // [56] DMA2 Stream0 global interrupt
    (uint32_t*)DMA2_Stream1_Handler,         // [57] DMA2 Stream1 global interrupt
    (uint32_t*)DMA1_Stream2_Handler,         // [58] DMA2 Stream2 global interrupt
    (uint32_t*)DMA2_Stream3_Handler,         // [59] DMA2 Stream3 global interrupt
    (uint32_t*)DMA2_Stream4_Handler,         // [60] DMA2 Stream4 global interrupt
    0,                                       // [61] 
    0,                                       // [62] 
    0,                                       // [63] 
    0,                                       // [64] 
    0,                                       // [65] 
    0,                                       // [66] 
    (uint32_t*)OTG_FS_Handler,               // [67] USB On The Go FS global interrupt
    (uint32_t*)DMA2_Stream5_Handler,         // [68] DMA2 Stream5 global interrupt
    (uint32_t*)DMA2_Stream6_Handler,         // [69] DMA2 Stream6 global interrupt
    (uint32_t*)DMA2_Stream7_Handler,         // [70] DMA2 Stream7 global interrupt
    (uint32_t*)USART6_Handler,               // [71] USART6 global interrupt
    (uint32_t*)I2C3_EV_Handler,              // [72] I2C3 event interrupt
    (uint32_t*)I2C3_ER_Handler,              // [73] I2C3 error interrupt
    0,                                       // [74] 
    0,                                       // [75] 
    0,                                       // [76] 
    0,                                       // [77] 
    0,                                       // [78] 
    0,                                       // [79] 
    0,                                       // [80] 
    (uint32_t*)FPU_Handler,                  // [81] FPU global interrupt
    0,                                       // [82] 
    0,                                       // [83] 
    (uint32_t*)SPI4_Handler,                 // [84] SPI 4 global interrupt
    (uint32_t*)SPI5_Handler                  // [85] SPI 5 global interrupt
};

#endif /* _VECTOR_TABLE_H_ */