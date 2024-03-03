#ifndef _INTERRUPTS_H_
#define _INTERRUPTS_H_

#define ISR_WEAK __attribute__ ((weak, alias("Default_Handler")))

//---- ARM Cortex M4 exeptions -----------------------------------------------------------------------------------------------------------------------------------

void          Reset_Handler();             // Reset Handler
void ISR_WEAK NMI_Handler();               // Non maskable interrupt, Clock Security System
void ISR_WEAK HardFault_Handler();         // All class of fault
void ISR_WEAK MemManage_Handler();         // Memory management
void ISR_WEAK BusFault_Handler();          // Pre-fetch fault, memory access fault
void ISR_WEAK UsageFault_Handler();        // Undefined instruction or illegal state
void ISR_WEAK SVCall_Handler();            // System Service call via SWI instruction
void ISR_WEAK Debug_Monitor_Handler();     // Debug Monitor
void ISR_WEAK PendSV_Handler();            // Pendable request for system service
void ISR_WEAK SysTick_Handler();           // System tick timer

//----------------------------------------------------------------------------------------------------------------------------------------------------------------

void ISR_WEAK FPU_Handler();           // FPU global interrupt
void ISR_WEAK WWDG_Handler();          // Window Watchdog interrupt
void ISR_WEAK PVD_Handler();           // EXTI Line 16 interrupt / PVD through EXTI line detection interrupt
void ISR_WEAK TAMP_STAMP_Handler();    // EXTI Line 21 interrupt / Tamper and TimeStamp interrupts through the EXTI line
void ISR_WEAK FLASH_Handler();         // Flash global interrupt
void ISR_WEAK RCC_Handler();           // RCC global interrupt
void ISR_WEAK SDIO_Handler();          // SDIO global interrupt

//---- EXTI Handlers ---------------------------------------------------------------------------------------------------------------------------------------------

void ISR_WEAK EXTI0_Handler();         // EXTI Line0 interrupt
void ISR_WEAK EXTI1_Handler();         // EXTI Line1 interrupt
void ISR_WEAK EXTI2_Handler();         // EXTI Line2 interrupt
void ISR_WEAK EXTI3_Handler();         // EXTI Line3 interrupt
void ISR_WEAK EXTI4_Handler();         // EXTI Line4 interrupt
void ISR_WEAK EXTI9_5_Handler();       // EXTI Line[9:5] interrupts
void ISR_WEAK EXTI15_10_Handler();     // EXTI Line[15:10] interrupts

//---- Timer Handlers --------------------------------------------------------------------------------------------------------------------------------------------

void ISR_WEAK TIM1_BRK_TIM9_Handler();         // TIM1 Break interrupt and TIM9 global interrupt
void ISR_WEAK TIM1_UP_TIM10_Handler();         // TIM1 Update interrupt and TIM10 global interrupt
void ISR_WEAK TIM1_TRG_COM_TIM11_Handler();    // TIM1 Trigger and Commutation interrupts and TIM11 global interrupt
void ISR_WEAK TIM1_CC_Handler();               // TIM1 Capture Compare interrupt
void ISR_WEAK TIM2_Handler();                  // TIM2 global interrupt
void ISR_WEAK TIM3_Handler();                  // TIM3 global interrupt
void ISR_WEAK TIM4_Handler();                  // TIM4 global interrupt
void ISR_WEAK TIM5_Handler();                  // TIM5 global interrupt

//---- ADC Handlers ----------------------------------------------------------------------------------------------------------------------------------------------
 
void ISR_WEAK ADC_Handler();       // ADC1 global interrupts

//---- USART Handlers --------------------------------------------------------------------------------------------------------------------------------------------

void ISR_WEAK USART1_Handler();        // USART1 global interrupt
void ISR_WEAK USART2_Handler();        // USART2 global interrupt
void ISR_WEAK USART6_Handler();        // USART6 global interrupt

//---- I2C Handlers ----------------------------------------------------------------------------------------------------------------------------------------------

void ISR_WEAK I2C1_EV_Handler();       // I2C1 event interrupt
void ISR_WEAK I2C1_ER_Handler();       // I2C1 error interrupt
void ISR_WEAK I2C2_EV_Handler();       // I2C2 event interrupt
void ISR_WEAK I2C2_ER_Handler();       // I2C2 error interrupt
void ISR_WEAK I2C3_EV_Handler();       // I2C3 event interrupt
void ISR_WEAK I2C3_ER_Handler();       // I2C3 error interrupt

//---- SPI Handlers ----------------------------------------------------------------------------------------------------------------------------------------------

void ISR_WEAK SPI1_Handler();      // SPI1 global interrupt
void ISR_WEAK SPI2_Handler();      // SPI2 global interrupt
void ISR_WEAK SPI3_Handler();      // SPI3 global interrupt
void ISR_WEAK SPI4_Handler();      // SPI4 global interrupt
void ISR_WEAK SPI5_Handler();      // SPI5 global interrupt

//---- OTG Handlers ----------------------------------------------------------------------------------------------------------------------------------------------

void ISR_WEAK OTG_FS_WKUP_Handler();       // EXTI Line 18 interrupt / USB On-The-Go FS Wakeup through EXTI line interrupt
void ISR_WEAK OTG_FS_Handler();            // USB On The Go FS global interrupt

//---- DMA Handlers ---------------------------------------------------------------------------------------------------------------------------------------------

void ISR_WEAK DMA1_Stream0_Handler();      // DMA1 Stream0 global interrupt
void ISR_WEAK DMA1_Stream1_Handler();      // DMA1 Stream1 global interrupt
void ISR_WEAK DMA1_Stream2_Handler();      // DMA1 Stream2 global interrupt
void ISR_WEAK DMA1_Stream3_Handler();      // DMA1 Stream3 global interrupt
void ISR_WEAK DMA1_Stream4_Handler();      // DMA1 Stream4 global interrupt
void ISR_WEAK DMA1_Stream5_Handler();      // DMA1 Stream5 global interrupt
void ISR_WEAK DMA1_Stream6_Handler();      // DMA1 Stream6 global interrupt
void ISR_WEAK DMA1_Stream7_Handler();      // DMA1 Stream7 global interrupt

void ISR_WEAK DMA2_Stream0_Handler();      // DMA2 Stream0 global interrupt
void ISR_WEAK DMA2_Stream1_Handler();      // DMA2 Stream1 global interrupt
void ISR_WEAK DMA2_Stream2_Handler();      // DMA2 Stream2 global interrupt
void ISR_WEAK DMA2_Stream3_Handler();      // DMA2 Stream3 global interrupt
void ISR_WEAK DMA2_Stream4_Handler();      // DMA2 Stream4 global interrupt
void ISR_WEAK DMA2_Stream5_Handler();      // DMA2 Stream5 global interrupt
void ISR_WEAK DMA2_Stream6_Handler();      // DMA2 Stream6 global interrupt
void ISR_WEAK DMA2_Stream7_Handler();      // DMA2 Stream7 global interrupt

//---- CAN Handlers ----------------------------------------------------------------------------------------------------------------------------------------------

void ISR_WEAK CAN1_TX_Handler();        // CAN1 TX interrupt
void ISR_WEAK CAN1_RX0_Handler();       // CAN1 RX0 interrupt
void ISR_WEAK CAN1_RX1_Handler();       // CAN1 RX1 interrupt
void ISR_WEAK CAN1_SCE_Handler();       // CAN1 SCE interrupt

//---- RTC Handlers ----------------------------------------------------------------------------------------------------------------------------------------------

void ISR_WEAK RTC_WKUP_Handler();      // EXTI Line 22 interrupt / RTC Wakeup interrupt through the EXTI line
void ISR_WEAK RTC_Alarm_Handler();     // EXTI Line 17 interrupt / RTC Alarms (A and B) through EXTI line interrupt

//----------------------------------------------------------------------------------------------------------------------------------------------------------------

// Default Handler for unhandled exeptions
void Default_Handler();

#endif /* _INTERRUPTS_H_ */