#ifndef _HAL_IWDG_H_
#define _HAL_IWDG_H_

/*
 *  STM32F4xx Independent Watchdog driver
 *  Martin Kopka 2024 
*/

#include "stm32f4xx.h"
#include "hal/rcc.h"

//---- ENUMERATIONS ----------------------------------------------------------------------------------------------------------------------------------------------

// IWDG prescaler options
typedef enum {

    IWDG_PR_4   = 0,
    IWDG_PR_8   = 1,
    IWDG_PR_16  = 2,
    IWDG_PR_32  = 3,
    IWDG_PR_64  = 4,
    IWDG_PR_128 = 5,
    IWDG_PR_256 = 6

} iwdg_prescaler_t;

//---- CONSTANTS -------------------------------------------------------------------------------------------------------------------------------------------------

#define IWDG_UNLOCK_KEY 0x5555      // writing this code to the IWDG_KR enables access to the IWDG_PR and IWDG_RLR registers
#define IWDG_START_KEY  0xCCCC      // writing this code to the IWDG_KR starts the independent watchdog
#define IWDG_RELOAD_KEY 0xAAAA      // writing this code to the IWDG_KR while the IWDG is running reloads the watchdog and prevents a system reset

//---- FUNCTIONS -------------------------------------------------------------------------------------------------------------------------------------------------

// initilizes the independent watchdog; after this the software needs to call iwdg_reload in regular intervals to prevent a system reset
static inline void iwdg_init(iwdg_prescaler_t prescale, uint16_t reload_val) {

    rcc_enable_lsi();

    IWDG->KR = IWDG_UNLOCK_KEY;
    IWDG->PR = prescale;
    IWDG->RLR = reload_val;
    IWDG->KR = IWDG_START_KEY;
}

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

// reloads the independent watchdog and prevents a system reset
static inline void iwdg_reload(void) {

    IWDG->KR = IWDG_RELOAD_KEY;
}

//----------------------------------------------------------------------------------------------------------------------------------------------------------------

#endif /* _HAL_IWDG_H_ */