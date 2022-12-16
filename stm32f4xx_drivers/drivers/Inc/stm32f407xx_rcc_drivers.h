/*
 * 									stm32f407xx_rcc_drivers.h
 *
 * This file contains all the RCC-related APIs supported by the driver.
 *
 */

#include "stm32f407xx.h"

#ifndef INC_STM32F407XX_RCC_DRIVERS_H_
#define INC_STM32F407XX_RCC_DRIVERS_H_

/* - To get System Clock Frequency - */

// To get Peripheral Clock APB1 Frequency (pCLK1)
uint32_t RCC_Pclk1_Value(void);

// To get Peripheral Clock APB2 Frequency (pCLK2)
uint32_t RCC_Pclk2_Value(void);



#endif /* INC_STM32F407XX_RCC_DRIVERS_H_ */
