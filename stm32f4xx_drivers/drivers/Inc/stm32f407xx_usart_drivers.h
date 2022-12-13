/*
 * 									stm32f407xx_usart_drivers.h
 *
 * This file contains all the USART-related APIs supported by the driver.
 *
 */

#ifndef INC_STM32F407XX_USART_DRIVERS_H_
#define INC_STM32F407XX_USART_DRIVERS_H_

#include "stm32f407xx.h"

/* -- CONFIGURATION Structure for a USART Peripheral -- */
typedef struct
{

	uint8_t		USART_Mode;				// Possible values:
	uint32_t	USART_BaudRate;			// Possible values:
	uint8_t		USART_NoOfStopBits;		// Possible values:
	uint8_t		USART_WordLength;		// Possible values:
	uint8_t 	USART_ParityControl;	// Possible values:
	uint8_t 	USART_HW_FlowControl;	// Possible values:


}USART_Config_t;

/* -- Handle Structure for USARTx Peripheral --  */
typedef struct
{
	// Holds the base address of the USARTx Peripheral
		// Initialized with USARTx (USARTx peripheral definitions in stm32f407xx.h and are already type-casted)
	USART_RegDef_t	*pUSARTx;

	// To hold different USART configuration
	USART_Config_t	USART_Config;

}USART_Handle_t;


/* -- USART Configuration Macros -- */

// USART Modes


// USART Baud Rate


// USART Number of Stop Bits


// USART Word Length


// USART Parity Control


// USART Hardware Flow Control


/* -- USART Status Flags (with Masking details) -- */



/* -- Possible USART Application States (used in Data Tx and Rx APIs in Interrupt Mode) -- */


/* -- Possible USART Application Events (Application callback) -- */


/* -- General MACROS -- */



#endif /* INC_STM32F407XX_USART_DRIVERS_H_ */
