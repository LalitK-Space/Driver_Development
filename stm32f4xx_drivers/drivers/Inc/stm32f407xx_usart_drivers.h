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

	uint8_t		USART_Mode;			// Possible values: USART Modes
	uint32_t	USART_BaudRate;			// Possible values: USART Baud Rate
	uint8_t		USART_NoOfStopBits;		// Possible values: USART Number of Stop Bits
	uint8_t		USART_WordLength;		// Possible values: USART Word Length
	uint8_t 	USART_ParityControl;		// Possible values: USART Parity Control
	uint8_t 	USART_HW_FlowControl;		// Possible values: USART Hardware Flow Control


}USART_Config_t;

/* -- Handle Structure for USARTx Peripheral --  */
typedef struct
{
	// Holds the base address of the USARTx Peripheral
	// Initialized with USART1, USART2, USART3, UART4, UART5, USART6
	// (USARTx peripheral definitions in stm32f407xx.h and are already type-casted)
	USART_RegDef_t	*pUSARTx;

	// To hold different USART configuration
	USART_Config_t	USART_Config;

	// Required for USART Data Tx & Rx APIs in Interrupt Mode
	uint8_t		*pTxBuffer;			// To store appplication's Tx Buffer address
	uint8_t		*pRxBuffer;			// To store appplication's Rx Buffer address
	uint32_t 	TxDataLength;			// Tx Length
	uint32_t 	RxDataLength;			// Rx Length
	uint8_t		TxState;			// Tx State
	uint8_t		RxState;			// Rx State

}USART_Handle_t;


/* -- USART Configuration Macros -- */

// USART Modes
#define USART_MODE_TX_ONLY		0
#define USART_MODE_RX_ONLY		1
#define USART_MODE_TX_RX		2

// USART Baud Rate (When fPCLK = 16MHz)
#define USART_STD_BAUDRATE_1200		1200
#define USART_STD_BAUDRATE_2400		2400
#define USART_STD_BAUDRATE_9600		9600
#define USART_STD_BAUDRATE_19200	19200
#define USART_STD_BAUDRATE_38400	38400
#define USART_STD_BAUDRATE_57600	57600
#define USART_STD_BAUDRATE_115200	115200
#define USART_STD_BAUDRATE_230400	230400
#define USART_STD_BAUDRATE_460800	460800
#define USART_STD_BAUDRATE_921600	921600
#define USART_STD_BAUDRATE_2M		2000000
#define USART_STD_BAUDRATE_3M		3000000

// USART Number of Stop Bits
#define USART_STOPBITS_1		0
#define USART_STOPBITS_0_5		1
#define USART_STOPBITS_2		2
#define USART_STOPBITS_1_5		3

// USART Word Length
#define USART_WORDLENGTH_8_BITS		0
#define USART_WORDLENGTH_9_BITS		1

// USART Parity Control
#define USART_PARITY_DISABLE		0
#define USART_PARITY_EVEN_EN		1
#define USART_PARITY_ODD_EN		2

// USART Hardware Flow Control
#define USART_HW_FLOW_CONTROL_NONE	0
#define USART_HW_FLOW_CONTROL_CTS	1
#define USART_HW_FLOW_CONTROL_RTS	2
#define USART_HW_FLOW_CONTROL_CTS_RTS	3

/* -- USART Status Flags (with Masking details) -- */
#define USART_FLAG_PE			(1 << USART_SR_PE)
#define USART_FLAG_FE			(1 << USART_SR_FE)
#define USART_FLAG_NF			(1 << USART_SR_NF)
#define USART_FLAG_ORE			(1 << USART_SR_ORE)
#define USART_FLAG_IDLE			(1 << USART_SR_IDLE)
#define USART_FLAG_RXNE			(1 << USART_SR_RXNE)
#define USART_FLAG_TC			(1 << USART_SR_TC)
#define USART_FLAG_TXE			(1 << USART_SR_TXE)
#define USART_FLAG_LBD			(1 << USART_SR_LBD)
#define USART_FLAG_CTS			(1 << USART_SR_CTS)


/* -- Possible USART Application States (used in Data Tx and Rx APIs in Interrupt Mode) -- */
#define USART_READY			0
#define USART_BUSY_IN_RX		1
#define USART_BUSY_IN_TX		2

/* -- Possible USART Application Events (Application callback) -- */
// Event
#define USART_EVENT_TX_COMPLETE		0
#define USART_EVENT_RX_COMPLETE		1
#define USART_EVENT_RX_IDLE		2
#define USART_EVENT_RX_CTS		3
#define USART_EVENT_RX_PE		4

// Errors
#define USART_ERROR_FE			5
#define USART_ERROR_NF			6
#define USART_ERROR_ORE			7

/* -- General MACROS -- */


/* -- APIs Supported by USART driver -- */

// Peripheral Clock Setup
// Enable/Disable Peripheral Clock for a given USART base address
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi);

// Peripheral Initialize and De-initialize APIs
void USART_Init(USART_Handle_t *pUSARTHandle);
void USART_DeInit(USART_RegDef_t *pUSARTx);

// Data Send and Receive
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t LenOfData);
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t LenOfData);

uint8_t USART_SendData_IT(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t LenOfData);
uint8_t USART_ReceiveData_IT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t LenOfData);


// IRQ Configuration and ISR Handling
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);     		// To configure IRQ number of the USART
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);		// To configure the priority
void USART_IRQHandling(USART_Handle_t *pUSARTHandle);				// To handle the ISR

// Other Helper APIs
uint8_t USART_getFlagStatus (USART_RegDef_t *pUSARTx, uint32_t FlagName);     	// To get Status Register Flags
void USART_ClearFlagStatus (USART_RegDef_t *pUSARTx, uint32_t FlagName);     	// To Clear Status Register Flags
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi);		// To enable or disable the USART peripheral
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t Baudrate);		// To Set Desired Baud Rate

// Application Callbacks [To be implemented in the application]
void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, uint8_t ApplicationEvent);

#endif /* INC_STM32F407XX_USART_DRIVERS_H_ */
