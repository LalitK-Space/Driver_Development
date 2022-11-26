/*
 * stm32f407xx_spi_drivers.h
 *
 *  Created on: Aug. 26, 2022
 *      Author: Lalit
 */

#ifndef INC_STM32F407XX_SPI_DRIVERS_H_
#define INC_STM32F407XX_SPI_DRIVERS_H_

#include "stm32f407xx.h"

/* -- CONFIGURATION Structure for SPIx Peripheral -- */
typedef struct
{
	// All configurable items
	uint8_t SPI_DeviceMOde;				// Possible values: @SPI_Device_Modes
	uint8_t SPI_BusConfig;				// Possible values: @SPI_Communication_Type
	uint8_t SPI_DFF;					// Possible values: @SPI_Data_Frame_Format
	uint8_t SPI_CPHA;					// Possible values: @SPI_Clock_Phase
	uint8_t SPI_COPL;					// Possible values: @SPI_Clock_Polarity
	uint8_t SPI_SSM;					// Possible values: @SPI_Slave_Select_Management
	uint8_t SPI_Speed;					// Possible values: @SPI_Serial_Clock_Speed

}SPI_Config_t;

/* -- Handle Structure for SPIx Peripheral -- */
typedef struct
{
	// Holds the base address of the SPIx Peripheral (SPI1, SPI2, SPI3, SPI4)
	//initialized with SPI1, SPI2, SPI3 (SPIx peripheral definitions in stm32f407xx.h and are already type-casted)
	SPI_RegDef_t *pSPIx;

	// Structure to hold different SPI configuration
	SPI_Config_t SPIConfig;

	// Required for SPI Data Tx & Rx APIs in Interrupt Mode
	uint8_t 	*pTxBuffer;					// To store application's Tx buffer address
	uint8_t 	*pRxBuffer;					// To store application's Rx buffer address
	uint32_t 	TxDataLength;				// Tx Length
	uint32_t 	RxDataLength;				// Rx Length
	uint8_t		TxState;					// Tx State
	uint8_t		RxState;					// Rx State

}SPI_Handle_t;


/* -- SPI Configuration Macros -- */

// @SPI_Device_Modes [SPI Control Register 1: MSTR bit]
#define SPI_DEVICE_MODE_MASTER				1
#define SPI_DEVICE_MODE_SLAVE				0								// [Default]

// @SPI_Communication_Type [SPI Control Register 1: BIDI OE and BIDI MODE]
#define SPI_BUS_CONFIG_FD					1								// Full-Duplex
#define SPI_BUS_CONFIG_HD					2								// Half-Duplex
		/*SIMPLEX Tx_only is same as FULL-DUPLEX, just disconnect MISO Line*/
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY		3								// Simplex Rx

// @SPI_Data_Frame_Format [SPI Control Register 1: DFF bit]
#define SPI_DFF_8BITS						0								// [Default]
#define SPI_DFF_16BITS						1

// @SPI_Clock_Phase [SPI Control Register 1:  CPHA bit]
#define SPI_CPOL_HIGH						1
#define SPI_CPOL_LOW						0								// [Default]

// @SPI_Clock_Polarity [SPI Control Register 1: CPOL bit]
#define SPI_CPHA_HIGH						1
#define SPI_CPHA_LOW						0								// [Default]

// @SPI_Slave_Select_Management [SPI Control Register 1: SSM bit]
#define SPI_SSM_SW_EN						1
#define SPI_SSM_SW_DI						0								// [Disabled by Default]

// @SPI_Serial_Clock_Speed [Check Reference manual : SPI Control Register : BR bits]
#define SPI_SCLK_SPEED_DIV_2				0			// peripheral_clock / 2 		[Default]
#define SPI_SCLK_SPEED_DIV_4				1			// peripheral_clock / 4
#define SPI_SCLK_SPEED_DIV_8				2			// peripheral_clock / 8
#define SPI_SCLK_SPEED_DIV_16				3			// peripheral_clock / 16
#define SPI_SCLK_SPEED_DIV_32				4			// peripheral_clock / 32
#define SPI_SCLK_SPEED_DIV_64				5			// peripheral_clock / 64
#define SPI_SCLK_SPEED_DIV_128				6			// peripheral_clock / 128
#define SPI_SCLK_SPEED_DIV_256				7			// peripheral_clock / 256


/* -- SPI Status Flags Definitions (with Masking details) -- */
#define SPI_TXE_FLAG						(1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG						(1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG						(1 << SPI_SR_BSY)

/* -- Possible SPI Application States (used in Data Tx and Rx APIs in Interrupt Mode) -- */
#define SPI_READY							0
#define SPI_BUSY_IN_RX						1
#define SPI_BUSY_IN_TX						2

/* -- Possible SPI Application Events (Application callback) -- */
#define SPI_EVENT_TX_COMPLETE				1
#define SPI_EVENT_RX_COMPLETE				2
#define SPI_EVENT_OVR_ERROR					3			// Only OVRERR is defined in code

/* -- APIs (prototypes) Supported by this SPI driver -- */

// Peripheral Clock Setup
// Enable/Disable Peripheral Clock for a given SPI base address
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

// Peripheral Initialize and De-initialize APIs
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

// Data Send and Receive
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t LenOfData);		// Polling based
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t LenOfData);	// Polling based

uint8_t SPI_SendData_IT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t LenOfData);		// Interrupt based
uint8_t SPI_ReceiveData_IT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t LenOfData);	// Interrupt based


// IRQ Configuration and ISR Handling
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);     	// To configure IRQ number of the SPI
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);	// To configure the priority
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);							// To process the interrupt

// Other Helper APIs
uint8_t SPI_getFlagStatus (SPI_RegDef_t *pSPIx, uint32_t FlagName);     // To get Status Register Flags
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);		// To enable or disable the SPI peripheral
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);				// To configure SSI bit
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);				// To configure SSOE bit
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);								// To clear OVR FLAG Error
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);					// To terminate SPI communication
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);						// To terminate SPI communication

// Application Callbacks [To be implemented by the application]
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t ApplicationEvent);

#endif /* INC_STM32F407XX_SPI_DRIVERS_H_ */
