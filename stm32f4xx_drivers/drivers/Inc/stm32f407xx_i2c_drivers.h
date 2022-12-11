/*
 * 									stm32f407xx_gpio_drivers.h
 *
 * This file contains all the I2C-related APIs supported by the driver.
 *
 */

#ifndef INC_STM32F407XX_I2C_DRIVERS_H_
#define INC_STM32F407XX_I2C_DRIVERS_H_

#include "stm32f407xx.h"

/* -- CONFIGURATION Structure for a I2C Peripheral -- */
typedef struct
{

	uint32_t	I2C_SCL_Speed;				// Possible values: I2C_SCL_SPEED
	uint8_t		I2C_Device_Address;			// Possible values: By user
	uint8_t		I2C_ACK_Control;			// Possible values: I2C_ACK_Control
	uint16_t	I2C_FM_DutyCycle;			// Possible values: I2C_FM_DutyCycle

}I2C_Config_t;

/* -- Handle Structure for I2Cx Peripheral --  */
typedef struct
{
	// Holds the base address of the I2Cx Peripheral (I2C1, I2C2, I2C3)
		// Initialized with I2C1, I2C2, I2C3 (I2Cx peripheral definitions in stm32f407xx.h and are already type-casted)
	I2C_RegDef_t	*pI2Cx;

	// To hold different I2C configuration
	I2C_Config_t	I2C_Config;

	// Required for I2C Data Tx & Rx APIs in Interrupt Mode
	uint8_t		*pTxBuffer;			// To store appplication's Tx Buffer address
	uint8_t		*pRxBuffer;			// To store appplication's Tx Buffer address
	uint32_t 	TxDataLength;			// Tx Length
	uint32_t 	RxDataLength;			// Rx Length
	uint8_t		TxRxState;			// Communication States (I2C is Half Duplex so 1 variable to represent state)
	uint8_t		DeviceAdddress;			// To store Slave/Device address
	uint32_t	RxSize;				// To store Rx size
	uint8_t		RepeatedStart;			// to store Repeated Start value (Sr)

}I2C_Handle_t;

/* -- I2C Configuration Macros -- */

// I2C_SCL_SPEED
#define I2C_SCL_SPEED_SM		100000			// Standard Mode Speed = 100KHz
#define I2C_SCL_SPEED_FM_4K		400000			// Fast Mode Speed = 400KHz

// I2C_ACK_Control [Disabled by default]
#define I2C_ACK_ENABLE			1
#define I2C_ACK_DISABLE			0			// [Default]

// I2C_FM_DutyCycle [Duty Cycle of Serial Clock in Fast Mode]
#define I2C_FM_DutyCycle_2		0			// FM Mode t(low)/t(high) = 2
#define I2C_FM_DutyCycle_16_9		1			// FM Mode t(low)/t(high) = 16/9

/* -- I2C Status Flags (with Masking details) -- */
#define I2C_FLAG_SB			(1 << I2C_SR1_SB)
#define I2C_FLAG_ADDR			(1 << I2C_SR1_ADDR)
#define I2C_FLAG_BTF			(1 << I2C_SR1_BTF)
#define I2C_FLAG_ADD10			(1 << I2C_SR1_ADD10)
#define I2C_FLAG_STOPF			(1 << I2C_SR1_STOPF)
#define I2C_FLAG_RXNE			(1 << I2C_SR1_RXNE)
#define I2C_FLAG_TXE			(1 << I2C_SR1_TXE)
#define I2C_FLAG_BERR			(1 << I2C_SR1_BERR)
#define I2C_FLAG_ARLO			(1 << I2C_SR1_ARLO)
#define I2C_FLAG_AF			(1 << I2C_SR1_AF)
#define I2C_FLAG_OVR			(1 << I2C_SR1_OVR)
#define I2C_FLAG_PECERR			(1 << I2C_SR1_PECERR)
#define I2C_FLAG_TIMEOUT		(1 << I2C_SR1_TIMEOUT)
#define I2C_FLAG_SMBALERT		(1 << I2C_SR1_SMBALERT)

/* -- Possible I2C Application States (used in Data Tx and Rx APIs in Interrupt Mode) -- */
#define I2C_READY			0
#define I2C_BUSY_IN_RX			1
#define I2C_BUSY_IN_TX			2

/* -- Possible I2C Application Events (Application callback) -- */
// Events
#define I2C_EVENT_TX_COMPLETE		0
#define I2C_EVENT_RX_COMPLETE		1
#define I2C_EVENT_STOP			2

// Errors
#define I2C_ERROR_BERR  		3
#define I2C_ERROR_ARLO 			4
#define I2C_ERROR_AF    		5
#define I2C_ERROR_OVR   		6
#define I2C_ERROR_TIMEOUT 		7

// More Events for Slave Mode
#define I2C_EVENT_DATA_REQUEST  	8
#define I2C_EVENT_DATA_RECEIVE  	9

/* -- General MACROS -- */
// Repeated Start
#define I2C_REPEATED_START_EN		ENABLE
#define I2C_REPEATED_START_DI		DISABLE

//


/* -- APIs Supported by SPI driver -- */

// Peripheral Clock Setup
// Enable/Disable Peripheral Clock for a given SPI base address
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

// Peripheral Initialize and De-initialize APIs
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

// Data Send and Receive
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t LenOfData, uint8_t SlaveAddress, uint8_t repeatedStart);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t LenOfData, uint8_t SlaveAddress, uint8_t repeatedStart);

uint8_t I2C_MasterSendData_IT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t LenOfData, uint8_t SlaveAddress, uint8_t repeatedStart);
uint8_t I2C_MasterReceiveData_IT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t LenOfData, uint8_t SlaveAddress, uint8_t repeatedStart);

void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t Data);
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx);

// Close Reception and Transmission of Data [In Interrupt Mode]
void I2C_Close_SendData(I2C_Handle_t *pI2CHandle);
void I2C_Close_ReceiveData(I2C_Handle_t *pI2CHandle);

// To generate START and STOP Conditions
void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);



// IRQ Configuration and ISR Handling
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);     	// To configure IRQ number of the I2C
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);	// To configure the priority

void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);			// TO handle interrupt by I2C EVENTS
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);			// To handle interrupts by I2C ERRORS

// Other Helper APIs
uint8_t I2C_getFlagStatus (I2C_RegDef_t *pI2Cx, uint32_t FlagName);     // To get Status Register Flags
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);	// To enable or disable the I2C peripheral
void I2C_ManageACK(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);		// To enable or disable ACKing

// Application Callbacks [To be implemented in the application]
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t ApplicationEvent);


#endif /* INC_STM32F407XX_I2C_DRIVERS_H_ */
