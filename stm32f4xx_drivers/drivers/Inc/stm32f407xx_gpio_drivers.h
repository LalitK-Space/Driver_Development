/*
 * 									stm32f407xx_gpio_drivers.h
 *
 * This file contains all the GPIO-related APIs supported by the driver.
 *
 */

#ifndef INC_STM32F407XX_GPIO_DRIVERS_H_
#define INC_STM32F407XX_GPIO_DRIVERS_H_

#include "stm32f407xx.h"


/* -- CONFIGURATION Structure for a GPIO pin -- */
typedef struct
{
	// All configurable items
	uint8_t GPIO_PinNumber;				// Possible values: GPIO_Pin_Numbers
	uint8_t GPIO_PinMode;				// Possible values: GPIO_Pin_Possible_Modes
	uint8_t GPIO_PinSpeed;				// Possible values: GPIO_Pin_Possible_Output_Speed
	uint8_t GPIO_PinPuPdControl;			// Possible values: GPIO_Pin_PULL_UP_and_PULL_DOWN_Configuration
	uint8_t GPIO_PinOPType;				// Possible values: GPIO_Pin_Possible_Output_Type
	uint8_t GPIO_PinAltFuncMode;

}GPIO_PinConfig_t;


/* -- HANDLE structure for a GPIO pin -- */
typedef struct
{
	// Holds the base address of the GPIO port to which the pin belongs
	// Initialized with GPIOA, GPIOB, GPIOx (GPIOx peripheral definitions in stm32f407xx.h and are already type-casted)
	GPIO_RegDef_t *pGPIOx;

	// Structure to hold different pin configuration
	GPIO_PinConfig_t GPIO_PinConfig;

}GPIO_Handle_t;


/* -- GPIO Configuration Macros -- */

// GPIO_Pin_Numbers
#define GPIO_Pin_0			0
#define GPIO_Pin_1			1
#define GPIO_Pin_2			2
#define GPIO_Pin_3			3
#define GPIO_Pin_4			4
#define GPIO_Pin_5			5
#define GPIO_Pin_6			6
#define GPIO_Pin_7			7
#define GPIO_Pin_8			8
#define GPIO_Pin_9			9
#define GPIO_Pin_10			10
#define GPIO_Pin_11			11
#define GPIO_Pin_12			12
#define GPIO_Pin_13			13
#define GPIO_Pin_14			14
#define GPIO_Pin_15			15


// GPIO_Pin_Possible_Modes
#define GPIO_MODE_IN			0				// Input Mode (reset state)
#define GPIO_MODE_OUT			1				// Output Mode
#define GPIO_MODE_ALTFUNC		2				// Alternate Function Mode
#define GPIO_MODE_ANALOG		3				// Analog Mode

// Interrupt Modes
#define GPIO_MODE_IT_FT			4				// Falling Edge Trigger
#define GPIO_MODE_IT_RT			5				// Rising Edge Trigger
#define GPIO_MODE_IT_RFT		6				// Rising Edge Falling Edge Trigger

// GPIO_Pin_Possible_Output_Type
#define GPIO_OP_TYPE_PP			0				// Output PUSH-PULL (reset state)
#define GPIO_OP_TYPE_OD			1				// Output Open Drain

// GPIO_Pin_Possible_Output_Speed
#define GPIO_SPEED_LOW			0
#define GPIO_SPEED_MEDIUM		1
#define GPIO_SPEED_HIGH			2
#define GPIO_SPEED_VERY_HIGH		3

// GPIO_Pin_PULL_UP_and_PULL_DOWN_Configuration
#define GPIO_NO_PUPD			0			       	// No Pull-up and Pull-down
#define GPIO_PIN_PU			1				// Pull Up
#define GPIO_PIN_PD			2				// Pull Down


/* -- APIs (prototypes) Supported by this GPIO driver -- */

// Peripheral Clock Setup API
// Enable/Disable Peripheral Clock for a given GPIO base address
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

// Peripheral Initialize and De-initialize APIs
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);				// To initialize the GPIO peripheral
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);				// To de-initialize the GPIO peripheral

// Data Read and Write APIs
uint8_t  GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

// IRQ Configuration and ISR Handling
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);    	// To configure IRQ number of the GPIO
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);	// To configure the priority
void GPIO_IRQHandling(uint8_t PinNumber);				// To process the interrupt

#endif /* INC_STM32F407XX_GPIO_DRIVERS_H_ */
