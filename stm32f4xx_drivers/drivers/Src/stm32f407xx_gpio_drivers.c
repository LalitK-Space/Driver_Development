/*
 * 											stm32f407xx_gpio_drivers.c
 *
 *  This file contains GPIO driver API implementations.
 *
 */

#include "stm32f407xx_gpio_drivers.h"


/* -- APIs (Definitions) Supported by this GPIO driver -- */

/* ------------------------------------------------------------------------------------------------------
 * Name			:	GPIO_PeriClockControl
 * Description	:	Peripheral Clock Setup API:
 * 					This function Enables or Disables peripheral clock for the given GPIO port
 * Parameter 1	:	Base address of the GPIO peripheral
 * Parameter 2	:	ENABLE or DISABLE Macro
 * Return Type	:	none (void)
 * Note			:	none
 * ------------------------------------------------------------------------------------------------------ */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}
		else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}
		else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
		else if (pGPIOx == GPIOI)
		{
			GPIOI_PCLK_EN();
		}
		else
		{
			// Meh
		}
	}
	else
	{
		if (EnorDi == DISABLE)
		{
			if (pGPIOx == GPIOA)
			{
				GPIOA_PCLK_DI();
			}
			else if (pGPIOx == GPIOB)
			{
				GPIOB_PCLK_DI();
			}
			else if (pGPIOx == GPIOC)
			{
				GPIOC_PCLK_DI();
			}
			else if (pGPIOx == GPIOD)
			{
				GPIOD_PCLK_DI();
			}
			else if (pGPIOx == GPIOE)
			{
				GPIOE_PCLK_DI();
			}
			else if (pGPIOx == GPIOF)
			{
				GPIOF_PCLK_DI();
			}
			else if (pGPIOx == GPIOG)
			{
				GPIOG_PCLK_EN();
			}
			else if (pGPIOx == GPIOH)
			{
				GPIOH_PCLK_DI();
			}
			else if (pGPIOx == GPIOI)
			{
				GPIOI_PCLK_DI();
			}
			else
			{
				// Meh
			}
		}

	}
}


// Peripheral Initialize and De-initialize APIs

/* ------------------------------------------------------------------------------------------------------
 * Name			:	GPIO_Init
 * Description	:	Peripheral Initialize API:
 *					To initialize the given GPIO port and the given GPIO pin.
 * Parameter 1	:
 * Return Type	:	none (void)
 * Note			:	Peripheral Clock is enabled at starting of the function, so users need not do it explicitly.
 * ------------------------------------------------------------------------------------------------------ */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	// Enable Peripheral Clock
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	uint32_t temp = 0;

	// ->  1. Configure the GPIO pin Mode

	// As defined in x_gpio_drivers.h, Pin Modes greater than 3 are interrupt modes
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		// In Pin Mode Register, each pin takes 2 bit fields [Shift value according to pin number]
		// Logic: Mode value left shifted by (2 * pin number)
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));

		// Store 'temp' i.e. Mode Value in MODE Register [Set/touch only required bit leave rest untouched '|']
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // Clear required bits
		pGPIOHandle->pGPIOx->MODER |= temp;													// Update required bits

		// Reset 'temp'
		temp = 0;

	}
	else
	{
		// Interrupt Modes
		/*
		 * 1. Pin must be in INPUT Configuration
		 * 2. Configure the edge trigger (RT,FT,RFT) [Done in EXTI Registers]
		 * 3. Enable interrupt delivery from peripheral to the processor side [in EXTI Registers using Interrupt Mask Register]
		 * 4. Identify the IRQ number on which the processor accepts the interrupt from that pin. [From Vector Table]
		 * 5. Configure the IRQ priority for the identified IRQ number [NVIC Registers]
		 * 6. Enable interrupt reception on that IRQ number [NVIC Registers]
		 * 7. Implement the IRQ handler.
		 * */

		// -> Configure the edge trigger
		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			// Configure the Falling Trigger Selection Register (FTSR)
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			// As a safe side, Clear same bit in RTSR so just FTSR is configured
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			// Configure the Rising Trigger Selection Register (RTSR)
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			// As a safe side, Clear same bit in FTSR so just RTSR is configured
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			// Configure the both FTSR and RTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		// -> Configure the GPIO Port Selection in SYSCFG_EXTICR
		// GPIO_Pin_x of WHICH port is delivering interrupt? decided by GPIO Port selection [By default, PORTA]

		uint8_t SYSArrayIndex, SYSBitField;

		SYSArrayIndex = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) / 4;
		SYSBitField	  = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) % 4;

		// Now store the Port Code
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_EN();												   // Clock Enable
		SYSCFG->EXTICR[SYSArrayIndex] = portcode << (SYSBitField * 4); // before configuring, enable clock


		// -> Enable EXTI Interrupt delivery using Interrupt Mask Register
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}

	// -> 2. Configure the GPIO Speed

	// Logic: Speed value left shifted by 2 * pin number
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // Clear required bits
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;												  // Update required bits

	// Reset 'temp'
	temp = 0;

	// -> 3. Configure the Pull-up and Pull-down setting

	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // Clear required bits
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	temp = 0;

	// -> 4. Configure the GPIO Output type

	// Each Pin takes only 1 bit field thats why (1 * pin number)
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (1 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // Clear required bits
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	temp = 0;

	// -> 5. Configure the Alternate Functionality

	// If mode is selected as Alternate Function then only configure for Alternate Functionality
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFUNC)
	{
		// In AFR, each pin takes 4 bit fields and there are two AF registers
		// AF Low and AF High, in this project AF is defined as an array (stm2f407xx.h)
		// AF[0]: Low (Pin 0 - 7), AF[1]: High (Pin 8 - 15)
		// Logic: (x = (Pin_number / 8) ) if 1 then AF[1] and if 0 then AF[0]
		//      : then, to get Bit position (y = (pin_number % 8))
		//		AF[x] = alt_fnc_value  << (4 * y)

		uint8_t AFarrayIndex, AFbitField;

		AFarrayIndex = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		AFbitField	 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[AFarrayIndex] &= ~(0xF << (4 * AFbitField));					// Clear required Bits
		pGPIOHandle->pGPIOx->AFR[AFarrayIndex] |= pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFuncMode << (4 * AFbitField);

	}

}


/* ------------------------------------------------------------------------------------------------------
 * Name			:	GPIO_DeInit
 * Description	:	GPIO Peripheral De-Initialize API:
 *					reset all the registers of GPIO port mentioned
 * Parameter 1	:	Base address of the GPIO Port
 * Return Type	:	none (void)
 * Note			:
 * ------------------------------------------------------------------------------------------------------ */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	// For Resetting GPIO, refer to RCC->AHB1RSTR (AHB1 Peripheral Reset Register)
	// Make respective bit 1 to reset then again make it 0, if kept 1 then Peripheral will always be in reset state
	// SET and RESET done in MACROS
	if (pGPIOx == GPIOA)
			{
				GPIOA_REG_RESET();
			}
			else if (pGPIOx == GPIOB)
			{
				GPIOB_REG_RESET();
			}
			else if (pGPIOx == GPIOC)
			{
				GPIOC_REG_RESET();
			}
			else if (pGPIOx == GPIOD)
			{
				GPIOD_REG_RESET();
			}
			else if (pGPIOx == GPIOE)
			{
				GPIOE_REG_RESET();
			}
			else if (pGPIOx == GPIOF)
			{
				GPIOF_REG_RESET();
			}
			else if (pGPIOx == GPIOG)
			{
				GPIOG_REG_RESET();
			}
			else if (pGPIOx == GPIOH)
			{
				GPIOH_REG_RESET();
			}
			else if (pGPIOx == GPIOI)
			{
				GPIOI_REG_RESET();
			}
			else
			{
				// Meh
			}

}


// Data Read and Write APIs

/* ------------------------------------------------------------------------------------------------------
 * Name			:	GPIO_ReadFromInputPin
 * Description	:	Data Read API:
 *					Read mentioned Pin value
 * Parameter 1	:	Base address of the GPIO peripheral
 * Parameter 2	: 	Pin Number to read from
 * Return Type	: 	uint8_t : either 0 or 1
 * Note			:
 * ------------------------------------------------------------------------------------------------------ */
uint8_t  GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t pinValue;

	// get value from IDR (Input Data Register)
	// Right Shift the value of IDR by PinNumber times (so that desired value reaches at bit position 0 LSB)
	// then, mask the remaining bit positions and extract the 0th (LSB) bit. That's the pin Value

	pinValue = (uint8_t) ((pGPIOx->IDR >> PinNumber) & 0x00000001);

	return pinValue;
}


/* ------------------------------------------------------------------------------------------------------
 * Name			:	GPIO_ReadFromInputPort
 * Description	:	To read from an input pin:
 *
 * Parameter 1	:	Base address of the GPIO peripheral
 * Return Type	:	uin16_t : 16 bit value, because of 16 pins
 * Note			:
 * ------------------------------------------------------------------------------------------------------ */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t portValue;

	// get values from IDR  (Input Data Register) READ ENTIRE IDR Register
	portValue = (uint16_t) (pGPIOx->IDR);

	return portValue;
}


/* ------------------------------------------------------------------------------------------------------
 * Name			:	GPIO_WriteToOutputPin
 * Description	:	To write to single pin
*					Write given Value to given PinNumber
 * Parameter 1	:	Base address of the GPIO peripheral
 * Parameter 2	:	Pin number to be written
 * Parameter 3	:	Value to be written
 * Return Type	:	none (void)
 * Note			:
 * ------------------------------------------------------------------------------------------------------ */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if (Value == GPIO_PIN_SET)
	{
		// write 1 to the OUTPUT DATA Register at the bit position corresponds to given pin number
		pGPIOx->ODR |= (1 << PinNumber);
	}
	else
	{
		// clear or write 0 to the OUTPUT DATA Register at bit position corresponds to given pin number
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}


/* ------------------------------------------------------------------------------------------------------
 * Name			:	GPIO_WriteToOutputPort
 * Description	:	To write to entire port
 *					Write given value to the entire mentioned port
 * Parameter 1	:	Base address of the GPIO peripheral
 * Parameter 2	: 	Value to be written (uint16_t <- because of 16 pins)
 * Return Type	:	none (void)
 * Note			:
 * ------------------------------------------------------------------------------------------------------ */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	// Write given value to entire port
	pGPIOx->ODR = Value;
}


/* ------------------------------------------------------------------------------------------------------
 * Name			:	GPIO_ToggleOutputPin
 * Description	:	To toggle the given pin number
 *
 * Parameter 1	:	Base address of the GPIO peripheral
 * Parameter 2	: 	Pin Number to toggle
 * Return Type	:	none (void)
 * Note			:
 * ------------------------------------------------------------------------------------------------------ */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}


// IRQ Configuration and ISR Handling

/* ------------------------------------------------------------------------------------------------------
 * Name			:	GPIO_IRQInterruptConfig
 * Description	:	To configure IRQ:
 *					Processor specific configurations (NVIC Registers)
 * Parameter 1	:	IRQ number
 * Parameter 2	:	Enable or Disable the IRQ (ENABLE or DISABLE Macro)
 * Return Type	:	none (void)
 * Note			:
 * ------------------------------------------------------------------------------------------------------ */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		// Interrupt Set Enable Registers NVIC_ISERx
		if (IRQNumber <= 31)
		{
			// Configure ISER0 Register
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if (IRQNumber > 31 && IRQNumber < 64)
		{
			// Configure ISER1 Register
			*NVIC_ISER1 |= (1 << IRQNumber % 32);  // x % 32 to get to second register set and from its 0th bit

		}
		else if (IRQNumber >= 64 && IRQNumber < 96)
		{
			// COnfigure ISER2 Register : Sufficient, no need to configure more ISERx Registers
			*NVIC_ISER2 |= (1 << IRQNumber % 64);  // x % 64 to get to third register set and from its 0th bit
		}
	}
	else	// Have to write 1 also to clear, writing 0 in ISER makes no effect
	{
		// Interrupt Clear Enable Registers NVIC_ISCRx
		if (IRQNumber <= 31)
		{
			// Configure ICER0 Register
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if (IRQNumber > 31 && IRQNumber < 64)
		{
			// Configure ICER1 Register
			*NVIC_ICER1 |= (1 << IRQNumber % 32);  // x % 32 to get to second register set and from its 0th bit
		}
		else if (IRQNumber >= 64 && IRQNumber < 96)
		{
			// COnfigure ICER2 Register : Sufficient, no need to configure more ICERx Registers
			*NVIC_ICER2 |= (1 << IRQNumber % 64);  // x % 64 to get to third register set and from its 0th bit
		}
	}

}


/* ------------------------------------------------------------------------------------------------------
 * Name			:	GPIO_IRQPriorityConfig
 * Description	:	To configure the priority of the interrupt:
 *
 * Parameter 1	:	IRQ Number
 * Parameter 2	:	IRQ Priority
 * Return Type	:	none (void)
 * Note			:
 * ------------------------------------------------------------------------------------------------------ */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	// There are 60 IPR (Interrupt Priority) registers
	// Each register is of 32 bits and divided into 4 sections to accommodate 4 Priority values

	// Now to get the right section and right bit field
	uint8_t iprx		 = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	// NVIC_PRI_BASEADDR + iprx to jump to the required address
	// shift value is calculated because lower 4 bits of each section are not implemented
	uint8_t shiftValue	 = (8 * iprx_section) + (8 - PRI_BITS_IMPLEMENTED);
	*(NVIC_PRI_BASEADDR + iprx) |= (IRQPriority << shiftValue);

}


/* ------------------------------------------------------------------------------------------------------
 * Name			:	GPIO_IRQHandling
 * Description	:	To Process the interrupt, when occurred:
 *
 * Parameter 1	:	Pin Number
 * Return Type	:	none (void)
 * Note			:	In case of GPIO, IRQ Handling is simple, just Clear the pending bit
 *
 * ------------------------------------------------------------------------------------------------------ */
void GPIO_IRQHandling(uint8_t PinNumber)
{
	// Clear the EXTI PR Register corresponds to the pin number
	if (EXTI->PR & (1 << PinNumber))	// if PR is set means interrupt is pended
	{
		// Clear it by writing 1
		EXTI->PR |= (1 << PinNumber);
	}
}
