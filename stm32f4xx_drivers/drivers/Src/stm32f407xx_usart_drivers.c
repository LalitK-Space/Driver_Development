/*
 * 									stm32f407xx_usart_drivers.c
 *
 *  This file contains USART driver API implementations.
 *
 */

#include "stm32f407xx_usart_drivers.h"





/* -- > Peripheral Clock Setup  < -- */
/* ------------------------------------------------------------------------------------------------------
 * Name		:  	USART_PeriClockControl
 * Description	:	Peripheral Clock Setup API:
 			This function Enables or Disables peripheral clock for the given UART peripheral
 * Parameter 1	:	Base address of the UART peripheral
 * Parameter 2	:	ENABLE or DISABLE Macro
 * Return Type	:	none (void)
 * Note		:
 * ------------------------------------------------------------------------------------------------------ */
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (pUSARTx == USART1)
		{
			USART1_PCLK_EN();
		}
		else if (pUSARTx == USART2)
		{
			USART2_PCLK_EN();
		}
		else if (pUSARTx == USART3)
		{
			USART3_PCLK_EN();
		}
		else if (pUSARTx == UART4)
		{
			UART4_PCLK_EN();
		}
		else if (pUSARTx == UART5)
		{
			UART5_PCLK_EN();
		}
		else if (pUSARTx == USART6)
		{
			USART6_PCLK_EN();
		}
		else
		{
			// Meh
		}
	}
	else if (EnorDi == DISABLE)
		{
			if (pUSARTx == USART1)
			{
				USART1_PCLK_DI();
			}
			else if (pUSARTx == USART2)
			{
				USART2_PCLK_DI();
			}
			else if (pUSARTx == USART3)
			{
				USART3_PCLK_DI();
			}
			else if (pUSARTx == UART4)
			{
				UART4_PCLK_DI();
			}
			else if (pUSARTx == UART5)
			{
				UART5_PCLK_DI();
			}
			else if (pUSARTx == USART6)
			{
				USART6_PCLK_DI();
			}
			else
			{
				// Meh
			}
		}

}


/* -- > Peripheral Initialize and De-initialize  < -- */
/* ------------------------------------------------------------------------------------------------------
 * Name		:	USART_Init
 * Description	:	Peripheral Initialize API:
 *			To initialize the given USART peripheral.
 * Parameter 1	:	Handle pointer variable
 * Return Type	:	none (void)
 * Note		:
 * ------------------------------------------------------------------------------------------------------ */
void USART_Init(USART_Handle_t *pUSARTHandle)
{

}


/* ------------------------------------------------------------------------------------------------------
 * Name		:	USART_DeInit
 * Description	:	USART Peripheral De-Initialize API:
 *			reset all the registers of USART peripheral mentioned
 * Parameter 1	:	Base address of the USART peripheral
 * Return Type	:	none (void)
 * Note		:
 * ------------------------------------------------------------------------------------------------------ */
void USART_DeInit(USART_RegDef_t *pUSARTx)
{
	// Make respective bit 1 to reset then again make it 0, if kept 1 then Peripheral will always be in reset state
	// SET and RESET done in MACROS
		if (pUSARTx == USART1)
		{
			USART1_REG_RESET();
		}
		else if (pUSARTx == USART2)
		{
			USART2_REG_RESET();
		}
		else if (pUSARTx == USART3)
		{
			USART3_REG_RESET();
		}
		else if (pUSARTx == UART4)
		{
			UART4_REG_RESET();
		}
		else if (pUSARTx == UART5)
		{
			UART5_REG_RESET();
		}
		else if (pUSARTx == USART6)
		{
			USART6_REG_RESET();
		}
		else
		{
			// Meh
		}
}


/* -- > SPI Send and Receive Data < -- */

/* ------------------------------------------------------------------------------------------------------
 * Name		:	USART_SendData
 * Description	:	USART Peripheral Send Data API:
 *
 * Parameter 1	:	Base address of the USART peripheral
 * Parameter 2 	:	Pointer to data
 * Parameter 3	:   	Length of the Data to send
 * Return Type	:	none (void)
 * Note		:	Blocking API (Polling), function call will wait until all the bytes are transmitted.
 * ------------------------------------------------------------------------------------------------------ */
void USART_SendData(USART_RegDef_t *pUSARTx, uint8_t *pTxBuffer, uint32_t LenOfData)
{

}


/* ------------------------------------------------------------------------------------------------------
 * Name		:	USART_ReceiveData
 * Description	:	USART Peripheral Receive Data API:
 *
 * Parameter 1	:	Base address of the USART peripheral
 * Parameter 2 	:	Pointer to Rx buffer
 * Parameter 3	:   	Length of the Data to receive
 * Return Type	:	none (void)
 * Note		:	Blocking API (Polling), function call will wait until all the bytes are received.
 * ------------------------------------------------------------------------------------------------------ */
void USART_ReceiveData(USART_RegDef_t *pUSARTx, uint8_t *pRxBuffer, uint32_t LenOfData)
{

}


/* ------------------------------------------------------------------------------------------------------
 * Name		:	USART_SendData_IT
 * Description	:	USART Peripheral Interrupt Based Send Data API:
 *
 * Parameter 1	:	Handle pointer variable
 * Parameter 2 	:	Pointer to data
 * Parameter 3	:   	Length of the Data to send
 * Return Type	:	uint8_t (State)
 * Note		:
 * ------------------------------------------------------------------------------------------------------ */
uint8_t USART_SendData_IT(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t LenOfData)
{

}


/* ------------------------------------------------------------------------------------------------------
 * Name		:	USART_ReceiveData_IT
 * Description	:	USART Peripheral Interrupt Based Receive Data API:
 *
 * Parameter 1	:	Handle pointer variable
 * Parameter 2 	:	Pointer to Rx buffer
 * Parameter 3	:   	Length of the Data to receive
 * Return Type	:	uint8_t (State)
 * Note		:
 * ------------------------------------------------------------------------------------------------------ */
uint8_t USART_ReceiveData_IT(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t LenOfData)
{

}


/* -- > IRQ Configuration and ISR Handling < -- */
/* ------------------------------------------------------------------------------------------------------
 * Name		:	USART_IRQInterruptConfig
 * Description	:	To configure IRQ:
 *			Processor specific configurations (NVIC Registers)
 * Parameter 1	:	IRQ number
 * Parameter 2	:	Enable or Disable the IRQ (ENABLE or DISABLE Macro)
 * Return Type	:	none (void)
 * Note		:
 * ------------------------------------------------------------------------------------------------------ */
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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
 * Name		:	USART_IRQPriorityConfig
 * Description	:	To configure the priority of the interrupt:
 *
 * Parameter 1	:	IRQ Number
 * Parameter 2	:	IRQ Priority
 * Return Type	:	none (void)
 * Note		:
 * ------------------------------------------------------------------------------------------------------ */
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
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
 * Name		:	USART_IRQHandling
 * Description	:	To Process the interrupt, when occurred:
 *
 * Parameter 1	:	Handle pointer variable
 * Return Type	:	none (void)
 * Note		:
 * ------------------------------------------------------------------------------------------------------ */
void USART_IRQHandling(USART_Handle_t *pUSARTHandle)
{

}


/* ------------------------------------------------------------------------------------------------------
 * Name		:	USART_getFlagStatus
 * Description	:	To get the Flags info from Status Register
 *
 * Parameter 1	:	Base address of the USART peripheral
 * Parameter 2  :   	Flag Name
 * Return Type	:	True or False (1 or 0)
 * Note		:
 * ------------------------------------------------------------------------------------------------------ */
uint8_t USART_getFlagStatus (USART_RegDef_t *pUSARTx, uint32_t FlagName)
{
	if (pUSARTx->SR & FlagName)  // if that Flag is set then execute
	{
		return FLAG_SET;

	}

	return FLAG_RESET;

}


/* ------------------------------------------------------------------------------------------------------
 * Name		:	USART_ClearFlagStatus
 * Description	:	To clear the Flags in Status Register
 * Parameter 1	:	Base address of the USART peripheral
 * Parameter 2  :   	Flag Name
 * Return Type	:	none (void)
 * Note		:
 * ------------------------------------------------------------------------------------------------------ */
void USART_ClearFlagStatus (USART_RegDef_t *pUSARTx, uint32_t FlagName)
{
	// Clear Flag in SR
	pUSARTx->SR &= ~(FlagName);
}


/* ------------------------------------------------------------------------------------------------------
 * Name		:	USART_PeripheralControl
 * Description	:	To Enable or Disable the USART Peripheral
 *
 * Parameter 1	:	Base address of the USART peripheral
 * Parameter 2  :   	Enable or Disable Macro
 * Return Type	:	none
 * Note		:	USART Peripherals are disabled by default
 *
 * ------------------------------------------------------------------------------------------------------ */
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		pUSARTx->CR1 |= (1 << USART_CR1_UE);	// UE : USART ENABLE
	}
	else if (EnorDi == DISABLE)
	{
		pI2Cx->CR1 &= ~(1 << USART_CR1_UE);
	}
	else
	{
		// Meh
	}

}


/* ------------------------------------------------------------------------------------------------------
 * Name		:	USART_ApplicationEventCallback
 * Description	:	Callback implementation
 *
 * Parameter 1	:	USART Handle Pointer
 * Parameter 2	:	Application Event Macro (Possible USART Application Events)
 * Return Type	:	none (void)
 * Note		:	This function will be implemented in the application. If not, to clear warnings or errors
 * 			weak implementation is done here. If application does not implement this function
 * 			then this implementation will be called. __attribute__((weak))
 * ------------------------------------------------------------------------------------------------------ */
__attribute__((weak))void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, uint8_t ApplicationEvent)
{
	// May or may not be implemented in the application file as per the requirements
}

