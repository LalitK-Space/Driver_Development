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
	/* - Enable Peripheral Clock - */
	USART_PeriClockControl(pUSARTHandle->pUSARTx, ENABLE);

	uint32_t tempReg = 0;

	/* - Configuring CR1 - */

	// a. Enabling USART Tx and Rx Blocks according to USART_MODE configuration
	if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_RX_ONLY)
	{
		// Enable Rx Block (CR1: Bit[2] RE)
		tempReg |= (1 << USART_CR1_RE);
	}
	else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TX_ONLY)
	{
		// Enable Rx Block (CR1: Bit[3] TE)
		tempReg |= (1 << USART_CR1_TE);
	}
	else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TX_RX)
	{
		// Enable both Tx and Rx Block (CR1: Bit[2] RE and Bit[3] TE)
		tempReg |= (1 << USART_CR1_RE);
		tempReg |= (1 << USART_CR1_TE);
	}
	else
	{
		// Invalid USART MODE
	}

	// b. Configuring the Word Length (CR1: Bit[12] M)
	tempReg |= pUSARTHandle->USART_Config.USART_WordLength << USART_CR1_M;

	// c. Configuring Parity Control
	if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EVEN_EN)
	{
		// Enable Parity Control (CR1: Bit[10] PCE)
		tempReg |= (1 << USART_CR1_PCE);

		// Enable Even Parity (CR1: Bit[9] PS)
		// when PCE is Enabled, by default EVEN parity is selected (PS = 0 for EVEN PARITY)
	}
	else if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_ODD_EN)
	{
		// Enable Parity Control (CR1: Bit[10] PCE)
		tempReg |= (1 << USART_CR1_PCE);

		//Enable Odd Parity (CR1: Bit[9] PS)
		tempReg |= (1 << USART_CR1_PS);
	}
	else if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
	{
		// Disabled by default
	}
	else
	{
		// Invalid Parity Configuration
	}

	// d. Configure CR1 with tempReg bit values
	pUSARTHandle->pUSARTx->CR1 = tempReg;

	/* - Configuring CR2 - */

	// Reset tempReg Variable
	tempReg = 0;

	// a. Configure Number of Stop Bits (CR2: Bits[13:12] STOP)
	tempReg |= pUSARTHandle->USART_Config.USART_NoOfStopBits << USART_CR2_STOP;

	// b. Configure CR2 with tempReg bit values
	pUSARTHandle->pUSARTx->CR2 = tempReg;

	/* - Configuring CR3 - */

	// Reset tempReg Variable
	tempReg = 0;

	// a. Configuring Hardware Flow Control
	if (pUSARTHandle->USART_Config.USART_HW_FlowControl == USART_HW_FLOW_CONTROL_CTS)
	{
		// Enable CTS (CR3: Bit[9] CTSE)
		tempReg |= (1 << USART_CR3_CTSE);
	}
	else if (pUSARTHandle->USART_Config.USART_HW_FlowControl == USART_HW_FLOW_CONTROL_RTS)
	{
		// Enable RTS (CR3: Bit[8] RTSE)
		tempReg |= (1 << USART_CR3_RTSE);
	}
	else if (pUSARTHandle->USART_Config.USART_HW_FlowControl == USART_HW_FLOW_CONTROL_CTS_RTS)
	{
		// Enable both CTS ans RTS (CR3: Bit[9] CTSE and Bit[8] RTSE)
		tempReg |= (1 << USART_CR3_CTSE);
		tempReg |= (1 << USART_CR3_RTSE);
	}
	else if (pUSARTHandle->USART_Config.USART_HW_FlowControl == USART_HW_FLOW_CONTROL_NONE)
	{
		// Disabled by default
	}
	else
	{
		// Invalid Hardware Flow Control Configurations
	}

	// b. Configure CR3 with tempReg bit Values
	pUSARTHandle->pUSARTx->CR3 = tempReg;


	/* - Configuring BRR - */


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
 * Parameter 1	:	Handle pointer variable
 * Parameter 2 	:	Pointer to data
 * Parameter 3	:   	Length of the Data to send
 * Return Type	:	none (void)
 * Note		:	Blocking API (Polling), function call will wait until all the bytes are transmitted.
 * ------------------------------------------------------------------------------------------------------ */
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t LenOfData)
{
	uint16_t *pData;

	/* - Step 1: Send the data until 'LenOfData' bytes are transferred - */
	for (uint32_t i = 0; i < LenOfData; i++ )
	{
		// a. Wait until TxE Flag is SET
		while (!USART_getFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TXE));

		// b. Check for Word Length
		if (pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLENGTH_9_BITS)
		{
			// 9 Bit Word Length is selected
			/*
			 * Load Data Register with 2 Bytes
			 * Mask bits other than first 9 bits
			 *
			 * */
			pData = (uint16_t *) pTxBuffer;								// 2 bytes for DR
			pUSARTHandle->pUSARTx->DR = (*pData & (uint16_t)0x01FF);	// Keeping 1st 9 bits

			// Check for Parity Control
			if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				// Parity is Disable, User Data of 9 bits will be transferred
				// Increment pTxBuffer (2 times) to transmit user data (because 9 bits)
				pTxBuffer++;
				pTxBuffer++;
			}
			else
			{
				// Parity Bit is used in Application, User Data of 8 Bits will be transferred
				// 9th Bit (Parity Bit) will be replaced by the Hardware
				// So, Increment pTxBuffer (1 time) to point to next data address
				pTxBuffer++;
			}
		}
		else
		{
			// 8 Bit Word Length is selected
			// Load Data Register with 8 bits of data
			pUSARTHandle->pUSARTx->DR = (*pTxBuffer & (uint8_t)0xFF);		// Dereference to send

			// Increment pTxBuffer (1 time)
			pTxBuffer++;

		}

	}

	/* - Step 2: Wait until TC Flag is SET (Transfer Complete) - */
	while (!USART_getFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TC));

	// Additional setting can be done here [after TC is SET](Ex: Disable Tx Block to save power, etc)

}


/* ------------------------------------------------------------------------------------------------------
 * Name		:	USART_ReceiveData
 * Description	:	USART Peripheral Receive Data API:
 *
 * Parameter 1	:	Handle pointer variable
 * Parameter 2 	:	Pointer to Rx buffer
 * Parameter 3	:   	Length of the Data to receive
 * Return Type	:	none (void)
 * Note		:	Blocking API (Polling), function call will wait until all the bytes are received.
 * ------------------------------------------------------------------------------------------------------ */
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t LenOfData)
{
	/* - Step 1: Receive/read the data until 'LenOfData' bytes are received - */
	for (uint32_t i = 0; i < LenOfData; i++)
	{
		// a. Wait until RxNE Flag is SET
		while (!USART_getFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_RXNE));

		// b. Check for Word Length (Receiving 9 bits or 8 bits)
		if (pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLENGTH_9_BITS)
		{
			// Receiving: 9 Bits of Data

			// Check for Parity Control
			if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				// Parity is Disable, User Data of 9 bits will be Received
				// Read first 2 bytes and mask out others
				*((uint16_t *)pRxBuffer) = (pUSARTHandle->pUSARTx->DR & (uint16_t)0x1FF);


				// Increment pRxBuffer (2 times) to transmit user data
				pRxBuffer++;
				pRxBuffer++;
			}
			else
			{
				// Parity Bit is used in Application, User Data of 8 Bits will be received + 1 parity bit
				// Read 8 bits of data
				*pRxBuffer = (pUSARTHandle->pUSARTx->DR & (uint8_t)0xFF);

				// So, Increment pRxBuffer (1 time)
				pRxBuffer++;
			}
		}
		else
		{
			// Receiving 8 Bits of Data

			// Check for Parity Control
			if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				// Parity is Disable, User Data of 8 bits will be Received
				// Read 8 bits of data
				*pRxBuffer = (pUSARTHandle->pUSARTx->DR & (uint8_t)0xFF);

			}
			else
			{
				// Parity Bit is used in Application, User Data of 7 Bits will be received + 1 parity bit
				// Read 7 bits of data
				*pRxBuffer = (pUSARTHandle->pUSARTx->DR & (uint8_t)0x7F);

			}

			// Increment pRxBuffer (1 time)
			pRxBuffer++;

		}

	}

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
	// Get state of USART peripheral
	uint8_t state = pUSARTHandle->TxState;

	// Only when Peripheral is NOT busy
	if (state != USART_BUSY_IN_TX)
	{
		// a. Save the Tx buffer address and length information in a global variable
		pUSARTHandle->pTxBuffer = pTxBuffer;		// Saving Tx Buffer Address
		pUSARTHandle->TxDataLength = LenOfData;		// Saving Length Information

		// b. Mark the USART state as busy in transmission
		pUSARTHandle->TxState = USART_BUSY_IN_TX; 	// State

		// c. Enable interrupt for TxE (CR1)
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TXEIE);

		// d. Enable interrupt for TC (CR1)
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TCIE);
	}

	return state;

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
uint8_t USART_ReceiveData_IT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t LenOfData)
{
	// Get state of USART peripheral
	uint8_t state = pUSARTHandle->RxState;

	// Only when Peripheral is NOT busy
	if (state != USART_BUSY_IN_RX)
	{
		// a. Save the Tx buffer address and length information in a global variable
		pUSARTHandle->pRxBuffer = pRxBuffer;		// Saving Rx Buffer Address
		pUSARTHandle->RxDataLength = LenOfData;		// Saving Length Information

		// b. Mark the USART state as busy in reception
		pUSARTHandle->RxState = USART_BUSY_IN_RX; 	// State

		// c. Enable interrupt for RxNE (CR1)
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_RXNEIE);
	}

	return state;
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
		pUSARTx->CR1 &= ~(1 << USART_CR1_UE);
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

