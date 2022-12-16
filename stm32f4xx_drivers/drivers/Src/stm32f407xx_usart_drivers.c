/*
 * 									stm32f407xx_usart_drivers.c
 *
 *  This file contains USART driver API implementations.
 *
 */

#include "stm32f407xx_usart_drivers.h"


// To get the value of Pclk1 [APB1] (defined in stm32f407xx_rcc_driver.c)
uint32_t RCC_Pclk1_Value(void);

// To get the value of Pclk2 [APB2] (defined in stm32f407xx_rcc_driver.c)
uint32_t RCC_Pclk2_Value(void);


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
	USART_SetBaudRate(pUSARTHandle->pUSARTx, pUSARTHandle->USART_Config.USART_BaudRate);

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


/* -- > USART Send and Receive Data < -- */

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
			pData = (uint16_t *) pTxBuffer;					// 2 bytes for DR
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
	/* - Check why interrupt is triggered and handle accordingly - */

	// Temporary variables to hold the status flag
	uint32_t temp_a, temp_b, temp_c;

	uint16_t *pData;

	/* - a. Handle for interrupt generated by TC Flag  - */

	// Check for TC Bit (SR)
	temp_a = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_TC);

	// Check for TCIE (CR1)
	temp_b = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_TCIE);

	if (temp_a && temp_b)
	{
		// Interrupt is triggered because of TC

		// a. Close Transmission and call Application Callback, if TxDataLength is 0
		if ((pUSARTHandle->TxState == USART_BUSY_IN_TX))
		{
			// Check TxDataLength
			// Close transmission if TxDataLength is 0
			if (!pUSARTHandle->TxDataLength)
			{
				// a. Clear TC Flag
				pUSARTHandle->pUSARTx->SR &= ~(1 << USART_SR_TC);

				// b. Clear TCIE bit
				pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_TCIE);

				// c. Change Application State to READY
				pUSARTHandle->TxState = USART_READY;

				// d. Reset Buffer Address
				pUSARTHandle->pTxBuffer = NULL;

				// e. Reset TxDataLength
				pUSARTHandle->TxDataLength = 0;

				// f. Call Application Callback [with EVENT: Tx Complete]
				USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_TX_COMPLETE);
			}
		}
	}

	/* - b. Handle for interrupt generated by TXE Flag  - */

	// Check for TXE Bit (SR)
	temp_a = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_TXE);

	// Check for TXEIE Bit (CR1)
	temp_b = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_TXEIE);

	if (temp_a && temp_b)
	{
		// Interrupt is generated because of TXE Flag

		// Only if Application is BUSY IN TX
		if ((pUSARTHandle->TxState == USART_BUSY_IN_TX))
		{
			// Keep sending data until TxDataLength is 0
			if (pUSARTHandle->TxDataLength > 0)
			{
				// Check for Word Length
				if (pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLENGTH_9_BITS)
				{
					// 9 Bit Word Length is selected
					/*
					 * Load Data Register with 2 Bytes
					 * Mask bits other than first 9 bits
					 *
					 * */
					pData = (uint16_t *) pUSARTHandle->pTxBuffer;					// 2 bytes for DR
					pUSARTHandle->pUSARTx->DR = (*pData & (uint16_t)0x01FF);	// Keeping 1st 9 bits

					// Check for Parity Control
					if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						// Parity is Disable, User Data of 9 bits will be transferred
						// Increment pTxBuffer (2 times) to transmit user data (because 9 bits)
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->pTxBuffer++;

						// Decrement TxDataLength (2 times)
						pUSARTHandle->TxDataLength -= 2;

					}
					else
					{
						// Parity Bit is used in Application, User Data of 8 Bits will be transferred
						// 9th Bit (Parity Bit) will be replaced by the Hardware
						// So, Increment pTxBuffer (1 time) to point to next data address
						pUSARTHandle->pTxBuffer++;

						// Decrement TxDataLength (1 time)
						pUSARTHandle->TxDataLength -= 1;

					}
				}
				else
				{
					// 8 Bit Word Length is selected
					// Load Data Register with 8 bits of data
					pUSARTHandle->pUSARTx->DR = (*pUSARTHandle->pTxBuffer & (uint8_t)0xFF);		// Dereference to send

					// Increment pTxBuffer (1 time)
					pUSARTHandle->pTxBuffer++;

					// Decrement TxDataLength (1 time)
					pUSARTHandle->TxDataLength -= 1;

				}
			}

			// All Data is transmitted
			if (pUSARTHandle->TxDataLength == 0)
			{
				// TxDataLength is 0

				// Disable TXE Interrupt (Clear TXEIE Bit)
				pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_TXEIE);

			}

		}

	}

	/* - c. Handle for interrupt generated by RXNE Flag  - */

	// Check for RXE Bit (SR)
	temp_a = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_RXNE);

	// Check for RXNEIE Bit (CR1)
	temp_b = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_RXNEIE);

	if (temp_a && temp_b)
	{
		// This interrupted is generated because of RXNE Flag

		// Only if Application is BUSY IN RX
		if (pUSARTHandle->RxState == USART_BUSY_IN_RX)
		{
			// Keep receiving until RxDataLength is 0
			if (pUSARTHandle->RxDataLength > 0)
			{
				// Check for Word Length
				if (pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLENGTH_9_BITS)
						{
							// Receiving: 9 Bits of Data

							// Check for Parity Control
							if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
							{
								// Parity is Disable, User Data of 9 bits will be Received
								// Read first 2 bytes and mask out others
								*((uint16_t *)pUSARTHandle->pRxBuffer) = (pUSARTHandle->pUSARTx->DR & (uint16_t)0x1FF);


								// Increment pRxBuffer (2 times) to transmit user data
								pUSARTHandle->pRxBuffer++;
								pUSARTHandle->pRxBuffer++;

								// Decrement RxDataLength (2 times)
								pUSARTHandle->RxDataLength -= 2;

							}
							else
							{
								// Parity Bit is used in Application, User Data of 8 Bits will be received + 1 parity bit
								// Read 8 bits of data
								*pUSARTHandle->pRxBuffer = (pUSARTHandle->pUSARTx->DR & (uint8_t)0xFF);

								// So, Increment pRxBuffer (1 time)
								pUSARTHandle->pRxBuffer++;

								// Decrement RxDataLength (1 time)
								pUSARTHandle->RxDataLength -= 1;
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
								*pUSARTHandle->pRxBuffer = (pUSARTHandle->pUSARTx->DR & (uint8_t)0xFF);

							}
							else
							{
								// Parity Bit is used in Application, User Data of 7 Bits will be received + 1 parity bit
								// Read 7 bits of data
								*pUSARTHandle->pRxBuffer = (pUSARTHandle->pUSARTx->DR & (uint8_t)0x7F);

							}

							// Increment pRxBuffer (1 time)
							pUSARTHandle->pRxBuffer++;

							// Decrement RxDataLenth (1 time)
							pUSARTHandle->RxDataLength -= 1;

						}

			}
			// All Data is received
			if (!pUSARTHandle->RxDataLength)
			{
				// RxDataLength is 0

				// Disable RXNE Interrupt (Clear RXEIE Bit)
				pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_RXNEIE);

				// Change Application State to READY
				pUSARTHandle->RxState = USART_READY;

				// Call Application Callback (Event: RX Complete)
				USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_RX_COMPLETE);

			}
		}
	}

	/* - d. Handle for interrupt generated by CTS Flag  - */

	/*
	 * CTS : NOT applicable in UART4 and UART5
	 * */

	// Check for CTS Bit (SR)
	temp_a = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_CTS);

	// Check for CTSE Bit (CR3)
	temp_b = pUSARTHandle->pUSARTx->CR3 & (1 << USART_CR3_CTSE);

	// Check for CTSEIE Bit (CR3)
	temp_c = pUSARTHandle->pUSARTx->CR3 & (1 << USART_CR3_CTSIE);

	if (temp_a && temp_c)
	{
		// Interrupt is generated because of CTS Flag

		// a. Clear the CTS Flag (SR)
		pUSARTHandle->pUSARTx->SR &= ~(1 << USART_SR_CTS);

		// b. Call Application Callback (Event: CTS)
		USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_RX_CTS);

	}

	/* - e. Handle for interrupt generated by IDLE Flag  - */

	// Check for IDLE Bit (SR)
	temp_a = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_IDLE);

	// Check for IDLEIE Bit (CR1)
	temp_b = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_IDLEIE);

	if (temp_a && temp_b)
	{
		// Interrupt is generated by IDLE Flag

		// a. Clear the IDLE Flag (SR)
		pUSARTHandle->pUSARTx->SR &= ~(1 << USART_SR_IDLE);

		// b. Call Application Callback (Event: IDLE)
		USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_RX_IDLE);

	}

	/* - f. Handle for interrupt generated by ORE Flag  - */

	// Check for ORE Bit (SR)
	temp_a = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_ORE);

	// Check for RXNEIE bit (CR1)
	temp_b = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_RXNEIE);

	if (temp_a && temp_b)
	{
		// Interrupt is generated by ORE Flag

		// a. Clear ORE Flag (In Application (according to requirements))

		// b. Call Application Callback (ERROR: ORE)
		USART_ApplicationEventCallback(pUSARTHandle, USART_ERROR_ORE);

	}

	/* - g. Handle for interrupt generated by other ERROR Flags  - */

	/*
	 * Noise Flag, Overrun error and Framing Error in Multibuffer Communication
	 *
	 * */

	// check for EIE Bit (CR3)
	temp_b = pUSARTHandle->pUSARTx->CR3 & (1 << USART_CR3_EIE);

	if (temp_b)
	{
		// Error Interrupts Enable

		// Check for Framing Error
		temp_a = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_FE);

		if (temp_a)
		{
			// Framing Error

			/*
				This bit is set by hardware when a de-synchronization, excessive noise or a break character
				is detected. It is cleared by reading to the USART_SR register
				followed by a read to the USART_DR register
			*/

			// Call Application Callback (ERROR: FE)
			USART_ApplicationEventCallback(pUSARTHandle, USART_ERROR_FE);
		}

		// Check for NF Error
		temp_a = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_NF);

		if (temp_a)
		{
			// Noise Detection Flag

			/*
				This bit is set by hardware when noise is detected on a received frame. It is cleared by reading
				to the USART_SR register followed by a read to the
				USART_DR register
			*/

			// Call Application Callback (ERROR: NF)
			USART_ApplicationEventCallback(pUSARTHandle, USART_ERROR_NF);

		}

		// Check for ORE Bit (SR)
		temp_a = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_ORE);

		if (temp_a)
		{
			// Overrun Error

			// Call Application Callback (ERROR: ORE)
			USART_ApplicationEventCallback(pUSARTHandle, USART_ERROR_ORE);

		}


	}



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
 * Name		:	USART_SetBaudRate
 * Description	:	To set the desired Baud Rate
 *
 * Parameter 1	:	Base address of the USART peripheral
 * Parameter 2  :   	Desired Baud Rate
 * Return Type	:	none
 * Note		: 	Baud Rate can be:
 * 					1200
					2400
					9600
					19200
					38400
					57600
					115200
					230400
					460800
					921600
					2000000
					3000000
 *
 * ------------------------------------------------------------------------------------------------------ */
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t Baudrate)
{
	// To hold APB Clock Frequency
	uint32_t Peri_Clkx;

	// USARTDIV value [Divide factor to generate different baud rates (minimum value = 1)]
	uint32_t usartDIV;

	// To hold Mantissa and Fraction values
	uint32_t mantissaValue;		// 12 Bits value
	uint32_t fractionValue;		// 4 Bits value

	uint32_t tempReg = 0;

	/*	- Calculations -
	 *
	 * 	BaudRate =  f(peripheralCLK) / (8 * USARTDIV)		if OVER8 = 1
     *
	 *	BaudRate = f(peripheralCLK) / (16 * USARTDIV)		if OVER8 = 0
	 *
	 * */

	/* - Step 1: Get Peripheral Clock Frequency - */
	if ((pUSARTx == USART1) || (pUSARTx ==  USART6))
	{
		// USART1 and USART6 are connected to APB2 Bus
		Peri_Clkx = RCC_Pclk2_Value();
	}
	else
	{
		// USART2, USART3, UART4, UART5 are connected to APB1
		Peri_Clkx = RCC_Pclk1_Value();
	}

	/* - Step 2: Check for OVER8 configuration - */

	if (pUSARTx->CR1 & (1 << USART_CR1_OVER8))
	{
		// OVER8 Bit is SET [Oversampling by 8]
		/*
		 *  Equation is multiplied by 100, to get a Whole Number (No need to deal with fractions)
		 * 	BaudRate =  [f(peripheralCLK) / (8 * USARTDIV)] x 100
		 *           =  [(25 * f(peripheralCLK)) / (2 * USARTDIV)]
		 * */

		usartDIV = ((25 * Peri_Clkx) / (2 * Baudrate));
	}
	else
	{
		// OVER8 Bit is Cleared [Oversampling by 16]
		/*
		 *  Equation is multiplied by 100, to get a Whole Number (No need to deal with fractions)
		 * 	BaudRate =  [f(peripheralCLK) / (16 * USARTDIV)] x 100
		 *           =  [(25 * f(peripheralCLK)) / (4 * USARTDIV)]
		 * */

		usartDIV = ((25 * Peri_Clkx) / (4 * Baudrate));
	}

	/* - Step 3: Calculate Mantissa and Fraction values from 'usartDIV' - */

	// Mantissa value
	mantissaValue = usartDIV / 100;		// Only need integer part (will be mantissa)

	// Store Mantissa value in proper bit fields in tempReg variable. USART_BRR DIV_Mantissa[15:4] 12 bits
	tempReg |= (mantissaValue << 4);

	// Fraction value
	fractionValue = (usartDIV - (mantissaValue * 100));		// Extracting fraction digits

	// Calculation: Fraction value
	if (pUSARTx->CR1 & (1 << USART_CR1_OVER8))
	{
		// OVER8 Bit is SET [Oversampling by 8]

		/*
		 * Multiply extracted Fraction value by 8
		 * +50 is roundoff factor (round off to nearest whole number)
		 * then, divide by 100 to get fraction value (rounded off to nearest whole number)
		 *
		 * */
		fractionValue = (((fractionValue * 8) + 50) / 100) & ((uint8_t) 0x07);
	}
	else
	{
		// OVER8 Bit is Cleared [Oversampling by 16]

		/*
		 * Multiply extracted Fraction value by 16
		 * +50 is roundoff factor (round off to nearest whole number)
		 * then, divide by 100 to get fraction value (rounded off to nearest whole number)
		 *
		 * */
		fractionValue = (((fractionValue * 16) + 50) / 100) & ((uint8_t) 0x0F);
	}

	// Store Fraction value in proper bit fields in tempReg variable. USART_BRR DIV_Fraction[3:0] 4 bits
	tempReg |= fractionValue;

	/* - Step 4: Configure the USART_BRR with Mantissa value and Fraction values (tempReg) - */
	pUSARTx->BRR = tempReg;

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

