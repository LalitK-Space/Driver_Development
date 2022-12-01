/*
 * 									stm32f407xx_gpio_drivers.c
 *
 *  This file contains I2C driver API implementations.
 *
 */

#include"stm32f407xx_i2c_drivers.h"


/* -- Helper Functions prototypes  -- */

// To get the value if Pclk1
uint32_t RCC_Pclk1_Value(void);

// To generate START condition
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);

// To generate STOP condition
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);

// To Execute Address Phase : for Writing (Master Tx -> Slave Rx)
static void I2C_ExecuteAddressPhase_Write(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddress);

// To Execute Address Phase : for Read (Master Rx <- Slave Tx)
static void I2C_ExecuteAddressPhase_Read(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddress);

// To clear ADDR Flag
static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx);

// array to store AHB prescaler values
uint16_t ahb_prescaler[8] = {2,4,8,16,64,128,256,512};

// array to store APB1 prescaler values
uint8_t apb1_prescaler[4] = {2,4,8,16};

/* -- > Peripheral Clock Setup  < -- */
/* ------------------------------------------------------------------------------------------------------
 * Name		:  	I2C_PeriClockControl
 * Description	:	Peripheral Clock Setup API:
 			This function Enables or Disables peripheral clock for the given I2C peripheral
 * Parameter 1	:	Base address of the I2C peripheral
 * Parameter 2	:	ENABLE or DISABLE Macro
 * Return Type	:	none (void)
 * Note		:
 * ------------------------------------------------------------------------------------------------------ */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		}
		else if (pI2Cx == I2C2)
		{
			I2C2_PCLK_EN();
		}
		else if (pI2Cx == I2C3)
		{
			I2C3_PCLK_EN();
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
			if (pI2Cx == I2C1)
			{
				I2C1_PCLK_DI();
			}
			else if (pI2Cx == I2C2)
			{
				I2C2_PCLK_DI();
			}
			else if (pI2Cx == I2C3)
			{
				I2C3_PCLK_DI();
			}
			else
			{
				// Meh
			}
		}

	}


}


/* -- > Peripheral Initialize and De-initialize  < -- */
/* ------------------------------------------------------------------------------------------------------
 * Name		:	I2C_Init
 * Description	:	Peripheral Initialize API:
 *			To initialize the given I2C peripheral.
 * Parameter 1	:	Pointer to I2C Handle
 * Return Type	:	none (void)
 * Note		:	Jobs:
 * 			1. Configure the Mode (Standard or Fast)
 * 			2. Configure the speed of the serial clock (SCL)
 * 			3. Configure the device address (only when device is Slave)
 * 			4. Enable ACking
 * 			5. Configure the rise time for I2C pins (TRISE)
 *			For FREQ configuration, it is known that HSI is 16 MHz, but still it is required to calculate
 *
 *			Also, Peripheral Clock is enabled at starting of the function, so users need not do it explicitly.
 * ------------------------------------------------------------------------------------------------------ */
void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	/* - Enable Peripheral Clock - */
	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

	uint32_t tempReg = 0;

	/* - Enabling ACKing (CR1 Register) - */

	// Store the value from ACK config variable (store at 10th bit position)
	tempReg |= (pI2CHandle->I2C_Config.I2C_ACK_Control << 10);
	// Configure CR1 Register
	pI2CHandle->pI2Cx->CR1 = tempReg;

	/* - Configure the FREQ fields (CR2 Register) - */

	tempReg = 0;
	// Get Pclk value [Returns 16HMz (STM32F407)]
	tempReg |= RCC_Pclk1_Value() / 1000000U;		// divide by 1000000 to get value 16

	// Configure CR2 with FREQ value
	pI2CHandle->pI2Cx->CR2 = (tempReg & 0x3F);		// Masking: Only need first 6 bits (FREQ[5:0])

	/* - Configure the Slave Address - */

	// Own Address Register (OAR1 Register) [using 7 bit address format in this driver development]
	// Bit 0	  : for 7 bit address -> DON'T CARE
	// Bits[7:1] 	  : 7 bit address
	// Bits[9:8]      : for 7 bit address -> DON'T CARE
	// Bits[13:10]    : RESERVED (MUST BE KEPT at reset value)
	// Bit 14	  : SHOULD always be at 1 by software
	// Bit 15	  : ADDMODE : 0 for 7 bit address (10 bit address not acknowledged)

	// Configure device address (own address)
	tempReg |= (pI2CHandle->I2C_Config.I2C_Device_Address << 1);	// shifted by 1 because bits are [7:1]

	// Configure OAR Register
	// Bit 15: ADDMODE: already 0 (reset value) so, 7 bit address mode is selected
	// Set 14th Bit as required
	tempReg |= (1 << 14);
	pI2CHandle->pI2Cx->OAR1 = tempReg;

	/* - Configure the Serial Clock Speed - */

	// Configure the CCR fields (CCR Register)
	// Bits[11:0]	: CCR field

	// CCR calculations
	uint16_t ccr_value = 0;
	tempReg = 0;

	if (pI2CHandle->I2C_Config.I2C_SCL_Speed <= I2C_SCL_SPEED_SM)
	{
		//STEP a:  Mode is Standard Mode (Configure the 15th bit in CCR register)
		// Bit 15: 0 for Standard Mode (by default (reset value))

		// STEP b: Calculate value of CCR for Standard Mode frequency
		/* Formula to calculate CCR
		 *	T(high scl) = CCR * T(pclk)
		 *	T(low scl) = CCR * T(pclk)
		 *   Assuming, T(high) = T(low) of SCL
		 *   => T(scl) = 2 * CCR * T(pclk1)
		 *   => CCR = T(scl) / 2 * T(pclk1)
		 *
		 *   In terms of frequency
		 *   => CCR = f(pclk1) / 2 * f(scl)
		 */

		// CCR = Pclk1 / I2C_SCL_Speed
		ccr_value = (RCC_Pclk1_Value() / (2 * pI2CHandle->I2C_Config.I2C_SCL_Speed));

		// Save CCR value in tempReg register and Mask out unnecessary bits (CCR Bits[11:0])
		tempReg |= (ccr_value & 0xFFF);
	}
	else
	{
		// STEP a:  Mode is Fast Mode
		// Set Bit 15: 1 for Fast Mode
		tempReg |= (1 << 15);

		// STEP b: Configure Duty Cycle
		// Bit 14 (user configured)
		tempReg |= (pI2CHandle->I2C_Config.I2C_FM_DutyCycle << 14);

		// STEP c: Calculate value of CCR for Fast Mode
		/* Formula to calculate CCR in FM
		 *
		 * if DUTY (I2C_FM_DutyCycle) = 0 then, T(low) = 2 * T(high)
		 *
		 * 	T(high) = CCR * T(pclk1)
		 * 	T(low)  = 2 * CCR * T(pclk1)
		 *
		 * 	CCR = f(pclk1) / (3 * f(scl))
		 *
		 * if DUTY (I2C_FM_DutyCycle) = 1 then, T(low) = ~ 1.7 * T(high)   [to reach 400kHz]
		 *
		 *  	T(high) = 9 * CCR * T(pclk1)
		 *	T(low)  = 16 * CCR * T(pclk1)
		 *
		 *	CCR = f(pclk1) / (25 * f(scl))
		 * */

		// Check for DUTY CYCLE (user configured)
		if(pI2CHandle->I2C_Config.I2C_FM_DutyCycle == I2C_FM_DutyCycle_2)
		{
			// CCR = f(pclk1) / (3 * f(scl))
			ccr_value = (RCC_Pclk1_Value() / (3 * pI2CHandle->I2C_Config.I2C_SCL_Speed));

		}
		else if (pI2CHandle->I2C_Config.I2C_FM_DutyCycle == I2C_FM_DutyCycle_16_9)
		{
			// CCR = f(pclk1) / (25 * f(scl))
			ccr_value = (RCC_Pclk1_Value() / (25 * pI2CHandle->I2C_Config.I2C_SCL_Speed));

		}
		else
		{
			// Meh
		}

		// Save CCR value in tempReg register and Mask out unnecessary bits (CCR Bits[11:0])
		tempReg |= (ccr_value & 0xFFF);

	}


	// Configure the CCR Register with value in tempReg
	pI2CHandle->pI2Cx->CCR = tempReg;

	/* - Configure the rise time for I2C pins - */

	// Configure the TRISE Register
	// TRISE[5:0] Maximum rise time in Fast Mode or Standard Mode
	// Get values from I2C specification
	/* Configure with value => (Max_SCL_rise_time / T(pclk1) + 1 [Reference Manual]
	 *			=> [Trise(max) / T(pclk1)] + 1
	 *			=> [Trise(max) * f(pclk1)] + 1
	 *
	 */

	// Check if Mode is FM or SM
	if (pI2CHandle->I2C_Config.I2C_SCL_Speed <= I2C_SCL_SPEED_SM)
	{
		// Mode: SM

		// [Trise(max) * f(pclk1)] + 1
		// Trise (max) for standard mode is 1000ns (I2C specification)
		tempReg = (RCC_Pclk1_Value() / 1000000U) + 1;

	}
	else
	{
		// Mode: FM

		// [Trise(max) * f(pclk1)] + 1
		// Trise (max) for fast mode is 300ns (I2C specification)
		tempReg = (((RCC_Pclk1_Value() * 300 ) / 1000000000U) + 1);

	}

	// Configure the TRISE Register with value in tempReg
	pI2CHandle->pI2Cx->TRISE = tempReg & 0x3F;		// TRISE[5:0] Mask others

}


/* ------------------------------------------------------------------------------------------------------
 * Name		:	I2C_DeInit
 * Description	:	I2C Peripheral De-Initialize API:
 *			reset all the registers of I2C peripheral mentioned
 * Parameter 1	:	Base address of the I2C peripheral
 * Return Type	:	none (void)
 * Note		:
 * ------------------------------------------------------------------------------------------------------ */
void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{
	// For Resetting I2C, refer to RCC -> APB1RSTR
	// Make respective bit 1 to reset then again make it 0, if kept 1 then Peripheral will always be in reset state
	// SET and RESET done in MACROS
	if (pI2Cx == I2C1)
			{
				I2C1_REG_RESET();
			}
			else if (pI2Cx == I2C2)
			{
				I2C2_REG_RESET();
			}
			else if (pI2Cx == I2C3)
			{
				I2C3_REG_RESET();
			}
			else
			{
				// Meh
			}

}


/* -- > SPI Send and Receive Data < -- */

/* ------------------------------------------------------------------------------------------------------
 * Name		:	I2C_MasterSendData
 * Description	:	I2C Peripheral Send Data API:
 *			Transmit data present in TX Buffer
 * Parameter 1	:	Base address of the I2C peripheral
 * Parameter 2 	:	Pointer to data
 * Parameter 3	:   	Length of the Data to send
 * Parameter 4	: 	Slave Address
 * Return Type	:	none (void)
 * Note		:	Blocking API (Polling), function call will wait until all the bytes are transmitted.
 * ------------------------------------------------------------------------------------------------------ */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t LenOfData, uint8_t SlaveAddress)
{
	/* - Step 1: Generate the START condition - */
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	/* - Step 2: Confirm generation of START condition - */
	// By checking SB Flag in SR1
	// Until SB is cleared, SCL will be stretched (SCL will be pulled to LOW)
	while (!(I2C_getFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB)));
	// Clearing is done (by reading)

	/* - Step 3: Send the address of the Slave with R/~W bit as 0 - */
	// Total 8 bits (7 bit Slave address + 1 R/~W bit)
	I2C_ExecuteAddressPhase_Write(pI2CHandle->pI2Cx, SlaveAddress);


	/* - Step 4: Confirm completetion of Address Phase - */
	// By checking the ADDR Flag in SR1
	while (!(I2C_getFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR)));


	/* - Step 5: Clear the ADDR Flag (according to its software sequence) - */
	// Until ADDR is cleared, SCL will be stretched (SCL will be pulled to LOW)
	// Clearing: Cleared by software by reading SR1 Register followed reading SR2 or by hardware when PE = 0
	I2C_ClearADDRFlag(pI2CHandle->pI2Cx);
	// Data reception begins: AFTER clearing ADDR Flag


	/* - Step 6: Send the data until LenOfData becomes 0 - */
	// Before sending, confirm DR is empty or not (TXE Flag)
	while (LenOfData > 0)
	{
		// Wait till TxE is Set
		while (!(I2C_getFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE)));

		// Send data (Copy to DR)
		pI2CHandle->pI2Cx->DR = *pTxBuffer;

		// Increment Tx Buffer to point at next memory
		pTxBuffer++;

		// Decrement Lenth of Data
		LenOfData--;

	}


	/* - Step 7: When LenOfData becomes 0, wait for TxE = 1 and BTF = 1 before generating STOP condition - */
	// TxE = 1 and BTF = 1 means, both Shoft register and DR are empty

	// Wait till TxE is Set
	while (!(I2C_getFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE)));

	// Wait till BTF is Set
	while (!(I2C_getFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF)));


	/* - Step 8: Generate the STOP condition - */
	// Master does not have to wait for the STOP condition completetion
	// Generating STOP condition, clears BTF
	I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

}


/* ------------------------------------------------------------------------------------------------------
 * Name		:	I2C_MasterReceiveData
 * Description	:	I2C Peripheral receive Data API:
 *			read data present in Shift register
 * Parameter 1	:	Base address of the I2C peripheral
 * Parameter 2 	:	Pointer to Rx buffer
 * Parameter 3	:   	Length of the Data to send
 * Parameter 4	: 	Slave Address
 * Return Type	:	none (void)
 * Note		:	Blocking API (Polling), function call will wait until all the bytes are received.
 * ------------------------------------------------------------------------------------------------------ */
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t LenOfData, uint8_t SlaveAddress)
{
	/* - Step 1: Generate the START condition - */
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	/* - Step 2: Confirm generation of START condition - */
	// By checking SB Flag in SR1
	// Until SB is cleared, SCL will be stretched (SCL will be pulled to LOW)
	while (!(I2C_getFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB)));
	// Clearing is done (by reading)

	/* - Step 3: Send the address of the Slave with R/~W bit as 1 - */
	// Total 8 bits (7 bit Slave address + 1 R/~W bit)
	I2C_ExecuteAddressPhase_Read(pI2CHandle->pI2Cx, SlaveAddress);

	/* - Step 4: Confirm completetion of Address Phase - */
	// By checking the ADDR Flag in SR1
	while (!(I2C_getFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR)));


	/* - Step 5: CHECK for length of data from the slave and follow procedures - */

	/* - PROCEDURE TO READ ONLY 1 BYTE FROM SLAVE - */
	if (LenOfData == 1)
	{
		// a. Set ACK bit to 0 [DISABLE ACKing (in CR)]
		I2C_ManageACK(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

		// b. Set STOP bit to 1 [STOP condition (in CR)]
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		// c. Clear ADDR flag [ADDR = 0]
		// Clearing: Cleared by software by reading SR1 Register followed reading SR2 or by hardware when PE = 0
		I2C_ClearADDRFlag(pI2CHandle->pI2Cx);
		// Data reception begins: AFTER clearing ADDR Flag


		// d. Wait until RxNE becomes 1
		while (!(I2C_getFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE)));

		// e. Read the data in Rx Buffer (Read DR)
		*pRxBuffer = pI2CHandle->pI2Cx->DR;

		return;

	}

	/* - PROCEDURE TO READ MORE THAN 1 BYTE FROM SLAVE - */
	if (LenOfData > 1)
	{
		// a. Clear ADDR flag [ADDR = 0]
		// Clearing: Cleared by software by reading SR1 Register followed reading SR2 or by hardware when PE = 0
		I2C_ClearADDRFlag(pI2CHandle->pI2Cx);
		// Data reception begins: AFTER clearing ADDR Flag


		// b. Read data until LenOfData becomes 0
		for (uint32_t i = LenOfData; i > 0; i--)
		{
			// c. Wait until RxNE becomes 1
			while (!(I2C_getFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE)));

			// d. Check: if only last 2 bytes are remaining
			if (i == 2)
			{
				// Set ACK bit to 0 [DISABLE ACKing (in CR)]
				I2C_ManageACK(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

				//  Set STOP bit to 1 [STOP condition (in CR)]
				I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

			}

			// e. Read the data in Rx Buffer (Read DR)
			*pRxBuffer = pI2CHandle->pI2Cx->DR;

			// f. Increment the buffer address
			pRxBuffer++;

		}

	}

	/* - Step 6: Set ACK bit back to 1 : Enable ACKing - */
	if(pI2CHandle->I2C_Config.I2C_ACK_Control == I2C_ACK_ENABLE)
	{
		I2C_ManageACK(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);
	}

}

/* -- > IRQ Configuration and ISR Handling < -- */
/* ------------------------------------------------------------------------------------------------------
 * Name		:	I2C_IRQInterruptConfig
 * Description	:	To configure IRQ:
 *			Processor specific configurations (NVIC Registers)
 * Parameter 1	:	IRQ number
 * Parameter 2	:	Enable or Disable the IRQ (ENABLE or DISABLE Macro)
 * Return Type	:	none (void)
 * Note		:
 * ------------------------------------------------------------------------------------------------------ */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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
 * Name		:	I2C_IRQPriorityConfig
 * Description	:	To configure the priority of the interrupt:
 *
 * Parameter 1	:	IRQ Number
 * Parameter 2	:	IRQ Priority
 * Return Type	:	none (void)
 * Note		:
 * ------------------------------------------------------------------------------------------------------ */
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
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
 * Name		:	I2C_getFlagStatus
 * Description	:	To get the Flags info from Status Register
 *
 * Parameter 1	:	Pointer to I2C handle
 * Parameter 2  :   	Flag Name
 * Return Type	:	True or False (1 or 0)
 * Note		:
 * ------------------------------------------------------------------------------------------------------ */
uint8_t I2C_getFlagStatus (I2C_RegDef_t *pI2Cx, uint32_t FlagName)
{
	if (pI2Cx->SR1 & FlagName)  // if that Flag is set then execute
	{
		return FLAG_SET;

	}

	return FLAG_RESET;
}

/* ------------------------------------------------------------------------------------------------------
 * Name		:	I2C_PeripheralControl
 * Description	:	To Enable or Disable the I2C Peripheral
 *
 * Parameter 1	:	Pointer to I2C peripheral base address
 * Parameter 2  :   	Enable or Disable Macro
 * Return Type	:	none
 * Note		:	I2C Peripherals are disabled by default
 *
 * ------------------------------------------------------------------------------------------------------ */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	}
	else
	{
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
	}
}


/* ------------------------------------------------------------------------------------------------------
 * Name		:	I2C_ManageACK
 * Description	:	To Enable or Disable the I2C ACKing
 *
 * Parameter 1	:	Pointer to I2C peripheral base address
 * Parameter 2  :   	Enable or Disable Macro
 * Return Type	:	none
 * Note		:
 *
 * ------------------------------------------------------------------------------------------------------ */
void I2C_ManageACK(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if (EnorDi == I2C_ACK_ENABLE)
	{
		// Enable ACKing: In CR1 set 10th bit
		pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
	}
	else if (EnorDi == I2C_ACK_DISABLE)
	{
		// Disable ACKing: In CR1 clear 10th bit
		pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
	}
	else
	{
		// Meh
	}
}

/*------------------------------------ HELPER FUNCTIONS IMPLEMENTATIONS ----------------------------*/

/* ------------------------------------------------------------------------------------------------------
 * Name		:	RCC_Pclk1_Value
 * Description	:	To get the value of Pclk1
 * Parameters	:	none
 * Return Type	:	(uint32_t) clock frequency
 * Note		:	For STM32F407G-DISC1 board,
 *			(From Clock Tree)
 *				 [HSE or HSI(used) or PLLCLK or PLLR]
 *				 -> [AHB_PRESC] -> [APB1_PRESC] -> [APB1 peripheral clock]
 *
 *			Steps:
 *				1. Find source [HSE or HSI(used) or PLLCLK or PLLR]  (from RCC)
 *				2. Calculate AHB Prescaler
 *				3. Calculate APB1 Prescaler
 *				4. Then, derive clock frequency for APB1 peripheral (I2Cx is connected to APB1 Bus)
 *
 * ------------------------------------------------------------------------------------------------------ */
uint32_t RCC_Pclk1_Value(void)
{
	uint32_t pClk1;

	/* - Step 1: Find Source - */

	// In RCC Cloak Configuration Register (RCC_CFGR)
	// Check bits 2 and 3 (SWS bits: System Clock Switch status)
	/*
	 * 00:	HSI oscillator used as the system clock
	 * 01:	HSE oscillator used as the system clock
	 * 10:  PLL used as the system clock
	 * 11:  Not applicable
	 *
	 * */
	uint8_t sysClkSource;
	uint32_t sysClockFreq;
	// Read 3rd and 2nd bit from CFGR register
	// To read, right shift CFGR by 2 (this will bring bit 3 and 2 to 0th and 1st bit position and then, Mask them)
	sysClkSource = ((RCC->CFGR >> 2) & 0x03);

	// Now check for System Clock Source
	if (sysClkSource == 0)
	{
		// HSI oscillator used as the system clock (16HMz)
		sysClockFreq = 16000000;
	}
	else if (sysClkSource == 1)
	{
		// HSE oscillator used as the system clock (8HMz)
		sysClockFreq = 8000000;
	}
	else if (sysClkSource == 2)
	{
		// PLL used as the system clock
	}
	else
	{
		// Not Applicable
	}

	/* - Step 2: Calculate AHB Prescalar - */
	// In RCC Cloak Configuration Register (RCC_CFGR)
	// Check bits 4 to 7 (HPRE: AHB Prescaler)
	/*
	 * 0xxx: (if value is < 8) System Clock is not divided
	 * 1000: system clock divided by 2
	 * 1001: system clock divided by 4
	 * 1010: system clock divided by 8
	 * 1011: system clock divided by 16
	 * 1100: system clock divided by 64
	 * 1101: system clock divided by 128
	 * 1110: system clock divided by 256
	 * 1111: system clock divided by 512
	 *
	 * */

	uint8_t temp;
	uint8_t ahb_pres;

	// Read the value of HPRE and shift the value to right and then, Mask them
	temp = ((RCC->CFGR >> 4) & 0xF);

	if (temp < 8)
	{
		// System Clock is not divided, AHB Prescaler is 1
		ahb_pres = 1;
	}
	else
	{
		// AHB prescaler value depends upon temp variable
		ahb_pres = ahb_prescaler[temp - 8];		// if temp is 8 then, fetch 1st array value (0th element), and so on.

	}

	/* - Step 3: Calculate APB1 Prescalar - */
	// In RCC Cloak Configuration Register (RCC_CFGR)
	// Check bits 10 to 12 (PPRE1: APB1 Prescaler)

	/*
	 * 0xx: (if value < 4) AHB clock not divided
	 * 100: AHB clock divided by 2
	 * 101: AHB clock divided by 4
	 * 110: AHB clock divided by 8
	 * 111: AHB clock divided by 16
	 *
	 * */

	uint8_t apb1_pres;

	// Read the value of PPRE1 and shift the value to right and then, Mask them
	temp = ((RCC->CFGR >> 10) & 0x7);

	if (temp < 4)
	{
		// AHB clock is not divided, APB1 Prescaler is 1
		apb1_pres = 1;
	}
	else
	{
		// APB1 prescaler value depends upon temp variable
		apb1_pres = apb1_prescaler[temp - 4];		// if temp is 4 then, fetch 1st array value (0th element), and so on.

	}

	/* - Step 4: Derive Clock (Pclk1) - */
	pClk1 = (sysClockFreq / ahb_pres ) / apb1_pres;


	return pClk1;

}


/* ------------------------------------------------------------------------------------------------------
 * Name		:	I2C_GenerateStartCondition
 * Description	:	To generate START condition
 *
 * Parameter 1	:	Base address of the I2C peripheral
 * Return Type	:	none (void)
 * Note		:	Private helper function
 * 			To generate START condition
 * ------------------------------------------------------------------------------------------------------ */
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	// Step a: In register CR1 (bit 8: START)
	/*
	 * SET and CLEARED by the software and CLEARED by hardware when start is senr or PE = 0
	 *
	 * In MASTER Mode
	 * 0: No start generation
	 * 1: Repeated start generation
	 *
	 * In SLAVE Mode
	 * 0: No start generation
	 * 1: Start generation when bus is free
	 *
	 * */
	pI2Cx->CR1 |= (1 << I2C_CR1_START);	// (1 << 8)

}


/* ------------------------------------------------------------------------------------------------------
 * Name		:	I2C_ExecuteAddressPhase_Write
 * Description	:	To Execute Address Phase
 *
 * Parameter 1	:	Base address of the I2C peripheral
 * Parameter 2	:       Slave Address
 * Return Type	:	none (void)
 * Note		:	Private helper function
 *				[7 bits] : Address
 *				[1 bit]	 : R/~W as 0 (indicating Master Tx -> Slave Rx)
 * ------------------------------------------------------------------------------------------------------ */
static void I2C_ExecuteAddressPhase_Write(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddress)
{
	/* - Total 8 bits (7 bit Slave address + 1 R/~W bit - */

	// Make space for R/~W bit (Shift Slave address by 1)
	SlaveAddress = (SlaveAddress << 1);

	// clear the 0th bit (0 -> writing)
	SlaveAddress &= ~(1);

	// Put data (SlaveAddress) in Data Register
	pI2Cx->DR = SlaveAddress;

}


/* ------------------------------------------------------------------------------------------------------
 * Name		:	I2C_ExecuteAddressPhase_Read
 * Description	:	To Execute Address Phase
 *
 * Parameter 1	:	Base address of the I2C peripheral
 * Parameter 2	:       Slave Address
 * Return Type	:	none (void)
 * Note		:	Private helper function
 *				[7 bits] : Address
 *				[1 bit]	 : R/~W as 1 (indicating Master Rx <- Slave Tx)
 * ------------------------------------------------------------------------------------------------------ */
static void I2C_ExecuteAddressPhase_Read(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddress)
{
	/* - Total 8 bits (7 bit Slave address + 1 R/~W bit - */

	// Make space for R/~W bit (Shift Slave address by 1)
	SlaveAddress = (SlaveAddress << 1);

	// set the 0th bit (1 -> Reading)
	SlaveAddress |= 1;

	// Put data (SlaveAddress) in Data Register
	pI2Cx->DR = SlaveAddress;

}

/* ------------------------------------------------------------------------------------------------------
 * Name		:	I2C_ClearADDRFlag
 * Description	:	To clear ADDR Flag
 *
 * Parameter 1	:	Base address of the I2C peripheral
 * Return Type	:	none (void)
 * Note		:	Private helper function
 *
 * ------------------------------------------------------------------------------------------------------ */
static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx)
{
	// Clearing: Cleared by software by reading SR1 Register followed reading SR2 or by hardware when PE = 0
	// Simply read SR1 and SR2
	uint32_t dummyRead;
	dummyRead = pI2Cx->SR1;
	dummyRead = pI2Cx->SR2;

	(void) dummyRead;

}


/* ------------------------------------------------------------------------------------------------------
 * Name		:	I2C_GenerateStopCondition
 * Description	:	To generate STOP condition
 *
 * Parameter 1	:	Base address of the I2C peripheral
 * Return Type	:	none (void)
 * Note		:	Private helper function
 * 			To generate STOP condition
 * ------------------------------------------------------------------------------------------------------ */
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	// Step a: In register CR1 (bit 9: STP)
	/*
	 * SET and CLEARED by the software and CLEARED by hardware when STOP condition is detected,
	 * set by hardware when a timeout error is detected
	 *
	 * In MASTER Mode
	 * 0: No stop generation
	 * 1: Stop generation AFTER the current byte transfer or after the current start condition is sent
	 *
	 * In SLAVE Mode
	 * 0: No stop generation
	 * 1: Release the SCL and SDA line after current byte transfer
	 *
	 * */

	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);	// (1 << 9)

}


