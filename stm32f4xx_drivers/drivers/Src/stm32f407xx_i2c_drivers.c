/*
 * 									stm32f407xx_gpio_drivers.c
 *
 *  This file contains I2C driver API implementations.
 *
 */

#include"stm32f407xx_i2c_drivers.h"

// To get the value if Pclk1
uint32_t RCC_Pclk1_Value(void);

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
 * 					1. Configure the Mode (Standard or Fast)
 * 					2. Configure the speed of the serial clock (SCL)
 * 					3. Configure the device address (only when device is Slave)
 * 					4. Enable ACking
 * 					5. Configure the rise time for I2C pins (TRISE)
 *				For FREQ configuration, it is known that HSI is 16 MHz, but still it is required to calculate
 * ------------------------------------------------------------------------------------------------------ */
void I2C_Init(I2C_Handle_t *pI2CHandle)
{
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
	pI2CHandle->pI2Cx->CR2 = (tempReg & 0x3F);	// Masking: Only need first 6 bits (FREQ[5:0])

	/* - Configure the Slave Address - */

	// Own Address Register (OAR1 Register) [using 7 bit address format in this driver development]
	// Bit 0	  : for 7 bit address -> DON'T CARE
	// Bits[7:1]  : 7 bit address
	// Bits[9:8]  : for 7 bit address -> DON'T CARE
	// Bits[13:10]: RESERVED (MUST BE KEPT at reset value)
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
	// Bits[11:0]	:	CCR field

	// CCR calculations
	uint16_t ccr_value = 0;
	tempReg = 0;

	if (pI2CHandle->I2C_Config.I2C_SCL_Speed <= I2C_SCL_SPEED_SM)
	{
		//STEP a:  Mode is Standard Mode (Configure the 15th bit in CCR register)
		// Bit 15: 0 for Standard Mode (by default (reset value))

		// STEP b: Calculate value of CCR for Standard Mode frequency
		/* Formula to calculate CCR
		 *	 T(high scl) = CCR * T(pclk)
		 *	 T(low scl)	= CCR * T(pclk)
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
		 * if DUTY (I2C_FM_DutyCycle) = 1 then, T(low) = ~ 1.7 * T(high)	[to reach 400kHz]
		 *
		 *  T(high) = 9 * CCR * T(pclk1)
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
 * Parameter 1	:	Pointer to I2C handle
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
 * Name		:	RCC_Pclk1_Value
 * Description	:	To get the value of Pclk1
 * Parameters	:	none
 * Return Type	:	(uint32_t) clock frequency
 * Note		:	For STM32F407G-DISC1 board,
 *				(From Clock Tree)
 *					 	[HSE or HSI(used) or PLLCLK or PLLR]
 *					 			-> [AHB_PRESC] -> [APB1_PRESC] -> [APB1 peripheral clock]
 *
 *				Steps:
 *					1. Find source [HSE or HSI(used) or PLLCLK or PLLR]  (from RCC)
 *					2. Calculate AHB Prescaler
 *					3. Calculate APB1 Prescaler
 *					4. Then, derive clock frequency for APB1 peripheral (I2Cx is connected to APB1 Bus)
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






