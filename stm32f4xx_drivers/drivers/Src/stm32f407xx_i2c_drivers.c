/*
 * 									stm32f407xx_i2c_drivers.c
 *
 *  This file contains I2C driver API implementations.
 *
 */

#include"stm32f407xx_i2c_drivers.h"


/* -- Helper Functions prototypes  -- */

// To get the value of Pclk1 (defined in stm32f407xx_rcc_driver.c)
uint32_t RCC_Pclk1_Value(void);

// To Execute Address Phase : for Writing (Master Tx -> Slave Rx)
static void I2C_ExecuteAddressPhase_Write(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddress);

// To Execute Address Phase : for Read (Master Rx <- Slave Tx)
static void I2C_ExecuteAddressPhase_Read(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddress);

// To clear ADDR Flag
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle);



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
 * Parameter 1	:	Handle pointer variable
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

	tempReg = 0;
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
 * Parameter 1	:	Handle pointer variable
 * Parameter 2 	:	Pointer to data
 * Parameter 3	:   	Length of the Data to send
 * Parameter 4	: 	Slave Address
 * Parameter 5	:	RepeatedStart (MACRO I2C_REPEATED_START_EN/_DI), to enable or disable repeated start
 * Return Type	:	none (void)
 * Note		:	Blocking API (Polling), function call will wait until all the bytes are transmitted.
 * ------------------------------------------------------------------------------------------------------ */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t LenOfData, uint8_t SlaveAddress, uint8_t repeatedStart)
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
	I2C_ClearADDRFlag(pI2CHandle);
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
	if (repeatedStart == I2C_REPEATED_START_DI)
	{
		// Check for Repeated Start then generate STOP condition
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
	}

}


/* ------------------------------------------------------------------------------------------------------
 * Name		:	I2C_MasterReceiveData
 * Description	:	I2C Peripheral receive Data API:
 *			read data present in Shift register
 * Parameter 1	:	Handle pointer variable
 * Parameter 2 	:	Pointer to Rx buffer
 * Parameter 3	:   	Length of the Data to send
 * Parameter 4	: 	Slave Address
 * Parameter 5	:	RepeatedStart (MACRO I2C_REPEATED_START_EN/_DI), to enable or disable repeated start
 * Return Type	:	none (void)
 * Note		:	Blocking API (Polling), function call will wait until all the bytes are received.
 * ------------------------------------------------------------------------------------------------------ */
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t LenOfData, uint8_t SlaveAddress, uint8_t repeatedStart)
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


		// b. Clear ADDR flag [ADDR = 0]
		// Clearing: Cleared by software by reading SR1 Register followed reading SR2 or by hardware when PE = 0
		I2C_ClearADDRFlag(pI2CHandle);
		// Data reception begins: AFTER clearing ADDR Flag


		// c. Wait until RxNE becomes 1
		while (!(I2C_getFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE)));

		// d. Set STOP bit to 1 [STOP condition (in CR)]
		if (repeatedStart == I2C_REPEATED_START_DI)
		{
			// Check for Repeated Start then generate STOP condition
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		}

		// e. Read the data in Rx Buffer (Read DR)
		*pRxBuffer = pI2CHandle->pI2Cx->DR;

	}

	/* - PROCEDURE TO READ MORE THAN 1 BYTE FROM SLAVE - */
	if (LenOfData > 1)
	{
		// a. Clear ADDR flag [ADDR = 0]
		// Clearing: Cleared by software by reading SR1 Register followed reading SR2 or by hardware when PE = 0
		I2C_ClearADDRFlag(pI2CHandle);
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
				if (repeatedStart == I2C_REPEATED_START_DI)
				{
					// Check for Repeated Start then generate STOP condition
					I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
				}

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


/* ------------------------------------------------------------------------------------------------------
 * Name		:	I2C_MasterSendData_IT
 * Description	:	I2C Peripheral Interrupt Based Send Data API:
 *			Transmit data present in TX Buffer
 * Parameter 1	:	Handle pointer variable
 * Parameter 2 	:	Pointer to data
 * Parameter 3	:   	Length of the Data to send
 * Parameter 4	: 	Slave Address
 * Parameter 5	:	RepeatedStart (MACRO I2C_REPEATED_START_EN/_DI), to enable or disable repeated start
 * Return Type	:	uint8_t (State)
 * Note		:	Triggers the START conditiona and enables all the required CONTROL BITS
 * ------------------------------------------------------------------------------------------------------ */
uint8_t I2C_MasterSendData_IT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t LenOfData, uint8_t SlaveAddress, uint8_t repeatedStart)
{
	// Get state of I2C peripheral
	uint8_t state = pI2CHandle->TxRxState;

	// Only when Peripheral is NOT busy
	if( (state != I2C_BUSY_IN_TX) && (state != I2C_BUSY_IN_RX))
	{
		// a. Save the Tx buffer address and length information in a global variable
		pI2CHandle->pTxBuffer = pTxBuffer;		// Saving Tx Buffer Address
		pI2CHandle->TxDataLength = LenOfData;		// Saving Length Information

		// b. Mark the I2C state as busy in transmission
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX; 	// State

		// c. Save Device/Slave address
		pI2CHandle->DeviceAdddress = SlaveAddress; 	// Device/Slave Address

		// d. Save Repeated Start (Enable or Disable)
		pI2CHandle->RepeatedStart = repeatedStart; 	// Repeated Start

		// e. Generate the START condition [SB will be SET and Interrupt will be generated (SB EVENT)]
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		// f. Enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);

		// g. Enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);

		// h. Enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);

		// i. Data transmision will be handled by the ISR code

	}

	return state;


}


/* ------------------------------------------------------------------------------------------------------
 * Name		:	I2C_MasterReceiveData_IT
 * Description	:	I2C Peripheral Interrupt Based Receive Data API:
 *			read data present in Shift register
 * Parameter 1	:	Handle pointer variable
 * Parameter 2 	:	Pointer to Rx buffer
 * Parameter 3	:   	Length of the Data to send
 * Parameter 4	: 	Slave Address
 * Parameter 5	:	RepeatedStart (MACRO I2C_REPEATED_START_EN/_DI), to enable or disable repeated start
 * Return Type	:	uint8_t (State)
 * Note		:
 * ------------------------------------------------------------------------------------------------------ */
uint8_t I2C_MasterReceiveData_IT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t LenOfData, uint8_t SlaveAddress, uint8_t repeatedStart)
{
	// Get state of I2C peripheral
	uint8_t state = pI2CHandle->TxRxState;

	// Only when Peripheral is NOT busy
	if( (state != I2C_BUSY_IN_TX) && (state != I2C_BUSY_IN_RX))
	{
		// a. Save the Tx buffer address and length information in a global variable
		pI2CHandle->pRxBuffer = pRxBuffer;		// Saving Rx Buffer Address
		pI2CHandle->RxDataLength = LenOfData;		// Saving Length Information

		// b. Mark the I2C state as busy in reception
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX; 	// State

		// c. Save Device/Slave address
		pI2CHandle->DeviceAdddress = SlaveAddress; 	// Device/Slave Address

		// d. Rx Data Size for ISR Code to manage Data Reception
		pI2CHandle->RxSize = LenOfData;

		// e. Save Repeated Start (Enable or Disable)
		pI2CHandle->RepeatedStart = repeatedStart; 	// Repeated Start

		// f. Generate the START condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		// g. Enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);

		// i. Enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);

		// j. Enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);

		// k. Data reception will be handled by the ISR code

	}

	return state;

}


/* ------------------------------------------------------------------------------------------------------
 * Name		:	I2C_SlaveSendData
 * Description	:	Receive Data from master
 * Parameter 1	:	Base address of the I2C peripheral
 * Parameter 2 	:	uint8_t DataToMaster
 * Return Type	:	none (void)
 * Note		:	When a request is received from the master, the Slave will send data one byte at a time
 * 			(uint8_t Data)
 * ------------------------------------------------------------------------------------------------------ */
void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t DataToMaster)
{
	/* -Load Data in Data Register (DR) - */
	pI2Cx->DR = DataToMaster;
}


/* ------------------------------------------------------------------------------------------------------
 * Name		:	I2C_SlaveReceiveData
 * Description	:	Receive Data from master
 * Parameter 1	:	Base address of the I2C peripheral
 * Return Type	:	uint8_t (dataFromMaster)
 * Note		:	Returns data a byte at a time received from the master.
 * ------------------------------------------------------------------------------------------------------ */
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx)
{
	/* -Return contents of Data Register (DR) - */
	return (uint8_t) pI2Cx->DR;
}


/* ------------------------------------------------------------------------------------------------------
 * Name		:	I2C_Close_SendData
 * Description	:	To close the Data Transmission in Interrupt Mode
 *
 * Parameter 1	:	Handle pointer variable
 * Return Type	:	none (void)
 * Note		:	Jobs:
 * 				1. Disable interrupt
 * 				2. Reset Member Elements (I2C Handle structure)
 *
 * ------------------------------------------------------------------------------------------------------ */
void I2C_Close_SendData(I2C_Handle_t *pI2CHandle)
{
	/* -Step 1. Disable Interupt Enable Bits- */
	// Disabling ITBUFEN Control Bit [Clearing ITBUFEN in CR2]
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	// Disabling ITEVFEN Control Bit [Clearing ITEVFEN in CR2]
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	/* -Step 2. Reset Member Elements- */
	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->TxDataLength = 0;

}


/* ------------------------------------------------------------------------------------------------------
 * Name		:	I2C_Close_ReceiveData
 * Description	:	To close the Data Reception in Interrupt Mode
 *
 * Parameter 1	:	Handle pointer variable
 * Return Type	:	none (void)
 * Note		:	Jobs:
 * 				1. Disable interrupt
 * 				2. Reset Member Elements (I2C Handle structure)
 *
 * ------------------------------------------------------------------------------------------------------ */
void I2C_Close_ReceiveData(I2C_Handle_t *pI2CHandle)
{
	/* -Step 1. Disable Interupt Enable Bits- */
	// Disabling ITBUFEN Control Bit [Clearing ITBUFEN in CR2]
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	// Disabling ITEVFEN Control Bit [Clearing ITEVFEN in CR2]
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	/* -Step 2. Reset Member Elements- */
	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxDataLength = 0;
	pI2CHandle->RxSize = 0;

	// Enable ACKing, ONLY if ACK Bit is SET
	if(pI2CHandle->I2C_Config.I2C_ACK_Control == I2C_ACK_ENABLE)
	{
		I2C_ManageACK(pI2CHandle->pI2Cx, ENABLE);
	}

}


/* ------------------------------------------------------------------------------------------------------
 * Name		:	I2C_GenerateStartCondition
 * Description	:	To generate START condition
 *
 * Parameter 1	:	Base address of the I2C peripheral
 * Return Type	:	none (void)
 * Note		:
 * 			To generate START condition
 * ------------------------------------------------------------------------------------------------------ */
void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
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
 * Name		:	I2C_GenerateStopCondition
 * Description	:	To generate STOP condition
 *
 * Parameter 1	:	Base address of the I2C peripheral
 * Return Type	:	none (void)
 * Note		:
 * 			To generate STOP condition
 * ------------------------------------------------------------------------------------------------------ */
void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
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
 * Name		:	I2C_EV_IRQHandling
 * Description	:	To handle the interrupts generated by I2C EVENTS:
 *
 * Parameter 1	:	Handle pointer variable
 * Return Type	:	none (void)
 * Note		:	Event Interrupt can be generated by:
 * 				-> SB, ADDR, ADD10, STOPF, BTF, TxE, ITBUFEN, RxNE
 * 				ADDR10 is not implemented (NOT using 10 bit Address Mode)
 * ------------------------------------------------------------------------------------------------------ */
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	/* - Interrupt handling for both Master and Slave Mode - */

	/* - Check why interrupt is triggered and handle accordingly - */

	// Temporary variables to hold the status flag
	uint32_t temp_a, temp_b, temp_c;

	// For Event Interrupt to Trigger, ITEVFEN Bit MUST be Enabled [CR2]
	temp_a = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITEVTEN);

	// Check for ITBUFEN Bit
	temp_b = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITBUFEN);

	// a. Handle for Interrupt Generated by SB Event [SB (Start Bit) -> applicable ONLY in Master Mode]
	temp_c =  pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_SB);
	if (temp_a && temp_c)
	{
		// Interrupt is triggered because of SB Event
		// [NOTE: For Slave SB flag is always RESET (in slave mode, if statement will never executes)]

		/* - STEP 1. Generate Start Condition- */
		//SB is SET when START Condition is generated (So, START Condition is already generated)

		/* -STEP 2. Execute Address Phase- */
		if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			// Write Phase
			I2C_ExecuteAddressPhase_Write(pI2CHandle->pI2Cx, pI2CHandle->DeviceAdddress);
		}
		else if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			// Read Phase
			I2C_ExecuteAddressPhase_Read(pI2CHandle->pI2Cx, pI2CHandle->DeviceAdddress);
		}
		else
		{
			// Meh
		}

	}


	// b. Handle for Interrupt Generated by ADDR Event
	/*
	 * if Mode = Master : Address is sent
	 * if Mode = Slave	: Address is matched with OWN address
	 *
	 * */
	temp_c =  pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ADDR);
	if (temp_a && temp_c)
	{
		// Interrupt is triggered because of ADDR Event

		/*
		 * When ADDR is SET, Clock will be Stretched
		 * When ADDR FLAG is SET, First Step will be CLEAR ADDR FLAG [Important]
		 */

		/* - STEP 1. Clear ADDR FLAG- */
		I2C_ClearADDRFlag(pI2CHandle);

		// That's it

	}

	// c. Handle for Interrupt Generated by BTF Event (BYTE TRANSFER FINISH)
	temp_c =  pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_BTF);
	if (temp_a && temp_c)
	{
		// Interrupt is triggered because of BTF Event

		/*
		 * When BTF is SET AND :
		 * 	a. TXE is SET: During Transmission: [Shift Register] AND [Data Register] are BOTH EMPTY
		 *			 		    [BTF = 1 AND TXE = 1] and Clock will be stretched
		 *
		 * 	b. RXNE is SET: During Reception: [Shift Register] AND [Data Register] are BOTH FULL
		 * 					  [BTF = 1 AND RXNE = 1] and Clock will be stretched
		 * 	*/

		// Decision is based on application STATE
		if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			// Check whether TXE is SET or NOT
			if (pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE))
			{
				/* -Here, TXE and BTF both are SET- */

				// Indication to close the transmission (ONLY when Length of Data is ZERO)
				if (pI2CHandle->TxDataLength == 0)
				{

					// a. Generate STOP Condition
					if (pI2CHandle->RepeatedStart == I2C_REPEATED_START_DI)
						{
							// Check for Repeated Start then generate STOP condition
							I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
						}

					// b. Reset member elements of Handle Structure
					// Close Send Data
					I2C_Close_SendData(pI2CHandle);


					// c. Notify the Application: Transmission Complete
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EVENT_TX_COMPLETE);
				}
			}

		}
		else if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			// Nothing to do
		}


	}

	// d. Handle for Interrupt Generated by STOPF Event [STOPF -> applicable ONLY in Slave Mode]
	temp_c =  pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_STOPF);
	if (temp_a && temp_c)
	{
		/* -Executes ONLY in Slave Mode- */
		// Interrupt is triggered because of STOPF Event

		// a. Clear the STOPF Flag
		/*
		 * [Clearing Procedure: 1. Read SR1 register then Write to CR1]
		 * 1. Reading SR1 is already DONE (step d.)
		 * 2. Write carefully so the content of CR1 does not corrupt [Cr1 | 0000]
		 *
		 * */
		pI2CHandle->pI2Cx->CR1 |= 0x0000;
		// STOPF Flag is now cleared

		// b. Notify the Application: STOP is generated by Master
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EVENT_STOP);

	}

	// e. Handle for Interrupt Generated by TXE Event
	temp_c =  pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE);
	if (temp_a && temp_b && temp_c)
	{
		/* -Executes ONLY in Master Mode- */
		// Interrupt is triggered because of TXE Event
		// [TxE = 1 : Data Register is empty] and software has to write data to Data Register

		// Check to ensure device is in Master Mode [MSL bit in SR2: if 1 -> Master Mode; 0 -> Slave Mode]
		if (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
		{

			// a. Data Transmission
			if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
			{
				// Data Transmission only when Application's state is BUSY_IN_TX
				if (pI2CHandle->TxDataLength > 0)
				{
					// a. Load Data in Data Register
					pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer); // Dereference to put data

					// b. Decrement TxDataLength
					pI2CHandle->TxDataLength--;

					// c. Increment TxBuffer Address
					pI2CHandle->pTxBuffer++;

				}

			}
		}
		else
		{
			/* -Executes ONLY in Slave Mode- */
			// TxE is SET meaning request for data

			// Only when TRA is 1 [SR2]
			/*
			 * if TRA = 1 Data Byte Transmitted     : Device is in Transmitter Mode
			 *    TRA = 0 Data Byte Received	: Device is in Receiver Mode
			 *
			 * */
			if ( (pI2CHandle->pI2Cx->SR2) & (1 << I2C_SR2_TRA) )
				{
					// Slave is in Transmitter Mode
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EVENT_DATA_REQUEST);
				}
		}

	}

	// f. Handle for Interrupt Generated by RXNE Event
	temp_c =  pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_RXNE);
	if (temp_a && temp_b && temp_c)
	{
		/* -Interrupt is triggered because of RXNE Event- */

		// Check to ensure device is in Master Mode [MSL bit in SR2: if 1 -> Master Mode; 0 -> Slave Mode]
		if (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
		{


			// a. Data Reception
			if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
			{
				// Check Length Information [Two Cases: RxSize = 1 and RxSize > 1]
				if (pI2CHandle->RxSize == 1)
				{
					// a. Read Data (1 byte) from Data Register to RxBuffer
					*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;

					// b. Decrement RxDataLength
					pI2CHandle->RxDataLength--;

				}

				if (pI2CHandle->RxSize > 1)
				{
					// a. if RxDataLength == 2, Disable ACking
					if(pI2CHandle->RxDataLength == 2)
					{
						// Disable ACKing
						I2C_ManageACK(pI2CHandle->pI2Cx, DISABLE);
					}

					// b. Keep on Reading Data Register
					*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;

					// c. Increment RxBuffer Address
					pI2CHandle->pRxBuffer++;

					// d. Decrement RxDataLength
					pI2CHandle->RxDataLength--;

				}

				if (pI2CHandle->RxDataLength == 0)
				{
					/* -Indication to Close the Data Reception- */
					// Also, Notify application about closing data reception

					// a. Generate STOP Condition
					if (pI2CHandle->RepeatedStart == I2C_REPEATED_START_DI)
						{
							// Check for Repeated Start then generate STOP condition
							I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
						}

					// b. Close Data Recption
					// Close Rx Data
					I2C_Close_ReceiveData(pI2CHandle);

					// c. Notify Application: Close Data Reception
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EVENT_RX_COMPLETE);

				}

			}
		}
		else
		{
			/* -Executes ONLY in Slave Mode- */
			// RxNE is SET meaning Receive Data

			// Only when TRA is 0 [SR2]
			/*
			 * if TRA = 1 Data Byte Transmitted 	: Device is in Transmitter Mode
			 *    TRA = 0 Data Byte Received	: Device is in Receiver Mode
			 *
			 * */
			if ( !((pI2CHandle->pI2Cx->SR2) & (1 << I2C_SR2_TRA)) )
				{
					// Slave is in Receiver Mode
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EVENT_DATA_RECEIVE);
				}

		}
	}

}


/* ------------------------------------------------------------------------------------------------------
 * Name		:	I2C_ER_IRQHandling
 * Description	:	To handle the interrupts generated by I2C ERRORS:
 *
 * Parameter 1	:	Handle pointer variable
 * Return Type	:	none (void)
 * Note		:
 * ------------------------------------------------------------------------------------------------------ */
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	/* - Check why interrupt is triggered and handle accordingly - */

	// Temporary variables to hold the status flag
	uint32_t temp_a, temp_b;

	// Check status of ITERREN Control Bit [CR2]
	temp_b = (pI2CHandle->pI2Cx->CR2) & (1 << I2C_CR2_ITERREN);

	/* -Check for Bus Error- */
	temp_a = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_BERR);
	if(temp_a && temp_b)
	{
		/* -True: Error is BUS ERROR- */

		// 1. Clear the BUS ERROR FLAG
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_BERR);

		// 2. Notify the Application: BUS ERROR
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_BERR);

	}


	/* -Check for Arbitration Lost Error- */
	temp_a = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_ARLO);
	if(temp_a && temp_b)
	{
		/* -True: Error is ARBITRATION LOST ERROR- */

		// 1. Clear the ARBITRATION LOST ERROR FLAG
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_ARLO);

		// 2. Notify the Application: ARBITRATION LOST ERROR
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_ARLO);

	}


	/* -Check for ACK Failure Error- */
	temp_a = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_AF);
	if(temp_a && temp_b)
	{
		/* -True: Error is ACK FAILURE ERROR- */

		// 1. Clear the ACK FAILURE ERROR FLAG
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_AF);

		// 2. Notify the Application: ACK FAILURE ERROR
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_AF);

	}


	/* -Check for Overrun/Underrun Error- */
	temp_a = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_OVR);
	if(temp_a && temp_b)
	{
		/* -True: Error is OVERRUN/UNDERUN ERROR- */

		// 1. Clear the OVERRUN/UNDERUN ERROR FLAG
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_OVR);

		// 2. Notify the Application: OVERRUN/UNDERUN ERROR
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_OVR);

	}


	/* -Check for Time-Out Error- */
	temp_a = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_TIMEOUT);
	if(temp_a && temp_b)
	{
		/* -True: Error is TIME-OUT ERROR- */

		// 1. Clear the TIME-OUT ERROR FLAG
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_TIMEOUT);

		// 2. Notify the Application: TIME-OUT ERROR
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_TIMEOUT);

	}

}


/* ------------------------------------------------------------------------------------------------------
 * Name		:	I2C_getFlagStatus
 * Description	:	To get the Flags info from Status Register
 *
 * Parameter 1	:	Base address of the I2C peripheral
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
 * Parameter 1	:	Base address of the I2C peripheral
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
 * Parameter 1	:	Base address of the I2C peripheral
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


/* ------------------------------------------------------------------------------------------------------
 * Name		:	I2C_Slave_ManageCallbackEvents
 * Description	:	To Enable or Disable the Callback Events in Slave Mode
 *
 * Parameter 1	:	Base address of the I2C peripheral
 * Parameter 2  :   	Enable or Disable Macro
 * Return Type	:	none
 * Note		:	Enables or disables the Interrupt Control Bits (CR2) in Slave Mode
 *
 * ------------------------------------------------------------------------------------------------------ */
void I2C_Slave_ManageCallbackEvents(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		/* -Enable Interrupt Control Bits- */

		// Enable ITEVTEN Bit
		pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);

		// Enable ITBUFEN Bit
		pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);

		// Enable ITERREN bit
		pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);

	}
	else if (EnorDi == DISABLE)
	{
		/* -Disable Interrupt Control Bits- */

		// Disable ITEVTEN Bit
		pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

		// Disable ITBUFEN Bit
		pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

		// Disable ITERREN bit
		pI2Cx->CR2 &= ~(1 << I2C_CR2_ITERREN);
	}
	else
	{
		// Meh
	}


}



/* ------------------------------------------------------------------------------------------------------
 * Name		:	I2C_ApplicationEventCallback
 * Description	:	Callback implementation
 *
 * Parameter 1	:	I2C Handle Pointer
 * Parameter 2	:	Application Event Macro (Possible I2C Application Events)
 * Return Type	:	none (void)
 * Note		:	This function will be implemented in the application. If not, to clear warnings or errors
 * 			weak implementation is done here. If application does not implement this function
 * 			then this implementation will be called. __attribute__((weak))
 * ------------------------------------------------------------------------------------------------------ */
__attribute__((weak))void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t ApplicationEvent)
{
	// May or may not be implemented in the application file as per the requirements
}


/*------------------------------------ HELPER FUNCTIONS IMPLEMENTATIONS ----------------------------*/

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
 * Parameter 1	:	Handle pointer variable
 * Return Type	:	none (void)
 * Note		:	Private helper function
 *
 * ------------------------------------------------------------------------------------------------------ */
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle)
{
	// Clearing: Cleared by software by reading SR1 Register followed reading SR2 or by hardware when PE = 0
	// Simply read SR1 and SR2
	uint32_t dummyRead;

	// 1. Check if for Device Mode [MSL bit in SR2: if 1 -> Master Mode; 0 -> Slave Mode]
	if (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
	{
		/* -True: Device is in Master Mode- */

		// Check Application's State [Should be Busy in Rx]
		if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			// Now check, RxSize
			if (pI2CHandle->RxSize == 1)
			{
				// a. Disable ACKing
				I2C_ManageACK(pI2CHandle->pI2Cx, DISABLE);

				// b. Now Clear the ADDR Flag [by Reading SR1 and then SR2]

				dummyRead = pI2CHandle->pI2Cx->SR1;
				dummyRead = pI2CHandle->pI2Cx->SR2;

				(void) dummyRead;

			}
		}
		else
		{
			/* -STATE is NOT BUSY IN RX- */

			// Simply clear by reading SR1 and then SR2
			dummyRead = pI2CHandle->pI2Cx->SR1;
			dummyRead = pI2CHandle->pI2Cx->SR2;

			(void) dummyRead;

		}

	}
	else
	{
		/* -False: Device is in Slave Mode- */

		// Simply clear by reading SR1 and then SR2
		dummyRead = pI2CHandle->pI2Cx->SR1;
		dummyRead = pI2CHandle->pI2Cx->SR2;

		(void) dummyRead;

	}


}
