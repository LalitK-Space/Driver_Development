/*
 * 									stm32f407xx_rcc_drivers.c
 *
 *  This file contains RCC driver API implementations.
 *
 */

#include "stm32f407xx_rcc_drivers.h"

// array to store AHB prescaler values
uint16_t ahb_prescaler[8] = {2,4,8,16,64,128,256,512};

// array to store APB1 prescaler values
uint8_t apb1_prescaler[4] = {2,4,8,16};

// array to store APB2 prescaler values
uint8_t apb2_prescaler[4] = {2,4,8,16};

/* ------------------------------------------------------------------------------------------------------
 * Name		:	RCC_Pclk1_Value
 * Description	:	To get the value of Pclk1
 * Parameters	:	none
 * Return Type	:	(uint32_t) clock frequency
 * Note		:	For STM32F407G-DISC1 board,
 *			(From Clock Tree)
 *			[HSE or HSI(used) or PLLCLK or PLLR]
 *			 -> [AHB_PRESC] -> [APB1_PRESC] -> [APB1 peripheral clock]
 *
 *			Steps:
 *			1. Find source [HSE or HSI(used) or PLLCLK or PLLR]  (from RCC)
 *			2. Calculate AHB Prescaler
 *			3. Calculate APB1 Prescaler
 *			4. Then, derive clock frequency for APB1 peripheral
 *
 *			USART2, USART3, UART4, UART5 are connected to APB1 Bus
 *			I2Cx is connected to APB1 Bus
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
 * Name		:	RCC_Pclk2_Value
 * Description	:	To get the value of Pclk2
 * Parameters	:	none
 * Return Type	:	(uint32_t) clock frequency
 * Note		:	For STM32F407G-DISC1 board,
 *			(From Clock Tree)
 *			[HSE or HSI(used) or PLLCLK or PLLR]
 *			-> [AHB_PRESC] -> [APB2_PRESC] -> [APB2 peripheral clock]
 *
 *			Steps:
 *			1. Find source [HSE or HSI(used) or PLLCLK or PLLR]  (from RCC)
 *			2. Calculate AHB Prescaler
 *			3. Calculate APB2 Prescaler
 *			4. Then, derive clock frequency for APB2 peripheral
 *
 *			USART1, USART6 are connected to APB2 Bus
 *
 * ------------------------------------------------------------------------------------------------------ */
uint32_t RCC_Pclk2_Value(void)
{
	uint32_t pClk2;

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

	uint8_t apb2_pres;

	// Read the value of PPRE2[15:13] and shift the value to right and then, Mask them
	temp = ((RCC->CFGR >> 13) & 0x7);

	if (temp < 4)
	{
		// AHB clock is not divided, APB2 Prescaler is 1
		apb2_pres = 1;
	}
	else
	{
		// APB2 prescaler value depends upon temp variable
		apb2_pres = apb2_prescaler[temp - 4];		// if temp is 4 then, fetch 1st array value (0th element), and so on.

	}

	/* - Step 4: Derive Clock (Pclk1) - */
	pClk2 = (sysClockFreq / ahb_pres ) / apb2_pres;


	return pClk2;

}


