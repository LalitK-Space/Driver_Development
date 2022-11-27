/*
 * 05_SPI_Send_Data_toSlave.c
 *
 * -> SPI Master and SPI Slave Communication <-
 *
 *	When the button on the master is pressed, the master will send a string of data to the connected slave device.
 *
 *	SPI CONFIGURATION USED:
 *		- Full Duplex Mode
 *		- STM (this application) is Master
 *		- DFF bit is 0 (8-Bits data)
 *		- Hardware slave Management is used (SSM bit is 0)
 *		- SCLK speed is 2MHz, peripheral clock is 16MHz
 *
 *	-> In this application, the Master will not receive anything from the Slave.
 *	   (Configuration of MISO pin can be ignored)
 *
 *	-> [IMPORTANT] : The Slave doesn't know how many bytes of data the Master will send.
 *					 So Master first should send the number of bytes of information that
 *					 the Slave is going to receive and then send the data.
 *
 *
 *	SPI Peripheral Used			:	SPI2
 *  Pins			   			:	PB(15) -> SPI2_MOSI
 *  								PB(14) -> SPI2_MISO
 *  								PB(13) -> SPI2_SCLK
 *  								PB(12) -> SPI2_NSS
 *
 *	Alternate Functionality Mode:	5
 *
 */


#include "stm32f407xx.h"

#include "stm32f407xx_gpio_drivers.h"

#include "stm32f407xx_spi_drivers.h"

#include <string.h>

// To configure GPIO pins to behave as SPI peripheral
void SPI2_GPIO_Init(void);

// To configure SPI2 peripheral
void SPI2_Init(void);

// To configure the GPIO Button
void GPIO_buttonInit(void);

// Software delay for de-bouncing
void softDelay(void);

int main()
{
	/* -- Configure GPIOs Alternate Functionality as SPI Peripheral -- */
	SPI2_GPIO_Init();

	/* -- Configure SPI2 Peripheral -- */
	SPI2_Init();

	/* -- GPIO Button Init -- */
	GPIO_buttonInit();

	/* -- SSOE Configuration -- */
	// NSS output Enable -> Please refer to the function description
	SPI_SSOEConfig(SPI2, ENABLE);

	/* -- Data to send -- */
	char user_data[]	= "SPI Master to Slave.. Tx only test..";				// Data Buffer


	/* -- Data is transmitted only when button is pressed -- */
	while (1)
	{
		/* -- Wait till button is pressed -- */
		while ( !(GPIO_ReadFromInputPin(GPIOA, GPIO_Pin_0)));							// hang here until button is pressed

		/* -- For De-bouncing -- */
		softDelay();

		/* -- NOW, Enable SPI and Send Data -- */

		/* -- Now, Enable the SPI (SPI Peripheral is disabled by default)-- */
		// MUST BE ENABLED AFTER ALL THE REQUIRED CONFIGURATIONS ARE DONE
		SPI_PeripheralControl(SPI2,ENABLE);

		/* --  FIRST send Length Information -- */
		uint8_t lengthOfData = strlen(user_data);
		SPI_SendData(SPI2, &lengthOfData, 1);


		/* -- Send Data -- */
		SPI_SendData(SPI2, (uint8_t *)user_data, strlen(user_data));	// (uint8_t*)user_data because user_data is char *


		/* -- Before Disabling the SPI, ensure all the Data is sent (check BSY Flag in SR)*/
		while (SPI_getFlagStatus(SPI2, SPI_BUSY_FLAG));		// Hang here, if BUSY FLAG is 1 means SPI is busy


		/* -- Now, Tx is done, Disable the peripheral -- */
		// Disabling abruptly may cause problems, ensure that all the data is transmitted before disabling
		SPI_PeripheralControl(SPI2, DISABLE);

	}

	return 0;

}


void SPI2_GPIO_Init(void)
{
	/* --  Handle variable -- */
	GPIO_Handle_t SPIpins;

	/* -- Port Selection -- */
	SPIpins.pGPIOx 								= GPIOB;			    // PORT B

	// Peripheral Configurations
	SPIpins.GPIO_PinConfig.GPIO_PinMode 		= GPIO_MODE_ALTFUNC;	// Mode as Alternate Functionality
	SPIpins.GPIO_PinConfig.GPIO_PinAltFuncMode 	= 5;					// ALT FUNCTION Mode is 5
	SPIpins.GPIO_PinConfig.GPIO_PinOPType		= GPIO_OP_TYPE_PP;		// Push Pull Mode
	SPIpins.GPIO_PinConfig.GPIO_PinPuPdControl	= GPIO_NO_PUPD;			// No Pull UP/DOWN
	SPIpins.GPIO_PinConfig.GPIO_PinSpeed		= GPIO_SPEED_VERY_HIGH;	// SPEED doesn't matter

	/* -- Pins Configuration -- */
	// PB(15) -> SPI2_MOSI
	SPIpins.GPIO_PinConfig.GPIO_PinNumber		= GPIO_Pin_15;
	GPIO_Init(&SPIpins);

/*
	// PB(14) -> SPI2_MISO [DISABLED BECAUSE NOT USED IN APPLICATION]
	SPIpins.GPIO_PinConfig.GPIO_PinNumber		= GPIO_Pin_14;
	GPIO_Init(&SPIpins);
*/

	// PB(13) -> SPI2_SCLK
	SPIpins.GPIO_PinConfig.GPIO_PinNumber		= GPIO_Pin_13;
	GPIO_Init(&SPIpins);


	// PB(12) -> SPI2_NSS
	SPIpins.GPIO_PinConfig.GPIO_PinNumber		= GPIO_Pin_12;
	GPIO_Init(&SPIpins);

}

void SPI2_Init(void)
{
	/* --  Handle variable -- */
	SPI_Handle_t SPI2Handle;

	/* -- Base Address of the SPI2 Peripheral -- */
	SPI2Handle.pSPIx							= SPI2;

	/* -- Peripheral Configuration -- */
	SPI2Handle.SPIConfig.SPI_BusConfig			= SPI_BUS_CONFIG_FD;		// Full-Duplex Communication
	SPI2Handle.SPIConfig.SPI_DeviceMOde			= SPI_DEVICE_MODE_MASTER;	// Application is MASTER
	SPI2Handle.SPIConfig.SPI_Speed				= SPI_SCLK_SPEED_DIV_8;		// Max Speed p-clk/8 : SCLK = 2MHz
	SPI2Handle.SPIConfig.SPI_DFF				= SPI_DFF_8BITS;			// 8Bits Data Frame Format
	SPI2Handle.SPIConfig.SPI_COPL				= SPI_CPOL_LOW;				// Default
	SPI2Handle.SPIConfig.SPI_CPHA				= SPI_CPHA_LOW;				// Default
	SPI2Handle.SPIConfig.SPI_SSM				= SPI_SSM_SW_DI;			// Software Slave Select Management Disabled

	/* -- Initialize the SPI Peripheral -- */
	SPI_Init(&SPI2Handle);

}

void GPIO_buttonInit(void)
{
	/* -- Variable for GPIO Handle Structure: for Button -- */
	GPIO_Handle_t gpio_button;

	/* ------------------------------- GPIO Button Pin CONFIGURATIONS -----------------------------------------*/
		// Select Port
		gpio_button.pGPIOx = GPIOA;

		// Pin Configuration: PIN NUMBER
		gpio_button.GPIO_PinConfig.GPIO_PinNumber 		= GPIO_Pin_0;

		// Pin Configuration: PIN MODE
		gpio_button.GPIO_PinConfig.GPIO_PinMode 		= GPIO_MODE_IN;

		// Pin Configuration: GPIO SPEED (Not important in this case)
		gpio_button.GPIO_PinConfig.GPIO_PinSpeed 		= GPIO_SPEED_VERY_HIGH;

		// Pin Configuration: PU-PD Configuration (Selected ->NO PU PD, As PIN is already PULLED-DOWN -> Schematics)
		gpio_button.GPIO_PinConfig.GPIO_PinPuPdControl 	= GPIO_NO_PUPD;

		/* -- Before Calling GPIO Init, Enable Peripheral Clock -- */
		// Clock will be enabled in GPIO_init();

		/* -- Call GPIO Initialization Function and pass address of handle variable as input argument -- */
		GPIO_Init(&gpio_button);

}


void softDelay(void)
{
	for(uint32_t i = 0; i < 500000/2; i++);
}
