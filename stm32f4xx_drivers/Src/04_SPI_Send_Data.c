/*
 * 04_SPI_Send_Data.c
 *
 *  -> SPI SEND DATA (Polling based)<-
 *  -> Without SLAVE
 *  -> JUST to test "SendData" Function
 *  -> [Important]Since, SLAVE device is not present, SSM is enabled -> refer : Description of SPI_SSIConfig();
 *
 *  1. Check which pins can be used as SPIx pins (ALternate Functions)
 *
 *  SPI Peripheral Used			:	SPI2
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

int main()
{
	/* -- Configure GPIOs Alternate Functionality as SPI Peripheral -- */
	SPI2_GPIO_Init();

	/* -- Configure SPI2 Peripheral -- */
	SPI2_Init();

	/* -- SSI Configuration -- */
	// MAkes NSS signal internally HIGH and avoid MODF error
	SPI_SSIConfig(SPI2, ENABLE);						// Please READ function description

	/* -- Now, Enable the SPI (SPI Peripheral is disabled by default)-- */
	// MUST BE ENABLED AFTER ALL THE REQUIRED CONFIGURATIONS
	SPI_PeripheralControl(SPI2,ENABLE);

	/* --  Data to send -- */
	char user_data[]	= "Hello World!";				// Data Buffer

	/* -- Send Data -- */
	SPI_SendData(SPI2, (uint8_t *)user_data, strlen(user_data));	// (uint8_t*)user_data because user_data is char *

	/* -- Before Disabling the SPI, ensure all the Data is sent (check BSY Flag in SR)*/
	while (SPI_getFlagStatus(SPI2, SPI_BUSY_FLAG));		// Hang here, if BUSY FLAG is 1 means SPI is busy


	/* -- Now, Tx is done, Disable the peripheral -- */
	// Disabling abruptly may cause problems, ensure that all the data is transmitted before disabling ^
	SPI_PeripheralControl(SPI2, DISABLE);

	while(1);

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
	SPIpins.GPIO_PinConfig.GPIO_PinAltFuncMode = 5;						// ALT FUNCTION Mode is 5
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

/*
	// PB(12) -> SPI2_NSS [DISABLED BECAUSE NOT USED IN APPLICATION]
	SPIpins.GPIO_PinConfig.GPIO_PinNumber		= GPIO_Pin_12;
	GPIO_Init(&SPIpins);
*/

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
	SPI2Handle.SPIConfig.SPI_Speed				= SPI_SCLK_SPEED_DIV_2;		// Max Speed p-clk/2 : SCLK = 8MHz
	SPI2Handle.SPIConfig.SPI_DFF				= SPI_DFF_8BITS;			// 8Bits Data Frame Format
	SPI2Handle.SPIConfig.SPI_COPL				= SPI_CPOL_LOW;				// Default
	SPI2Handle.SPIConfig.SPI_CPHA				= SPI_CPHA_LOW;				// Default
	SPI2Handle.SPIConfig.SPI_SSM				= SPI_SSM_SW_EN;			// Software Slave Select Management EN

	/* -- Initialize the SPI Peripheral -- */
	SPI_Init(&SPI2Handle);




}
