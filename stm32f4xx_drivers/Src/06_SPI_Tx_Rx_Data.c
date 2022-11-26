/*
 * 05_SPI_Send_Data_toSlave.c
 *
 * -> SPI Master (STM) and SPI Slave (Arduino) Communication <-
 *
 *	When the button on the master is pressed, master should send string of data to the Arduino slave connected.
 *	The data received by the Arduino will be displayed on the Arduino Serial Monitor.
 *
 *	SPI CONFIGURATION USED:
 *		- Full Duplex Mode
 *		- STM (this application) is Master and Arduino is Slave
 *		- DFF bit is 0 (8-Bits data)
 *		- Hardware slave Management is used (SSM bit is 0)
 *		- SCLK speed is 2MHz, peripheral clock is 16MHz
 *
 *	-> [IMPORTANT] : Slave doesn't know how many bytes of data master is going to send. So Master first should
 *					 send the number of bytes information which slave is going to receive and then send the data.
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

/*---------------------- LED CODE ---------------------------------------*/
#define LED_GREEN  12
#define LED_ORANGE 13
#define LED_RED    14
#define LED_BLUE   15

void led_off(uint8_t led_no);
void led_on(uint8_t led_no);
void led_init_all(void);
/* -----------------------------------------------------------------------*/

// To configure GPIO pins to behave as SPI peripheral
void SPI2_GPIO_Init(void);

// To configure SPI2 peripheral
void SPI2_Init(void);

// To configure the GPIO Button
void GPIO_buttonInit(void);

// Software delay for de-bouncing
void softDelay(void);

// To verify response from slave
uint8_t SPI_VerifyResponse(uint8_t ackByte);

/* ---------------------------- LED CODE -------------------------------------- */

/* -----------------------------^ LED CODE ^------------------------------------ */

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

	/* -- Dummy Data -- */
	uint8_t dummyWrite = 0x11;
	uint8_t dummyRead;
/* ---------------------------- LED CODE -------------------------------------- */
	led_init_all();
/* -----------------------------^ LED CODE ^------------------------------------ */

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

		/* -- Send Data -- */

		// Test Data
		uint8_t toArduino = 0xF1;	// F1 - F5		// Data to send
		uint8_t ackBytefromSlave;	// ACK Byte
/* -- ------------------------------------------------------------------------ -- */
		// SEND DATA
		SPI_SendData(SPI2,&toArduino, 1);

		// DUMMY READ
		SPI_ReceiveData(SPI2, &dummyRead, 1);

		// SEND DUMMY BYTES TO RECEIVE RESPONSE
		SPI_SendData(SPI2,&dummyWrite, 1);

		// READ ACTUAL DATA (ACK BYTE FROM SLAVE)
		SPI_ReceiveData(SPI2, &ackBytefromSlave, 1);
/* -- ------------------------------------------------------------------------ -- */
		if (ackBytefromSlave == (uint8_t)0xFF)
		{
			led_on(15);

			led_on(14);

			led_on(13);

			led_on(12);
		}


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


	// PB(14) -> SPI2_MISO
	SPIpins.GPIO_PinConfig.GPIO_PinNumber		= GPIO_Pin_14;
	GPIO_Init(&SPIpins);


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


uint8_t SPI_VerifyResponse(uint8_t ackByte)
{
	if (ackByte == 0xFF)	// Defined in Arduino Sketch
	{
		return 1;
	}
	return 0;
}


/* ---------------------- LED CODE ---------------------------------- */
void led_init_all(void)
{

	uint32_t *pRccAhb1enr = (uint32_t*)0x40023830;
	uint32_t *pGpiodModeReg = (uint32_t*)0x40020C00;


	*pRccAhb1enr |= ( 1 << 3);
	//configure LED_GREEN
	*pGpiodModeReg |= ( 1 << (2 * LED_GREEN));
	*pGpiodModeReg |= ( 1 << (2 * LED_ORANGE));
	*pGpiodModeReg |= ( 1 << (2 * LED_RED));
	*pGpiodModeReg |= ( 1 << (2 * LED_BLUE));

    led_off(LED_GREEN);
    led_off(LED_ORANGE);
    led_off(LED_RED);
    led_off(LED_BLUE);
}

void led_on(uint8_t led_no)
{
  uint32_t *pGpiodDataReg = (uint32_t*)0x40020C14;
  *pGpiodDataReg |= ( 1 << led_no);

}

void led_off(uint8_t led_no)
{
	  uint32_t *pGpiodDataReg = (uint32_t*)0x40020C14;
	  *pGpiodDataReg &= ~( 1 << led_no);

}
