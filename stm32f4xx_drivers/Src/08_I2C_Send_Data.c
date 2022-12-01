/*
 * 08_I2C_Send_Data.c
 *
 *	-> I2C Send Data
 *
 *	When the button on the master is pressed, the master will send a string of data to the connected slave device.
 *
 *	-> I2C CONFIGURATION:
 *		-> I2C in Standard Mode (SCL = 100kHz)
 *		-> Use external pull-up resisters (3.3kOhms) for SDA and SCL lines
 *
 *	-> I2C Perpheral used		:	I2C1
 *	-> Pins				:	PB6 -> I2C1_SCL
 *						PB7 -> I2C1_SDA
 *
 *	-> Alternate Functionality 	:	AF4
 *   */


#include "stm32f407xx.h"

#include "stm32f407xx_gpio_drivers.h"

#include "stm32f407xx_i2c_drivers.h"

#include <string.h>



// Device Address (in SLAVE mode)
#define MY_ADDRESS	0x61

// SLAVE Address
#define SLAVE_ADDRESS	0x68

// I2C1  Handle variable
I2C_Handle_t I2C1Handle;

// Data to transmit
uint8_t TxData[] = "Lalitk.space \n";

// To configure GPIO pins to behave as I2C peripheral
void I2C1_GPIO_Init(void);

// To configure I2C1 peripheral
void I2C1_Init(void);

// To configure the GPIO Button
void GPIO_buttonInit(void);

// Software delay for de-bouncing
void softDelay(void);

int main()
{
	/* -- Configure GPIOs Alternate Functionality as I2C Peripheral -- */
	I2C1_GPIO_Init();

	/* -- Configure I2C1 Peripheral -- */
	I2C1_Init();

	/* -- GPIO Button Init -- */
	GPIO_buttonInit();

	/* -- Now, Enable the I2C (I2C Peripheral is disabled by default)-- */
	// MUST BE ENABLED AFTER ALL THE REQUIRED CONFIGURATIONS
	I2C_PeripheralControl(I2C1,ENABLE);

	/* -- Data is transmitted only when button is pressed -- */
	while (1)
	{

		/* -- Wait till button is pressed -- */
		while ( !(GPIO_ReadFromInputPin(GPIOA, GPIO_Pin_0)));		// hang here until button is pressed

		/* -- For De-bouncing -- */
		softDelay();

		/* -- Send Data -- */
		I2C_MasterSendData(&I2C1Handle, TxData, strlen((char *)TxData), SLAVE_ADDRESS);

	}

	return 0;
}


void I2C1_GPIO_Init(void)
{
	/* --  Handle variable -- */
	GPIO_Handle_t I2CPins;

	/* -- Port Selection -- */
	I2CPins.pGPIOx	=	GPIOB;

	/* -- Peripheral Configurations -- */
	I2CPins.GPIO_PinConfig.GPIO_PinMode		= GPIO_MODE_ALTFUNC;	// Mode as Alternate Functionality
	I2CPins.GPIO_PinConfig.GPIO_PinAltFuncMode 	= 4;			// ALT FUNCTION Mode is 4
	I2CPins.GPIO_PinConfig.GPIO_PinOPType		= GPIO_OP_TYPE_OD;	// Open Drain
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl	= GPIO_PIN_PU;		// Pull UP
	I2CPins.GPIO_PinConfig.GPIO_PinSpeed		= GPIO_SPEED_VERY_HIGH;	// SPEED doesn't matter


	/* -- Pins Configuration -- */

	// I2C1_SCL -> PB6
	I2CPins.GPIO_PinConfig.GPIO_PinNumber		= GPIO_Pin_6;
	GPIO_Init(&I2CPins);

	// I2C1_SDA -> PB7
	I2CPins.GPIO_PinConfig.GPIO_PinNumber		= GPIO_Pin_7;
	GPIO_Init(&I2CPins);

}


void I2C1_Init(void)
{
	// Handle defined in global space
	/* -- Base Address of the I2C1 Peripheral -- */
	I2C1Handle.pI2Cx			 = I2C1;

	/* -- Peripheral Configuration -- */
	I2C1Handle.I2C_Config.I2C_ACK_Control	 =	I2C_ACK_ENABLE;   	// Enable ACKing
	I2C1Handle.I2C_Config.I2C_FM_DutyCycle	 = 	I2C_FM_DutyCycle_2;	// Can be ignored (Device is in SM)
	I2C1Handle.I2C_Config.I2C_Device_Address =  	MY_ADDRESS;		// Can be ignored (Device is Master)
	I2C1Handle.I2C_Config.I2C_SCL_Speed	 =  	I2C_SCL_SPEED_SM;	// SCL Speed (Standard)

	/* -- Initialize the I2C Peripheral -- */
	I2C_Init(&I2C1Handle);

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





