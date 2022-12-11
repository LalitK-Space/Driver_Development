/*
 * 010_I2C_Tx_Rx_IT.c
 *
 *	-> Interrupt Based Communication
 *
 *	When the button on the master is pressed, the master will read and display data from the connected slave device.
 *	First Master will receive the length of data from the slave and then read the data
 *
 *	PRODUDURE:
 *  	1. Master writes to Slave in interrupt mode (request for length of data (1 Byte)) [R/~W = 0]
 *  	2. Master reads from Slave in interrupt mode (length information (1 Byte)) [R/~W = 1]
 *  	3. Master writes to Slave in interrupt mode (request for data) [R/~W = 0]
 *  	4. Master reads from Slave in interrupt mode (data (x Bytes)) [R/~W = 1]
 *
 *  	[NOTEs]
 *  	-> Check return value (returns State of Application) of I2C_MasterReceiveData_IT and I2C_SendData_IT APIs
 *  	   and wait until State is READY.
 *  	-> Since Interrupt Mode
 *  		- Configure the IRQ for I2Cx (I2C1) Peripheral
 *  		- Configure ISRs for I2Cx (I2C1) Events Interrupts and I2Cx (I2C1) Errors interrupts
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
 *
 *	-> Semihosting is enabled for this application
 *		-> To enable Semihosting:
 *		   - Set Linker Flags								: -specs=rdimon.specs -lc -lrdimon
 *		   - In Debug Configuration > Run Cummands (enter)	: monitor arm semihosting enable
 *		   - In Debug Configuration, change debugger 		: to ST_LINK (OpenOCD)
 *		   - In application (xxx.c)							: add prototype -> extern void initialise_monitor_handles(void);
 *															  and before using any printfs -> initialise_monitor_handles();
 *		   - Exclude 'syscalls.c' from build process
 *
 */


#include "stm32f407xx.h"

#include "stm32f407xx_gpio_drivers.h"

#include "stm32f407xx_i2c_drivers.h"

#include <string.h>
#include <stdio.h>

/* -- To initialize the semi-hosting features -- */
extern void initialise_monitor_handles(void);

// Device Address (in SLAVE mode)
#define MY_ADDRESS	0x61

// SLAVE Address
#define SLAVE_ADDRESS	0x68

uint8_t RxCompleteFlag = 0;	// Application specific global variable

// I2C1  Handle variable
I2C_Handle_t I2C1Handle;

// Rx Buffer
uint8_t RxData[32];

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
	/* -- To enable Semi-Hosting [before using any printfs] -- */
	initialise_monitor_handles();
	printf("I2C Master Data Transmission \n");

	/* -- Configure GPIOs Alternate Functionality as I2C Peripheral -- */
	I2C1_GPIO_Init();

	/* -- Configure I2C1 Peripheral -- */
	I2C1_Init();

	/* -- I2C IRQ Configurations -- */
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_EV, ENABLE);	// Enabling for EVENTS
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_ER, ENABLE); // Enabling for ERRORS

	/* -- I2C Interrupt Priority Configurations -- */
	// Not required in Application


	/* -- GPIO Button Init -- */
	GPIO_buttonInit();

	/* -- Now, Enable the I2C (I2C Peripheral is disabled by default)-- */
	// MUST BE ENABLED AFTER ALL THE REQUIRED CONFIGURATIONS [PE = 1]
	I2C_PeripheralControl(I2C1,ENABLE);

	// Enable ACKing after Enabling I2C Peripheral (if PE = 1 then only ACK = 1 [CR1])
	// for reading data from the Slave (> 1 byte)
	I2C_ManageACK(I2C1, ENABLE);


	/* -- Request Code -- */
	uint8_t requestCode;		// 0x51 -> Request for length information
					// 0x52 -> Request for data

	/* -- Length of data (slave wants to send) -- */
	uint8_t lengthRxData;

	/* -- Data is transmitted only when button is pressed -- */
	while (1)
	{

		/* -- Wait till button is pressed -- */
		while ( !(GPIO_ReadFromInputPin(GPIOA, GPIO_Pin_0)));		// hang here until button is pressed

		/* -- For De-bouncing -- */
		softDelay();

		/* -- Procedure -- */

		// 1. Master writes to Slave (request for length of data (1 Byte)) [R/~W = 0]
		requestCode = 0x51;		// Request length information

		// Wait until State is READY
		while (I2C_MasterSendData_IT(&I2C1Handle, &requestCode, 1,SLAVE_ADDRESS,I2C_REPEATED_START_EN) != I2C_READY);

		// 2. Master reads from Slave (length information (1 Byte)) [R/~W = 1]
		// Store length information in 'lengthRxData'

		// Wait until State is READY
		while (I2C_MasterReceiveData_IT(&I2C1Handle, &lengthRxData, 1,SLAVE_ADDRESS,I2C_REPEATED_START_EN) != I2C_READY);

		// 3. Master writes to Slave (request for data) [R/~W = 0]
		requestCode = 0x52;		// Request data

		// Wait until State is READY
		while (I2C_MasterSendData_IT(&I2C1Handle, &requestCode, 1,SLAVE_ADDRESS,I2C_REPEATED_START_EN) != I2C_READY);

		// 4. Master reads from Slave (data (x Bytes)) [R/~W = 1]
		// Repeated Start is disabled [wants to end the communication]

		// Wait until State is READY
		while (I2C_MasterReceiveData_IT(&I2C1Handle, RxData, lengthRxData,SLAVE_ADDRESS,I2C_REPEATED_START_DI) != I2C_READY);

		// RESET the RxCompleteFlag variable (SET in first I2C_MasterReceiveData_IT)
		RxCompleteFlag = 0;

		// 5. Print Received Data

		// Wait until I2C_EVENT_RX_COMPLETE (Application Event) to print data
		while (RxCompleteFlag != 1);	// wait till RxCompleteFlag is not SET to print RxData properly

		RxData[lengthRxData + 1] = '\0';	// Making last element of array as 0 [because of %s in printf]
		printf("Received Data from Slave: %s",RxData);

		// Again RESET the RxCompleteFlag variable
		RxCompleteFlag = 0;
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
		gpio_button.GPIO_PinConfig.GPIO_PinPuPdControl 		= GPIO_NO_PUPD;

		/* -- Before Calling GPIO Init, Enable Peripheral Clock -- */
		// Clock will be enabled in GPIO_init();

		/* -- Call GPIO Initialization Function and pass address of handle variable as input argument -- */
		GPIO_Init(&gpio_button);

}


// I2C1 Event Interrupt IRQ handler
void I2C1_EV_IRQHandler(void)
{
	/* -- Call IRQ Event Handling API  -- */
	I2C_EV_IRQHandling(&I2C1Handle);

	// Handler will notify application


}

// I2C1 Error Interrupt IRQ Handler
void I2C1_ER_IRQHandler(void)
{
	/* -- Call IRQ Error Handling API  -- */
	I2C_ER_IRQHandling(&I2C1Handle);

}

// Application Callbacks
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t ApplicationEvent)
{
	/* -- Events Generated by I2C (implemented in the driver)
	 *
	 * I2C_EVENT_TX_COMPLETE
 	 * I2C_EVENT_RX_COMPLETE
 	 * I2C_EVENT_STOP
 	 * I2C_ERROR_BERR
	 * I2C_ERROR_ARLO
	 * I2C_ERROR_AF
	 * I2C_ERROR_OVR
	 * I2C_ERROR_TIMEOUT
	 *
	 * --*/

	if (ApplicationEvent == I2C_EVENT_TX_COMPLETE)
	{
		// Tx is complete
		printf("Data Transmission is complete \n");
	}
	else if (ApplicationEvent == I2C_EVENT_RX_COMPLETE)
	{
		// Rx is complete
		printf("Data Reception is complete \n");
		RxCompleteFlag = 1;
	}
	// Taking an error case to ensure working
	else if (ApplicationEvent == I2C_ERROR_AF)
	{
		// ERROR: ACK Failed

		/* -- In case of Error -- */

		// Close Communication
		I2C_Close_SendData(&I2C1Handle);	// Close Send Data because Device is Master

		// Generate STOP Condition (to free the bus)
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		// Take action accordingly
		// Example: hang application here to process [while(1);]
		printf("!!! ACK ERROR !!! \n");

	}
	else
	{
		// Some other Event
	}


}


// Software Delay
void softDelay(void)
{
	for(uint32_t i = 0; i < 500000/2; i++);
}


