/*
 * 	11_I2C_SlaveMode_Tx_Rx.c
 *
 *	-> STM32 Board will be in Slave Mode
 *	-> Slave's data communications will be in interrupt mode
 *
 *
 *	First Master will receive the length of data from the slave and then read the data
 *
 *	PROCEDURE:
 *		1. Master will send a request to read the length of data
 *		2. Then, after receiving length information
 *		3. Master will send a request to read the data
 *
 *
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
 *
 */

#include "stm32f407xx.h"

#include "stm32f407xx_gpio_drivers.h"

#include "stm32f407xx_i2c_drivers.h"

#include <string.h>



// SLAVE Address (This Application)
#define SLAVE_ADDRESS	0x68

/*
 * Defined in Master
 * 0x51 -> request for length information (when received send length information to Master)
 * 0x52 -> request for data (when received send data to Master)
 *
 * */
uint8_t RequestCode = 0;	// Application specific global variable
uint8_t byteTx = 0;


// I2C1  Handle variable
I2C_Handle_t I2C1Handle;

// Tx Buffer
uint8_t TxData[32] = " -> lalitk.Space";

// To configure GPIO pins to behave as I2C peripheral
void I2C1_GPIO_Init(void);

// To configure I2C1 peripheral
void I2C1_Init(void);


int main()
{

	/* -- Configure GPIOs Alternate Functionality as I2C Peripheral -- */
	I2C1_GPIO_Init();

	/* -- Configure I2C1 Peripheral -- */
	I2C1_Init();

	/* -- I2C IRQ Configurations -- */
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_EV, ENABLE);	// Enabling for EVENTS
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_ER, ENABLE); // Enabling for ERRORS

	// Enable Application Callbacks (Enables Interrupt Control Bits)
	I2C_Slave_ManageCallbackEvents(I2C1, ENABLE);

	/* -- I2C Interrupt Priority Configurations -- */
	// Not required in Application


	/* -- Now, Enable the I2C (I2C Peripheral is disabled by default)-- */
	// MUST BE ENABLED AFTER ALL THE REQUIRED CONFIGURATIONS [PE = 1]
	I2C_PeripheralControl(I2C1,ENABLE);

	// Enable ACKing after Enabling I2C Peripheral (if PE = 1 then only ACK = 1 [CR1])
	// for reading data from the Slave (> 1 byte)
	I2C_ManageACK(I2C1, ENABLE);


	/* -- Slave Hangs Here -- */
	/*
	 * (Until request is received from the Master then Application Event Callback will be called)
	 * */

	while(1);

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
	I2C1Handle.I2C_Config.I2C_Device_Address =  	SLAVE_ADDRESS;		// Device Address (in Slave Mode)
	I2C1Handle.I2C_Config.I2C_SCL_Speed	 =  	I2C_SCL_SPEED_SM;	// SCL Speed (Standard)

	/* -- Initialize the I2C Peripheral -- */
	I2C_Init(&I2C1Handle);

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
	 * I2C_EVENT_DATA_REQUEST
	 * I2C_EVENT_DATA_RECEIVE
	 *
	 * --*/

	if (ApplicationEvent == I2C_EVENT_DATA_REQUEST)
	{
		/* -Master is requesting data- */

		if (RequestCode == 0x51)		// Defined in Master
		{
			// Send the length information (length of TxData)
			I2C_SlaveSendData(pI2CHandle->pI2Cx, strlen((char *)TxData));
		}
		else if (RequestCode == 0x52)
		{
			// Send Data (TxData) to Master
			I2C_SlaveSendData(pI2CHandle->pI2Cx, TxData[byteTx++]);

			// Slave will transmit byte by byte, once completed AF Event is generated
		}
		else
		{
			// Invalid Request from Master
		}


	}
	else if (ApplicationEvent == I2C_EVENT_DATA_RECEIVE)
	{
		/* -Data is waiting- */

		// Request Code (read from Master)
		RequestCode = I2C_SlaveReceiveData(pI2CHandle->pI2Cx);

	}
	else if (ApplicationEvent == I2C_ERROR_AF)
	{
		// ACK Failure
		/*
		 * Occurs ONLY during SLAVE TRANSMISSION
		 * When Master NACKs
		 *  	-> if NACK (AF) is received, STOP sending Data
		 *
		 *  Also, Slave is done with Transmission
		 *
		 * */

		// Stop Transmission
		RequestCode = 0x1;	// Invalid code to stop transmission

		// Reset byteTx variable
		byteTx = 0;

	}
	else if (ApplicationEvent == I2C_EVENT_STOP)
	{
		/*
		 * Occurs ONLY during SLAVE RECEPTION
		 * 		-> Master has ended communication
		 *
		 * */
	}


}




