/*
 * 12_USART_Tx_Data.c
 *
 *  -> USART SEND DATA (Polling based) <-
 *
 *	-> Configuration:
 *		- baud Rate		:115200
 *		- Frame Format	:1 Stop Bit, 8 Bits Data, No Parity
 *
 *	USART Peripheral Used			:	USART2
 *  Pins			   	:	PA(2) -> USART2_Tx
 *  						PA(3) -> USART2_Rx

 *
 *  -> Alternate Functionality Mode:	7
 */

#include "stm32f407xx.h"

#include "stm32f407xx_gpio_drivers.h"

#include "stm32f407xx_usart_drivers.h"

#include<string.h>

/* --  USART Handle variable -- */
USART_Handle_t USART2Handle;

// Data to transmit
char TxData[1024] = "(Lalitk.space) testing USART\n";

// To configure GPIO pins to behave as USART peripheral
void USART2_GPIO_Init(void);

// To configure USART2 peripheral
void USART2_Init(void);

// To configure the GPIO Button
void GPIO_buttonInit(void);

// Software delay for de-bouncing
void softDelay(void);

int main()
{
	/* -- Configure GPIOs Alternate Functionality as USART Peripheral -- */
	USART2_GPIO_Init();

	/* -- Configure USART2 Peripheral -- */
	USART2_Init();

	/* -- GPIO Button Init -- */
	GPIO_buttonInit();

	/* -- Enable USART Peripheral -- */
	USART_PeripheralControl(USART2, ENABLE);

	/* -- Data is transmitted only when button is pressed -- */
	while (1)
	{
		/* -- Wait till button is pressed -- */
		while ( !(GPIO_ReadFromInputPin(GPIOA, GPIO_Pin_0)));				// hang here until button is pressed

		/* -- For De-bouncing -- */
		softDelay();

		/* -- Send Data -- */
		USART_SendData(&USART2Handle, (uint8_t *)TxData, strlen(TxData));


	}

	return 0;

}


void USART2_GPIO_Init(void)
{
	/* --  Handle variable -- */
	GPIO_Handle_t USARTpins;

	/* -- Port Selection -- */
	USARTpins.pGPIOx 					= GPIOA;		// PORT A

	// Peripheral Configurations
	USARTpins.GPIO_PinConfig.GPIO_PinMode 		= GPIO_MODE_ALTFUNC;	// Mode as Alternate Functionality
	USARTpins.GPIO_PinConfig.GPIO_PinAltFuncMode 	= 7;			// ALT FUNCTION Mode is 7
	USARTpins.GPIO_PinConfig.GPIO_PinOPType		= GPIO_OP_TYPE_PP;	// Push Pull Mode
	USARTpins.GPIO_PinConfig.GPIO_PinPuPdControl	= GPIO_PIN_PU;		// Pulled UP
	USARTpins.GPIO_PinConfig.GPIO_PinSpeed		= GPIO_SPEED_VERY_HIGH;	// SPEED doesn't matter

	/* -- Pins Configuration -- */
	// PA(2) -> USART2_Tx
	USARTpins.GPIO_PinConfig.GPIO_PinNumber = GPIO_Pin_2;
	GPIO_Init(&USARTpins);


	// PA(3) -> USART2_Rx
	USARTpins.GPIO_PinConfig.GPIO_PinNumber = GPIO_Pin_3;
	GPIO_Init(&USARTpins);

}

void USART2_Init(void)
{

	/* -- Base Address of the USART2 Peripheral -- */
	USART2Handle.pUSARTx	= USART2;

	/* -- Peripheral Configuration -- */
	USART2Handle.USART_Config.USART_BaudRate = USART_STD_BAUDRATE_115200;	// Baud Rate = 115200
	USART2Handle.USART_Config.USART_HW_FlowControl = USART_HW_FLOW_CONTROL_NONE; // No HW Flow Control
	USART2Handle.USART_Config.USART_Mode = USART_MODE_TX_ONLY;		// Mode: ONLY_TX (Only sending)
	USART2Handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;	// No. of Stop Bits = 1
	USART2Handle.USART_Config.USART_WordLength = USART_WORDLENGTH_8_BITS;	// Word Length of 8 Bits
	USART2Handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;	// Parity is disabled

	/* -- Initialize the USART Peripheral -- */
	USART_Init(&USART2Handle);

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
