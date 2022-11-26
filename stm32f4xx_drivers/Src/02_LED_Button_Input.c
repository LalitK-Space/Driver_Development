/*
 * 02_LED_Button_Input.c
 *
 * LEDs are connected (on STM32F407G-DISC1 Board) as:
 * 	USER LED1 	(LD4 GREEN)		->	PD12
 *	USER LED2 	(LD3 ORANGE)	->  PD13
 *	USER LED3 	(LD5 RED)		->	PD14
 *	USER LED4 	(LD6 BLUE)		->	PD15
 *
 *	USER Button (B1)			-> 	PA0
 *
 */

/* -- Device Specific header file -- */
#include "stm32f407xx.h"

/* -- GPIO Drivers header file -- */
#include "stm32f407xx_gpio_drivers.h"

/* -- Software Delay -- */
void softDelay(void);

int main(void)
{
	/* -- Variable for GPIO Handle Structure: for LED -- */
	GPIO_Handle_t gpio_led;

	/* -- Variable for GPIO Handle Structure: for Button -- */
	GPIO_Handle_t gpio_button;

/* ------------------------------- GPIO LED Pin CONFIGURATIONS -----------------------------------------*/
	/* -- Now, initialize the structure variable: gpio_led -- */

	// Select Port
	gpio_led.pGPIOx = GPIOD;

	// Pin Configuration: PIN NUMBER
	gpio_led.GPIO_PinConfig.GPIO_PinNumber = GPIO_Pin_12;

	// Pin Configuration: PIN MODE
	gpio_led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;

	// Pin Configuration: GPIO SPEED (Not important in this case)
	gpio_led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_VERY_HIGH;

	// Pin Configuration: OUTPUT TYPE (Selected -> PUSH-PULL)
	gpio_led.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;

	// Pin Configuration: PU-PD Configuration (Selected -> NO PU PD)
	gpio_led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	/* -- Before Calling GPIO Init, Enable Peripheral Clock -- */
	GPIO_PeriClockControl(GPIOD, ENABLE);

	/* -- Call GPIO Initialization Function and pass address of handle variable as input argument -- */
	GPIO_Init(&gpio_led);

/* ------------------------------- GPIO Button Pin CONFIGURATIONS -----------------------------------------*/
	// Select Port
	gpio_button.pGPIOx = GPIOA;

	// Pin Configuration: PIN NUMBER
	gpio_button.GPIO_PinConfig.GPIO_PinNumber = GPIO_Pin_0;

	// Pin Configuration: PIN MODE
	gpio_button.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;

	// Pin Configuration: GPIO SPEED (Not important in this case)
	gpio_button.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_VERY_HIGH;

	// Pin Configuration: PU-PD Configuration (Selected ->NO PU PD, As PIN is already PULLED-DOWN -> Schematics)
	gpio_button.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	/* -- Before Calling GPIO Init, Enable Peripheral Clock -- */
	GPIO_PeriClockControl(GPIOA, ENABLE);

	/* -- Call GPIO Initialization Function and pass address of handle variable as input argument -- */
	GPIO_Init(&gpio_button);

	while(1)
	{
		uint8_t buttonState = GPIO_ReadFromInputPin(GPIOA, GPIO_Pin_0);
		if (buttonState)
		{
			softDelay();	// For De-bouncing
			GPIO_ToggleOutputPin(GPIOD, GPIO_Pin_12);
		}
	}

}


/* -- Software Delay Implementation -- */
void softDelay(void)
{
	for(uint32_t i = 0; i < 500000; i++);
}

