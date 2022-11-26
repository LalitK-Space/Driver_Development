/*
 * 01_Led_toggle.c
 *
 * LEDs are connected as:
 * 	USER LED1 (LD4 GREEN)	->	PD12
 *	USER LED2 (LD3 ORANGE)	->  PD13
 *	USER LED3 (LD5 RED)		->	PD14
 *	USER LED4 (LD6 BLUE)	->	PD15
 *
 *	-> LED Toggling with PUSH-PULL Configuration <-
 *
 */

/* -- Device Specific header file -- */
#include "stm32f407xx.h"

/* -- GPIO Drivers header file -- */
#include "stm32f407xx_gpio_drivers.h"

/* -- Software Delay -- */
// Just to observe the toggling
void softDelay(void);


int main(void)
{
	/* -- Variable for GPIO Handle Structure -- */
	GPIO_Handle_t gpio_led;

	/* -- Now, initialize the structure variable -- */

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

	/* -- REQUIRED Configurations are done -- */
	// Toggle LED
	while(1)
	{
		GPIO_ToggleOutputPin(GPIOD, GPIO_Pin_12);
		softDelay();
	}

	return 0;
}


/* -- Software Delay Implementation -- */
void softDelay(void)
{
	for(uint32_t i = 0; i < 500000; i++);
}
