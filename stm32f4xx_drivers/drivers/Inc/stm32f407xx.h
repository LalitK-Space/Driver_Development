/*
 * 									stm32f407xx.h
 *
 * This file Contains all the micro-controller specific details:
 * 	- Base addresses of various Memories
 * 	- Base addresses of Buses
 * 	- Base addresses of Peripherals
 * 	- Clock Management MACROS
 * 	- IRQ definitions
 * 	- Peripheral Registers definition structures
 * 	- Peripheral Register bit definition
 * 	- Other useful micro-controller configuration MACROS
 *
 *	- NOTE (Important) - :
 * A new folder named 'drivers' is created with subfolders 'Inc' and 'Src'to keep the drivers' specific header and source file.
 * To include this 'driver > Src' folder in the build process, this folder must be manually added to the build path.
 * To do so (in eclipse-based IDEs):
 * - Firstly, in the project's properties.
			In the C/C++ build, make sure 'exclude resources from the build' is unchecked.
   - To include the path if 'drivers > Inc,' follow the following steps:
            Go to Project Properties -> C/C++ Build -> Settings -> MCU GCC Compiler -> Include Path
            							->  Add new include path of folder -> (select) Workspace
            							-> Select project  -> Select 'Inc' in 'drivers' folder
            							-> (click) OK
 * Now, This new folder will be included in the project's build process.
*/

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>

// To define NULL
#include <stddef.h>


/* -- Processor Specific Details -- */

// ARM Cortex Mx NVIC_ISERx (Interrupt Set Enable) Registers Addresses
#define NVIC_ISER0					((volatile uint32_t *)0xE000E100)
#define NVIC_ISER1					((volatile uint32_t *)0xE000E104)
#define NVIC_ISER2					((volatile uint32_t *)0xE000E108)
#define NVIC_ISER3					((volatile uint32_t *)0xE000E10C) // up-to ISER7

// ARM Cortex Mx NVIC_ICERx (Interrupt Clear Enable) Registers Addresses
#define NVIC_ICER0					((volatile uint32_t *)0XE000E180)
#define NVIC_ICER1					((volatile uint32_t *)0XE000E184)
#define NVIC_ICER2					((volatile uint32_t *)0XE000E188)
#define NVIC_ICER3					((volatile uint32_t *)0XE000E18C) // up-to ICER7

// ARM Cortex Mx NVIC_IPRx (Interrupt Priority) Register Address
#define NVIC_PRI_BASEADDR			((volatile uint32_t *)0xE000E400)

// Number of Priority Bit Implemented
#define PRI_BITS_IMPLEMENTED		4

/* -- Base Addresses of Memories -- */
#define FLASH_BASEADDR				0x08000000U
#define SRAM1_BASEADDR				0x20000000U				// 112 KB
#define SRAM2_BASEADDR				0x20001C00U				// 16 KB
#define SRAM					SRAM1_BASEADDR
#define ROM_BASEADDR				0x1FFF0000U  		 		// System Memory


/* -- Base Addresses of Bus Domains: AHBx and APBx Bus Peripheral Base Addresses -- */
#define PERIPH_BASEADDR				0x40000000U			 	// Peripheral Base Address
#define APB1PERIPH_BASEADDR        		PERIPH_BASEADDR				// APB1 base = Peripheral base
#define APB2PERIPH_BASEADDR 			0x40010000U				// Peripheral Base + Offset 0x00010000
#define AHB1PERIPH_BASEADDR			0x40020000U			 	// Peripheral Base + Offset 0x00020000
#define AHB2PERIPH_BASEADDR			0x50000000U			 	// Peripheral Base + Offset 0x10000000

/* -- Base Addresses of peripherals on AHB1 Bus -- */
#define GPIOA_BASEADDR				((AHB1PERIPH_BASEADDR) + (0x0000)) 	// AHB1PERIPH_BASE + Offset
#define GPIOB_BASEADDR				((AHB1PERIPH_BASEADDR) + (0x0400))
#define GPIOC_BASEADDR				((AHB1PERIPH_BASEADDR) + (0x0800))
#define GPIOD_BASEADDR				((AHB1PERIPH_BASEADDR) + (0x0C00))
#define GPIOE_BASEADDR				((AHB1PERIPH_BASEADDR) + (0x1000))
#define GPIOF_BASEADDR				((AHB1PERIPH_BASEADDR) + (0x1400))
#define GPIOG_BASEADDR				((AHB1PERIPH_BASEADDR) + (0x1800))
#define GPIOH_BASEADDR				((AHB1PERIPH_BASEADDR) + (0x1C00))
#define GPIOI_BASEADDR				((AHB1PERIPH_BASEADDR) + (0x2000))

#define RCC_BASEADDR				((AHB1PERIPH_BASEADDR) + (0x3800))

/* -- Base Addresses of peripherals on APB1 Bus -- */
#define I2C1_BASEADDR				((APB1PERIPH_BASEADDR) + (0x5400)) 	// APB1PERIPH_BASE + Offset
#define I2C2_BASEADDR				((APB1PERIPH_BASEADDR) + (0x5800))
#define I2C3_BASEADDR				((APB1PERIPH_BASEADDR) + (0x5C00))

#define SPI2_BASEADDR				((APB1PERIPH_BASEADDR) + (0x3800))
#define SPI3_BASEADDR				((APB1PERIPH_BASEADDR) + (0x3C00))

#define USART2_BASEADDR				((APB1PERIPH_BASEADDR) + (0x4400))
#define USART3_BASEADDR				((APB1PERIPH_BASEADDR) + (0x4800))
#define UART4_BASEADDR				((APB1PERIPH_BASEADDR) + (0x4C00))
#define UART5_BASEADDR				((APB1PERIPH_BASEADDR) + (0x5000))

/* -- Base Addresses of peripherals on APB2 Bus -- */
#define EXTI_BASEADDR				((APB2PERIPH_BASEADDR) + (0x3C00)) 	// APB2PERIPH_BASE + Offset
#define SPI1_BASEADDR				((APB2PERIPH_BASEADDR) + (0x3000))
#define SPI4_BASEADDR				((APB2PERIPH_BASEADDR) + (0x3400))
#define SYSCFG_BASEADDR				((APB2PERIPH_BASEADDR) + (0x3800))
#define USART1_BASEADDR				((APB2PERIPH_BASEADDR) + (0x1000))
#define USART6_BASEADDR				((APB2PERIPH_BASEADDR) + (0x1400))


/* -- Peripheral Register Definition Structure -- */

// Generic Registers Structure for all GPIOs (Some registers may be highly volatile. That's why 'volatile' is used)
typedef struct
{
	volatile uint32_t MODER;	/* - port mode register 								  - Offset :0x00 */
	volatile uint32_t OTYPER;	/* - port output type register 								  - Offset :0x04 */
	volatile uint32_t OSPEEDR;	/* - port output speed register 							  - Offset :0x08 */
	volatile uint32_t PUPDR;	/* - port pull-up/pull-down register 							  - Offset :0x0C */
	volatile uint32_t IDR;		/* - port input data register 								  - Offset :0x10 */
	volatile uint32_t ODR;		/* - port output data register 								  - Offset :0x14 */
	volatile uint32_t BSRR;		/* - port bit set/reset register 							  - Offset :0x18 */
	volatile uint32_t LCKR;		/* - port configuration lock register 							  - Offset :0x1C */
	volatile uint32_t AFR[2];	/* - alternate function,[0] - LOW Registers [1] - HIGH Registers  - Offset :0x20-0x24 */

}GPIO_RegDef_t;

// Structure for RCC peripheral Registers (Reset and Clock Control)
typedef struct
{
	volatile uint32_t CR;		   /* - clock control register 								 - Offset :0x00 */
	volatile uint32_t PLLCFGR;	   /* - PLL configuration register 							 - Offset :0x04 */
	volatile uint32_t CFGR;		   /* - clock configuration register 							 - Offset :0x08 */
	volatile uint32_t CIR;		   /* - clock interrupt register 							 - Offset :0x0C */
	volatile uint32_t AHB1RSTR;	   /* - AHB1 peripheral reset register 							 - Offset :0x10 */
	volatile uint32_t AHB2RSTR;	   /* - AHB2 peripheral reset register 							 - Offset :0x14 */
	volatile uint32_t AHB3RSTR;        /* - AHB3 peripheral reset register 							 - Offset :0x18 */
	volatile uint32_t RESERVED0;  	   /* - RESERVED	 								 - Offset :0x1C */
	volatile uint32_t APB1RSTR;	   /* - APB1 peripheral reset register 							 - Offset :0x20 */
	volatile uint32_t APB2RSTR;	   /* - APB2 peripheral reset register 							 - Offset :0x24 */
	volatile uint32_t RESERVED1[2];	   /* - RESERVED 									 - Offset :0x28-0x2C */
	volatile uint32_t AHB1ENR;	   /* - AHB1 peripheral clock register							 - Offset :0x30 */
	volatile uint32_t AHB2ENR;	   /* - AHB2 peripheral clock enable register					  	 - Offset :0x34 */
	volatile uint32_t AHB3ENR;	   /* - AHB3 peripheral clock enable register					  	 - Offset :0x38 */
	volatile uint32_t RESERVED2;   	   /* - RESERVED									 - Offset :0x3C */
	volatile uint32_t APB1ENR;	   /* - APB1 peripheral clock enable register					   	 - Offset :0x40 */
	volatile uint32_t APB2ENR;	   /* - APB2 peripheral clock enable register					    	 - Offset :0x44 */
	volatile uint32_t RESERVED3[2];	   /* - RESERVED									 - Offset :0x48-0x4C */
	volatile uint32_t AHB1LPENR;  	   /* - AHB1 peripheral clock enable in low power mode register	  			 - Offset :0x50 */
	volatile uint32_t AHB2LPENR;   	   /* - AHB2 peripheral clock enable in low power mode register	 			 - Offset :0x54 */
	volatile uint32_t AHB3LPENR;       /* - AHB3 peripheral clock enable in low power mode register	 			 - Offset :0x58 */
	volatile uint32_t RESERVED4;       /* - RESERVED									 - Offset :0x5C */
	volatile uint32_t APB1LPENR;       /* - APB1 peripheral clock enable in low power mode register	 			 - Offset :0x60 */
	volatile uint32_t APB2LPENR;       /* - APB2 peripheral clock enabled in low power mode register 			 - Offset :0x64 */
	volatile uint32_t RESERVED5[2];    /* - RESERVED									 - Offset :0x68-0x6C */
	volatile uint32_t BDCR;		   /* - Backup domain control register							 - Offset :0x70 */
	volatile uint32_t CSR;		   /* - clock control & status register							 - Offset :0x74 */
	volatile uint32_t RESERVED6[2];    /* - RESERVED									 - Offset :0x78-0x7C */
	volatile uint32_t SSCGR;	   /* - spread spectrum clock generation register				 	 - Offset :0x80 */
	volatile uint32_t PLLI2SCFGR;      /* - PLLI2S configuration register							 - Offset :0x84 */
	volatile uint32_t PLLSAICFGR;      /* - PLL configuration register							 - Offset :0x88 */
	volatile uint32_t DCKCFGR;	   /* - Dedicated Clock Configuration Register					 	 - Offset :0x8C */

}RCC_RegDef_t;

// Structure for EXTI peripheral Registers (External interrupt)
typedef struct
{
	volatile uint32_t IMR;		   /* - Interrupt Mask Register 							 - Offset :0x00 */
	volatile uint32_t EMR;		   /* - Event Mask Register 								 - Offset :0x04 */
	volatile uint32_t RTSR;		   /* - Rising Trigger Selection Register						 - Offset :0x08 */
	volatile uint32_t FTSR;		   /* - Falling Trigger Selection Register						 - Offset :0x0C */
	volatile uint32_t SWIER;	   /* - Software Interrupt Event Register						 - Offset :0x10 */
	volatile uint32_t PR;		   /* - Pending Register	 							 - Offset :0x14 */

}EXTI_RegDef_t;

// Structure for SYSCFG peripheral Registers (System Configuration)
typedef struct
{
	volatile uint32_t MEMRMP;	   /* - Memory Re-map Register  							 - Offset :0x00 */
	volatile uint32_t PMC;		   /* - Peripheral Mode Configuration Register					  	 - Offset :0x04 */
	volatile uint32_t EXTICR[4];   	   /* - External Interrupt Configuration Registers [1:4]     	 			 - Offset :0x08-0x14 */
	volatile uint32_t RESERVED1[2];    /* - RESERVED									 - Offset :0x18-0x1C */
	volatile uint32_t CMPCR;	   /* - Compensation Cell Control Register						 - Offset :0x20 */
	volatile uint32_t RESERVED2[2];    /* - RESERVED			 						 - Offset :0x14 */
	volatile uint32_t CFGR;		   /* - x					 					 - Offset :0x20 */

}SYSCFG_RegDef_t;

// Generic Registers Structure for all SPI Peripherals
typedef struct
{
	volatile uint32_t CR1;		/* - Control Register 1 								- Offset :0x00 */
	volatile uint32_t CR2;		/* - Control Register 2		 							- Offset :0x04 */
	volatile uint32_t SR;		/* - Status Register			 						- Offset :0x08 */
	volatile uint32_t DR;		/* - Data Register 									- Offset :0x0C */
	volatile uint32_t CRCPR;	/* - CRC Polynomial Register 								- Offset :0x10 */
	volatile uint32_t RXCRCR;	/* - RX CRC Register		 							- Offset :0x14 */
	volatile uint32_t TXCRCR;	/* - TX CRC Register 									- Offset :0x18 */
	volatile uint32_t I2SCFGR;	/* - I2S Configuration Register	 							- Offset :0x1C */
	volatile uint32_t I2SPR;	/* - I2S Pre-scalar Register								- Offset :0x20 */

}SPI_RegDef_t;

// Generic Registers Structure for all I2C Peripherals
typedef struct
{
	volatile uint32_t CR1;		/* - Control Register 1 								- Offset :0x00 */
	volatile uint32_t CR2;		/* - Control Register 2		 							- Offset :0x04 */
	volatile uint32_t OAR1;		/* - Own Address Register 1			 					- Offset :0x08 */
	volatile uint32_t OAR2;		/* - Own Address Register 2 								- Offset :0x0C */
	volatile uint32_t DR;		/* - Data Register 									- Offset :0x10 */
	volatile uint32_t SR1;		/* - Status Register 1		 							- Offset :0x14 */
	volatile uint32_t SR2;		/* - Status Register 2 									- Offset :0x18 */
	volatile uint32_t CCR;		/* - Clock Control Register	 							- Offset :0x1C */
	volatile uint32_t TRISE;	/* - TRISE Register									- Offset :0x20 */
	volatile uint32_t FLTR;		/* - FLTR Register									- Offset :0x24 */

}I2C_RegDef_t;

// Generic Registers Structure for all USART Peripherals
typedef struct
{
	volatile uint32_t SR;		/* - Status Register									- Offset :0x00 */
	volatile uint32_t DR;		/* - Data Register		 							- Offset :0x04 */
	volatile uint32_t BRR;		/* - Baud Rate Register			 						- Offset :0x08 */
	volatile uint32_t CR1;		/* - Control Register 1 								- Offset :0x0C */
	volatile uint32_t CR2;		/* - Control Register 2 								- Offset :0x10 */
	volatile uint32_t CR3;		/* - Control Register 3		 							- Offset :0x14 */
	volatile uint32_t GTPR;		/* - Guard Time and Prescaler Register 							- Offset :0x18 */

}USART_RegDef_t;

/* -- Peripheral Definitions (Peripheral Base Address type-casted to x_RegDef_t) -- */

// For GPIO
#define GPIOA					((GPIO_RegDef_t *)GPIOA_BASEADDR)
#define GPIOB					((GPIO_RegDef_t *)GPIOB_BASEADDR)
#define GPIOC					((GPIO_RegDef_t *)GPIOC_BASEADDR)
#define GPIOD					((GPIO_RegDef_t *)GPIOD_BASEADDR)
#define GPIOE					((GPIO_RegDef_t *)GPIOE_BASEADDR)
#define GPIOF					((GPIO_RegDef_t *)GPIOF_BASEADDR)
#define GPIOG					((GPIO_RegDef_t *)GPIOG_BASEADDR)
#define GPIOH					((GPIO_RegDef_t *)GPIOH_BASEADDR)
#define GPIOI					((GPIO_RegDef_t *)GPIOI_BASEADDR)

// For RCC
#define RCC					((RCC_RegDef_t *)RCC_BASEADDR)

// For EXTI
#define EXTI					((EXTI_RegDef_t *)EXTI_BASEADDR)

// For SYSCFG
#define SYSCFG					((SYSCFG_RegDef_t *)SYSCFG_BASEADDR)

// For SPI
#define SPI1					((SPI_RegDef_t *)SPI1_BASEADDR)
#define SPI2					((SPI_RegDef_t *)SPI2_BASEADDR)
#define SPI3					((SPI_RegDef_t *)SPI3_BASEADDR)
#define SPI4					((SPI_RegDef_t *)SPI4_BASEADDR)

// For I2C
#define I2C1					((I2C_RegDef_t *)I2C1_BASEADDR)
#define I2C2					((I2C_RegDef_t *)I2C2_BASEADDR)
#define I2C3					((I2C_RegDef_t *)I2C3_BASEADDR)

// For USART
#define USART1					((USART_RegDef_t *)USART1_BASEADDR)
#define USART2					((USART_RegDef_t *)USART2_BASEADDR)
#define USART3					((USART_RegDef_t *)USART3_BASEADDR)
#define UART4					((USART_RegDef_t *)UART4_BASEADDR)
#define UART5					((USART_RegDef_t *)UART5_BASEADDR)
#define USART6					((USART_RegDef_t *)USART6_BASEADDR)


/* -- Peripheral Clock Enable and Disable MACROS  --*/

// Clock Enable MACROS for GPIOx Peripherals
#define GPIOA_PCLK_EN()			(RCC -> AHB1ENR |= (1 << 0))		// SET 0th Bit to enable
#define GPIOB_PCLK_EN()			(RCC -> AHB1ENR |= (1 << 1))		// SET 1st Bit to enable
#define GPIOC_PCLK_EN()			(RCC -> AHB1ENR |= (1 << 2))		// SET 2nd Bit to enable
#define GPIOD_PCLK_EN()			(RCC -> AHB1ENR |= (1 << 3))		// SET 3rd Bit to enable
#define GPIOE_PCLK_EN()			(RCC -> AHB1ENR |= (1 << 4))		// SET 4th Bit to enable
#define GPIOF_PCLK_EN()			(RCC -> AHB1ENR |= (1 << 5))		// SET 5th Bit to enable
#define GPIOG_PCLK_EN()			(RCC -> AHB1ENR |= (1 << 6))		// SET 6th Bit to enable
#define GPIOH_PCLK_EN()			(RCC -> AHB1ENR |= (1 << 7))		// SET 7th Bit to enable
#define GPIOI_PCLK_EN()			(RCC -> AHB1ENR |= (1 << 8))		// SET 8th Bit to enable


// Clock Enable MACROS for I2Cx Peripherals
#define I2C1_PCLK_EN()			(RCC -> APB1ENR |= (1 << 21))		// SET 21st Bit to enable
#define I2C2_PCLK_EN()			(RCC -> APB1ENR |= (1 << 22))		// SET 22nd Bit to enable
#define I2C3_PCLK_EN()			(RCC -> APB1ENR |= (1 << 23))		// SET 23rd Bit to enable

// Clock Enable MACROS for SPIx Peripherals
#define SPI1_PCLK_EN()			(RCC -> APB2ENR |= (1 << 12))		// SET 12th Bit to enable
#define SPI2_PCLK_EN()			(RCC -> APB1ENR |= (1 << 14))		// SET 14th Bit to enable
#define SPI3_PCLK_EN()			(RCC -> APB1ENR |= (1 << 15))		// SET 15th Bit to enable
#define SPI4_PCLK_EN()			(RCC -> APB2ENR |= (1 << 13))		// SET 13th Bit to enable


// Clock Enable MACROS for USARTx Peripherals
#define USART1_PCLK_EN()		(RCC -> APB2ENR |= (1 << 4))		// SET 4th Bit to enable
#define USART2_PCLK_EN()		(RCC -> APB1ENR |= (1 << 17))		// SET 17th Bit to enable
#define USART3_PCLK_EN()		(RCC -> APB1ENR |= (1 << 18))		// SET 18th Bit to enable
#define UART4_PCLK_EN()			(RCC -> APB1ENR |= (1 << 19))		// SET 19th Bit to enable
#define UART5_PCLK_EN()			(RCC -> APB1ENR |= (1 << 20))		// SET 20th Bit to enable
#define USART6_PCLK_EN()		(RCC -> APB2ENR |= (1 << 5))		// SET 5th Bit to enable

// Clock Enable MACROS for SYSCFG Peripheral
#define SYSCFG_EN()			(RCC -> APB2ENR |= (1 << 14))		// SET 14th Bit to enable

// Clock Disable MACROS for GPIOx Peripherals
#define GPIOA_PCLK_DI()			(RCC -> AHB1ENR &= ~(1 << 0))		// CLEAR 0th Bit to disable
#define GPIOB_PCLK_DI()			(RCC -> AHB1ENR &= ~(1 << 1))		// CLEAR 1st Bit to disable
#define GPIOC_PCLK_DI()			(RCC -> AHB1ENR &= ~(1 << 2))		// CLEAR 2nd Bit to disable
#define GPIOD_PCLK_DI()			(RCC -> AHB1ENR &= ~(1 << 3))		// CLEAR 3rd Bit to disable
#define GPIOE_PCLK_DI()			(RCC -> AHB1ENR &= ~(1 << 4))		// CLEAR 4th Bit to disable
#define GPIOF_PCLK_DI()			(RCC -> AHB1ENR &= ~(1 << 5))		// CLEAR 5th Bit to disable
#define GPIOG_PCLK_DI()			(RCC -> AHB1ENR &= ~(1 << 6))		// CLEAR 6th Bit to disable
#define GPIOH_PCLK_DI()			(RCC -> AHB1ENR &= ~(1 << 7))		// CLEAR 7th Bit to disable
#define GPIOI_PCLK_DI()			(RCC -> AHB1ENR &= ~(1 << 8))		// CLEAR 8th Bit to disable

// Clock Disable MACROS for I2Cx Peripherals
#define I2C1_PCLK_DI()			(RCC -> APB1ENR &= ~(1 << 21))		// CLEAR 21st Bit to disable
#define I2C2_PCLK_DI()			(RCC -> APB1ENR &= ~(1 << 22))		// CLEAR 22nd Bit to disable
#define I2C3_PCLK_DI()			(RCC -> APB1ENR &= ~(1 << 23))		// CLEAR 23rd Bit to disable

// Clock Disable MACROS for SPIx Peripherals
#define SPI1_PCLK_DI()			(RCC -> APB2ENR &= ~(1 << 12))		// CLEAR 12th Bit to disable
#define SPI2_PCLK_DI()			(RCC -> APB1ENR &= ~(1 << 14))		// CLEAR 14th Bit to disable
#define SPI3_PCLK_DI()			(RCC -> APB1ENR &= ~(1 << 15))		// CLEAR 15th Bit to disable
#define SPI4_PCLK_DI()			(RCC -> APB2ENR &= ~(1 << 13))		// CLEAR 13th Bit to disable

// Clock Disable MACROS for USARTx Peripherals
#define USART1_PCLK_DI()		(RCC -> APB2ENR &= ~(1 << 4))		// CLEAR 4th Bit to disable
#define USART2_PCLK_DI()		(RCC -> APB1ENR &= ~(1 << 17))		// CLEAR 17th Bit to disable
#define USART3_PCLK_DI()		(RCC -> APB1ENR &= ~(1 << 18))		// CLEAR 18th Bit to disable
#define UART4_PCLK_DI()			(RCC -> APB1ENR &= ~(1 << 19))		// CLEAR 19th Bit to disable
#define UART5_PCLK_DI()			(RCC -> APB1ENR &= ~(1 << 20))		// CLEAR 20th Bit to disable
#define USART6_PCLK_DI()		(RCC -> APB2ENR &= ~(1 << 5))		// CLEAR 5th Bit to disable

// Clock Disable MACROS for SYSCFG Peripheral
#define SYSCFG_DI()			(RCC -> APB2ENR &= ~(1 << 14))		// CLEAR 14th Bit to disable

/* -- GPIOx Peripheral Reset Macros -- */
#define GPIOA_REG_RESET()		do {(RCC -> AHB1RSTR |= (1 << 0)); (RCC -> AHB1RSTR &= ~(1 << 0)); } while(0)
#define GPIOB_REG_RESET()		do {(RCC -> AHB1RSTR |= (1 << 1)); (RCC -> AHB1RSTR &= ~(1 << 1)); } while(0)
#define GPIOC_REG_RESET()		do {(RCC -> AHB1RSTR |= (1 << 2)); (RCC -> AHB1RSTR &= ~(1 << 2)); } while(0)
#define GPIOD_REG_RESET()		do {(RCC -> AHB1RSTR |= (1 << 3)); (RCC -> AHB1RSTR &= ~(1 << 3)); } while(0)
#define GPIOE_REG_RESET()		do {(RCC -> AHB1RSTR |= (1 << 4)); (RCC -> AHB1RSTR &= ~(1 << 4)); } while(0)
#define GPIOF_REG_RESET()		do {(RCC -> AHB1RSTR |= (1 << 5)); (RCC -> AHB1RSTR &= ~(1 << 5)); } while(0)
#define GPIOG_REG_RESET()		do {(RCC -> AHB1RSTR |= (1 << 6)); (RCC -> AHB1RSTR &= ~(1 << 6)); } while(0)
#define GPIOH_REG_RESET()		do {(RCC -> AHB1RSTR |= (1 << 7)); (RCC -> AHB1RSTR &= ~(1 << 7)); } while(0)
#define GPIOI_REG_RESET()		do {(RCC -> AHB1RSTR |= (1 << 8)); (RCC -> AHB1RSTR &= ~(1 << 8)); } while(0)

/* -- Port Code for given GPIOx Base Address (can be a c function)-- */
#define GPIO_BASEADDR_TO_CODE(x) ((x == GPIOA) ? 0 :\
								  (x == GPIOB) ? 1 :\
								  (x == GPIOC) ? 2 :\
								  (x == GPIOD) ? 3 :\
								  (x == GPIOE) ? 4 :\
								  (x == GPIOF) ? 5 :\
								  (x == GPIOG) ? 6 :\
								  (x == GPIOH) ? 7 :\
								  (x == GPIOI) ? 8 : 0)

/* -- SPI Peripheral Reset Macros -- */
#define SPI1_REG_RESET()		do {(RCC -> APB2RSTR |= (1 << 12)); (RCC -> APB2RSTR &= ~(1 << 12)); } while(0)
#define SPI2_REG_RESET()		do {(RCC -> APB1RSTR |= (1 << 14)); (RCC -> APB1RSTR &= ~(1 << 14)); } while(0)
#define SPI3_REG_RESET()		do {(RCC -> APB1RSTR |= (1 << 15)); (RCC -> APB1RSTR &= ~(1 << 15)); } while(0)
#define SPI4_REG_RESET()		do {(RCC -> APB2RSTR |= (1 << 13)); (RCC -> APB2RSTR &= ~(1 << 13)); } while(0)


/* -- I2C Peripheral Reset Macros -- */
#define I2C1_REG_RESET()		do {(RCC -> APB1RSTR |= (1 << 21)); (RCC -> APB1RSTR &= ~(1 << 21)); } while(0)
#define I2C2_REG_RESET()		do {(RCC -> APB1RSTR |= (1 << 22)); (RCC -> APB1RSTR &= ~(1 << 22)); } while(0)
#define I2C3_REG_RESET()		do {(RCC -> APB1RSTR |= (1 << 23)); (RCC -> APB1RSTR &= ~(1 << 23)); } while(0)


/* -- USART Peripheral Reset Macros -- */
#define USART1_REG_RESET()		do {(RCC -> APB2RSTR |= (1 << 4));  (RCC -> APB2RSTR &= ~(1 << 4));  } while(0)
#define USART2_REG_RESET()		do {(RCC -> APB1RSTR |= (1 << 17)); (RCC -> APB1RSTR &= ~(1 << 17)); } while(0)
#define USART3_REG_RESET()		do {(RCC -> APB1RSTR |= (1 << 18)); (RCC -> APB1RSTR &= ~(1 << 18)); } while(0)
#define UART4_REG_RESET()		do {(RCC -> APB1RSTR |= (1 << 19)); (RCC -> APB1RSTR &= ~(1 << 19)); } while(0)
#define UART5_REG_RESET()		do {(RCC -> APB1RSTR |= (1 << 20)); (RCC -> APB1RSTR &= ~(1 << 20)); } while(0)
#define USART6_REG_RESET()		do {(RCC -> APB2RSTR |= (1 << 5));  (RCC -> APB2RSTR &= ~(1 << 5));  } while(0)


/* -- IRQ Numbers Macros-- */

// For EXTI Lines
#define IRQ_NO_EXTI0			6
#define IRQ_NO_EXTI1			7
#define IRQ_NO_EXTI2			8
#define IRQ_NO_EXTI3			9
#define IRQ_NO_EXTI4			10
#define IRQ_NO_EXTI5_9			23
#define IRQ_NO_EXTI10_15		40

// For SPI
#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2			36
#define IRQ_NO_SPI3			51
#define IRQ_NO_SPI4			84

// For I2C
#define IRQ_NO_I2C1_EV			31
#define IRQ_NO_I2C1_ER			32
#define IRQ_NO_I2C2_EV			33
#define IRQ_NO_I2C2_ER			34
#define IRQ_NO_I2C3_EV			79
#define IRQ_NO_I2C3_ER			80

// For USART
#define IRQ_NO_USART1	    		37
#define IRQ_NO_USART2	    		38
#define IRQ_NO_USART3	    		39
#define IRQ_NO_UART4	    		52
#define IRQ_NO_UART5	   		53
#define IRQ_NO_USART6	    		71

/* -- Bit Position Definitions of SPI Peripheral -- */

// For SPI_CR1 Register
#define SPI_CR1_CPHA			0
#define SPI_CR1_COPL			1
#define SPI_CR1_MSTR			2
#define SPI_CR1_BR			3
#define SPI_CR1_SPE			6
#define SPI_CR1_LSBFIRST		7
#define SPI_CR1_SSI			8
#define SPI_CR1_SSM			9
#define SPI_CR1_RXONLY			10
#define SPI_CR1_DFF			11
#define SPI_CR1_CRCNEXT			12
#define SPI_CR1_CRCEN			13
#define SPI_CR1_BIDIOE			14
#define SPI_CR1_BIDIMODE		15

// For SPI_CR2 Register
#define SPI_CR2_RXDMAEN			0
#define SPI_CR2_TXDMAEN			1
#define SPI_CR2_SSOE			2
#define SPI_CR2_FRF			4
#define SPI_CR2_ERRIE			5
#define SPI_CR2_RXNEIE			6
#define SPI_CR2_TXEIE			7

// For SPI_SR
#define SPI_SR_RXNE			0
#define SPI_SR_TXE			1
#define SPI_SR_CHSIDE			2
#define SPI_SR_UDR			3
#define SPI_SR_CRCERR			4
#define SPI_SR_MODF			5
#define SPI_SR_OVR			6
#define SPI_SR_BSY			7
#define SPI_SR_FRE			8

/* -- Bit Position Definitions of I2C Peripheral -- */

// For I2C_CR1
#define I2C_CR1_PE			0
#define I2C_CR1_SMBUS			1
#define I2C_CR1_SMBTYPE			3
#define I2C_CR1_ENARP			4
#define I2C_CR1_ENPEC			5
#define I2C_CR1_ENGC			6
#define I2C_CR1_NOSTRETCH  		7
#define I2C_CR1_START			8
#define I2C_CR1_STOP			9
#define I2C_CR1_ACK			10
#define I2C_CR1_POS			11
#define I2C_CR1_PEC			12
#define I2C_CR1_ALERT			13
#define I2C_CR1_SWRST			14

// For I2C_CR2
#define I2C_CR2_FREQ			0	// FREQ[5:0]
#define I2C_CR2_ITERREN			8
#define I2C_CR2_ITEVTEN			9
#define I2C_CR2_ITBUFEN			10
#define I2C_CR2_DMAEN			11
#define I2C_CR2_LAST			12

// For I2C_SR1
#define I2C_SR1_SB			0
#define I2C_SR1_ADDR			1
#define I2C_SR1_BTF			2
#define I2C_SR1_ADD10			3
#define I2C_SR1_STOPF			4
#define I2C_SR1_RXNE			6
#define I2C_SR1_TXE			7
#define I2C_SR1_BERR			8
#define I2C_SR1_ARLO			9
#define I2C_SR1_AF			10
#define I2C_SR1_OVR			11
#define I2C_SR1_PECERR			12
#define I2C_SR1_TIMEOUT			14
#define I2C_SR1_SMBALERT		15

// FOR I2C_SR2
#define I2C_SR2_MSL			0
#define I2C_SR2_BUSY			1
#define I2C_SR2_TRA			2
#define I2C_SR2_GENCALL			4
#define I2C_SR2_SMBDEFAULT		5
#define I2C_SR2_SMBHOST			6
#define I2C_SR2_DUALF			7
#define I2C_SR2_PEC			8	// PEC[7:0]

// FOR I2C_CCR
#define I2C_CCR_CCR			0	// CCR[11:0]
#define I2C_CCR_DUTY			14
#define I2C_CCR_FS			15


/* -- Bit Position Definitions of USART Peripheral -- */

// For USART_SR
#define USART_SR_PE			0
#define USART_SR_FE			1
#define USART_SR_NF			2
#define USART_SR_ORE			3
#define USART_SR_IDLE			4
#define USART_SR_RXNE			5
#define USART_SR_TC			6
#define USART_SR_TXE			7
#define USART_SR_LBD			8
#define USART_SR_CTS			9

// For USART_CR1
#define USART_CR1_SBK			0
#define USART_CR1_RWU			1
#define USART_CR1_RE			2
#define USART_CR1_TE			3
#define USART_CR1_IDLEIE		4
#define USART_CR1_RXNEIE		5
#define USART_CR1_TCIE			6
#define USART_CR1_TXEIE			7
#define USART_CR1_PEIE			8
#define USART_CR1_PS			9
#define USART_CR1_PCE			10
#define USART_CR1_WAKE			11
#define USART_CR1_M			12
#define USART_CR1_UE			13
#define USART_CR1_OVER8			15

// For USART_CR2
#define USART_CR2_ADD			0	// ADD[3:0]
#define USART_CR2_LBDL			5
#define USART_CR2_LBDIE			6
#define USART_CR2_LBCL			8
#define USART_CR2_CPHA			9
#define USART_CR2_CPOL			10
#define USART_CR2_CLKEN			11
#define USART_CR2_STOP			12 	// STOP[13:12]
#define USART_CR2_LINEN			14

// For USART_CR3
#define USART_CR3_EIE			0
#define USART_CR3_IREN			1
#define USART_CR3_IRLP			2
#define USART_CR3_HDSEL			3
#define USART_CR3_NACK			4
#define USART_CR3_SCEN			5
#define USART_CR3_DMAR			6
#define USART_CR3_DMAT			7
#define USART_CR3_RTSE			8
#define USART_CR3_CTSE			9
#define USART_CR3_CTSIE			10
#define USART_CR3_ONEBIT		11


/* -- General MACROS -- */
#define ENABLE				1
#define DISABLE				0
#define SET				ENABLE
#define RESET				DISABLE
#define GPIO_PIN_SET			SET
#define GPIO_PIN_RESET			RESET
#define FLAG_SET			SET
#define FLAG_RESET			RESET



#endif /* INC_STM32F407XX_H_ */
