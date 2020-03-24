/*
 * stm32f446xx.h
 *
 *  Created on: Feb 11, 2020
 *      Author: $NghiaPham$
 */

#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_

#include <stdint.h>
#include <stddef.h>
#include <stdio.h>

#define ENABLE							1
#define DISABLE							0
#define SET								ENABLE
#define RESET							DISABLE
#define GPIO_PIN_SET					SET
#define GPIO_PIN_RESET					RESET
#define FLAG_SET						SET
#define FLAG_RESET						RESET
#define NO_PR_BITS_IMPLEMENTED			4

#define __vo 							volatile
#define __weak 							__attribute__((weak))
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//								BASE ADDRESSES OF NESTED VECTORED INTERRUPT CONTROLLER								//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*
 * ARM Cortex Mx Processor NVIC ISERx register Addresses
 */
#define NVIC_ISER0          			( (__vo uint32_t*)0xE000E100 )
#define NVIC_ISER1          			( (__vo uint32_t*)0xE000E104 )
#define NVIC_ISER2          			( (__vo uint32_t*)0xE000E108 )
#define NVIC_ISER3         				( (__vo uint32_t*)0xE000E10C )

/*
 * ARM Cortex Mx Processor NVIC ICERx register Addresses
 */
#define NVIC_ICER0 						( (__vo uint32_t*)0XE000E180 )
#define NVIC_ICER1						( (__vo uint32_t*)0XE000E184 )
#define NVIC_ICER2  					( (__vo uint32_t*)0XE000E188 )
#define NVIC_ICER3						( (__vo uint32_t*)0XE000E18C )

/*
 * ARM Cortex Mx Processor Priority Register Address Calculation
 */
#define NVIC_PR_BASE_ADDR 				( (__vo uint32_t*)0xE000E400 )

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//											BASE ADDRESSES OF FLASH AND SRAM										//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define FLASH_BASEADDR					0x08000000U
#define SRAM1_BASEADDR					0x20000000U
#define SRAM2_BASEADDR					0x2001C000U
#define ROM_BASEADDR					0x1FFF0000U
#define SRAM							SRAM1_BASEADDR

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//										AHBx AND APBx BUS PERIPHERAL BASE ADDRESSES									//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define PERIPH_BASE						0x40000000U
#define APB1PERIPH_BASEARR				PERIPH_BASE
#define APB2PERIPH_BASEARR				0x40010000U
#define AHB1PERIPH_BASEARR				0x40020000U
#define AHB2PERIPH_BASEARR				0x50000000U

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//								BASE ADDRESSES OF PERIPHERAL WHICH IS ARE HANGING ON AHB1							//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define GPIOA_BASEADDR					(AHB1PERIPH_BASEARR + 0x0000)
#define GPIOB_BASEADDR					(AHB1PERIPH_BASEARR + 0x0400)
#define GPIOC_BASEADDR					(AHB1PERIPH_BASEARR + 0x0800)
#define GPIOD_BASEADDR					(AHB1PERIPH_BASEARR + 0x0C00)
#define GPIOE_BASEADDR					(AHB1PERIPH_BASEARR + 0x1000)
#define GPIOF_BASEADDR					(AHB1PERIPH_BASEARR + 0x1400)
#define GPIOG_BASEADDR					(AHB1PERIPH_BASEARR + 0x1800)
#define GPIOH_BASEADDR					(AHB1PERIPH_BASEARR + 0x1C00)
#define RCC_BASEADDR					(AHB1PERIPH_BASEARR + 0x3800)

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//				   				BASE ADDRESSES OF PERIPHERAL WHICH IS ARE HANGING ON APB1							//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define I2C1_BASEADDR					(APB1PERIPH_BASEARR + 0X5400)
#define I2C2_BASEADDR					(APB1PERIPH_BASEARR + 0X5800)
#define I2C3_BASEADDR					(APB1PERIPH_BASEARR + 0X5C00)

#define SPI2_BASEADDR					(APB1PERIPH_BASEARR + 0X3800)
#define SPI3_BASEADDR					(APB1PERIPH_BASEARR + 0X3C00)

#define USART2_BASEADDR					(APB1PERIPH_BASEARR + 0X4400)
#define USART3_BASEADDR					(APB1PERIPH_BASEARR + 0X4800)
#define UART4_BASEADDR					(APB1PERIPH_BASEARR + 0X4C00)
#define UART5_BASEADDR					(APB1PERIPH_BASEARR + 0X5000)

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//									BASE ADDRESSES OF PERIPHERAL WHICH IS ARE HANGING ON APB2	 				 	//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define SPI1_BASEADDR					(APB2PERIPH_BASEARR + 0X3000)
#define SPI4_BASEADDR					(APB2PERIPH_BASEARR + 0X3400)

#define USART1_BASEADDR					(APB2PERIPH_BASEARR + 0X1000)
#define USART6_BASEADDR					(APB2PERIPH_BASEARR + 0X1400)

#define EXTI_BASEADDR					(APB2PERIPH_BASEARR + 0X3C00)
#define SYSCFG_BASEADDR					(APB2PERIPH_BASEARR + 0X3800)

//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
//_/									PERIPHERAL REGISTER DEFINITIONS STRUCTURES									_/
//_/							SPI PERIPHERAL OF STM32F446XX FAMILY OF MCUs MAY BE DIFFERENT						_/
//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/

/*
 * GPIO Registers
 */
typedef struct {
	__vo uint32_t MODER;				/*!GPIO port mode register 																			Address offset 0x00											*/
	__vo uint32_t OTYPER;				/*!GPIO port output type register 																	Address offset 0x04											*/
	__vo uint32_t OSPEEDER;				/*!GPIO port output speed register 																	Address offset 0x08											*/
	__vo uint32_t PUPDR;				/*!GPIO port pull-up/pull-down register																Address offset 0x0C											*/
	__vo uint32_t IDR;					/*!GPIO port input data register																	Address offset 0x10											*/
	__vo uint32_t ODR;					/*!GPIO port output data register 																	Address offset 0x14											*/
	__vo uint32_t BSRR;					/*!GPIO port bit set/reset register																	Address offset 0x18											*/
	__vo uint32_t LCKR;					/*!GPIO port configuration lock register															Address offset 0x1C											*/
	__vo uint32_t AFR[2];				/*!AFR[0]: GPIO AF low register, AFR[1]:GPIO AF high register										Address offset AF[0]:0x20  AF[1]:0x24						*/
}GPIO_RegDef_t;

/*
 * RCC Registers
 */
typedef struct {
	__vo uint32_t CR;					/*!RCC clock control register 																		Address offset 0x00											*/
	__vo uint32_t PLLCFGR;				/*!RCC PLL configuration register 																	Address offset 0x04											*/
	__vo uint32_t CFGR;					/*!RCC clock configuration register 																Address offset 0x08											*/
	__vo uint32_t CIR;					/*!RCC clock interrupt register 																	Address offset 0x0C											*/
	__vo uint32_t AHB1RSTR;				/*!RCC AHB1 peripheral reset register 																Address offset 0x10											*/
	__vo uint32_t AHB2RSTR;				/*!RCC AHB2 peripheral reset register 																Address offset 0x14											*/
	__vo uint32_t AHB3RSTR;				/*!RCC AHB3 peripheral reset register 																Address offset 0x18											*/
		 uint32_t RESERVED0;			/*!Reserved 																						Address offset 0x1C											*/
	__vo uint32_t APB1RSTR;				/*!RCC APB1 peripheral reset register 																Address offset 0x20											*/
	__vo uint32_t APB2RSTR;				/*!RCC APB2 peripheral reset register 																Address offset 0x24											*/
		 uint32_t RESERVED1[2];			/*!Reserved 																						Address offset RESERVED1[0]:0x28 RESERVED1[1]:0X2C			*/
	__vo uint32_t AHB1ENR;				/*!RCC AHB1 peripheral clock enable register 														Address offset 0x30											*/
	__vo uint32_t AHB2ENR;				/*!RCC AHB2 peripheral clock enable register 														Address offset 0x34											*/
	__vo uint32_t AHB3ENR;				/*!RCC AHB3 peripheral clock enable register 														Address offset 0x38											*/
		 uint32_t RESERVED2;			/*!Reserved 																						Address offset 0x3C											*/
	__vo uint32_t APB1ENR;				/*!RCC APB1 peripheral clock enable register 														Address offset 0x40											*/
	__vo uint32_t APB2ENR;				/*!RCC APB2 peripheral clock enable register 														Address offset 0x44											*/
		 uint32_t RESERVED3[2];			/*!Reserved 																						Address offset RESERVED3[0]:0x48 RESERVED3[1]:0X4C			*/
	__vo uint32_t AHB1LPENR;			/*!RCC AHB1 peripheral clock enable in low power mode register 										Address offset 0x50											*/
	__vo uint32_t AHB2LPENR;			/*!RCC AHB2 peripheral clock enable in low power mode register 										Address offset 0x54											*/
	__vo uint32_t AHB3LPENR;			/*!RCC AHB3 peripheral clock enable in low power mode register 										Address offset 0x58											*/
	     uint32_t RESERVED4;			/*!Reserved 																						Address offset 0x5C											*/
	__vo uint32_t APB1LPENR;			/*!RCC APB1 peripheral clock enable in low power mode register 										Address offset 0x60											*/
	__vo uint32_t APB2LPENR;			/*!RCC APB2 peripheral clock enabled in low power mode register 									Address offset 0x64											*/
		 uint32_t RESERVED5[2];			/*!Reserved 																						Address offset RESERVED5[0]:0x68 RESERVED5[1]:0X6C			*/
	__vo uint32_t BDCR;					/*!RCC Backup domain control register 																Address offset 0x70											*/
	__vo uint32_t CSR;					/*!RCC clock control & status register 																Address offset 0x74											*/
	     uint32_t RESERVED6[2];			/*!Reserved 																						Address offset RESERVED6[0]:0x78 RESERVED6[1]:0X7C			*/
	__vo uint32_t SSCGR;				/*!RCC spread spectrum clock generation register 													Address offset 0x80											*/
	__vo uint32_t PLLI2SCFGR;			/*!RCC PLLI2S configuration register 																Address offset 0x84											*/
	__vo uint32_t PLLSAICFGR;			/*!RCC PLL configuration register 																	Address offset 0x88											*/
	__vo uint32_t DCKCFGR;				/*!RCC Dedicated Clock Configuration Register 														Address offset 0x8C											*/
	__vo uint32_t CKGATENR;				/*!RCC clocks gated enable register 																Address offset 0x90											*/
	__vo uint32_t DCKCFGR2;				/*!RCC dedicated clocks configuration register 2 													Address offset 0x94											*/

}RCC_RegDef_t;

/*
 * EXTI Registers
 */
typedef struct {
	__vo uint32_t IMR;					/*!Interrupt mask register																			Address offset 0x00											*/
	__vo uint32_t EMR;					/*!Event mask register																				Address offset 0x04											*/
	__vo uint32_t RTSR;					/*!Rising trigger selection register																Address offset 0x08											*/
	__vo uint32_t FTSR;					/*!Falling trigger selection register																Address offset 0x0C											*/
	__vo uint32_t SWIER;				/*!Software interrupt event register																Address offset 0x10											*/
	__vo uint32_t PR;					/*!Pending register																					Address offset 0x14											*/
}EXTI_RegDef_t;

/*
 * SYSCFG Registers
 */
typedef struct {
	__vo uint32_t MEMRMP;				/*!SYSCFG memory remap register																	Address offset 0x00												*/
	__vo uint32_t PMC;					/*!SYSCFG peripheral mode configuration register												Address offset 0x04												*/
	__vo uint32_t EXTICR[4];			/*!SYSCFG external interrupt configuration register 1											Address offset CR1: 0x08 CR2: 0x0C CR3: 0x10 CR4: 0x14			*/
		 uint32_t RESERVED1[2];			/*!Reserved 																					Address offset RESERVED1[0]:0x18 RESERVED1[1]:0X1C				*/
	__vo uint32_t CMPCR;				/*!Compensation cell control register															Address offset 0x20												*/
		 uint32_t RESERVED2[2];			/*!Reserved 																					Address offset RESERVED2[0]:0x24 RESERVED2[1]:0X28				*/
	__vo uint32_t CFGR;					/*!SYSCFG configuration register																Address offset 0x2C												*/
}SYSCFG_RegDef_t;

/*
 * SPI Registers
 */
typedef struct {
	__vo uint32_t CR1;					/*!SPI control register 1																		Address offset 0x00												*/
	__vo uint32_t CR2;					/*!SPI control register 2																		Address offset 0x04												*/
	__vo uint32_t SR;					/*!SPI status register																			Address offset 0x08												*/
	__vo uint32_t DR;					/*!SPI data register																			Address offset 0x0C												*/
	__vo uint32_t CRCPR;				/*!SPI CRC polynomial register																	Address offset 0x10												*/
	__vo uint32_t RXCRCR;				/*!SPI RX CRC register																			Address offset 0x14												*/
	__vo uint32_t TXCRCR;				/*!SPI TX CRC register																			Address offset 0x18												*/
	__vo uint32_t I2SCFGR;				/*!SPI_I2S configuration register 																Address offset 0x1C												*/
	__vo uint32_t I2SPR;				/*!SPI_I2S prescaler register																	Address offset 0x20												*/
}SPI_RegDef_t;

/*
 * I2C Registers
 */
typedef struct {
	__vo uint32_t CR1;					/*!I2C Control register 1																		Address offset 0x00												*/
	__vo uint32_t CR2;					/*!I2C Control register 2																		Address offset 0x04												*/
	__vo uint32_t OAR1;					/*!I2C Own address register 1																	Address offset 0x08												*/
	__vo uint32_t OAR2;					/*!I2C Own address register 2																	Address offset 0x0C												*/
	__vo uint32_t DR;					/*!I2C Data register																			Address offset 0x10												*/
	__vo uint32_t SR1;					/*!I2C Status register 1																		Address offset 0x14												*/
	__vo uint32_t SR2;					/*!I2C Status register 2																		Address offset 0x18												*/
	__vo uint32_t CCR;					/*!I2C Clock control register 																	Address offset 0x1C												*/
	__vo uint32_t TRISE;				/*!I2C TRISE register																			Address offset 0x20												*/
	__vo uint32_t FLTR;					/*!I2C FLTR register																			Address offset 0x24												*/
}I2C_RegDef_t;

/*
 * USART Registers
 */
typedef struct {
	__vo uint32_t SR;         			/*!Status register      																		Address offset: 0x00 											*/
	__vo uint32_t DR;         			/*!Data register     																			Address offset: 0x04 											*/
	__vo uint32_t BRR;        			/*!Baud rate register   																		Address offset: 0x08 											*/
	__vo uint32_t CR1;        			/*!Control register 1     																		Address offset: 0x0C 											*/
	__vo uint32_t CR2;        			/*!Control register 2    																		Address offset: 0x10 											*/
	__vo uint32_t CR3;        			/*!Control register 3   																		Address offset: 0x14 											*/
	__vo uint32_t GTPR;       			/*!Guard time and prescaler register     														Address offset: 0x18 											*/
} USART_RegDef_t;

//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
//_/											 PERIPHERAL DEFINITIONS 											_/
//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/

/*
 * GPIOx Registers
 */
#define GPIOA							((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB							((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC							((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD							((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE							((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF							((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG							((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH							((GPIO_RegDef_t*)GPIOH_BASEADDR)

/*
 * RCC Registers
 */
#define RCC								((RCC_RegDef_t*)RCC_BASEADDR)

/*
 * EXTI Registers
 */
#define EXTI							((EXTI_RegDef_t*)EXTI_BASEADDR)

/*
 * SYSCFG Registers
 */
#define SYSCFG							((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

/*
 * I2C Registers
 */
#define I2C1							((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2							((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3							((I2C_RegDef_t*)I2C3_BASEADDR)

/*
 * SPI Registers
 */
#define SPI1							((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2							((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3							((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4							((SPI_RegDef_t*)SPI4_BASEADDR)

/*
 * USART Registers
 */
#define USART1  						((USART_RegDef_t*)USART1_BASEADDR)
#define USART2  						((USART_RegDef_t*)USART2_BASEADDR)
#define USART3  						((USART_RegDef_t*)USART3_BASEADDR)
#define UART4  							((USART_RegDef_t*)UART4_BASEADDR)
#define UART5  							((USART_RegDef_t*)UART5_BASEADDR)
#define USART6  						((USART_RegDef_t*)USART6_BASEADDR)

//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
//_/										 CLOCK ENABLE MACROs FOR PERIPHERAL 									_/
//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/

/*
 * Clock Enable Macros For GPIOx Peripheral
 */
#define GPIOA_PCLK_EN()					( RCC->AHB1ENR |= ( 1 << 0 ) )
#define GPIOB_PCLK_EN()					( RCC->AHB1ENR |= ( 1 << 1 ) )
#define GPIOC_PCLK_EN()					( RCC->AHB1ENR |= ( 1 << 2 ) )
#define GPIOD_PCLK_EN()					( RCC->AHB1ENR |= ( 1 << 3 ) )
#define GPIOE_PCLK_EN()					( RCC->AHB1ENR |= ( 1 << 4 ) )
#define GPIOF_PCLK_EN()					( RCC->AHB1ENR |=  (1 << 5 ) )
#define GPIOG_PCLK_EN()					( RCC->AHB1ENR |=  (1 << 6 ) )
#define GPIOH_PCLK_EN()					( RCC->AHB1ENR |=  (1 << 7 ) )

/*
 * Clock Enable Macros For SYSCFG Peripheral
 */
#define SYSCFG_PCLK_EN()				( RCC->APB2ENR |= ( 1 << 14 ) )

/*
 * Clock Enable Macros For I2Cx Peripheral
 */
#define I2C1_PCLK_EN()					( RCC->APB1ENR |= ( 1 << 21 ) )
#define I2C2_PCLK_EN()					( RCC->APB1ENR |= ( 1 << 22 ) )
#define I2C3_PCLK_EN()					( RCC->APB1ENR |= ( 1 << 23 ) )

/*
 * Clock Enable Macros For SPIx Peripheral
 */
#define SPI1_PCLK_EN()					( RCC->APB2ENR |= ( 1 << 12 ) )
#define SPI2_PCLK_EN()					( RCC->APB1ENR |= ( 1 << 14 ) )
#define SPI3_PCLK_EN()					( RCC->APB1ENR |= ( 1 << 15 ) )
#define SPI4_PCLK_EN()					( RCC->APB2ENR |= ( 1 << 13 ) )

/*
 * Clock Enable Macros For USARTx Peripheral
 */
#define USART1_PCCK_EN() 				( RCC->APB2ENR |= (1 << 4) )
#define USART2_PCCK_EN() 				( RCC->APB1ENR |= (1 << 17) )
#define USART3_PCCK_EN() 				( RCC->APB1ENR |= (1 << 18) )
#define UART4_PCCK_EN()  				( RCC->APB1ENR |= (1 << 19) )
#define UART5_PCCK_EN()  				( RCC->APB1ENR |= (1 << 20) )
#define USART6_PCCK_EN() 				( RCC->APB2ENR |= (1 << 5) )

//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
//_/									 CLOCK DISABLE MACROs FOR PERIPHERAL										_/
//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/

/*
 * Clock Disable Macros For GPIOx Peripheral
 */
#define GPIOA_PCLK_DI()					( RCC->AHB1ENR &= ~( 1 << 0 ) )
#define GPIOB_PCLK_DI()					( RCC->AHB1ENR &= ~( 1 << 1 ) )
#define GPIOC_PCLK_DI()					( RCC->AHB1ENR &= ~( 1 << 2 ) )
#define GPIOD_PCLK_DI()					( RCC->AHB1ENR &= ~( 1 << 3 ) )

/*
 * Clock Disable Macros For I2Cx Peripheral
 */
#define I2C1_PCLK_DI()					( RCC->APB1ENR &= ~( 1 << 21 ) )
#define I2C2_PCLK_DI()					( RCC->APB1ENR &= ~( 1 << 22 ) )
#define I2C3_PCLK_DI()					( RCC->APB1ENR &= ~( 1 << 23 ) )

/*
 * Clock Disable Macros For SPIx Peripheral
 */
#define SPI1_PCLK_DI()					( RCC->APB2ENR &= ~( 1 << 12 ) )
#define SPI2_PCLK_DI()					( RCC->APB1ENR &= ~( 1 << 14 ) )
#define SPI3_PCLK_DI()					( RCC->APB1ENR &= ~( 1 << 15 ) )
#define SPI4_PCLK_DI()					( RCC->APB2ENR &= ~( 1 << 13 ) )

/*
 * Clock Disable Macros For USARTx Peripheral
 */
#define USART1_PCCK_DI() 				( RCC->APB2ENR &= ~(1 << 4) )
#define USART2_PCCK_DI() 				( RCC->APB1ENR &= ~(1 << 17) )
#define USART3_PCCK_DI() 				( RCC->APB1ENR &= ~(1 << 18) )
#define UART4_PCCK_DI()  				( RCC->APB1ENR &= ~(1 << 19) )
#define UART5_PCCK_DI()  				( RCC->APB1ENR &= ~(1 << 20) )
#define USART6_PCCK_DI() 				( RCC->APB2ENR &= ~(1 << 5) )

//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
//_/								***** CLOCK RESET MACROs FOR PERIPHERAL *****									_/
//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
/*
 * Macros Reset GPIOx Peripheral
 */
#define GPIOA_REG_RESET()				do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET()				do{ (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET()				do{ (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET()				do{ (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); }while(0)

/*
 * Macros Reset I2Cx Peripheral
 */
#define I2C1_REG_RESET()				do{ (RCC->APB1RSTR |= (1 << 21)); (RCC->APB1RSTR &= ~(1 << 21)); }while(0)
#define I2C2_REG_RESET()				do{ (RCC->APB1RSTR |= (1 << 22)); (RCC->APB1RSTR &= ~(1 << 22)); }while(0)
#define I2C3_REG_RESET()				do{ (RCC->APB1RSTR |= (1 << 23)); (RCC->APB1RSTR &= ~(1 << 23)); }while(0)

/*
 * Macros Reset SPIx Peripheral
 */
#define SPI1_REG_RESET()				do{ (RCC->APB2RSTR |= (1 << 12)); (RCC->APB2RSTR &= ~(1 << 12)); }while(0)
#define SPI2_REG_RESET()				do{ (RCC->APB1RSTR |= (1 << 14)); (RCC->APB1RSTR &= ~(1 << 14)); }while(0)
#define SPI3_REG_RESET()				do{ (RCC->APB1RSTR |= (1 << 15)); (RCC->APB1RSTR &= ~(1 << 15)); }while(0)
#define SPI4_REG_RESET()				do{ (RCC->APB2RSTR |= (1 << 13)); (RCC->APB2RSTR &= ~(1 << 13)); }while(0)

/*
 * Macros Reset USARTx Peripheral
 */
#define USART1_REG_RESET()				do{ (RCC->APB2RSTR |= (1 << 4)); (RCC->APB2RSTR &= ~(1 << 12)); }while(0)
#define USART2_REG_RESET()				do{ (RCC->APB1RSTR |= (1 << 17)); (RCC->APB1RSTR &= ~(1 << 12)); }while(0)
#define USART3_REG_RESET()				do{ (RCC->APB1RSTR |= (1 << 18)); (RCC->APB1RSTR &= ~(1 << 12)); }while(0)
#define UART4_REG_RESET()				do{ (RCC->APB1RSTR |= (1 << 19)); (RCC->APB1RSTR &= ~(1 << 12)); }while(0)
#define UART5_REG_RESET()				do{ (RCC->APB1RSTR |= (1 << 20)); (RCC->APB1RSTR &= ~(1 << 12)); }while(0)
#define USART6_REG_RESET()				do{ (RCC->APB2RSTR |= (1 << 5)); (RCC->APB2RSTR &= ~(1 << 12)); }while(0)

//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
//_/											***** DEFINE MACRO *****											_/
//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/

/*
 * This macro returns a code( between 0 to 7) for a given GPIO base address(x)
 */
#define GPIO_BASEADDR_TO_CODE(x)      ( (x == GPIOA)?0:					\
										(x == GPIOB)?1:					\
										(x == GPIOC)?2:					\
										(x == GPIOD)?3:					\
								        (x == GPIOE)?4:					\
								        (x == GPIOF)?5:					\
								        (x == GPIOG)?6:					\
								        (x == GPIOH)?7:0)

/*
 * IRQ(Interrupt Request) Numbers of STM32F446x MCU
 * NOTE: update these macros with valid values according to your MCU
 */

#define IRQ_NO_EXTI0 					6
#define IRQ_NO_EXTI1 					7
#define IRQ_NO_EXTI2 					8
#define IRQ_NO_EXTI3 					9
#define IRQ_NO_EXTI4 					10
#define IRQ_NO_EXTI9_5 					23
#define IRQ_NO_EXTI15_10 				40

#define IRQ_NO_SPI1						35
#define IRQ_NO_SPI2         			36
#define IRQ_NO_SPI3         			51
#define IRQ_NO_SPI4						84

#define IRQ_NO_I2C1_EV     				31
#define IRQ_NO_I2C1_ER     				32
#define IRQ_NO_I2C2_EV     				33
#define IRQ_NO_I2C2_ER     				34
#define IRQ_NO_I2C3_EV     				79
#define IRQ_NO_I2C3_ER     				80

#define IRQ_NO_USART1	    			37
#define IRQ_NO_USART2	    			38
#define IRQ_NO_USART3	    			39
#define IRQ_NO_UART4	    			52
#define IRQ_NO_UART5	    			53
#define IRQ_NO_USART6	    			71

/*
 * macros for all the possible priority levels
 */
#define NVIC_IRQ_PRI0    				0
#define NVIC_IRQ_PRI1    				1
#define NVIC_IRQ_PRI2    				2
#define NVIC_IRQ_PRI3    				3
#define NVIC_IRQ_PRI4    				4
#define NVIC_IRQ_PRI5    				5
#define NVIC_IRQ_PRI6    				6
#define NVIC_IRQ_PRI7    				7
#define NVIC_IRQ_PRI8    				8
#define NVIC_IRQ_PRI9    				9
#define NVIC_IRQ_PRI15    				15

#endif /* INC_STM32F446XX_H_ */
