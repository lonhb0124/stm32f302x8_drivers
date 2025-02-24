/*
 * stm32f302x8.h
 *
 *  Created on: Feb 19, 2025
 *      Author: Hyunbin
 */

#ifndef INC_STM32F302X8_H_
#define INC_STM32F302X8_H_

#include <stdint.h>

#define FLASH_BASE_ADDR		0x08000000U // 32KB to 64KB
#define SRAM_BASE_ADDR		0x20000000U // 16KB
#define ROM_BASE_ADDR		0x1FFFD800U // 8KB System memory

#define PERIPH_BASE_ADDR 	0x40000000U
#define APB1_BASE_ADDR		(PERIPH_BASE_ADDR)
#define APB2_BASE_ADDR		0x40010000U
#define AHB1_BASE_ADDR		0x40020000U
#define AHB2_BASE_ADDR		0x48000000U

/* AHB1 BUS Peripheral */
#define RCC_BASE_ADDR		((AHB1_BASE_ADDR) + (0x1000))


/* AHB2 BUS Peripheral */
#define GPIOA_BASE_ADDR		((AHB2_BASE_ADDR) + (0x0000))
#define GPIOB_BASE_ADDR		((AHB2_BASE_ADDR) + (0x0400))
#define GPIOC_BASE_ADDR		((AHB2_BASE_ADDR) + (0x0800))
#define GPIOD_BASE_ADDR		((AHB2_BASE_ADDR) + (0x0C00))
#define GPIOF_BASE_ADDR		((AHB2_BASE_ADDR) + (0x1400))

/* APB1 BUS Peripheral */
#define SPI2_BASE_ADDR		((PERIPH_BASE_ADDR) + (0x3800))
#define SPI3_BASE_ADDR		((PERIPH_BASE_ADDR) + (0x3C00))

#define USART2_BASE_ADDR	((PERIPH_BASE_ADDR) + (0x4400))
#define USART3_BASE_ADDR	((PERIPH_BASE_ADDR) + (0x4800))

#define I2C1_BASE_ADDR		((PERIPH_BASE_ADDR) + (0x5400))
#define I2C2_BASE_ADDR		((PERIPH_BASE_ADDR) + (0x5800))
#define I2C3_BASE_ADDR		((PERIPH_BASE_ADDR) + (0x7800))

/* APB2 BUS Peripheral */

#define SYSCFG_BASE_ADDR	((APB2_BASE_ADDR) + (0x0000))
#define EXTI_BASE_ADDR		((APB2_BASE_ADDR) + (0x0400))
#define USART1_BASE_ADDR	((APB2_BASE_ADDR) + (0x3800))


/* GPIO structure */
typedef struct {
	volatile uint32_t MODER;
	volatile uint32_t OTYPER;
	volatile uint32_t OSPEEDR;
	volatile uint32_t PUPDR;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSRR;
	volatile uint32_t LCKR;
	volatile uint32_t AFRL;
	volatile uint32_t AFRH;
	volatile uint32_t BRR;
} GPIO_REG_t;

#define GPIOA 				((GPIO_REG_t*) GPIOA_BASE_ADDR)
#define GPIOB 				((GPIO_REG_t*) GPIOB_BASE_ADDR)
#define GPIOC 				((GPIO_REG_t*) GPIOC_BASE_ADDR)
#define GPIOD 				((GPIO_REG_t*) GPIOD_BASE_ADDR)
#define GPIOF 				((GPIO_REG_t*) GPIOF_BASE_ADDR)

/* RCC structure */
typedef struct {
	volatile uint32_t CR;
	volatile uint32_t CFGR;
	volatile uint32_t CIR;
	volatile uint32_t APB2RSTR;
	volatile uint32_t APB1RSTR;
	volatile uint32_t AHBENR;
	volatile uint32_t APB2ENR;
	volatile uint32_t APB1ENR;
	volatile uint32_t BDCR;
	volatile uint32_t CSR;
	volatile uint32_t AHBRSTR;
	volatile uint32_t CFGR2;
	volatile uint32_t CFGR3;
} RCC_REG_t;

#define RCC					((RCC_REG_t*) RCC_BASE_ADDR)

/* I2C structure */
typedef struct {
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t OAR1;
	volatile uint32_t OAR2;
	volatile uint32_t TIMINGR;
	volatile uint32_t TIMEOUTR;
	volatile uint32_t ISR;
	volatile uint32_t ICR;
	volatile uint32_t PECR;
	volatile uint32_t RXDR;
	volatile uint32_t TXDR;
} I2C_REG_t;

#define I2C1					((I2C_REG_t*) I2C1_BASE_ADDR)
#define I2C2					((I2C_REG_t*) I2C2_BASE_ADDR)
#define I2C3					((I2C_REG_t*) I2C3_BASE_ADDR)

/* SPI structure */
typedef struct {
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t SR;
	volatile uint32_t DR;
	volatile uint32_t CRCPR;
	volatile uint32_t RXCRCR;
	volatile uint32_t TXCRCR;
	volatile uint32_t I2SCFGR;
	volatile uint32_t I2SPR;
} SPI_REG_t;

#define SPI2					((SPI_REG_t*) SPI2_BASE_ADDR)
#define SPI3					((SPI_REG_t*) SPI3_BASE_ADDR)

/* USART structure */
typedef struct {
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t CR3;
	volatile uint32_t BRR;
	volatile uint32_t GTPR;
	volatile uint32_t RTOR;
	volatile uint32_t RQR;
	volatile uint32_t ISR;
	volatile uint32_t ICR;
	volatile uint32_t RDR;
	volatile uint32_t TDR;
} USART_REG_t;

#define USART1					((USART_REG_t*) USART1_BASE_ADDR)
#define USART2					((USART_REG_t*) USART2_BASE_ADDR)
#define USART3					((USART_REG_t*) USART3_BASE_ADDR)

/* SYSCFG structure */
typedef struct {
	volatile uint32_t CFGR1;
	uint32_t RESERVED0;
	volatile uint32_t EXTICR1;
	volatile uint32_t EXTICR2;
	volatile uint32_t EXTICR3;
	volatile uint32_t EXTICR4;
	volatile uint32_t CFGR2;
} SYSCFG_REG_t;

#define SYSCFG					((SYSCFG_REG_t*) SYSCFG_BASE_ADDR)

/* Clock Enable & Disable Macros for GPIOx */

#define GPIOA_PCLK_EN()	(RCC->AHBENR |= (1 << 17))
#define GPIOB_PCLK_EN()	(RCC->AHBENR |= (1 << 18))
#define GPIOC_PCLK_EN()	(RCC->AHBENR |= (1 << 19))
#define GPIOD_PCLK_EN()	(RCC->AHBENR |= (1 << 20))
#define GPIOF_PCLK_EN()	(RCC->AHBENR |= (1 << 22))

#define GPIOA_PCLK_DI()	(RCC->AHBENR &= ~(1 << 17))
#define GPIOB_PCLK_DI()	(RCC->AHBENR &= ~(1 << 18))
#define GPIOC_PCLK_DI()	(RCC->AHBENR &= ~(1 << 19))
#define GPIOD_PCLK_DI()	(RCC->AHBENR &= ~(1 << 20))
#define GPIOF_PCLK_DI()	(RCC->AHBENR &= ~(1 << 22))

/* Clock Enable & Disable Macros for I2C */

#define I2C1_PCLK_EN()	(RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()	(RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()	(RCC->APB1ENR |= (1 << 30))

#define I2C1_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 30))

/* Clock Enable & Disable Macros for SPI */
#define SPI2_PCLK_EN()	(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()	(RCC->APB1ENR |= (1 << 15))

#define SPI2_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 15))

/* Clock Enable & Disable Macros for USART */
#define USART1_PCLK_EN()	(RCC->APB2ENR |= (1 << 14))
#define USART2_PCLK_EN()	(RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN()	(RCC->APB1ENR |= (1 << 18))

#define USART1_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 14))
#define USART2_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 18))

/* Clock Enable & Disable Macros for SYSCFG */
#define SYSCFG_PCLK_EN()	(RCC->APB2ENR |= (1 << 0))

#define SYSCFG_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 0))

/* Clock reset Macros for GPIOx */
#define GPIOA_REG_RESET()	do {(RCC->AHBRSTR |= (1 << 17)); (RCC->AHBRSTR &= ~(1 << 17));} while(0)
#define GPIOB_REG_RESET()	do {(RCC->AHBRSTR |= (1 << 18)); (RCC->AHBRSTR &= ~(1 << 18));} while(0)
#define GPIOC_REG_RESET()	do {(RCC->AHBRSTR |= (1 << 19)); (RCC->AHBRSTR &= ~(1 << 19));} while(0)
#define GPIOD_REG_RESET()	do {(RCC->AHBRSTR |= (1 << 20)); (RCC->AHBRSTR &= ~(1 << 20));} while(0)
#define GPIOF_REG_RESET()	do {(RCC->AHBRSTR |= (1 << 22)); (RCC->AHBRSTR &= ~(1 << 22));} while(0)

#define ENABLE 				1
#define DISABLE 			0
#define SET					ENABLE
#define RESET				DISABLE
#define GPIO_PIN_SET		SET
#define GPIO_PIN_RESET		RESET

#endif /* INC_STM32F302X8_H_ */
