/*
 * stm32f302x8_spi_drivers.h
 *
 *  Created on: Feb 28, 2025
 *      Author: Hyunbin
 */

#ifndef INC_STM32F302X8_SPI_DRIVERS_H_
#define INC_STM32F302X8_SPI_DRIVERS_H_

#include "stm32f302x8.h"

typedef struct {
	uint8_t SPI_Device_Mode;
	uint8_t SPI_Bus_Config;
	uint8_t SPI_SCLK_Speed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
} SPI_Config_t;

typedef struct {
	SPI_REG_t *pSPIx;
	SPI_Config_t SPI_Config;
} SPIx_Handle_t;

/* Clock */
void SPI_PCLK_CTRL(SPI_REG_t *pSPIx, uint8_t EnDi);

/* Init */
void SPI_Init(SPIx_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_REG_t *pSPIx);

/* Data */
void SPI_Transmit_Data(SPI_REG_t *pSPIx, uint8_t *pTx_Buffer, uint32_t Len);
void SPI_Receive_Data(SPI_REG_t *pSPIx, uint8_t *pTx_Buffer, uint32_t Len);

/* IRQ */
void SPI_IRQ_Interrupt_Config(uint8_t IRQNumber, uint8_t EnDi);
void SPI_IRQ_Priority_Config(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQ_Handling(SPIx_Handle_t *pSPIHandle);


#endif /* INC_STM32F302X8_SPI_DRIVERS_H_ */
