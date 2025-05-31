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
	uint8_t SPI_CRCL;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
} SPI_Config_t;

typedef struct {
	SPI_REG_t *pSPIx;
	SPI_Config_t SPI_Config;
} SPIx_Handle_t;

/* SPI MODE */
#define SPI_DEVICE_MODE_S	0
#define SPI_DEVICE_MODE_M	1


/* SPI BUS */
#define SPI_BUS_FULL_DUPLEX	1
//#define SPI_BUS_HALF_DUPLEX	2
#define SPI_BUS_SIMPLEX_TX_ONLY	3
#define SPI_BUS_SIMPLEX_RX_ONLY	4

/* SPI SCLK SPEED */
#define SPI_SCLK_SPEED_DIV_2	0
#define SPI_SCLK_SPEED_DIV_4	1
#define SPI_SCLK_SPEED_DIV_8	2
#define SPI_SCLK_SPEED_DIV_16	3
#define SPI_SCLK_SPEED_DIV_32	4
#define SPI_SCLK_SPEED_DIV_64	5
#define SPI_SCLK_SPEED_DIV_128	6
#define SPI_SCLK_SPEED_DIV_256	7

/* SPI CRCL */
#define SPI_CRCL_8_BITS		0
#define SPI_CRCL_16_BITS		1

/* SPI CPLO */
#define SPI_CPLO_LOW		0
#define SPI_CPLO_HIGH		1

/* SPI CPHA */
#define SPI_CPHA_LOW		0
#define SPI_CPHA_HIGH		1

/* SPI SSM */
#define SPI_SSM_SW_DI			0
#define SPI_SSM_SW_EN			1


/* SPI flag */
#define SPI_RXNE_FLAG			(1 << SPI_SR_RXNE)
#define SPI_TXE_FLAG			(1 << SPI_SR_TXE)
#define SPI_CHSIDE_FLAG			(1 << SPI_SR_CHSIDE)
#define SPI_UDR_FLAG			(1 << SPI_SR_UDR)
#define SPI_CRCERR_FLAG			(1 << SPI_SR_CRCERR)
#define SPI_MODF_FLAG			(1 << SPI_SR_MODF)
#define SPI_OVR_FLAG			(1 << SPI_SR_OVR)
#define SPI_BUSY_FLAG			(1 << SPI_SR_BSY)
#define SPI_FRE_FLAG			(1 << SPI_SR_FRE)
#define SPI_FRLVL_FLAG			(1 << SPI_SR_FRLVL)
#define SPI_FTLVL_FLAG			(1 << SPI_SR_FTLVL)



/* Clock */
void SPI_PCLK_CTRL(SPI_REG_t *pSPIx, uint8_t EnDi);

/* Init */
void SPI_Init(SPIx_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_REG_t *pSPIx);

/* Data */
void SPI_Transmit_Data(SPI_REG_t *pSPIx, uint8_t *pTx_Buffer, uint32_t Len);
void SPI_Receive_Data(SPI_REG_t *pSPIx, uint8_t *pRx_Buffer, uint32_t Len);

/* IRQ */
void SPI_IRQ_Interrupt_Config(uint8_t IRQNumber, uint8_t EnDi);
void SPI_IRQ_Priority_Config(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQ_Handling(SPIx_Handle_t *pSPIHandle);

/* */
uint8_t SPI_Get_Flag_Status(SPI_REG_t *pSPIx, uint32_t Flag_name);
void SPI_Peri_CTRL(SPI_REG_t *pSPIx, uint8_t EnDi);
void SPI_SSI_Config(SPI_REG_t *pSPIx, uint8_t EnDi);
void SPI_SSOE_Config(SPI_REG_t *pSPIx, uint8_t EnDi);


#endif /* INC_STM32F302X8_SPI_DRIVERS_H_ */
