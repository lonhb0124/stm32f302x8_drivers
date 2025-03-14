/*
 * stm32f302x8_spi_drivers.c
 *
 *  Created on: Feb 28, 2025
 *      Author: Hyunbin
 */


#include "stm32f302x8_spi_drivers.h"


/* ==========================================================================
 * Function			: SPI_PCLK_CTRL
 *
 * Description		: Enable or Disable peripheral clock for SPI
 *
 * Parameter[in]	: base address of the SPI
 * Parameter[in]	: Enable or Disable
 * Parameter[in]	:
 *
 * Return			: None
 *
 * Note				: N/A
 */
void SPI_PCLK_CTRL(SPI_REG_t *pSPIx, uint8_t EnDi) {

	if (EnDi == ENABLE) {
		if (pSPIx == SPI2) {
			SPI2_PCLK_EN();
		} else if (pSPIx == SPI3) {
			SPI3_PCLK_EN();
		}
	}

	else {
		if (pSPIx == SPI2) {
			SPI2_PCLK_DI();
		} else if (pSPIx == SPI3) {
			SPI3_PCLK_DI();
		}
	}

}



/* ==========================================================================
 * Function			: SPI_Init
 *
 * Description		: Initialize SPI protocol
 *
 * Parameter[in]	: base address of the GPIO
 * Parameter[in]	: Enable or Disable
 * Parameter[in]	:
 *
 * Return			: None
 *
 * Note				: N/A
 */
void SPI_Init(SPIx_Handle_t *pSPIHandle) {

	// peripheral clk enable
	SPI_PCLK_CTRL(pSPIHandle->pSPIx, ENABLE);

	// configure the SPI_CR1 register
	uint32_t temp = 0;

	// 1. Configure the device mode
	temp |= pSPIHandle->SPI_Config.SPI_Device_Mode << 2;

	// 2. Configure the bus
	if (pSPIHandle->SPI_Config.SPI_Bus_Config == SPI_BUS_FULL_DUPLEX) {
		// Full Duplex mode
		temp &= ~(1 << SPI_CR1_BIDI_MODE);
	} else if (pSPIHandle->SPI_Config.SPI_Bus_Config == SPI_BUS_SIMPLEX_RX_ONLY) {
		// Simple receive only
		temp &= ~(1 << SPI_CR1_BIDI_MODE);
		temp |= (1 << SPI_CR1_RX_ONLY);
	}

	else if (pSPIHandle->SPI_Config.SPI_Bus_Config == SPI_BUS_SIMPLEX_TX_ONLY) {
		// Simple transmit only
	}

	// 3. Configure the clock speed
	temp |= pSPIHandle->SPI_Config.SPI_SCLK_Speed << SPI_CR1_BR;

	// 4. Configure the CRCL
	temp |= pSPIHandle->SPI_Config.SPI_CRCL << SPI_CR1_CRCL;

	// 5. Configure the CPOL
	temp |= pSPIHandle->SPI_Config.SPI_CPOL << SPI_CR1_CPOL;

	// 6. Configure the CPHA
	temp |= pSPIHandle->SPI_Config.SPI_CPHA << SPI_CR1_CPHA;

	// 7. Configure the SSM
	temp |= pSPIHandle->SPI_Config.SPI_SSM << SPI_CR1_SSM;

	pSPIHandle->pSPIx->CR1 |= temp;
}



/* ========================================================================
 * Function			: SPI_DeInit
 *==
 * Description		: Reset SPI protocol
 *
 * Parameter[in]	: base address of the SPI
 * Parameter[in]	:
 * Parameter[in]	:
 *
 * Return			: None
 *
 * Note				: N/A
 */
void SPI_DeInit(SPI_REG_t *pSPIx) {

	if (pSPIx == SPI2) {
		SPI2_REG_RESET();
	} else if (pSPIx == SPI3) {
		SPI3_REG_RESET();
	}

}


uint8_t SPI_Get_Flag_Status(SPI_REG_t *pSPIx, uint32_t Flag_name) {

	if (pSPIx->SR & Flag_name) {
		return FLAG_SET;
	}

	return FLAG_RESET;
}

/* ==========================================================================
 * Function			: SPI_Transmit_Data
 *
 * Description		: send data to SPI
 *
 * Parameter[in]	: base address of the GPIO
 * Parameter[in]	: Enable or Disable
 * Parameter[in]	:
 *
 * Return			: None
 *
 * Note				: Blocking call
 */
void SPI_Transmit_Data(SPI_REG_t *pSPIx, uint8_t *pTx_Buffer, uint32_t Len) {

	while (Len > 0) {

		// 1. wait until TXE set
		while(SPI_Get_Flag_Status(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

		// 2. check the DFF bit in CR1
		if (pSPIx->CR1 & (1 << SPI_CR1_CRCL)) {
			// 16 bit
			// load the data from TX buffer to DR
			pSPIx->DR = *((uint16_t*) pTx_Buffer);
			Len--;
			Len--;
			(uint16_t*) pTx_Buffer++;
		} else {
			// 8 bit
			pSPIx->DR = *pTx_Buffer;
			Len--;
			pTx_Buffer++;
		}


	}

}



/* ==========================================================================
 * Function			: SPI_Receive_Data
 *
 * Description		: Reset GPIO port
 *
 * Parameter[in]	: base address of the GPIO
 * Parameter[in]	: Enable or Disable
 * Parameter[in]	:
 *
 * Return			: None
 *
 * Note				: N/A
 */
void SPI_Receive_Data(SPI_REG_t *pSPIx, uint8_t *pRx_Buffer, uint32_t Len) {

	while (Len > 0) {

		// 1. wait until RXNE set
		while(SPI_Get_Flag_Status(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);

		// 2. check the DFF bit in CR1
		if (pSPIx->CR1 & (1 << SPI_CR1_CRCL)) {
			// 16 bit
			// load the data from DR to RX buffer
			*((uint16_t*) pRx_Buffer) = pSPIx->DR;
			Len--;
			Len--;
			(uint16_t*) pRx_Buffer++;
		} else {
			// 8 bit
			*pRx_Buffer = pSPIx->DR;
			Len--;
			pRx_Buffer++;
		}


	}
}


/* ==========================================================================
 * Function			: GPIO_DeInit
 *
 * Description		: Reset GPIO port
 *
 * Parameter[in]	: base address of the GPIO
 * Parameter[in]	: Enable or Disable
 * Parameter[in]	:
 *
 * Return			: None
 *
 * Note				: N/A
 */
void SPI_Peri_CTRL(SPI_REG_t *pSPIx, uint8_t EnDi) {

	if (EnDi == ENABLE) {
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	} else {
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}

}

void SPI_SSI_Config(SPI_REG_t *pSPIx, uint8_t EnDi) {

	if (EnDi == ENABLE) {
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	} else {
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}

void SPI_SSOE_Config(SPI_REG_t *pSPIx, uint8_t EnDi) {

	if (EnDi == ENABLE) {
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	} else {
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}

}




/* ==========================================================================
 * Function			: GPIO_DeInit
 *
 * Description		: Reset GPIO port
 *
 * Parameter[in]	: base address of the GPIO
 * Parameter[in]	: Enable or Disable
 * Parameter[in]	:
 *
 * Return			: None
 *
 * Note				: N/A
 */
void SPI_IRQ_Interrupt_Config(uint8_t IRQNumber, uint8_t EnDi);

/* ==========================================================================
 * Function			: GPIO_DeInit
 *
 * Description		: Reset GPIO port
 *
 * Parameter[in]	: base address of the GPIO
 * Parameter[in]	: Enable or Disable
 * Parameter[in]	:
 *
 * Return			: None
 *
 * Note				: N/A
 */
void SPI_IRQ_Priority_Config(uint8_t IRQNumber, uint32_t IRQPriority);

/* ==========================================================================
 * Function			: GPIO_DeInit
 *
 * Description		: Reset GPIO port
 *
 * Parameter[in]	: base address of the GPIO
 * Parameter[in]	: Enable or Disable
 * Parameter[in]	:
 *
 * Return			: None
 *
 * Note				: N/A
 */
void SPI_IRQ_Handling(SPIx_Handle_t *pSPIHandle);
