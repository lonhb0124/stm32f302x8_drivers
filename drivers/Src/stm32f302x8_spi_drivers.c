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
void SPI_Init(SPIx_Handle_t *pSPIHandle) {

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
void SPI_DeInit(SPI_REG_t *pSPIx) {

	if (pSPIx == SPI2) {
		SPI2_REG_RESET();
	} else if (pSPIx == SPI3) {
		SPI3_REG_RESET();
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
void SPI_Transmit_Data(SPI_REG_t *pSPIx, uint8_t *pTx_Buffer, uint32_t Len);

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
void SPI_Receive_Data(SPI_REG_t *pSPIx, uint8_t *pTx_Buffer, uint32_t Len);

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
