/*
 * stm32f302x8_i2c_drivers.c
 *
 *  Created on: Mar 13, 2025
 *      Author: Hyunbin
 */

#include "stm32f302x8_i2c_drivers.h"

uint16_t AHB_Pre_Scaler[8] = {2,4,8,16,64,128,256,512};


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
void I2C_PCLK_CTRL(I2C_REG_t *pI2Cx, uint8_t EnDi) {

	if (EnDi == ENABLE) {
		if (pI2Cx == I2C1) {
			I2C1_PCLK_EN();
		} else if (pI2Cx == I2C2){
			I2C2_PCLK_EN();
		} else if (pI2Cx == I2C3){
			I2C3_PCLK_EN();
		}
	}

	else {
		if (pI2Cx == I2C1) {
			I2C1_PCLK_DI();
		} else if (pI2Cx == I2C2){
			I2C2_PCLK_DI();
		} else if (pI2Cx == I2C3){
			I2C3_PCLK_DI();
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

uint32_t RCC_GET_PLL(void) {
	uint32_t pllclk;

	return pllclk;
}





uint32_t RCC_Get_PCKL(void) {

	uint32_t pclk, System_Clk;

	uint8_t clkused, temp_clk, AHB_clock, APB1_clock;
	//uint8_t APB2_clock;

	clkused = (RCC->CFGR >> 2) & 0x3;

	if (clkused == 0) {
		// HSI oscillator used as system clock - 00
		System_Clk = 8000000;

	} else if (clkused == 1) {
		// HSE oscillator used as system clock - 01
		System_Clk = 4000000;

	} else if (clkused == 2) {
		// PLL used as system clock - 10
		System_Clk = RCC_GET_PLL();
	}

	// AHB Clock
	temp_clk = (RCC->CFGR >> 4) & 0xF;

	if (temp_clk < 8) {

		AHB_clock = 1;
	}

	else {
		AHB_clock = AHB_Pre_Scaler[temp_clk - 8];

	}

	// ABP1 Clock - Low-speed
	temp_clk = (RCC->CFGR >> 8) & 0x7;

	if (temp_clk < 4) {

		APB1_clock = 1;
	}

	else {
		APB1_clock = AHB_Pre_Scaler[temp_clk - 4];

	}

	// ABP2 Clock - High-speed
	/*temp_clk = (RCC->CFGR >> 11) & 0x7;

	if (temp_clk < 4) {

		AHB_clock = 1;
	}

	else {
		AHB_clock = AHB_Pre_Scaler[temp_clk - 4];

	}*/

	pclk = (System_Clk / AHB_clock) / APB1_clock;

	return pclk;

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
void I2C_Init(I2Cx_Handle_t *I2CHandle) {

	uint32_t temp1 = 0, temp2 = 0;

	// follow I2C initialization flow chart
	// 1. Clear PE bit in I2C_CR1
	I2CHandle->pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);


	// 2. Configure ANFOFF and DNF[3:0] in I2C_CR1
	I2CHandle->pI2Cx->CR1 &= ~(1 << I2C_CR1_ANFOFF);
	I2CHandle->pI2Cx->CR1 |= (0xf << I2C_CR1_DNF);


	// 3. Configure PRESC[3:0], SDADEL[3:0], SCLDEL[3:0], SCLH[7:0], SCLL[7:0] in I2C_TIMINGR
	temp1 |= I2CHandle->I2C_Config.I2C_PRESC << I2C_TIMINGR_PRESC;
	temp1 |= I2CHandle->I2C_Config.I2C_SCLDEL << I2C_TIMINGR_SCLDEL;
	temp1 |= I2CHandle->I2C_Config.I2C_SDADEL << I2C_TIMINGR_SDADEL;
	temp1 |= I2CHandle->I2C_Config.I2C_SCLH << I2C_TIMINGR_SCLH;
	temp1 |= I2CHandle->I2C_Config.I2C_SCLL << I2C_TIMINGR_SCLL;

	I2CHandle->pI2Cx->TIMINGR |= temp1;

	// 4. Configure NOSTRETCH in I2C_CR1 - must be 0 in master mode
	I2CHandle->pI2Cx->CR1 &= ~(1 << I2C_CR1_NOSTRETCH);


	// 5. Set PE bit in I2C_CR1
	I2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_PE);

	// 6. Set address
	temp2 |= I2CHandle->I2C_Config.I2C_Device_Address << I2C_CR2_SADD;
	temp2 |= (1 << I2C_CR2_ADD10); // 7 bits;
	I2CHandle->pI2Cx->CR2 |= temp2;

	//temp = 0;
	//temp = RCC_Get_PCKL() / 10000000U;

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
void I2C_DeInit(I2C_REG_t *pI2Cx) {

	if (pI2Cx == I2C1) {
		I2C1_REG_RESET();
	} else if (pI2Cx == I2C2){
		I2C2_REG_RESET();
	} else if (pI2Cx == I2C3){
		I2C3_REG_RESET();
	}

}

/* Data */


/* IRQ */
void I2C_IRQ_Interrupt_Config(uint8_t IRQNumber, uint8_t EnDi);
void I2C_IRQ_Priority_Config(uint8_t IRQNumber, uint32_t IRQPriority);

uint8_t I2C_Get_Flag_Status(I2C_REG_t *pI2Cx, uint32_t Flag_name) {

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
void I2C_Peri_CTRL(I2C_REG_t *pI2Cx, uint8_t EnDi) {

	if (EnDi == ENABLE) {
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	}

	else {
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
	}

}
