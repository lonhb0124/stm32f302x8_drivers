/*
 * stm32f302x8_i2c_drivers.c
 *
 *  Created on: Mar 13, 2025
 *      Author: Hyunbin
 */

#include "stm32f302x8_i2c_drivers.h"

/* Clock */
void I2C_PCLK_CTRL(I2C_REG_t *pI2Cx, uint8_t EnDi) {

}

/* Init */
void I2C_Init(I2Cx_Handle_t *I2CHandle) {

}
void I2C_DeInit(I2C_REG_t *pI2Cx) {

}

/* Data */


/* IRQ */
void I2C_IRQ_Interrupt_Config(uint8_t IRQNumber, uint8_t EnDi);
void I2C_IRQ_Priority_Config(uint8_t IRQNumber, uint32_t IRQPriority);
