/*
 * stm32f302x8_i2c_drivers.h
 *
 *  Created on: Mar 13, 2025
 *      Author: Hyunbin
 */

#ifndef INC_STM32F302X8_I2C_DRIVERS_H_
#define INC_STM32F302X8_I2C_DRIVERS_H_

#include "stm32f302x8.h"

typedef struct {
	uint8_t I2C_SCL_Speed;
	uint8_t I2C_Device_Address;
	uint8_t I2C_ACK_Control;
	uint8_t I2C_FM_Duty_Cycle;
} I2C_Config_t;

typedef struct {
	I2C_REG_t *pI2Cx;
	I2C_Config_t I2C_Config;
} I2Cx_Handle_t;

/* I2C Clock Speed */
#define I2C_SCL_SPEED_1K	100000
#define I2C_SCL_SPEED_2K	200000
#define I2C_SCL_SPEED_4K	400000

/* I2C ACK Control */
//#define I2C_ACK_ENABLE
//#define I2C_ACK_DISABLE

/* I2C Duty Cycle */
#define I2C_FM_Duty_Cycle_2		0 // SCLH = 7, SCLL = 15
#define I2C_FM_Duty_Cycle_16_9	1 // SCLH = 8, SCLL = 15

/* Clock */
void I2C_PCLK_CTRL(I2C_REG_t *pI2Cx, uint8_t EnDi);

/* Init */
void I2C_Init(I2Cx_Handle_t *I2CHandle);
void I2C_DeInit(I2C_REG_t *pI2Cx);

/* Data */


/* IRQ */
void I2C_IRQ_Interrupt_Config(uint8_t IRQNumber, uint8_t EnDi);
void I2C_IRQ_Priority_Config(uint8_t IRQNumber, uint32_t IRQPriority);

uint8_t I2C_Get_Flag_Status(I2C_REG_t *pI2Cx, uint32_t Flag_name);
void I2C_Peri_CTRL(I2C_REG_t *pI2Cx, uint8_t EnDi);


#endif /* INC_STM32F302X8_I2C_DRIVERS_H_ */
