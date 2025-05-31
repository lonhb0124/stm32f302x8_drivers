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
	uint8_t I2C_SCLL;
	uint8_t I2C_SCLH;
	uint8_t I2C_SDADEL;
	uint8_t I2C_SCLDEL;
	uint8_t I2C_PRESC;
} I2C_Config_t;

typedef struct {
	I2C_REG_t *pI2Cx;
	I2C_Config_t I2C_Config;
} I2Cx_Handle_t;

/*
 * tSCL = tSYNC1 + tSYNC2 + {[(SCLH+1) + (SCLL+1)] x (PRESC+1) x tI2CCLK}
 * tSYNC1 + tSYNC2 >= 4 * tSCL
 * tSDADEL = SDADEL x tPRESC + tI2CCLK = SDADEL x (PRESC+1)x tI2CCLK + tI2CCLK
 * tSCLDEL = (SCLDEL+1) x tPRESC = (SCLDEL+1) x (PRESC+1) x tI2CCLK
 *
 * */

/* I2C Clock Speed */
#define I2C_SCL_SPEED_1K	100000 // PRESC = 1, SCLL = 0x13, SCLH = 0xF, SDADEL = 0x2, SCLDEL = 0x4
#define I2C_SCL_SPEED_2K	200000
#define I2C_SCL_SPEED_4K	400000 // PRESC = 0, SCLL = 0x9, SCLH = 0x3, SDADEL = 0x1, SCLDEL = 0x3

/* I2C ACK Control */
//#define I2C_ACK_ENABLE
//#define I2C_ACK_DISABLE

/* I2C Duty Cycle */
#define I2C_FM_Duty_Cycle_5_4	0 // SCLH = 0xF, SCLL = 0x13	(19 + 1) / (15 + 1) = 5 / 4
#define I2C_FM_Duty_Cycle_5_2	1 // SCLH = 0x3, SCLL = 0x9		(9 + 1) / (3 + 1) = 5 / 2

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
