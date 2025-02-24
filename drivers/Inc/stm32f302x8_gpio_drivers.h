/*
 * stm32f302x8_gpio_drivers.h
 *
 *  Created on: Feb 21, 2025
 *      Author: Hyunbin
 */

#ifndef INC_STM32F302X8_GPIO_DRIVERS_H_
#define INC_STM32F302X8_GPIO_DRIVERS_H_

#include "stm32f302x8.h"

typedef struct {
	uint8_t GPIO_Pin_Number; 	// 0 ~ 15
	uint8_t GPIO_Pin_Mode;		// GPIO MODE
	uint8_t GPIO_Pin_Speed;		// GPIO SOUTPUT SPEED
	uint8_t GPIO_Pin_PuPd;		// GPIO PULL UP DOWN
	uint8_t GPIO_Pin_OPType; 	// GPIO OUTPUT TYPES
	uint8_t GPIO_Pin_AltFun;	// GPIO Alternative MODE
} GPIO_Pin_Config_t;


typedef struct {
	GPIO_REG_t *pGPIOx;		// pointer to hold the base of the GPIO peripheral
	GPIO_Pin_Config_t GPIO_PinConfig;
} GPIOx_Handle_t;


/* GPIO NUMBER */
#define GPIO_PIN_0			0
#define GPIO_PIN_1			1
#define GPIO_PIN_2			2
#define GPIO_PIN_3			3
#define GPIO_PIN_4			4
#define GPIO_PIN_5			5
#define GPIO_PIN_6			6
#define GPIO_PIN_7			7
#define GPIO_PIN_8			8
#define GPIO_PIN_9			9
#define GPIO_PIN_10			10
#define GPIO_PIN_11			11
#define GPIO_PIN_12			12
#define GPIO_PIN_13			13
#define GPIO_PIN_14			14
#define GPIO_PIN_15			15

/* GPIO MODE */
#define GPIO_MODE_IN 		0
#define GPIO_MODE_OUT 		1
#define GPIO_MODE_ALT 		2
#define GPIO_MODE_ANALOG 	3
#define GPIO_MODE_IT_FT 	4 // input falling edge
#define GPIO_MODE_IT_RT 	5 // input rising edge
#define GPIO_MODE_IT_RFT 	6 // rising/falling edge trigger

/* GPIO OUTPUT TYPES */
#define GPIO_OP_TYPE_PP 	0
#define GPIO_OP_TYPE_OD 	1

/* GPIO OUTPUT SPEED */
#define GPIO_OP_LOW			0
#define GPIO_OP_MED			1
#define GPIO_OP_HIGH		3

/* GPIO PULL UP DOWN */
#define GPIO_NO_PUPD		0
#define GPIO_PIN_PU			1
#define GPIO_PIN_PD			2


void GPIO_Init(GPIOx_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_REG_t *pGPIOx);

void GPIO_PCLK_CTRL(GPIO_REG_t *pGPIOx, uint8_t EnDi);

uint8_t GPIO_Read_In_Pin(GPIO_REG_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_Read_In_Port(GPIO_REG_t *pGPIOx);
void GPIO_Write_Out_Pin(GPIO_REG_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_Write_Out_Port(GPIO_REG_t *pGPIOx, uint16_t Value);
void GPIO_Toggle_Out_Pin(GPIO_REG_t *pGPIOx, uint8_t PinNumber);

void GPIO_IRQ_Config(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnDi);
void GPIO_IRQ_Handling(uint8_t PinNumber);


#endif /* INC_STM32F302X8_GPIO_DRIVERS_H_ */
