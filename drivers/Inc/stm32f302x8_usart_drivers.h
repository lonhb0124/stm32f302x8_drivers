/*
 * stm32f302x8_usart_drivers.h
 *
 *  Created on: Mar 20, 2025
 *      Author: Hyunbin
 */

#ifndef INC_STM32F302X8_USART_DRIVERS_H_
#define INC_STM32F302X8_USART_DRIVERS_H_

#include "stm32f302x8.h"

typedef struct {
	uint8_t USART_Mode;
	uint32_t USART_Baud_Rate;
	uint8_t USART_No_Stop_Bits;
	uint8_t USART_Word_Length;
	uint8_t USART_Parity_Control;
	uint8_t USART_HW_Flow_Control;
} USART_Config_t;

typedef struct {
	USART_REG_t *pUSARTx;
	USART_Config_t USART_Config;
} USARTx_Handle_t;

/* USART Mode */
#define USART_MODE_TX	0
#define USART_MODE_RX	1
#define USART_MODE_TXRX	2

/* USART Baud Rate */
#define USART_BR_1200		1200
#define USART_BR_2400		2400
#define USART_BR_9600		9600
#define USART_BR_19200		19200
#define USART_BR_38400		38400
#define USART_BR_57600		57600
#define USART_BR_115200		115200
#define USART_BR_230400		230400
#define USART_BR_460800		460800
#define USART_BR_921600		921600
#define USART_BR_2M			2000000
#define USART_BR_3M			3000000





/* Clock */
void USART_PCLK_CTRL(USART_REG_t *pUSARTx, uint8_t EnDi);

/* Init */
void USART_Init(USARTx_Handle_t *pUSARTHandle);
void USART_DeInit(USART_REG_t *pUSARTx);

/* Data */
void USART_Transmit_Data(USART_REG_t *pUSARTx, uint8_t *pTx_Buffer, uint32_t Len);
void USART_Receive_Data(USART_REG_t *pUSARTx, uint8_t *pRx_Buffer, uint32_t Len);
uint8_t USART_Transmit_Data_IT(USARTx_Handle_t *pUSARTHandle, uint8_t *pTx_Buffer, uint32_t Len);
uint8_t USART_Receive_Data_IT(USARTx_Handle_t *pUSARTHandle, uint8_t *pRx_Buffer, uint32_t Len);

/* IRQ */
void USART_IRQ_Priority_Config(uint8_t IRQNumber, uint32_t IRQPriority);
void USART_IRQ_Handling(USARTx_Handle_t *pUSARTHandle);

/* */
uint8_t USART_Get_Flag_Status(USART_REG_t *pUSARTx, uint32_t Flag_name);
void USART_Clear_Flag(USART_REG_t *pUSARTx, uint32_t Flag_name);
void USART_Peri_CTRL(USART_REG_t *pUSARTx, uint8_t EnDi);

#endif /* INC_STM32F302X8_USART_DRIVERS_H_ */
