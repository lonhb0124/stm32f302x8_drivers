/*
 * stm32f302x8_usart_drivers.c
 *
 *  Created on: Mar 20, 2025
 *      Author: Hyunbin
 */


#include "stm32f302x8_usart_drivers.h"



/* Clock */
void USART_PCLK_CTRL(USART_REG_t *pUSARTx, uint8_t EnDi) {

}

/* Init */
void USART_Init(USARTx_Handle_t *pUSARTHandle) {

}
void USART_DeInit(USART_REG_t *pUSARTx) {

}

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
