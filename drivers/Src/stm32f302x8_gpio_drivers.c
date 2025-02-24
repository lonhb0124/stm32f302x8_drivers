/*
 * stm32f302x8_gpio_drivers.c
 *
 *  Created on: Feb 21, 2025
 *      Author: Hyunbin
 */


#include "stm32f302x8_gpio_drivers.h"

/* ==========================================================================
 * Function			: GPIO_Init
 *
 * Description		: Initialize GPIO port
 *
 * Parameter[in]	:
 * Parameter[in]	:
 * Parameter[in]	:
 *
 * Return			: None
 *
 * Note				: N/A
 */
void GPIO_Init(GPIOx_Handle_t *pGPIOHandle) {
	// 1. configure the mode of GPIO pin

	uint32_t temp = 0;

	if (pGPIOHandle->GPIO_PinConfig.GPIO_Pin_Mode <= GPIO_MODE_ANALOG) {
		// non interrupt mode
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_Pin_Mode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_Pin_Number));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_Pin_Number)); // clearing
		pGPIOHandle->pGPIOx->MODER |= temp; // setting

	} else {
		// interrupt mode
	}
	temp = 0;

	// 2. configure the speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_Pin_Speed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_Pin_Number));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_Pin_Number));
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	// 3. configure pull up / down setting
	temp = 0;
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_Pin_PuPd << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_Pin_Number));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_Pin_Number));
	pGPIOHandle->pGPIOx->PUPDR |= temp;


	// 4. configure the output type
	temp = 0;
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_Pin_OPType << (pGPIOHandle->GPIO_PinConfig.GPIO_Pin_Number));
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << (pGPIOHandle->GPIO_PinConfig.GPIO_Pin_Number));
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	// 5. configure the alt functionality
	temp = 0;
	if (pGPIOHandle->GPIO_PinConfig.GPIO_Pin_Mode == GPIO_MODE_ALT) {

		uint8_t temp1, temp2 = 0;

		temp1 = (pGPIOHandle->GPIO_PinConfig.GPIO_Pin_Number) / 8;
		temp2 = (pGPIOHandle->GPIO_PinConfig.GPIO_Pin_Number) % 8;

		if (temp1) {
			temp = (pGPIOHandle->GPIO_PinConfig.GPIO_Pin_AltFun << (4 * temp2));
			pGPIOHandle->pGPIOx->AFRH &= ~(0xF << (4 * temp2));
			pGPIOHandle->pGPIOx->AFRH |= temp;
		} else {
			temp = (pGPIOHandle->GPIO_PinConfig.GPIO_Pin_AltFun << (4 * temp2));
			pGPIOHandle->pGPIOx->AFRL &= ~(0xF << (4 * temp2));
			pGPIOHandle->pGPIOx->AFRL |= temp;
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

void GPIO_DeInit(GPIO_REG_t *pGPIOx){

	if (pGPIOx == GPIOA) {
		GPIOA_REG_RESET();
	} else if (pGPIOx == GPIOB) {
		GPIOB_REG_RESET();
	} else if (pGPIOx == GPIOC) {
		GPIOC_REG_RESET();
	} else if (pGPIOx == GPIOD) {
		GPIOD_REG_RESET();
	} else if (pGPIOx == GPIOF) {
		GPIOF_REG_RESET();
	}
}


/* ==========================================================================
 * Function			: GPIO_PCLK_CTRL
 *
 * Description		: Enable or Disable peripheral clock for GPIO port
 *
 * Parameter[in]	: base address of the GPIO
 * Parameter[in]	: Enable or Disable
 * Parameter[in]	:
 *
 * Return			: None
 *
 * Note				: N/A
 */
void GPIO_PCLK_CTRL(GPIO_REG_t *pGPIOx, uint8_t EnDi){


	if (EnDi == ENABLE) {
		if (pGPIOx == GPIOA) {
			GPIOA_PCLK_EN();
		} else if (pGPIOx == GPIOB) {
			GPIOB_PCLK_EN();
		} else if (pGPIOx == GPIOC) {
			GPIOC_PCLK_EN();
		} else if (pGPIOx == GPIOD) {
			GPIOD_PCLK_EN();
		} else if (pGPIOx == GPIOF) {
			GPIOF_PCLK_EN();
		}
	}

	else {
		if (pGPIOx == GPIOA) {
			GPIOA_PCLK_DI();
		} else if (pGPIOx == GPIOB) {
			GPIOB_PCLK_DI();
		} else if (pGPIOx == GPIOC) {
			GPIOC_PCLK_DI();
		} else if (pGPIOx == GPIOD) {
			GPIOD_PCLK_DI();
		} else if (pGPIOx == GPIOF) {
			GPIOF_PCLK_DI();
		}
	}
}



/* ==========================================================================
 * Function			: GPIO_Read_In_Pin
 *
 * Description		: Read data from GPIO pin
 *
 * Parameter[in]	: base address of the GPIO
 * Parameter[in]	: Pin number
 * Parameter[in]	:
 *
 * Return			: GPIO pin value
 *
 * Note				: N/A
 */
uint8_t GPIO_Read_In_Pin(GPIO_REG_t *pGPIOx, uint8_t PinNumber){

	uint8_t value;
	value = (uint8_t) ((pGPIOx->IDR >> PinNumber) & ~(0xFE)); // 0x00000001

	return value;
}



/* ==========================================================================
 * Function			: GPIO_Read_In_Port
 *
 * Description		: Read data from GPIO port (0 ~ 15)
 *
 * Parameter[in]	: base address of the GPIO
 * Parameter[in]	:
 * Parameter[in]	:
 *
 * Return			: GPIO port value
 *
 * Note				: N/A
 */

uint16_t GPIO_Read_In_Port(GPIO_REG_t *pGPIOx){

	uint16_t value;
	value = (uint16_t) (pGPIOx->IDR);

	return value;

}


/* ==========================================================================
 * Function			: GPIO_Write_Out_Pin
 *
 * Description		: Write data into GPIO pin
 *
 * Parameter[in]	: base address of the GPIO
 * Parameter[in]	: Pin number
 * Parameter[in]	: Value
 *
 * Return			: None
 *
 * Note				: N/A
 */

void GPIO_Write_Out_Pin(GPIO_REG_t *pGPIOx, uint8_t PinNumber, uint8_t Value){

	if (Value == GPIO_PIN_SET) {
		pGPIOx->ODR |=  (1 << PinNumber);
	} else {
		pGPIOx->ODR &=  ~(1 << PinNumber);
	}

}


/* ==========================================================================
 * Function			: GPIO_Write_Out_Port
 *
 * Description		: Write data into GPIO port
 *
 * Parameter[in]	: base address of the GPIO
 * Parameter[in]	: Value
 * Parameter[in]	:
 *
 * Return			: None
 *
 * Note				: N/A
 */

void GPIO_Write_Out_Port(GPIO_REG_t *pGPIOx, uint16_t Value){

	pGPIOx->ODR =  Value;

}


/* ==========================================================================
 * Function			: GPIO_Toggle_Out_Pin
 *
 * Description		: Toggle GPIO pin
 *
 * Parameter[in]	: base address of the GPIO
 * Parameter[in]	: Pin number
 * Parameter[in]	:
 *
 * Return			: None
 *
 * Note				: N/A
 */

void GPIO_Toggle_Out_Pin(GPIO_REG_t *pGPIOx, uint8_t PinNumber){

	pGPIOx->ODR ^= (1 << PinNumber);

}


/* ==========================================================================
 * Function			: GPIO_Init
 *
 * Description		: Enable or Disable peripheral clock for GPIO port
 *
 * Parameter[in]	: base address of the GPIO
 * Parameter[in]	: Enable or Disable
 * Parameter[in]	:
 *
 * Return			: None
 *
 * Note				: N/A
 */

void GPIO_IRQ_Config(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnDi){

}


/* ==========================================================================
 * Function			: GPIO_Init
 *
 * Description		: Enable or Disable peripheral clock for GPIO port
 *
 * Parameter[in]	: base address of the GPIO
 * Parameter[in]	: Enable or Disable
 * Parameter[in]	:
 *
 * Return			: None
 *
 * Note				: N/A
 */

void GPIO_IRQ_Handling(uint8_t PinNumber){

}
