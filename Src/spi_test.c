/*
 * spi_test.c
 *
 *  Created on: Mar 11, 2025
 *      Author: Hyunbin
 */


#include <string.h>
#include <stdint.h>
#include "stm32f302x8.h"

/* PB12 - NSS
 * PB13 - SCLK
 * PB14 - MISO
 * PB15 - MOSI
 * ALT function mode : 5
 */

void SPI2_GPIO_Init(void) {

	GPIOx_Handle_t SPI_Pin;

	SPI_Pin.pGPIOx = GPIOB;
	SPI_Pin.GPIO_PinConfig.GPIO_Pin_Mode = GPIO_MODE_ALT;
	SPI_Pin.GPIO_PinConfig.GPIO_Pin_Speed = GPIO_OP_HIGH;
	SPI_Pin.GPIO_PinConfig.GPIO_Pin_PuPd = GPIO_NO_PUPD;
	SPI_Pin.GPIO_PinConfig.GPIO_Pin_OPType = GPIO_OP_TYPE_PP;
	SPI_Pin.GPIO_PinConfig.GPIO_Pin_AltFun = 5;

	// NSS
	SPI_Pin.GPIO_PinConfig.GPIO_Pin_Number = GPIO_PIN_12;
	GPIO_Init(&SPI_Pin);

	// SCLK
	SPI_Pin.GPIO_PinConfig.GPIO_Pin_Number = GPIO_PIN_13;
	GPIO_Init(&SPI_Pin);

	// MISO
	//SPI_Pin.GPIO_PinConfig.GPIO_Pin_Number = GPIO_PIN_14;
	//GPIO_Init(&SPI_Pin);

	// MOSI
	SPI_Pin.GPIO_PinConfig.GPIO_Pin_Number = GPIO_PIN_15;
	GPIO_Init(&SPI_Pin);


}

void SPI2_Init(void) {

	SPIx_Handle_t SPI2_Handle;

	SPI2_Handle.pSPIx = SPI2;
	SPI2_Handle.SPI_Config.SPI_Bus_Config = SPI_BUS_FULL_DUPLEX;
	SPI2_Handle.SPI_Config.SPI_SCLK_Speed = SPI_SCLK_SPEED_DIV_4; // SCLK = 2 MHZ
	SPI2_Handle.SPI_Config.SPI_CRCL = SPI_CRCL_8_BITS;
	SPI2_Handle.SPI_Config.SPI_CPOL = SPI_CPLO_LOW;
	SPI2_Handle.SPI_Config.SPI_CPHA = SPI_CPLO_LOW;
	SPI2_Handle.SPI_Config.SPI_SSM = SPI_SSM_SW_DI;
	SPI2_Handle.SPI_Config.SPI_Device_Mode = SPI_DEVICE_MODE_M;

	SPI_Init(&SPI2_Handle);
}

void GPIO_Button_Init(void) {

	GPIOx_Handle_t GPIO_BUTTON;
	memset(&GPIO_BUTTON,0,sizeof(GPIO_BUTTON));

	GPIO_BUTTON.pGPIOx = GPIOC;
	GPIO_BUTTON.GPIO_PinConfig.GPIO_Pin_Number = GPIO_PIN_13;
	GPIO_BUTTON.GPIO_PinConfig.GPIO_Pin_Mode = GPIO_MODE_IT_FT;
	GPIO_BUTTON.GPIO_PinConfig.GPIO_Pin_Speed = GPIO_OP_HIGH;
	GPIO_BUTTON.GPIO_PinConfig.GPIO_Pin_PuPd = GPIO_NO_PUPD;


	GPIO_Init(&GPIO_BUTTON);

}

void delay(void) {

	for(uint32_t i = 0; i< 125000; i++);
}

int main(void) {

	char data[] = "Hello World, I'm Hyunbin";

	GPIO_Button_Init();

	// This is spi test to send_data "Hello World"
	SPI2_GPIO_Init();

	SPI2_Init();

	// To resolve MODF error - NSS signal high internally
	//SPI_SSI_Config(SPI2, ENABLE);

	SPI_SSOE_Config(SPI2, ENABLE);
	while(1) {
		// wait button is pressed
		while(!(GPIO_Read_In_Pin(GPIOA, GPIO_PIN_13)));

		delay();
		// Enable SPI2 peripheral
		SPI_Peri_CTRL(SPI2, ENABLE);

		// Send length of information
		uint8_t data_len = strlen(data);
		SPI_Transmit_Data(SPI2, &data_len, 1);

		// Send data
		SPI_Transmit_Data(SPI2, (uint8_t*) data, strlen(data));

		// check SPI is busy
		while(SPI_Get_Flag_Status(SPI2, SPI_BUSY_FLAG));

		// Disable SPI2 peripheral
		SPI_Peri_CTRL(SPI2, DISABLE);

	}

	return 0;
}
