/*
 * 004spiTX.c
 *
 *  Created on: Mar 20, 2020
 *      Author: Sarthak
 */
#include "stm32f407xx.h"
/*

 PB15 -->SPI2 MOSI
 PB14 -->SPI2 MISO
 PB13 -->SPI2 SCLK    pin13
 PB12 -->SPI2 NSS
 Alternate Functionality Mode  : 5 ie AF5 for Port B

*/
void SPI2_GPIOInits(void)
{
	GPIO_Handle_t SPI2_pins;

	SPI2_pins.pGPIOx =GPIOB;
	SPI2_pins.GPIO_PinConfig.Pin_Mode= GPIO_MODE_ALTFUNC;
	SPI2_pins.GPIO_PinConfig.Pin_AltFuncMode = 5;
	SPI2_pins.GPIO_PinConfig.Pin_OType =GPIO_OTYPE_PP;
	SPI2_pins.GPIO_PinConfig.Pin_PuPdType =GPIO_PUPD_NO_PU_PD;

	//pin13 provides clock for SPI2 for port B
	SPI2_pins.GPIO_PinConfig.Pin_Num = PIN_13;
	GPIO_Init(&SPI2_pins);
	//MOSI is located at pin 15 for SPI2 of port B
	SPI2_pins.GPIO_PinConfig.Pin_Num= PIN_15;
	GPIO_Init(&SPI2_pins);
	//MISO is located at pin 14 for SPI2 of port B
	SPI2_pins.GPIO_PinConfig.Pin_Num=PIN_14;
	GPIO_Init(&SPI2_pins);
	//NNS is located at pin 12 for SPI2 of port B
	SPI2_pins.GPIO_PinConfig.Pin_Num= PIN_12;
	GPIO_Init(&SPI2_pins);
}

void SPI2_Inits(void)
{
	SPI_Handle_t SPI2handle;

	SPI2handle.pSPIx=SPI2;
	SPI2handle.SPI_PinConfig.SPI_BusConfig =SPI_BUSCONFIG_FULL_DUPLEX;
	SPI2handle.SPI_PinConfig.SPI_DeviceMode= SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPI_PinConfig.SPI_SclkSpeed =SPI_SCLK_SPEED_DIV2;
	SPI2handle.SPI_PinConfig.SPI_DFF =SPI_DFF_8BITS;
	SPI2handle.SPI_PinConfig.SPI_CPOL= SPI_CPOL_0_IDLESTATE;
	SPI2handle.SPI_PinConfig.SPI_CPHA = SPI_CPHA_DATACAPTURE_1ST_EDGE;
	SPI2handle.SPI_PinConfig.SPI_SSM=SPI_SSM_ENABLE;

	SPI_Init(&SPI2handle);
}
int main(void)
{
 //this function is used to initialize GPIO pins behave as SPI2 pins
		SPI2_GPIOInits();

		SPI2_Inits();

		//enable SPI2 peripheral
		SPI_PeripheralControl(SPI2, ENABLE);
		//this makes NSS signal internally HIGHa and avoids MODF error which is a flag
		SPI_SSIconfig(SPI2,ENABLE);

		char user_data[]="Hello World";

		SPI_SendData(SPI2,(uint8_t *)user_data,strlen(user_data));

		while(GetFlagStatus(SPI2, SPI_BSY_FLAG));

		//disable the transmission by disabling the peripheral
		SPI_PeripheralControl(SPI2, DISABLE);

		while(1);

	return 0;
}

