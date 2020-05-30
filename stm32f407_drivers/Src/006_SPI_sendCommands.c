/*
 * 006_SPI_sendCommands.c
 *
 *  Created on: Mar 21, 2020
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
//command codes
#define COMMAND_LED_CTRL          0x50
#define COMMAND_SENSOR_READ       0x51
#define COMMAND_LED_READ          0x52
#define COMMAND_PRINT             0x53
#define COMMAND_ID_READ           0x54

#define LED_ON        1
#define LED_OFF       0

//arduino analog pins
#define ANALOG_PIN0   0
#define ANALOG_PIN1   1
#define ANALOG_PIN2   2
#define ANALOG_PIN3   3
#define ANALOG_PIN4   4

#define NACK         0xA5
#define ACK          0xF5

//External LED connected to pin 9
#define LED_PIN       9


void SPI2_GPIOInits(void)
{
	GPIO_Handle_t SPI2_pins;

	SPI2_pins.pGPIOx =GPIOB;
	SPI2_pins.GPIO_PinConfig.Pin_Mode= GPIO_MODE_ALTFUNC;
	SPI2_pins.GPIO_PinConfig.Pin_AltFuncMode = 5;
	SPI2_pins.GPIO_PinConfig.Pin_OType =GPIO_OTYPE_PP;
	SPI2_pins.GPIO_PinConfig.Pin_PuPdType =GPIO_PUPD_PU;

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
	SPI2handle.SPI_PinConfig.SPI_SclkSpeed =SPI_SCLK_SPEED_DIV8;
	SPI2handle.SPI_PinConfig.SPI_DFF =SPI_DFF_8BITS;
	SPI2handle.SPI_PinConfig.SPI_CPOL= SPI_CPOL_0_IDLESTATE;
	SPI2handle.SPI_PinConfig.SPI_CPHA = SPI_CPHA_DATACAPTURE_1ST_EDGE;
	SPI2handle.SPI_PinConfig.SPI_SSM=SPI_SSM_ENABLE;

	SPI_Init(&SPI2handle);
}

void GPIO_btnInit()
{
	GPIO_Handle_t gpiobtn;

	 GPIO_input(&gpiobtn ,GPIOA, PIN_0,GPIO_MODE_INPUT, GPIO_OSPEED_HIGH, GPIO_PUPD_NO_PU_PD);
	 GPIOA_PCLK_EN();
	 GPIO_Init(&gpiobtn);
}

uint8_t SPI_VerifyResponse(uint8_t ackbyte)
{
	if(ackbyte == 0xF5)
		return 1;
else return 0;
}

void delay(void)
{
	for(uint32_t i=0 ; i<300000 ; i++);
}
int main(void)
{
	    uint8_t dummywrite= 0xff;
	    uint8_t ackbyte;
	    uint8_t dummyread;
        //this function is used to initialize GPIO pins behave as SPI2 pins
		SPI2_GPIOInits();

		SPI2_Inits();
       GPIO_btnInit();
		/*
		 * Making SSOE 1 does NSS output enable
		 * the NSS automatically managed by hardware
		 * i.e when SPE=1 ,NSS will be pulled to LOW.
		 *     and when SPE=0, NSS will be HIGH.
		 *
		 */
		SPI_SSOEconfig(SPI2,ENABLE);

		while(1)
		{
		 while(!(GPIO_ReadPin(GPIOA, PIN_0)));
		 delay();

		 //enable SPI2 peripheral
		 SPI_PeripheralControl(SPI2, ENABLE);
		 //this makes NSS signal internally HIGHa and avoids MODF error which is a flag
		 SPI_SSIconfig(SPI2,ENABLE);


		 //1.  COMMAND_LED_CTRL  <pin (1)>      <value(1)>
		 uint8_t commandcode = COMMAND_LED_CTRL;
		 SPI_SendData(SPI2, &commandcode, 1);
		 /*after sending Commandcode master will receive a byte too bcz MISO is connected
		  so Do dummy read to clear the RXNE flag.
		 */SPI_ReceiveData(SPI2,&dummyread, 1);

		 //lets send some dummy bits in order to get response from slave. spi on itself does not initiate the salve to respond
		 SPI_SendData(SPI2,&dummywrite, 1);
		 SPI_ReceiveData(SPI2,&ackbyte, 1);

		 uint8_t args[2];
        if( SPI_VerifyResponse(ackbyte) )
        {//send arguements

        	args[0]= LED_PIN;
        	args[1]= LED_ON;
        	SPI_SendData(SPI2, args, 2);
        }


#if(0)
		 char user_data[]="Hello World";
		 //first send data length to slave
		 uint8_t dataLen=strlen(user_data);

		 SPI_SendData(SPI2, &dataLen, 1);

		 SPI_SendData(SPI2,(uint8_t*)user_data,strlen(user_data));
#endif
         while(GetFlagStatus(SPI2, SPI_BSY_FLAG));

		 //disable the transmission by disabling the peripheral
		 SPI_PeripheralControl(SPI2, DISABLE);

		}


	return 0;
}



