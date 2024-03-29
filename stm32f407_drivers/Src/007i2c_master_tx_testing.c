/*
 * 007i2c_master_tx_testing.c
 *
 *  Created on: Mar 26, 2020
 *      Author: Sarthak
 */
#include "stm32f407xx.h"
#include <string.h>
#include <stdio.h>

#define MY_ADDR         0x61
#define SLAVE_ADDR      0x68
//some data for slave
uint8_t somedata[]="We are testing i2c Master Tx\n";
/*
 * PB6 -> SCL
 * PB9 -> SDA
 * MY_ADDR = 0x61
 *
 */
void I2C1_GPIOInits(void)
{
 GPIO_Handle_t I2CPins;

 I2CPins.pGPIOx=GPIOB;
 I2CPins.GPIO_PinConfig.Pin_Mode=GPIO_MODE_ALTFUNC;
 I2CPins.GPIO_PinConfig.Pin_OType=GPIO_OTYPE_OD;   //must for i2c
 I2CPins.GPIO_PinConfig.Pin_PuPdType=GPIO_PUPD_PU;
 I2CPins.GPIO_PinConfig.Pin_AltFuncMode=4;
 I2CPins.GPIO_PinConfig.Pin_Speed=GPIO_OSPEED_HIGH;

 //scl
 I2CPins.GPIO_PinConfig.Pin_Num=PIN_6;
 GPIO_Init(&I2CPins);

 //sda
 I2CPins.GPIO_PinConfig.Pin_Num=PIN_9;
 GPIO_Init(&I2CPins);

}
I2C_Handle_t I2C1handle;
void I2C1_Inits(void)
{


 I2C1handle.pI2Cx=I2C1;
 I2C1handle.I2C_Config.I2C_ACKControl=I2C_ACK_EN;
 I2C1handle.I2C_Config.I2C_DeviceAddress=MY_ADDR;
 I2C1handle.I2C_Config.I2C_FMDutyCycle=I2C_FM_DUTY_2;
 I2C1handle.I2C_Config.I2C_SCLSpeed=I2C_SCL_SPEED_SM;

 I2C_Init(&I2C1handle);
}

void GPIO_btnInit()
{
	GPIO_Handle_t gpiobtn;

	 GPIO_input(&gpiobtn ,GPIOA, PIN_0,GPIO_MODE_INPUT, GPIO_OSPEED_HIGH, GPIO_PUPD_PU);
	 GPIOA_PCLK_EN();
	 GPIO_Init(&gpiobtn);
}

void delay(void)
{
	for(uint32_t i=0;i<5000000/2;i++);
}
int main()
{
	GPIO_btnInit();

    //i2c pin initialise ..gpio pin as i2c pin
	I2C1_GPIOInits();
	//i2c peripheral configuration
	I2C1_Inits();

	//enable the i2c peripheral
	I2C_PeripheralControl(I2C1, ENABLE);
	while(1)
	{
		//wait till button is pressed
		while(! GPIO_ReadPin(GPIOA, PIN_0));
		//to avoid button de-bouncing related issues 200ms of delay
		delay();


        //send some data to slave
        I2C_MasterSendData(&I2C1handle,somedata, strlen((char*)somedata), SLAVE_ADDR , I2C1);
	}


}

