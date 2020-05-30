/*
 * 001LEDtesting.c
 *
 *  Created on: Mar 18, 2020
 *      Author: Sarthak
 */
#include "stm32f407xx.h"

void delay(int n)
{  for(; n>0 ; n--)
	for(int  i=0;i<1203;i++);
}
int main(void)
{


	GPIO_Handle_t gpioled12,gpioled13,gpioled14,gpioled15;


	GPIO_output(&gpioled12, GPIOD, PIN_12, GPIO_MODE_OUPUT, GPIO_OSPEED_HIGH, GPIO_OTYPE_PP, GPIO_PUPD_PU);
	GPIO_output(&gpioled13, GPIOD, PIN_13, GPIO_MODE_OUPUT, GPIO_OSPEED_HIGH, GPIO_OTYPE_PP, GPIO_PUPD_PU);
	GPIO_output(&gpioled14, GPIOD, PIN_14, GPIO_MODE_OUPUT, GPIO_OSPEED_HIGH, GPIO_OTYPE_PP, GPIO_PUPD_PU);
	GPIO_output(&gpioled15, GPIOD, PIN_15, GPIO_MODE_OUPUT, GPIO_OSPEED_HIGH, GPIO_OTYPE_PP, GPIO_PUPD_PU);




    GPIOD_PCLK_EN();


	GPIO_Init(&gpioled12);
	GPIO_Init(&gpioled13);
	GPIO_Init(&gpioled14);
	GPIO_Init(&gpioled15);



	while(1)
{

	   GPIO_WritePin(GPIOD,PIN_12, ENABLE);
	   delay(40);
	   GPIO_WritePin(GPIOD, PIN_12, DISABLE);
	   delay(40);
	   GPIO_WritePin(GPIOD,PIN_13, ENABLE);
	   delay(40);
	   GPIO_WritePin(GPIOD, PIN_13, DISABLE);
	   delay(40);
	   GPIO_WritePin(GPIOD,PIN_14, ENABLE);
	   delay(40);
	   GPIO_WritePin(GPIOD,PIN_14, DISABLE);
	   delay(40);
	   GPIO_WritePin(GPIOD,PIN_15, ENABLE);
	   delay(40);
	   GPIO_WritePin(GPIOD,PIN_15, DISABLE);
       delay(40);

}


return 0;
}

