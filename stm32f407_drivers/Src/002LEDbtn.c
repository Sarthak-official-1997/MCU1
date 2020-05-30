/*
 * 002LEDbtn.c
 *
 *  Created on: Mar 17, 2020
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
	GPIO_Handle_t gpiobtn;

	GPIO_output(&gpioled12, GPIOD, PIN_12, GPIO_MODE_OUPUT, GPIO_OSPEED_HIGH, GPIO_OTYPE_PP, GPIO_PUPD_PU);
	GPIO_output(&gpioled13, GPIOD, PIN_13, GPIO_MODE_OUPUT, GPIO_OSPEED_HIGH, GPIO_OTYPE_PP, GPIO_PUPD_PU);
	GPIO_output(&gpioled14, GPIOD, PIN_14, GPIO_MODE_OUPUT, GPIO_OSPEED_HIGH, GPIO_OTYPE_PP, GPIO_PUPD_PU);
	GPIO_output(&gpioled15, GPIOD, PIN_15, GPIO_MODE_OUPUT, GPIO_OSPEED_HIGH, GPIO_OTYPE_PP, GPIO_PUPD_PU);


    GPIO_input(&gpiobtn ,GPIOA, PIN_0,GPIO_MODE_INPUT, GPIO_OSPEED_HIGH, GPIO_PUPD_NO_PU_PD);

    GPIOD_PCLK_EN();
    GPIOA_PCLK_EN();

	GPIO_Init(&gpioled12);
	GPIO_Init(&gpioled13);
	GPIO_Init(&gpioled14);
	GPIO_Init(&gpioled15);

	GPIO_Init(&gpiobtn);

	while(1)
{
    if(GPIO_ReadPin(GPIOA, PIN_0))
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
    }//end of if
}


return 0;
}
