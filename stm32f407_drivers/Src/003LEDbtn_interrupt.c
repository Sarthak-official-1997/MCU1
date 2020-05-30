/*
 * 003LEDbtn_interrupt.c
 *
 *  Created on: Mar 18, 2020
 *      Author: Sarthak
 */


#include "stm32f407xx.h"



int main(void)
{
	GPIO_Handle_t gpioled, gpiobtn;
	memset(&gpioled,0,sizeof(gpioled));
	memset(&gpiobtn,0,sizeof(gpiobtn));

	GPIO_output(&gpioled, GPIOD, PIN_12, GPIO_MODE_OUPUT, GPIO_OSPEED_HIGH, GPIO_OTYPE_PP, GPIO_PUPD_PU); //mode : OUPUT

	GPIO_Init(&gpioled);
	GPIOD_PCLK_EN();





	GPIO_input(&gpiobtn, GPIOA, PIN_0, GPIO_MODE_IT_FT, GPIO_OSPEED_HIGH, GPIO_PUPD_PU); //mode : IT_FT

	GPIO_Init(&gpiobtn);
	GPIOA_PCLK_EN();
	//GPIO_WritePin(GPIOD, PIN_12, DISABLE);

	/********************************* Interrupt Section  ****************************************/

	GPIO_IRQPriorityConfig(IRQ_NUM_EXTI0,NVIC_IRQ_PRI15 );
	GPIO_IRQInterruptConfig(IRQ_NUM_EXTI0, ENABLE);

while(0);
}
void delay(int n)
{
	for(;n>0;n--)
		for(int i=0;i<1203;i++);
}

void EXTI0_IRQHandler(void)
{
	 GPIO_IRQHandling(PIN_0);
 //delay(200);
 GPIO_WritePin(GPIOD, PIN_12, HIGH);
}
