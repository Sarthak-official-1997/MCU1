/**
 * gpio_driver.c
 *
 *  Created on: Mar 16, 2020
 *      Author: Sarthak
 */
#include "gpio_driver.h"


/***********************************************************************************************************************
 *                                              A P I's supported by  GPIO driver
 ***********************************************************************************************************************/






/**
 * @brief GPIO Init and DeInit
 */

/**
 * @fn
 * @param pGPIOx_handle
 */
/*************************************************************************************************
 * @funName   -GPIO_Init
 *
 * @brief     - 1.config the mode of GPIO pin
 *              2.config the speed
 *              3.config the pupd settings
 *              4.config the output type
 *              5.config the alternate functionality (if required)
 *
 *
 * @para[1]   -base address of peripheral
 *
 *
 *
 * @return    -none
 * @Note      -
 *
 *
 **************************************************************************************************/

void GPIO_Init(GPIO_Handle_t  *pGPIOx_handle) // used for configuring the port and pin setting
 {   //peripheral clcok enable
	GPIO_PclkControl(pGPIOx_handle->pGPIOx, ENABLE);


/*/***********************************************  1.config the mode of GPIO pin   **********************************************************/
	uint32_t vola tempGPIO=0;


	if(pGPIOx_handle->GPIO_PinConfig.Pin_Mode <= GPIO_MODE_ANALOG /*3*/)   //0 to 3 are normal modes ,4,5and 6 are IT (interrupt) modes
	{
      tempGPIO = pGPIOx_handle->GPIO_PinConfig.Pin_Mode << (2 * pGPIOx_handle->GPIO_PinConfig.Pin_Num);
      pGPIOx_handle->pGPIOx->MODER &=~(0x3 << pGPIOx_handle->GPIO_PinConfig.Pin_Num);
      pGPIOx_handle->pGPIOx->MODER |= tempGPIO;
	}
	else
	{
	/*
	*Configuring The IT (interrupt) Modes 4, 5 and 6
	* 1. Config the RT and FT and RFT
	* 2. Config GPIO port selection in SYSCFG_EXTICR
	* 3. Enable EXTI interrupt delivery using IMR (interrupt mask register)
	*/

		if(pGPIOx_handle->GPIO_PinConfig.Pin_Mode == GPIO_MODE_IT_FT)
		{  //config FTSR
            EXTI->FTSR |= (1<< pGPIOx_handle->GPIO_PinConfig.Pin_Num);

            //but same time clear RTSR for safety
            EXTI->RTSR &= ~(1<< pGPIOx_handle->GPIO_PinConfig.Pin_Num);
		}
		else if(pGPIOx_handle->GPIO_PinConfig.Pin_Mode == GPIO_MODE_IT_RT)
		{  //config RTSR
			EXTI->RTSR |= (1<< pGPIOx_handle->GPIO_PinConfig.Pin_Num);

			//but same time clear FETSR for safety
			EXTI->FTSR &= ~(1<< pGPIOx_handle->GPIO_PinConfig.Pin_Num);
		}else if(pGPIOx_handle->GPIO_PinConfig.Pin_Mode == GPIO_MODE_IT_RFT)
		{   //config both RTSR and FTSR
			EXTI->RTSR &= ~(1<< pGPIOx_handle->GPIO_PinConfig.Pin_Num); //Clearing of
			EXTI->FTSR &= ~(1<< pGPIOx_handle->GPIO_PinConfig.Pin_Num); //both of them first...

			EXTI->RTSR |= (1<< pGPIOx_handle->GPIO_PinConfig.Pin_Num);  //Now setting
			EXTI->FTSR |= (1<< pGPIOx_handle->GPIO_PinConfig.Pin_Num);  //both of them.
		}


		//2. Config GPIO port selection in SYSCFG_EXTICR
		     uint8_t temp1,temp2;
		         temp1 = pGPIOx_handle->GPIO_PinConfig.Pin_Num / 4 ;
		         temp2 = pGPIOx_handle->GPIO_PinConfig.Pin_Num % 4;
		         SYSCFG_PCLK_EN();
		     uint32_t portcode;
		         portcode =GPIO_BASEADDR_TO_ITS_CODE(pGPIOx_handle->pGPIOx);
                 SYSCFG->EXTICR[temp1] =  portcode << (4 *temp2);
		//3. Enable EXTI interrupt delivery using IMR (interrupt mask register)
		    EXTI->IMR |= (1<< pGPIOx_handle->GPIO_PinConfig.Pin_Num);

	}
/*/****************************************************  2.config the Ospeed  **********************************************************************/
	tempGPIO=0;
	tempGPIO = pGPIOx_handle->GPIO_PinConfig.Pin_Speed << (2* pGPIOx_handle->GPIO_PinConfig.Pin_Num);
	pGPIOx_handle->pGPIOx->OSPEEDR &=~(0x3 << pGPIOx_handle->GPIO_PinConfig.Pin_Num);
	pGPIOx_handle->pGPIOx->OSPEEDR |= tempGPIO;


/*/***************************************************   3.config the PUPD    **********************************************************************/
	tempGPIO=0;
	tempGPIO = pGPIOx_handle->GPIO_PinConfig.Pin_PuPdType << (2 *pGPIOx_handle->GPIO_PinConfig.Pin_Num);
	pGPIOx_handle->pGPIOx->PUPDR &=~(0x3 << pGPIOx_handle->GPIO_PinConfig.Pin_Num);
	pGPIOx_handle->pGPIOx->PUPDR |=tempGPIO;

/*/***************************************************   4.config the Otype   **********************************************************************/
	tempGPIO=0;
	tempGPIO= pGPIOx_handle->GPIO_PinConfig.Pin_OType << (1 * pGPIOx_handle->GPIO_PinConfig.Pin_OType);
	pGPIOx_handle->pGPIOx->OTYPER &=~(0x1 << pGPIOx_handle->GPIO_PinConfig.Pin_Num);
	pGPIOx_handle->pGPIOx->OTYPER |=tempGPIO;

/*/***************************************************   5.config the Alternate Functionalities   **************************************************/
	if(pGPIOx_handle->GPIO_PinConfig.Pin_Mode == GPIO_MODE_ALTFUNC)
	{
		uint8_t temp1=0 ,temp2=0;
		 temp1=pGPIOx_handle->GPIO_PinConfig.Pin_Num / 8;
		 temp2=pGPIOx_handle->GPIO_PinConfig.Pin_Num % 8;
		 pGPIOx_handle->pGPIOx->AFR[temp1] &=~(0xF << (4*temp2));
		 pGPIOx_handle->pGPIOx->AFR[temp1] |= pGPIOx_handle->GPIO_PinConfig.Pin_AltFuncMode << (4 * temp2);

	}

}
/**************************************************************************************************
 *
 * @name  GPIO_DeInit
 * @brief used for resetting all values of registers of a port in one go!
 * @param pGPIOx
 *
 **************************************************************************************************/
void GPIO_DeInit(GPIO_RegisDef_t * pGPIOx)
{
	if (pGPIOx == GPIOA)
	{
     GPIOA_REG_RESET();
	}
	else if(pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}else if(pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}else if(pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}else if(pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}else if(pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}else if(pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();
	}else if(pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}else if(pGPIOx == GPIOI)
	{
		GPIOI_REG_RESET();
	} // end of nested if's

}//end of if

/*
 * GPIO Peripheral clock setting
 */
/*************************************************************************************************
 * @funName   -  GPIO_PclkControl
 *
 * @brief     - This function enables or disable peripheral clock for the GPIO
 *
 *
 * @para[1]   -base address of the GPIO peripheral
 * @para[2]   -ENABLE or DISABLE
 *
 *
 *
 * @return    - none
 * @Note      - none
 *
 *
 **************************************************************************************************/
void GPIO_PclkControl(GPIO_RegisDef_t * pGPIOx , uint8_t EnorDis)
{
	if( EnorDis == ENABLE )
	{
		if( pGPIOx == GPIOA )
		{
			GPIOA_PCLK_EN();
		}
		else if( pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if ( pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}else if ( pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}else if ( pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}else if ( pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}else if ( pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}else if ( pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}else if ( pGPIOx == GPIOI)
		{
			GPIOI_PCLK_EN();
		}

	}// end of ENABLE
	if( EnorDis == DISABLE )
	{
		if( pGPIOx == GPIOA )
		{
			GPIOA_PCLK_DIS();
		}
		else if ( pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DIS();
		}else if ( pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DIS();
		}else if ( pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DIS();
		}else if ( pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DIS();
		}else if ( pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DIS();
		}else if ( pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DIS();
		}else if ( pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DIS();
		}else if ( pGPIOx == GPIOI)
		{
			GPIOI_PCLK_DIS();
		}

	}// end of DISABLE
}// end of function GPIO_PclckControl


/*
 * GPIO Read  function from PIN
 */
/*************************************************************************************************
 * @funName   -  GPIO_ReadPin
 *
 * @brief     - This function fetches value of IDR of that pin number
 *
 *
 * @para[1]   -base address of the GPIO peripheral
 * @para[2]   -Pin Number
 *
 *
 *
 * @return    - uint8_t type variable to store fetched value
 * @Note      - none
 *
 *
 **************************************************************************************************/
uint8_t GPIO_ReadPin(GPIO_RegisDef_t * pGPIOx, uint8_t Pin_Num)
{  uint8_t temp=0;
   // we are reading so we fetch from the Register , not store
  temp=  (uint8_t)   ( (pGPIOx->IDR >> Pin_Num) & 0x1);
  /*
   * we typecasted to uint8_t because all registers of RegisDef structure are of uint32_t
   * but temp is of uint8_t
   */


  return temp;

}

/*
 * GPIO Write to a  PIN
 */
/*************************************************************************************************
 * @funName   -  GPIO_WritePin
 *
 * @brief     - This function loads value to ODR of that pin number
 *
 *
 * @para[1]   -base address of the GPIO peripheral
 * @para[2]   -Pin Number
 * @para[3]   -Value to be written ; 0 or 1
 *
 *
 * @return    - void
 * @Note      - none
 *
 *
 **************************************************************************************************/
void GPIO_WritePin(GPIO_RegisDef_t * pGPIOx, uint8_t Pin_Num ,uint8_t value)
{  if(value == ( HIGH || SET || ENABLE || 1))
{
	   pGPIOx->ODR &=~ (1 << Pin_Num);
	   pGPIOx->ODR |= (1 << Pin_Num);
}//end nested if
else if(value == (LOW || RESET || DISABLE || 0 ))
{
	pGPIOx->ODR &=~ (1 << Pin_Num);
}//end else if


}//end if
/*
 * GPIO Read from PORT
 */
/*************************************************************************************************
 * @funName   -  GPIO_ReadPort
 *
 * @brief     - This function fetches/ reads the whole Register of that pin
 *
 *
 * @para[1]   -base address of the GPIO peripheral
 *
 * @return    - uint32_t value
 * @Note      - none
 *
 *
 **************************************************************************************************/
uint32_t GPIO_ReadPort(GPIO_RegisDef_t * pGPIOx)
{
 uint32_t value = pGPIOx->IDR ;
 return value;
}
/*
 * GPIO Write to whole Port
 */
/*************************************************************************************************
 * @funName   -  GPIO_WritePort
 *
 * @brief     - This function writes to complete port
 *
 *
 * @para[1]   -base address of the GPIO peripheral
 * @para[2]   -Value to be written
 *
 *
 * @return    - void
 * @Note      - none
 *
 *
 **************************************************************************************************/
void GPIO_WritePort(GPIO_RegisDef_t * pGPIOx , uint32_t value)
{
	pGPIOx->ODR = 0;       //clearing whole port
	pGPIOx->ODR = value;   //writing value to whole port

}
/*
 * GPIO Toggle Output PIN
 */
/*************************************************************************************************
 * @funName   -  GPIO_TogglePin
 *
 * @brief     - This function toggles the bit
 *
 *
 * @para[1]   -base address of the GPIO peripheral
 * @para[2]   -Pin Number
 *
 *
 * @return    - void
 * @Note      - none
 *
 *
 **************************************************************************************************/
void GPIO_TogglePin(GPIO_RegisDef_t *pGPIOx , uint8_t Pin_Num )
{
		pGPIOx->ODR ^=  1 << Pin_Num;
}

/**
 * @name  GPIO_IRQInterruptConfig
 * @brief GPIO IRQ configuration (setting the interrupt config)
 * @param IRQ_Num
 * @param IRQ_Priority
 * @param EnorDis
 */
void GPIO_IRQInterruptConfig(uint8_t IRQ_Num, uint8_t EnorDis)
{
	if(EnorDis == (ENABLE || SET || HIGH || 1))
		{
		  if(IRQ_Num <= 31) //0 to 31
		  {//for ISER0 Interrupt Set-Enable Register 0
             *NVIC_ISER0 |= (1<< IRQ_Num);
		  }
		  else if (IRQ_Num >= 32 && IRQ_Num <=63) // 32 to 63
		  {//for ISER1 Interrupt Set-Enable Register 1
             *NVIC_ISER1 |= (1<< (IRQ_Num % 32 ) );
		  }else if(IRQ_Num >= 64) // 64 and above
		  {//for ISER2 Interrupt Set-Enable Register 2
             *NVIC_ISER2 |= (1<< (IRQ_Num % 64 ) );
		  }

		}
	else
	{
		  if(IRQ_Num <= 31) //0 to 31
		  {//for ICER0 Interrupt Set-Enable Register 0
		     *NVIC_ICER0 |= (1<< IRQ_Num);
		  }
	      else if (IRQ_Num >= 32 && IRQ_Num <=63) // 32 to 63
		  {//for ICER1 Interrupt Set-Enable Register 1
		     *NVIC_ICER1 |= (1<< (IRQ_Num % 32 ) );
		  }else if(IRQ_Num >= 64) // 64 and above
		  {//for ICER2 Interrupt Set-Enable Register 2
		     *NVIC_ICER2 |= (1<< (IRQ_Num % 64 ) );
		  }
	}



}
/**
 * @name  GPIO_IRQPriorityConfig
 * @brief Configures Priority of interrupts
 * @param IRQ_PriorityNum
 * @param IRQ_Num
 */
void GPIO_IRQPriorityConfig(uint8_t IRQ_Num, uint32_t IRQ_PriorityNum)
{
	//1. Finding out IPR register number out of 60 registers!!!!

	uint8_t         IPRx = IRQ_Num /4;           //Reason being : each 4 IRQ register of 8bit sections  in 32bits
	                                              //Usage        : to calculate which IRQ register number to select

	uint8_t IPRx_section = IRQ_Num %4;           //Usage        : to choose the 0th ,1st 2nd or 3rd section of that IRQx Register

	uint8_t shift_amount = (8 * IPRx_section) +(8 - NO_OF_PR_BITS_IMPLEMENTED);
	*(NVIC_IPR_BASEADDR + IPRx ) |= (IRQ_PriorityNum << shift_amount);
}
/**
 * @name  GPIO_IRQHandling
 * @brief GPIO IRQ task to handle when interrupt is triggered
 * @param Pin_Num
 */
void GPIO_IRQHandling(uint8_t Pin_Num)
{   //Very SIMPLE in case of GPIO peripheral
	// handling here means  CLEARING the PR ( Pending Register) of NVIC which is part of processor
	if ( EXTI->PR  &  (1<< Pin_Num)  )    //checking the PR bit at that pin number
	{
	       EXTI->PR |= (1<< Pin_Num);            // Writing '1' into PR register clears it. INSTRUCTED in Cortex-M manual
	}
}





/**
 * @name  GPIO_used
 * @brief configures the port,pin,mode,opseed,otype,pupdtype
 * @param handling_variable
 * @param port
 * @param Pin_Number
 * @param Mode
 * @param Ospeed
 * @param Otype
 * @param PUPDtype
 */
void GPIO_output(GPIO_Handle_t *handling_variable,GPIO_RegisDef_t * port, uint8_t Pin_Number, uint8_t Mode ,uint8_t Ospeed,uint8_t Otype, uint8_t PUPDtype )
{
	handling_variable->pGPIOx = port;
	handling_variable->GPIO_PinConfig.Pin_Num=Pin_Number;
	handling_variable->GPIO_PinConfig.Pin_Mode=Mode;
	handling_variable->GPIO_PinConfig.Pin_Speed=Ospeed;
	handling_variable->GPIO_PinConfig.Pin_OType=Otype;
	handling_variable->GPIO_PinConfig.Pin_PuPdType=PUPDtype;


}
void GPIO_input(GPIO_Handle_t *handling_variable,GPIO_RegisDef_t * port, uint8_t Pin_Number, uint8_t Mode ,uint8_t Ospeed, uint8_t PUPDtype)
{
	    handling_variable->pGPIOx = port;
		handling_variable->GPIO_PinConfig.Pin_Num=Pin_Number;
		handling_variable->GPIO_PinConfig.Pin_Mode=Mode;
		handling_variable->GPIO_PinConfig.Pin_Speed=Ospeed;
		handling_variable->GPIO_PinConfig.Pin_PuPdType=PUPDtype;
}








