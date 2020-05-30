/*
 * spi_driver.c
 *
 *  Created on: Mar 20, 2020
 *      Author: Sarthak
 */

#include "spi_driver.h"


uint8_t GetFlagStatus(SPI_RegisDef_t *pSPIx , uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	else
		return FLAG_RESET;
}


/***********************************************************************************************************************
 *                                              A P I's supported by  SPI driver
 ***********************************************************************************************************************/

/*
 * SPI Init and DeInit
 */
/********************************************************************************************************
 * @name  SPI_Init
 * @brief configs the CR1
 * @param pSPIHandle
 *********************************************************************************************************/
void SPI_Init(SPI_Handle_t *pSPIHandle )
{   //perpheral clock enable
	SPI_PclkControl(pSPIHandle->pSPIx, ENABLE);

	//config the CR1 control register 1

	uint32_t tempreg=0;

	//1.configuring the Device Mode

	tempreg |= pSPIHandle->SPI_PinConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	//2. configuring the busconfig
	 if(pSPIHandle->SPI_PinConfig.SPI_BusConfig == SPI_BUSCONFIG_FULL_DUPLEX)
	 {
		 //BIDImode should be cleared
		 tempreg &=~(1<<SPI_CR1_BIDIMODE);
	 }else if (pSPIHandle->SPI_PinConfig.SPI_BusConfig == SPI_BUSCONFIG_HALF_DUPLEX)
	 {
		 //BIDImode should be set
		 tempreg|=(1<<SPI_CR1_BIDIMODE);
	 }else if(pSPIHandle->SPI_PinConfig.SPI_BusConfig == SPI_BUSCONFIG_SIMPLEX_RX_ONLY)
	 {
		 //BIDImode should be cleared
		 tempreg &=~(1<<SPI_CR1_BIDIMODE);
		 //RXONLY bit should be set
		 tempreg |= (1<<SPI_CR1_RXONLY);
	 }
     //3.config the SPI SCLK SPEED (baud rate)
	 tempreg|= pSPIHandle->SPI_PinConfig.SPI_SclkSpeed << SPI_CR1_BR;

	 //4.config the DFF
	 tempreg |=pSPIHandle->SPI_PinConfig.SPI_DFF <<SPI_CR1_DFF;

	 //5.config the CPOL
	 tempreg|= pSPIHandle->SPI_PinConfig.SPI_CPOL <<SPI_CR1_CPOL;
	  //6.config the CPHA
	 tempreg|= pSPIHandle->SPI_PinConfig.SPI_CPHA <<SPI_CR1_CPHA;

	 //7.config the SSM
	 tempreg|= pSPIHandle->SPI_PinConfig.SPI_SSM <<SPI_CR1_SSM;

/*/****  we set all the required bits of tempreg , now setting the complete CR1 register ****/
	 pSPIHandle->pSPIx->CR1 = tempreg;

}
/**
 * @name  SPI_DeInit
 * @brief resets all the values of the SPIx
 * @param pSPIx
 */
void SPI_DeInit(SPI_RegisDef_t * pSPIx)
{
	if (pSPIx == SPI1)
		{
	     SPI1_REG_RESET();
		}
		else if(pSPIx == SPI2)
		{
			SPI2_REG_RESET();
		}else if(pSPIx == SPI3)
		{
			SPI3_REG_RESET();
		}

}

/*
 * SPI Peripheral clock setting
 */
void SPI_PclkControl(SPI_RegisDef_t *pSPIx , uint8_t EnorDis)
{
	if( EnorDis == ENABLE )
		{
			if( pSPIx == SPI1 )
			{
				SPI1_PCLK_EN();
			}
			else if( pSPIx == SPI2)
			{
				SPI2_PCLK_EN();
			}
			else if ( pSPIx == SPI3)
			{
				SPI3_PCLK_EN();
			}

		}// end of ENABLE
		if( EnorDis == DISABLE )
		{
			if( pSPIx == SPI1 )
			{
				SPI1_PCLK_DIS();
			}
			else if ( pSPIx == SPI2)
			{
				SPI2_PCLK_DIS();
			}else if ( pSPIx == SPI3)
			{
				SPI3_PCLK_DIS();
			}

		}// end of DISABLE

}//end of if

/*
 * Data Send and Receive
 */
/**
 * @name  SPI_SendData
 * @brief sends 8/16 bit data
 * @param pSPIx
 * @param pTxBuffer
 * @param len
 * @note  this is a blocking call
 */
void SPI_SendData(SPI_RegisDef_t *pSPIx , uint8_t *pTxBuffer , uint32_t len)
{
	while(len>0)
	{
		//1. wait till flag TXE is set, TXE is enabled when TX buffer is emptied
		 while ( GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET );

		 //2.check DFF bit in CR1
		 if( pSPIx->CR1 & (1<< SPI_CR1_DFF) )
		 { //then 16bit data frame format
			 //2.1 Load the data into Data register DR to send it to txbuffer to shift register
			  pSPIx->DR |= *((uint16_t *) pTxBuffer);        //typecasted to 16 cz 16bit DFF
			                                                 //dereferenced the pointer pTxBuffer to get value of it.
			  len--;
			  len--;          //decreased 2 times because 2Bytes ~ 16 bits were sent
		   //increase the buffer to point to next 2 bytes data
			  (uint16_t *)pTxBuffer++;

		 }else
		 {// 8bit data fram format
			 pSPIx->DR |= *pTxBuffer;                       //dereferenced the pointer pTxBuffer to get value of it.
			 len--;           //decreased by 1byte because 8bits were sent
			 //increase the buffer to point to next 1 byte data
			  pTxBuffer++;
		 }
	}
}

void SPI_ReceiveData(SPI_RegisDef_t *pSPIx , uint8_t *pRxBuffer , uint32_t len)
{
	while(len>0)
	{
		//1. wait till flag RXNE is set, RXNE is enabled when RX buffer is full
		 while ( GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET );

		 //2.check DFF bit in CR1
		 if( pSPIx->CR1 & (1<< SPI_CR1_DFF) )
		 { //then 16bit data frame format
			 //2.1 Load the data from  Data register DR
			  *((uint16_t *) pRxBuffer) = pSPIx->DR;        //typecasted to 16 cz 16bit DFF
			                                                 //dereferenced the pointer pRxBuffer to get value of it.
			  len--;
			  len--;          //decreased 2 times because 2Bytes ~ 16 bits were received
		   //increase the buffer to point to next 2 bytes data
			  (uint16_t *)pRxBuffer++;

		 }else
		 {// 8bit data fram format
			 *pRxBuffer = pSPIx->DR;                       //dereferenced the pointer pRxBuffer to get value of it.
			 len--;           //decreased by 1byte because 8bits were sent
			 //increase the buffer to point to next 1 byte data
			  pRxBuffer++;
		 }
	}



}



void SPI_PeripheralControl(SPI_RegisDef_t *pSPIx, uint8_t EnorDis)
{
	if(EnorDis == ENABLE)
	{
		pSPIx->CR1 |= ( 1<< SPI_CR1_SPE);
	}else
		pSPIx->CR1 &=~(1<< SPI_CR1_SPE);



}
void SPI_SSIconfig(SPI_RegisDef_t *pSPIx, uint8_t EnorDis)
{
	if(EnorDis == ENABLE)
		{
			pSPIx->CR1 |= ( 1<< SPI_CR1_SSI);
		}else
			pSPIx->CR1 &=~(1<< SPI_CR1_SSI);


}

void SPI_SSOEconfig(SPI_RegisDef_t *pSPIx, uint8_t EnorDis)
{
	if(EnorDis == ENABLE)
	{
		pSPIx->CR2 |= (1<< SPI_CR2_SSOE);

	}else
		pSPIx->CR2 &=~(1<< SPI_CR2_SSOE);

}

