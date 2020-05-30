/*
 * spi_driver.h
 *
 *  Created on: Mar 20, 2020
 *      Author: Sarthak
 */

#ifndef INC_SPI_DRIVER_H_
#define INC_SPI_DRIVER_H_

#include "stm32f407xx.h"

/*****************************************************************************************************************
 *                                    This SPI header file contains
 * @1. Register Definition structure for a SPI
 * @2. SPIx Pin Configuration structure implemented here
 * @3. SPI Handle Structure to handle 1 and 2 both.
 * @4. SPI
 * @5. API's supported by this SPI driver
 *
 *****************************************************************************************************************/



/*************************************************************************************************************
 * @Name                                        : @1. Register Definition structure for a SPI PIN
 * @Variable type used to access this structure : pointer variable
 * @Reason for variable type					: Reference to addresses (an address is a pointer to its data)
 *************************************************************************************************************/

typedef struct
{
uint32_t CR1;                          /*!< control register 1,     						    Addr offset: 0x00 */
uint32_t CR2;                          /*!< control register 2,     						    Addr offset: 0x00 */
uint32_t SR;                           /*!< status register,     						        Addr offset: 0x00 */
uint32_t DR;                           /*!< data register,     						        	Addr offset: 0x00 */
uint32_t CRCPR;                        /*!< CRC polynomial register,     				    	Addr offset: 0x00 */
uint32_t RXCRCR;                       /*!< CRC register,     						            Addr offset: 0x00 */
uint32_t TXCRCR;                       /*!< CRC register,     						        	Addr offset: 0x00 */
uint32_t I2SCFGR;                      /*!< configuration register,     						Addr offset: 0x00 */
uint32_t I2SPR;                        /*!< prescaler register,     						    Addr offset: 0x00 */

}SPI_RegisDef_t;


/*************************************************************************************************************
 * @Name                                        : @2. SPI Pin Config structure
 * @Variable type used to access this structure : Normal variable
 * @Reason for variable type                    : uint8_t data compared to uint8_t or Macros
 *************************************************************************************************************/
typedef struct
{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;

}SPI_PinConfig_t;

/*************************************************************************************************************
 *@Name                                       : @3.     HANDLE structure for SPI
 *@Variable type used to access this structure: pointer variable
 *@Reason for variable type                   : No comparing of stdint data | reference to another structure
 *************************************************************************************************************/

typedef struct
{
    SPI_RegisDef_t *pSPIx;

    SPI_PinConfig_t SPI_PinConfig;

}SPI_Handle_t;

/*
 * FLAGS related to SPI peripheral
 */
#define SPI_TXE_FLAG                          (1<< SPI_SR_TXE)
#define SPI_RXNE_FLAG                         (1<< SPI_SR_RXNE)
#define SPI_BSY_FLAG                          (1<< SPI_SR_BSY)
#define SPI_FRE_FLAG                          (1<< SPI_SR_FRE)
#define SPI_OVR_FLAG                          (1<< SPI_SR_OVR)
#define SPI_MODF_FLAG                         (1<< SPI_SR_MODF)
#define SPI_UDR_FLAG                          (1<< SPI_SR_UDR)
#define SPI_CRCERR_FLAG                       (1<< SPI_SR_CRCERR)
#define SPI_CHSIDE_FLAG                       (1<< SPI_SR_CHSIDE)
/*
 * @SPI_DeviceMode
 */
#define SPI_DEVICE_MODE_MASTER                     1
#define SPI_DEVICE_MODE_SLAVE                      0

/*
 * @SPI_BusConfig
 */
#define SPI_BUSCONFIG_FULL_DUPLEX                  1
#define SPI_BUSCONFIG_HALF_DUPLEX                  2
//#define SPI_BUSCONFIG_SIMPLEX_TX_ONLY              3
#define SPI_BUSCONFIG_SIMPLEX_RX_ONLY              3

/*
 * @SPI_SclkSpeed
 */
#define SPI_SCLK_SPEED_DIV2                        0
#define SPI_SCLK_SPEED_DIV4                        1
#define SPI_SCLK_SPEED_DIV8                        2
#define SPI_SCLK_SPEED_DIV16                       3
#define SPI_SCLK_SPEED_DIV32                       4
#define SPI_SCLK_SPEED_DIV64                       5
#define SPI_SCLK_SPEED_DIV128                      6
#define SPI_SCLK_SPEED_DIV256                      7

/*
 * @SPI_DFF
 */
#define SPI_DFF_8BITS                              0
#define SPI_DFF_16BITS                             1

/*
 * @SPI_CPOL
 */
#define SPI_CPOL_0_IDLESTATE                       0
#define SPI_CPOL_1_IDLESTATE                       1

/*
 * @SPI_CPHA
 */

#define SPI_CPHA_DATACAPTURE_1ST_EDGE              0
#define SPI_CPHA_DATACAPTURE_2ND_EDGE              1

/*
 * @SPI_SSM
 */
#define SPI_SSM_ENABLE                             1
#define SPI_SSM_DISABLE                            0

/********************************************************************************************
 *                         @. API's supported by  SPI driver
 ********************************************************************************************/
/*
 * SPI Init and DeInit
 */
void SPI_Init(SPI_Handle_t *pSPIHandle /*pointer variable to access elements of handle structure*/); // used for configuring the port and pin setting
void SPI_DeInit(SPI_RegisDef_t * /*base address of the SPIx*/);          //used for resetting all values of registers of a port in one go

/*
 * SPI Periphal clock setting
 */
void SPI_PclkControl(SPI_RegisDef_t * /*base address of the SPIx*/ , uint8_t /*enable or disable variable*/);

/*
 * Data Send and Receive
 */

void SPI_SendData(SPI_RegisDef_t *pSPIx , uint8_t *pTxBuffer , uint32_t len);

void SPI_ReceiveData(SPI_RegisDef_t *pSPIx , uint8_t *pRxBuffer , uint32_t len);



/*********************************************************************************************************************************
 *                                  Processor Specific Fucntions to handle Interrupts
 *********************************************************************************************************************************/
/*
 * SPI IRQ configuration (setting the interrupt config)
 */
void SPI_IRQInterruptConfig(uint8_t IRQ_Num, uint8_t EnorDis);

/*
 * SPI IRQ Priority configuration
 */
void SPI_IRQPriorityConfig(uint8_t IRQ_Num, uint32_t IRQ_PriorityNum);

/*
 * SPI IRQ task to handle when interrupt is triggered
 */
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);

/*********************************************************************************************************************************/



/*
 * other peripheral control API's
 */
void SPI_PeripheralControl(SPI_RegisDef_t *pSPIx, uint8_t EnorDis);

void SPI_SSIconfig(SPI_RegisDef_t *pSPIx, uint8_t EnorDis);

void SPI_SSOEconfig(SPI_RegisDef_t *pSPIx, uint8_t EnorDis);


#endif /* INC_SPI_DRIVER_H_ */
