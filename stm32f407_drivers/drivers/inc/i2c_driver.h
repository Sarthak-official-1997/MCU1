/*
 * i2c_driver.h
 *
 *  Created on: Mar 26, 2020
 *      Author: Sarthak
 */

#ifndef INC_I2C_DRIVER_H_
#define INC_I2C_DRIVER_H_
#include "stm32f407xx.h"

/*
 * Register Definition for I2Cx peripheral
 */

typedef struct
{
	vola uint32_t CR1;
    vola uint32_t CR2;
    vola uint32_t OAR1;
    vola uint32_t DR;
    vola uint32_t SR1;
    vola uint32_t SR2;
    vola uint32_t CCR;
    vola uint32_t TRISE;
    vola uint32_t FLTR;

}I2C_RegisDef_t;

/*
 * Configuration structure for I2Cx peripheral
 */
typedef struct
{
uint32_t I2C_SCLSpeed;
uint8_t  I2C_DeviceAddress;
uint32_t I2C_ACKControl;
uint32_t I2C_FMDutyCycle;

}I2C_PinConfig_t;

/**
 * Handle structure for I2Cx peripheral
 */
typedef struct
{
 I2C_RegisDef_t *pI2Cx;

 I2C_PinConfig_t I2C_Config;
 uint8_t 		*pTxBuffer; /* !< To store the app. Tx buffer address > */
 	uint8_t 		*pRxBuffer;	/* !< To store the app. Rx buffer address > */
 	uint32_t 		TxLen;		/* !< To store Tx len > */
 	uint32_t 		RxLen;		/* !< To store Tx len > */
 	uint8_t 		TxRxState;	/* !< To store Communication state > */
 	uint8_t 		DevAddr;	/* !< To store slave/device address > */
     uint32_t        RxSize;		/* !< To store Rx size  > */
     uint8_t         Sr;			/* !< To store repeated start value  > */

}I2C_Handle_t;


/*
 * I2C application states
 */
#define I2C_READY 					0
#define I2C_BUSY_IN_RX 				1
#define I2C_BUSY_IN_TX 				2
/*
 * @I2C_SCLSPeed
 */
#define I2C_SCL_SPEED_SM     100000    // standard speed 100kbits per sec
#define I2C_SCL_SPEED_FM4K   400000
#define I2C_SCL_SPEED_FM2K   200000



/*
 * @I2C_ACKControl
 */
#define I2C_ACK_EN           1
#define I2C_ACK_DIS          0
/*
 * @I2C_FMDutyCycle
 */
#define I2C_FM_DUTY_2         0
#define I2C_FM_DUTY_16_9      1


/*
 * I2C related status flags definitions
 */
#define I2C_FLAG_TXE   		( 1 << I2C_SR1_TxE)
#define I2C_FLAG_RXNE   	( 1 << I2C_SR1_RxNE)
#define I2C_FLAG_SB			( 1 << I2C_SR1_SB)
#define I2C_FLAG_OVR  		( 1 << I2C_SR1_OVR)
#define I2C_FLAG_AF   		( 1 << I2C_SR1_AF)
#define I2C_FLAG_ARLO 		( 1 << I2C_SR1_ARLO)
#define I2C_FLAG_BERR 		( 1 << I2C_SR1_BERR)
#define I2C_FLAG_STOPF 		( 1 << I2C_SR1_STOPF)
#define I2C_FLAG_ADD10 		( 1 << I2C_SR1_ADD10)
#define I2C_FLAG_BTF  		( 1 << I2C_SR1_BTF)
#define I2C_FLAG_ADDR 		( 1 << I2C_SR1_ADDR)
#define I2C_FLAG_TIMEOUT 	( 1 << I2C_SR1_TIMEOUT)

#define I2C_DISABLE_SR  	RESET
#define I2C_ENABLE_SR   	SET


/*
 * I2C application events macros
 */
#define I2C_EV_TX_CMPLT  	 	0
#define I2C_EV_RX_CMPLT  	 	1
#define I2C_EV_STOP       		2
#define I2C_ERROR_BERR 	 		3
#define I2C_ERROR_ARLO  		4
#define I2C_ERROR_AF    		5
#define I2C_ERROR_OVR   		6
#define I2C_ERROR_TIMEOUT 		7
#define I2C_EV_DATA_REQ         8
#define I2C_EV_DATA_RCV         9


/**********************************************************************************
 *                           API's Supported by this I2C driver
 *********************************************************************************/
/*
 * peripheral clock setup
 */
void I2C_PclkControl(I2C_RegisDef_t *pI2Cx, uint8_t EnorDis);

/*
 * I2C Init and DeInit
 */
void I2C_Init(I2C_Handle_t * /*pointer variable to access elements of handle structure*/); // used for configuring the port and pin setting
void I2C_DeInit(I2C_RegisDef_t * /*base address of the I2C port*/);          //used for resetting all values of registers of a port in one go


/*
 * Send Data Receive Data
 */

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle,uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr,uint8_t Sr);
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr);
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr,uint8_t Sr);

void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle);
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle);


void I2C_SlaveSendData(I2C_RegisDef_t *pI2C,uint8_t data);
uint8_t I2C_SlaveReceiveData(I2C_RegisDef_t *pI2C);



/*
 * I2C IRQ configuration (setting the interrupt config)
 */
void I2C_IRQInterruptConfig(uint8_t IRQ_Num, uint8_t EnorDis);

/*
 * I2C IRQ Priority configuration
 */
void I2C_IRQPriorityConfig(uint8_t IRQ_Num, uint32_t IRQ_PriorityNum);

void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);

/*
 * Other Peripheral Control APIs
 */
void I2C_PeriheralControl(I2C_RegisDef_t *pI2Cx, uint8_t EnorDis);

uint8_t I2C_GetFlagStatus(I2C_RegisDef_t *pI2Cx, uint32_t FlagName);
void I2C_ManageAcking(I2C_RegisDef_t *pI2Cx, uint8_t EnorDi);
void I2C_GenerateStopCondition(I2C_RegisDef_t *pI2Cx);

void I2C_SlaveEnableDisableCallbackEvents(I2C_RegisDef_t *pI2Cx,uint8_t EnorDi);


/*
 * Application Callback
 */
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle,uint8_t AppEvent);












#endif /* INC_I2C_DRIVER_H_ */
