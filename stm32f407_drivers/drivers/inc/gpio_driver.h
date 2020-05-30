/*
 * gpio_driver.h
 *
 *  Created on: Mar 16, 2020
 *      Author: Sarthak
 */

#ifndef INC_GPIO_DRIVER_H_
#define INC_GPIO_DRIVER_H_

#include "stm32f407xx.h"
/*****************************************************************************************************************
 *                                    This GPIO header file contains
 * @1. Register Definition structure for a GPIO PIN
 * @2. GPIOx Pin Configuration structure implemented here
 * @3. GPIO Handle Structure to handle 1 and 2 both.
 * @4. GPIO MODES --- input,ouput,alternate functionality, analog
 * @5. API's supported by this GPIO driver
 *
 *****************************************************************************************************************/


/*************************************************************************************************************
 * @Name                                        : @1. Register Definition structure for a GPIO PIN
 * @Variable type used to access this structure : pointer variable
 * @Reason for variable type					: Reference to addresses (an address is a pointer to its data)
 *************************************************************************************************************/


typedef struct{
vola uint32_t MODER;                       //Addr offset :0x00
vola uint32_t OTYPER;                      //Addr offset :0x04
vola uint32_t OSPEEDR;                     //Addr offset :0x08
vola uint32_t PUPDR;                       //Addr offset :0x0C
vola uint32_t IDR;                         //Addr offset :0x10
vola uint32_t ODR;                         //Addr offset :0x14
vola uint32_t BSRR;                        //Addr offset :0x18
vola uint32_t LCKR;                        //Addr offset :0x1C
vola uint32_t AFR[2];                      //Addr offset :0x20  // AFR[0] is AFRL , AFR[1] is AFRH
} GPIO_RegisDef_t;


/*************************************************************************************************************
 * @Name                                        : @2. PIN CONFIG structure for GPIO PIN
 * @Variable type used to access this structure : Normal variable
 * @Reason for variable type                    : uint8_t data compared to uint8_t or Macros
 *************************************************************************************************************/
typedef struct
{
	uint8_t Pin_Num;                  //pin number 0-15
	uint8_t Pin_Mode;                 //ouput/input/alternate function / analog
	uint8_t Pin_Speed;                //clock speed low,medium,high,very high
	uint8_t Pin_PuPdType;             //pull-up/pull-down  resistor
	uint8_t Pin_OType;                //Open Drain or Push-pull
	uint8_t Pin_AltFuncMode;          //config the pin for AF0-Af15
}GPIO_PinConfig_t;



/*************************************************************************************************************
 *@Name                                       : @3.     HANDLE structure for GPIO
 *@Variable type used to access this structure: pointer variable
 *@Reason for variable type                   : No comparing of stdint data | reference to another structure
 *************************************************************************************************************/

typedef struct
{
	GPIO_RegisDef_t *pGPIOx;           /* pGPIOx is a pointer to base address of the GPIO port x ,x=A,B,C....I
	                                      same as GPIO_RegisDef_t *ptr = GPIOA / GPIOB/ .....GPIOI
	                                   */
	GPIO_PinConfig_t GPIO_PinConfig;



}GPIO_Handle_t;



/********************************************************************************************
 *                          @4.     GPIO pin MODES
 ********************************************************************************************/
#define GPIO_MODE_INPUT                   0                 //Input mode
#define GPIO_MODE_OUPUT                   1                 //Output mode
#define GPIO_MODE_ALTFUNC                 2                 //Alternate functionality mode
#define GPIO_MODE_ANALOG                  3                 //Analog mode
#define GPIO_MODE_IT_FT                   4                 //Interrup Falling edge Trigger
#define GPIO_MODE_IT_RT                   5                 //Interrupt Rising edge Trigger
#define GPIO_MODE_IT_RFT                  6                 //Interrupt rising and falling T

/********************************************************************************************
 *                          @5.     GPIO pin OUTPUT TYPE
 ********************************************************************************************/
#define GPIO_OTYPE_PP                     0                 //Push-Pull
#define GPIO_OTYPE_OD                     1                 //Op en Drain

/********************************************************************************************
 *                          @6.     GPIO pin SPEED
 ********************************************************************************************/
#define GPIO_OSPEED_LOW                   0
#define GPIO_OSPEED_MED                   1
#define GPIO_OSPEED_HIGH                  2
#define GPIO_OSPEED_VERYHIGH              3

/********************************************************************************************
 *                          @7.     GPIO pin PUPD
 ********************************************************************************************/
#define GPIO_PUPD_NO_PU_PD                0                  //No PU/PD
#define GPIO_PUPD_PU                      1                  //Pull UP
#define GPIO_PUPD_PD                      2                  //Pull Down


/********************************************************************************************
 *                         @. API's supported by  GPIO driver
 ********************************************************************************************/
/*
 * GPIO Init and DeInit
 */
void GPIO_Init(GPIO_Handle_t * /*pointer variable to access elements of handle structure*/); // used for configuring the port and pin setting
void GPIO_DeInit(GPIO_RegisDef_t * /*base address of the GPIO port*/);          //used for resetting all values of registers of a port in one go

/*
 * GPIO Periphal clock setting
 */
void GPIO_PclkControl(GPIO_RegisDef_t * /*base address of the GPIO port*/ , uint8_t /*enable or disable variable*/);

/*
 * GPIO Read / Write function to PIN
 */
uint8_t GPIO_ReadPin(GPIO_RegisDef_t * /*base address of the GPIO port*/, uint8_t /*variable to store pin number*/);
void GPIO_WritePin(GPIO_RegisDef_t * /*base address of the GPIO port*/, uint8_t /*variable to store pin number*/ ,uint8_t /*variable to store value to be written*/);
/*
 * GPIO Read/Write function to PORT
 */
uint32_t GPIO_ReadPort(GPIO_RegisDef_t * /*base address of the GPIO port*/) ;
void GPIO_WritePort(GPIO_RegisDef_t * /*base address of the GPIO port*/ , uint32_t /*variable to store value to be written*/);
/*
 * GPIO Toggle Output PIN
 */
void GPIO_TogglePin(GPIO_RegisDef_t * /*base address of the GPIO port*/ , uint8_t /*variable to store pin number*/ );


/*********************************************************************************************************************************
 *                                  Processor Specific Fucntions to handle Interrupts
 *********************************************************************************************************************************/
/*
 * GPIO IRQ configuration (setting the interrupt config)
 */
void GPIO_IRQInterruptConfig(uint8_t IRQ_Num, uint8_t EnorDis);

/*
 * GPIO IRQ Priority configuration
 */
void GPIO_IRQPriorityConfig(uint8_t IRQ_Num, uint32_t IRQ_PriorityNum);

/*
 * GPIO IRQ task to handle when interrupt is triggered
 */
void GPIO_IRQHandling(uint8_t Pin_Num);

/*********************************************************************************************************************************/







/**
 * @name  GPIO_output
 * @brief configures the port,pin,mode,opseed,otype,pupdtype as in output mode
 * @param handling_variable
 * @param port
 * @param Pin_Number
 * @param Mode
 * @param Ospeed
 * @param Otype
 * @param PUPDtype
 */
void GPIO_output(GPIO_Handle_t *handling_variable,GPIO_RegisDef_t * port, uint8_t Pin_Number, uint8_t Mode ,uint8_t Ospeed,uint8_t Otype, uint8_t PUPDtype);


/**
 * @name  GPIO_input
 * @brief configures the port,pin,mode,opseed,pupdtype as in input mode
 * @param handling_variable
 * @param port
 * @param Pin_Number
 * @param Mode
 * @param Ospeed
 * @param PUPDtype
 */
void GPIO_input(GPIO_Handle_t *handling_variable,GPIO_RegisDef_t * port, uint8_t Pin_Number, uint8_t Mode ,uint8_t Ospeed, uint8_t PUPDtype);












#endif /* INC_GPIO_DRIVER_H_ */
