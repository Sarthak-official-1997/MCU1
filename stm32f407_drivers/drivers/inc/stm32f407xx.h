/*
 * stm32f407xx.h
 *
 *  Created on: Mar 16, 2020
 *      Author: Sarthak
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include<stdint.h>
#include<string.h>


#define vola                       volatile
#define  NO_PR_BITS_IMPLEMENTED      4

/*
 *                                   BASE ADDRESS
 *
 *                                   M A C R O 's
 */

/********************base addresses of Flash and SRAM memories *****************************/
#define FLASH_BASEADDR                    0x08000000U
#define SRAM1_BASEADDR                    0x20000000U
#define SRAM2_BASEADDR                    ((uint32_t )0x20001C00)               //0x20001C00U
#define SRAM3_BASEADDR                    ((uint32_t )0x20002000)               //0x20002000U
#define ROM_BASEADDR                      0x1FFF0000U
#define SRAM                              SRAM1_BASEADDR



 /*************************AHBx and APBx bus peripheral addresses********************************/

#define PERIPH_BASEADDR                   0x40000000U
#define APB1PERIPH_BASEADDR               0x40000000U                       //same as PERIPH_BASE
#define APB2PERIPH_BASEADDR               0x40010000U
#define AHB1PERIPH_BASEADDR               (PERIPH_BASEADDR +0x00020000U)          //0x4002 0000
#define AHB2PERIPH_BASEADDR               0x50000000U


/************************base address of peripherals hanging on AHB1 bus ************************/

#define GPIOA_BASEADDR                    (AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR                    (AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR                    (AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR                    (AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR                    (AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR                    (AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR                    (AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR                    (AHB1PERIPH_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR                    (AHB1PERIPH_BASEADDR + 0x2000)
#define RCC_BASEADDR                      (AHB1PERIPH_BASEADDR + 0x3800)


/*************************base address of peripherals hanging on APB1 bus ************************/

#define I2C1_BASEADDR                     (APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR                     (APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR                     (APB1PERIPH_BASEADDR + 0x5C00)
#define SPI2_BASEADDR                     (APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR                     (APB1PERIPH_BASEADDR + 0x3C00)
#define USART2_BASEADDR                   (APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR                   (APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR                    (APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR                    (APB1PERIPH_BASEADDR + 0x5000)

/*************************base address of peripherals hanging on APB2 bus ************************/

#define SPI1_BASEADDR                     (APB2PERIPH_BASEADDR + 0x3000)
//#define SPI4_BASEADDR                     (APB2PERIPH_BASEADDR + 0x3400)
#define EXTI_BASEADDR                     (APB2PERIPH_BASEADDR + 0x3C00)
#define SYSCFG_BASEADDR                   (APB2PERIPH_BASEADDR + 0x3800)
#define USART1_BASEADDR                   (APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR                   (APB2PERIPH_BASEADDR + 0x1400)


/**************************************************************************************************
 *                                     S T R U C T U R E S
 **************************************************************************************************/



#if(0)      //Using Pre-Processor #if(conditon) to exclude from build


/*********************************NOW using this structure*******************************************/

    GPIO_RegisDef_t *ptr= (GPIO_RegisDef_t *) 0x40020000;    //where 0x4002000 is BASEADDR of GPIOA

/************************************** IS same as **************************************************/
    #define GPIOA_BASEADDR           0x4002000
    #define GPIOA                    ( (GPIO_RegisDef_t *) GPIOA_BASEADDR )
    GPIO_RegisDef_t  *ptr =GPIOA;

#endif

typedef struct
{
 vola uint32_t CR;            /*!< clock control register,     								Addr offset: 0x00 */
 vola uint32_t PLLCFGR;       /*!< PLL configuration register,     							Addr offset: 0x04 */
 vola uint32_t CFGR;          /*!< clock configuration register,     						Addr offset: 0x08 */
 vola uint32_t CIR;           /*!< clock interrupt register,     							Addr offset: 0x0C */
 vola uint32_t AHB1RSTR;      /*!< AHB1 peripheral reset register,     						Addr offset: 0x10 */
 vola uint32_t AHB2RSTR;      /*!< AHB2 peripheral reset register,     						Addr offset: 0x14 */
 vola uint32_t AHB3RSTR;      /*!< AHB3 peripheral reset register     						Addr offset: 0x18 */
      uint32_t      RESERVED0;/*!< Reserved, 0x1C                                    		                  */
 vola uint32_t APB1RSTR;      /*!< APB1 peripheral reset register,     						Addr offset: 0x20 */
 vola uint32_t APB2RSTR;      /*!< APB2 peripheral reset register,     						Addr offset: 0x24 */
      uint32_t      RESERVED1[2];  /*!< Reserved, 0x28-0x2C                                                   */
 vola uint32_t AHB1ENR;       /*!< AHB1 peripheral clock register,     						Addr offset: 0x30 */
 vola uint32_t AHB2ENR;       /*!< AHB2 peripheral clock register,     						Addr offset: 0x34 */
 vola uint32_t AHB3ENR;       /*!< AHB3 peripheral clock register,     						Addr offset: 0x38 */
      uint32_t      RESERVED2;     /*!< Reserved, 0x3C                                                        */
 vola uint32_t APB1ENR;       /*!< APB1 peripheral clock enable register,     				Addr offset: 0x40 */
 vola uint32_t APB2ENR;       /*!< APB2 peripheral clock enable register,     				Addr offset: 0x44 */
      uint32_t      RESERVED3[2];  /*!< Reserved, 0x48-0x4C                                                   */
 vola uint32_t AHB1LPENR;     /*!< AHB1 peripheral clock enable in low power mode register, Addr offset: 0x50 */
 vola uint32_t AHB2LPENR;     /*!< AHB2 peripheral clock enable in low power mode register, Addr offset: 0x54 */
 vola uint32_t AHB3LPENR;     /*!< AHB3 peripheral clock enable in low power mode register, Addr offset: 0x58 */
      uint32_t      RESERVED4;     /*!< Reserved, 0x5C                                                        */
 vola uint32_t APB1LPENR;     /*!< APB1 peripheral clock enable in low power mode register, Addr offset: 0x60 */
 vola uint32_t APB2LPENR;     /*!< APB2 peripheral clock enable in low power mode register, Addr offset: 0x64 */
      uint32_t      RESERVED5[2];  /*!< Reserved, 0x68-0x6C                                                   */
 vola uint32_t BDCR;          /*!< Backup domain control register,     						Addr offset: 0x70 */
 vola uint32_t CSR;           /*!< clock control & status register,     					Addr offset: 0x74 */
      uint32_t      RESERVED6[2];  /*!< Reserved, 0x78-0x7C                                                   */
 vola uint32_t SSCGR;         /*!< spread spectrum clock generation register,     			Addr offset: 0x80 */
 vola uint32_t PLLI2SCFGR;    /*!< PLLI2S configuration register,     						Addr offset: 0x84 */
 //vola uint32_t PLLSAICFGR;    /*!< PLL configuration register,     							Addr offset: 0x88 */
 //vola uint32_t DCKCFGR;       /*!< Dedicated Clock Configuration Register,     				Addr offset: 0x8C */

 } RCC_RegisDef_t;

typedef struct
{
vola uint32_t IMR;             /*!< Interrupt mask register,     						    Addr offset: 0x00 */
vola uint32_t EMR;             /*!< Event mask register,     						        Addr offset: 0x04 */
vola uint32_t RTSR;            /*!< Rising trigger selection register,     				    Addr offset: 0x08 */
vola uint32_t FTSR;            /*!< Falling trigger selection register,     				Addr offset: 0x0C */
vola uint32_t SWIER;           /*!< Software interrupt event register,     					Addr offset: 0x10 */
vola uint32_t PR;              /*!< Pending register,     									Addr offset: 0x14 */

}EXTI_RegisDef_t;

typedef struct
{
vola uint32_t MEMRMP;          /*!< memory remap register,     							    Addr offset: 0x00 */
vola uint32_t PMC;             /*!< peripheral mode configuration register,     			Addr offset: 0x04 */
vola uint32_t EXTICR[4];         /*!< external interrupt configuration register 1,     	    Addr offset: 0x08-014 */
vola uint32_t CMPCR;           /*!< Compensation cell control register,     				Addr offset: 0x20 */

}SYSCFG_RegisDef_t;


/**************************************************************************************************
 *
 * Peripheral Definitions  (( Base addresses typecasted to  xxx_RegisDef_t*))
 *
 **************************************************************************************************/

#define GPIOA  				((GPIO_RegisDef_t*)GPIOA_BASEADDR)              // G
#define GPIOB  				((GPIO_RegisDef_t*)GPIOB_BASEADDR)
#define GPIOC  				((GPIO_RegisDef_t*)GPIOC_BASEADDR)              // P
#define GPIOD  				((GPIO_RegisDef_t*)GPIOD_BASEADDR)
#define GPIOE  				((GPIO_RegisDef_t*)GPIOE_BASEADDR)              // I
#define GPIOF  				((GPIO_RegisDef_t*)GPIOF_BASEADDR)
#define GPIOG  				((GPIO_RegisDef_t*)GPIOG_BASEADDR)              // O
#define GPIOH  				((GPIO_RegisDef_t*)GPIOH_BASEADDR)
#define GPIOI  				((GPIO_RegisDef_t*)GPIOI_BASEADDR)              // S

#define RCC 				((RCC_RegisDef_t*)RCC_BASEADDR)                 // RCC
#define EXTI				((EXTI_RegisDef_t*)EXTI_BASEADDR)               // EXTI
#define SYSCFG				((SYSCFG_RegisDef_t*)SYSCFG_BASEADDR)           // SYSCFG


#define SPI1  				((SPI_RegisDef_t*)SPI1_BASEADDR)                // S
#define SPI2  				((SPI_RegisDef_t*)SPI2_BASEADDR)                // P
#define SPI3  				((SPI_RegisDef_t*)SPI3_BASEADDR)                // I
//#define SPI4                ((SPI_RegisDef_t*)SPI4_BASEADDR)

#define I2C1  				((I2C_RegisDef_t*)I2C1_BASEADDR)                // I
#define I2C2  				((I2C_RegisDef_t*)I2C2_BASEADDR)                // 2
#define I2C3  				((I2C_RegisDef_t*)I2C3_BASEADDR)                // C

#define USART1  			((USART_RegisDef_t*)USART1_BASEADDR)            // USART
#define USART2  			((USART_RegisDef_t*)USART2_BASEADDR)
#define USART3  			((USART_RegisDef_t*)USART3_BASEADDR)            // and
#define UART4  				((USART_RegisDef_t*)UART4_BASEADDR)
#define UART5  				((USART_RegisDef_t*)UART5_BASEADDR)             // UART
#define USART6  			((USART_RegisDef_t*)USART6_BASEADDR)


/****************************************************************************************************
 *                                     CLOCK    ENABLE
 *                                       M A C R O 's
 ****************************************************************************************************/


/*
 * Clock Enable Macros for GPIOx peripherals
 */
#define GPIOA_PCLK_EN()    	(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()		(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()		(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()		(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()		(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()		(RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()		(RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()		(RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN()		(RCC->AHB1ENR |= (1 << 8))



/*
 * Clock Enable Macros for I2Cx peripherals
 */
#define I2C1_PCLK_EN()      (RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()      (RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()      (RCC->APB1ENR |= (1 << 23))


/*
 * Clock Enable Macros for SPIx peripherals
 */
#define SPI1_PCLK_EN()      (RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()      (RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()      (RCC->APB1ENR |= (1 << 15))
//#define SPI4_PCLK_EN()      (RCC->APB2ENR |= (1 << 13))

/*
 * Clock Enable Macros for USARTx peripherals
 */
#define USART1_PCLK_EN()    (RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN()    (RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN()    (RCC->APB1ENR |= (1 << 18))
#define UART4_PCLK_EN()     (RCC->APB1ENR |= (1 << 19))
#define UART5_PCLK_EN()     (RCC->APB1ENR |= (1 << 20))
#define USART6_PCLK_EN()    (RCC->APB1ENR |= (1 << 5))


/*
 * Clock Enable Macros for SYSCFG peripheral
 */
#define SYSCFG_PCLK_EN() (RCC->APB2ENR |= (1 << 14))


/****************************************************************************************************
 *                                     CLOCK    DISABLE
 *                                       M A C R O 's
 ****************************************************************************************************/
 /*
  * Clock Disable macros for GPIOx peripherals
  */
 #define GPIOA_PCLK_DIS()     ( RCC->AHB1ENR &= ~(1<<0) )
 #define GPIOB_PCLK_DIS()     ( RCC->AHB1ENR &= ~(1<<1) )
 #define GPIOC_PCLK_DIS()     ( RCC->AHB1ENR &= ~(1<<2) )
 #define GPIOD_PCLK_DIS()     ( RCC->AHB1ENR &= ~(1<<3) )
 #define GPIOE_PCLK_DIS()     ( RCC->AHB1ENR &= ~(1<<4) )
 #define GPIOF_PCLK_DIS()     ( RCC->AHB1ENR &= ~(1<<5) )
 #define GPIOG_PCLK_DIS()     ( RCC->AHB1ENR &= ~(1<<6) )
 #define GPIOH_PCLK_DIS()     ( RCC->AHB1ENR &= ~(1<<7) )
 #define GPIOI_PCLK_DIS()     ( RCC->AHB1ENR &= ~(1<<8) )


 /*
  * Clock Disable macors for I2Cx peripherals
  */
 #define I2C1_PCLK_DIS()      ( RCC->APB1ENR &= ~(1<<21))
 #define I2C2_PCLK_DIS()      ( RCC->APB1ENR &= ~(1<<22))
 #define I2C3_PCLK_DIS()      ( RCC->APB1ENR &= ~(1<<23))



 /*
  * Clock Disable macros for SPIx peripherals
   */
 #define SPI1_PCLK_DIS()      ( RCC->APB2ENR &= ~(1<<12))
 #define SPI2_PCLK_DIS()      ( RCC->APB1ENR &= ~(1<<14))
 #define SPI3_PCLK_DIS()      ( RCC->APB1ENR &= ~(1<<15))
 //#define SPI4_PCLK_DIS()      ( RCC->APB2ENR &= ~(1<<13))


 /*
  * Clock Disable macros for USARTx peripherals
  */
 #define USART1_PCLK_DIS()    ( RCC->APB2ENR &= ~(1<<4))
 #define USART2_PCLK_DIS()    ( RCC->APB1ENR &= ~(1<<17))
 #define USART3_PCLK_DIS()    ( RCC->APB1ENR &= ~(1<<18))
 #define UART4_PCLK_DIS()     ( RCC->APB1ENR &= ~(1<<19))
 #define UART5_PCLK_DIS()     ( RCC->APB1ENR &= ~(1<<20))
 #define USART6_PCLK_DIS()    ( RCC->APB2ENR &= ~(1<<5))


/*
 * Clock Disable macros for SYSCFG peripheral
 */
 #define SYSCFG_PCLK_DIS()    (RCC->APB2ENR &= ~(1<<14))

/**********************************************************************************************************************
 *                                     GPIOx    Registers RESET
 *                                          M A C R O 's
 **********************************************************************************************************************/
#define GPIOA_REG_RESET()                     do{( RCC->AHB1RSTR |= (1 << 0) ); ( RCC->AHB1RSTR &= ~(1 << 0) ); }while(0)
#define GPIOB_REG_RESET()                     do{( RCC->AHB1RSTR |= (1 << 1) ); ( RCC->AHB1RSTR &= ~(1 << 1) ); }while(0)
#define GPIOC_REG_RESET()                     do{( RCC->AHB1RSTR |= (1 << 2) ); ( RCC->AHB1RSTR &= ~(1 << 2) ); }while(0)
#define GPIOD_REG_RESET()                     do{( RCC->AHB1RSTR |= (1 << 3) ); ( RCC->AHB1RSTR &= ~(1 << 3) ); }while(0)
#define GPIOE_REG_RESET()                     do{( RCC->AHB1RSTR |= (1 << 4) ); ( RCC->AHB1RSTR &= ~(1 << 4) ); }while(0)
#define GPIOF_REG_RESET()                     do{( RCC->AHB1RSTR |= (1 << 5) ); ( RCC->AHB1RSTR &= ~(1 << 5) ); }while(0)
#define GPIOG_REG_RESET()                     do{( RCC->AHB1RSTR |= (1 << 6) ); ( RCC->AHB1RSTR &= ~(1 << 6) ); }while(0)
#define GPIOH_REG_RESET()                     do{( RCC->AHB1RSTR |= (1 << 7) ); ( RCC->AHB1RSTR &= ~(1 << 7) ); }while(0)
#define GPIOI_REG_RESET()                     do{( RCC->AHB1RSTR |= (1 << 8) ); ( RCC->AHB1RSTR &= ~(1 << 8) ); }while(0)



/**********************************************************************************************************************
 *                                     SPIx    Registers RESET
 *                                          M A C R O 's
 **********************************************************************************************************************/
#define SPI1_REG_RESET()                     do{( RCC->APB2RSTR |= (1 << 12) ); ( RCC->APB2RSTR &= ~(1 << 12) ); }while(0)
#define SPI2_REG_RESET()                     do{( RCC->APB1RSTR |= (1 << 14) ); ( RCC->APB1RSTR &= ~(1 << 14) ); }while(0)
#define SPI3_REG_RESET()                     do{( RCC->APB1RSTR |= (1 << 15) ); ( RCC->APB1RSTR &= ~(1 << 15) ); }while(0)
//#define SPI4_REG_RESET()                     do{( RCC->AHB1RSTR |= (1 << 3) ); ( RCC->AHB1RSTR &= ~(1 << 3) ); }while(0)


/**********************************************************************************************************************
 *                                     I2Cx    Registers RESET
 *                                          M A C R O 's
 **********************************************************************************************************************/
#define I2C1_REG_RESET()                     do{( RCC->APB1RSTR |= (1 << 21) ); ( RCC->APB2RSTR &= ~(1 << 21) ); }while(0)
#define I2C2_REG_RESET()                     do{( RCC->APB1RSTR |= (1 << 22) ); ( RCC->APB1RSTR &= ~(1 << 22) ); }while(0)
#define I2C3_REG_RESET()                     do{( RCC->APB1RSTR |= (1 << 23) ); ( RCC->APB1RSTR &= ~(1 << 23) ); }while(0)


/**********************************************************************************************************************
 *                                     USARTx    Registers RESET
 *                                          M A C R O 's
 **********************************************************************************************************************/
#define USART1_REG_RESET()                     do{( RCC->APB2RSTR |= (1 << 4) );  ( RCC->APB2RSTR &= ~(1 << 4) ); }while(0)
#define USART2_REG_RESET()                     do{( RCC->APB1RSTR |= (1 << 17) ); ( RCC->APB1RSTR &= ~(1 << 17) ); }while(0)
#define USART3_REG_RESET()                     do{( RCC->APB1RSTR |= (1 << 18) ); ( RCC->APB1RSTR &= ~(1 << 18) ); }while(0)
#define UART4_REG_RESET()                     do{( RCC->APB1RSTR |= (1 << 19) ); ( RCC->APB1RSTR &= ~(1 << 19) ); }while(0)
/**********************************************************************************************************************
 *                                         GPIO_BASEADDR_TO_ITS_CODE
 *                                                M A C R O
 *                                                used in gpio_driver.c IT(interrupt) modes
 **********************************************************************************************************************/

#define GPIO_BASEADDR_TO_ITS_CODE(x)           (  (x==GPIOA) ? 0 :\
		                                          (x==GPIOB) ? 1 :\
 /*NOTE : for myself    */                        (x==GPIOC) ? 2 :\
 /*The (x) should match */                        (x==GPIOD) ? 3 :\
 /*of the statement.    */                        (x==GPIOE) ? 4 :\
                                                  (x==GPIOF) ? 5 :\
                                                  (x==GPIOG) ? 6 :\
                                                  (x==GPIOH) ? 7 :\
                                                  (x==GPIOI) ? 8 :0 )

/**********************************************************************************************************************
 *                                               A    R    M   Cortex-M
 *
 *                                   I R Q  (Interrupt Request ) Numbers for STM32F407
 *                                                M A C R O 's
 *                                   @todo complete for other peripherals
 **********************************************************************************************************************/
#define IRQ_NUM_EXTI0        /* one */                    6
#define IRQ_NUM_EXTI1        /* one */                    7
#define IRQ_NUM_EXTI2        /* one */                    8
#define IRQ_NUM_EXTI3        /* one */                    9
#define IRQ_NUM_EXTI4        /* one */                    10
#define IRQ_NUM_EXTI9_5      /* five*/                    23
#define IRQ_NUM_EXTI15_10    /* six */                    40


#define NVIC_IRQ_PRI0                                     0
#define NVIC_IRQ_PRI1                                     1
#define NVIC_IRQ_PRI2                                     2
#define NVIC_IRQ_PRI3                                     3
#define NVIC_IRQ_PRI4                                     4
#define NVIC_IRQ_PRI5                                     5
#define NVIC_IRQ_PRI6                                     6
#define NVIC_IRQ_PRI7                                     7
#define NVIC_IRQ_PRI8                                     8
#define NVIC_IRQ_PRI9                                     9
#define NVIC_IRQ_PRI10                                    10
#define NVIC_IRQ_PRI11                                    11
#define NVIC_IRQ_PRI12                                    12
#define NVIC_IRQ_PRI13                                    13
#define NVIC_IRQ_PRI14                                    14
#define NVIC_IRQ_PRI15                                    15
/**********************************************************************************************************************
 *                                                A   R   M    Cortex-M
 *
 *                                   NVIC - ISERx (Interrupt Set-Enable Register ) Addresses
 *
 **********************************************************************************************************************/
#define NVIC_ISER0                                       ((vola uint32_t *)0xE000E100)
#define NVIC_ISER1                                       ((vola uint32_t *)0xE000E104)
#define NVIC_ISER2                                       ((vola uint32_t *)0xE000E108)
#define NVIC_ISER3                                       ((vola uint32_t *)0xE000E10C)
#define NVIC_ISER4                                       ((vola uint32_t *)0xE000E110)
#define NVIC_ISER5                                       ((vola uint32_t *)0xE000E114)
#define NVIC_ISER6                                       ((vola uint32_t *)0xE000E118)
#define NVIC_ISER7                                       ((vola uint32_t *)0xE000E11C)

/**********************************************************************************************************************
 *                                                A   R   M    Cortex-M
 *
 *                                   NVIC - ICERx (Interrupt Clear-Enable Register ) Addresses
 *
 **********************************************************************************************************************/
#define NVIC_ICER0                                       ((vola uint32_t *)0XE000E180)
#define NVIC_ICER1                                       ((vola uint32_t *)0XE000E184)
#define NVIC_ICER2                                       ((vola uint32_t *)0XE000E188)
#define NVIC_ICER3                                       ((vola uint32_t *)0XE000E18C)
#define NVIC_ICER4                                       ((vola uint32_t *)0XE000E190)
#define NVIC_ICER5                                       ((vola uint32_t *)0XE000E194)
#define NVIC_ICER6                                       ((vola uint32_t *)0XE000E198)
#define NVIC_ICER7                                       ((vola uint32_t *)0XE000E19C)

/**********************************************************************************************************************
 *                                                A   R   M    Cortex-M
 *
 *                                   NVIC - ISPRx (Interrupt Set-Pending Register ) Addresses
 *
 **********************************************************************************************************************/
#define NVIC_ISPR0                                       ((vola uint32_t *)0xE000E200)
#define NVIC_ISPR1                                       ((vola uint32_t *)0xE000E204)
#define NVIC_ISPR2                                       ((vola uint32_t *)0xE000E208)
#define NVIC_ISPR3                                       ((vola uint32_t *)0xE000E20C)
#define NVIC_ISPR4                                       ((vola uint32_t *)0xE000E210)
#define NVIC_ISPR5                                       ((vola uint32_t *)0xE000E214)
#define NVIC_ISPR6                                       ((vola uint32_t *)0xE000E218)
#define NVIC_ISRP7                                       ((vola uint32_t *)0xE000E21C)

/**********************************************************************************************************************
 *                                                A   R   M    Cortex-M
 *
 *                                   NVIC - ICERx (Interrupt Clear-Pending Register ) Addresses
 *
 **********************************************************************************************************************/
#define NVIC_ICPR0                                       ((vola uint32_t *)0XE000E280)
#define NVIC_ICPR1                                       ((vola uint32_t *)0XE000E284)
#define NVIC_ICPR2                                       ((vola uint32_t *)0XE000E288)
#define NVIC_ICPR3                                       ((vola uint32_t *)0XE000E28C)
#define NVIC_ICPR4                                       ((vola uint32_t *)0XE000E290)
#define NVIC_ICPR5                                       ((vola uint32_t *)0XE000E294)
#define NVIC_ICPR6                                       ((vola uint32_t *)0XE000E298)
#define NVIC_ICPR7                                       ((vola uint32_t *)0XE000E29C)

/**********************************************************************************************************************
 *                                                A   R   M    Cortex-M
 *
 *                                   NVIC - IABRx (Interrupt Active-Bit Register ) Addresses
 *
 **********************************************************************************************************************/
#define NVIC_IABR0                                       ((vola uint32_t *)0xE000E300)
#define NVIC_IABR1                                       ((vola uint32_t *)0xE000E304)
#define NVIC_IABR2                                       ((vola uint32_t *)0xE000E308)
#define NVIC_IABR3                                       ((vola uint32_t *)0xE000E30C)
#define NVIC_IABR4                                       ((vola uint32_t *)0xE000E310)
#define NVIC_IABR5                                       ((vola uint32_t *)0xE000E314)
#define NVIC_IABR6                                       ((vola uint32_t *)0xE000E318)
#define NVIC_IABR7                                       ((vola uint32_t *)0xE000E31C)

/**********************************************************************************************************************
 *                                                A   R   M    Cortex-M
 *
 *                                   NVIC - IPR (Interrupt Priority Register ) BASE - Address
 *
 **********************************************************************************************************************/
#define NVIC_IPR_BASEADDR                                ((vola uint32_t *)0xE000E400)
                     //BASE address because there are total 60 of these Registers



#define NO_OF_PR_BITS_IMPLEMENTED                         4
/*****************************************************************************************************************
 *                                              Some @GENERIC MACRO's
 *****************************************************************************************************************/

#define ENABLE                                 1
#define DISABLE                                0
#define SET                                    1
#define RESET                                  0
#define HIGH                                   1
#define LOW                                    0
#define FLAG_RESET                             0
#define FLAG_SET                               1


#define PIN_0                             0
#define PIN_1                             1
#define PIN_2							  2
#define PIN_3                             3
#define PIN_4                             4
#define PIN_5                             5
#define PIN_6                             6
#define PIN_7                             7
#define PIN_8                             8
#define PIN_9                             9
#define PIN_10                            10
#define PIN_11                            11
#define PIN_12                            12
#define PIN_13                            13
#define PIN_14                            14
#define PIN_15                            15

/****************************************************************************************
 *                              BIT definition MACROS for SPI peripheral
 ****************************************************************************************/
/*
 * bit definition for CR1
 */
#define SPI_CR1_CPHA                      0
#define SPI_CR1_CPOL                      1
#define SPI_CR1_MSTR                      2
#define SPI_CR1_BR                        3
#define SPI_CR1_SPE                       6
#define SPI_CR1_LSB_FIRST                 7
#define SPI_CR1_SSI                       8
#define SPI_CR1_SSM                       9
#define SPI_CR1_RXONLY                    10
#define SPI_CR1_DFF                       11
#define SPI_CR1_CRCNEXT                   12
#define SPI_CR1_CRCEN                     13
#define SPI_CR1_BIDIOE                    14
#define SPI_CR1_BIDIMODE                  15

/*
 * bit definitions for CR2
 */
#define SPI_CR2_RXDMAEN                   0
#define SPI_CR2_TXDMAEN                   1
#define SPI_CR2_SSOE                      2
#define SPI_CR2_FRF                       4
#define SPI_CR2_ERRIE                     5
#define SPI_CR2_RXNEIE                    6
#define SPI_CR2_TXEIE                     7

/*
 * bit definitons for SR
 */
#define SPI_SR_RXNE                       0
#define SPI_SR_TXE                        1
#define SPI_SR_CHSIDE                     2
#define SPI_SR_UDR                        3
#define SPI_SR_CRCERR                     4
#define SPI_SR_MODF                       5
#define SPI_SR_OVR                        6
#define SPI_SR_BSY                        7
#define SPI_SR_FRE                        8
/*
 * bit definition for DR
 */
#define SPI_DR_15_0                     0

/****************************************************************************************
 *                              BIT definition MACROS for I2C peripheral
 ****************************************************************************************/
/*
 * Bit defitions for CR1
 */
#define I2C_CR1_SWRST                     15
#define I2C_CR1_ALERT                     13
#define I2C_CR1_PEC                       12
#define I2C_CR1_POS                       11
#define I2C_CR1_ACK                       10
#define I2C_CR1_STOP                      9
#define I2C_CR1_START                     8
#define I2C_CR1_NOSTRETCH                 7
#define I2C_CR1_ENGC                      6
#define I2C_CR1_ENPEC                     5
#define I2C_CR1_ENARP                     4
#define I2C_CR1_SMBTYPE                   3
#define I2C_CR1_SMBUS                     1
#define I2C_CR1_PE                        0
/*
 * bit definitions for CR2
 */
#define I2C_CR2_FREQ5_0                   0
#define I2C_CR2_ITERREN                   8
#define I2C_CR2_ITEVTEN                   9
#define I2C_CR2_ITBUFEN                   10
#define I2C_CR2_DMAEN                     11
#define I2C_CR2_LAST                      12

/*
 * bit definitions for OAR1
 */
#define I2C_OAR1_ADD0                     0
#define I2C_OAR1_ADD7_1                   1
#define I2C_OAR1_ADD9_8                   8
#define I2C_OAR1_ADDMODE                  15
/*
 * bit definitions for OAR2
 */
#define I2C_OAR2_ENDUAL                   0
#define I2C_OAR2_ADD2_7_1                 1

/*
 * bit definiton for DR
 */
#define I2C_DR8_0                         0
/*
 * bit definitions for SR1
 */
#define I2C_SR1_SB                        0
#define I2C_SR1_ADDR                      1
#define I2C_SR1_BTF                       2
#define I2C_SR1_ADD10                     3
#define I2C_SR1_STOPF                     4
#define I2C_SR1_RxNE                      6
#define I2C_SR1_TxE                       7
#define I2C_SR1_BERR                      8
#define I2C_SR1_ARLO                      9
#define I2C_SR1_AF                        10
#define I2C_SR1_OVR                       11
#define I2C_SR1_PECERR                    12
#define I2C_SR1_TIMEOUT                   14
#define I2C_SR1_SMBALERT                  15

/*
 * bit definitions for SR2
 */
#define I2C_SR2_MSL                       0
#define I2C_SR2_BUSY                      1
#define I2C_SR2_TRA                       2
#define I2C_SR2_GENCALL                   4
#define I2C_SR2_SMBDEFAULT                5
#define I2C_SR2_SMBHOST                   6
#define I2C_SR2_DUALF                     7
#define I2C_SR2_PEC15_8                   8
/*
 * bit definitions for CCR
 */
#define I2C_CCR11_0                       0
#define I2C_CCR_DUTY                      14
#define I2C_CCR_FS                        15
/*
 * bit definition for TRISE
 */
#define I2C_TRISE5_0                      0

/*
 * bit definitions for FLTR
 */
#define I2C_FLTR_DNF3_0                   0
#define I2C_FLTR_ANOFF                    4


/****************************************************************************************
 *                              BIT definition MACROS for I2C peripheral
 ****************************************************************************************/
/*
 * bit definitions for SR
 */
#define USART_SR_PE                      0
#define USART_SR_FE                      1
#define USART_SR_NF                      2
#define USART_SR_ORE                     3
#define USART_SR_IDLE                    4
#define USART_SR_RXNE                    5
#define USART_SR_TC                      6
#define USART_SR_TXE                     7
#define USART_SR_LBD                     8
#define USART_SR_CTS                     9
/*
 * bit definitons for DR
 */
#define USART_DR_8_0
/*
 * bit definitons fot BRR
 */
#define USART_BRR_DIV_FRACTION_3_0        0
#define USART_BRR_DIV_MANTSSA_11_0        4
/*
 * bit definitions for CR1
 */
#define USART_CR1_SBK                     0
#define USART_CR1_RWU                     1
#define USART_CR1_RE                      2
#define USART_CR1_TE                      3
#define USART_CR1_IDLEIE                  4
#define USART_CR1_RXNEIE                  5
#define USART_CR1_TCIE                    6
#define USART_CR1_TXEIE                   7
#define USART_CR1_PEIE                    8
#define USART_CR1_PS                      9
#define USART_CR1_PCE                     10
#define USART_CR1_WAKE                    11
#define USART_CR1_M                       12
#define USART_CR1_UE                      13
#define USART_CR1_RESERVED                14
#define USART_CR1_OVER8                   15

/*
 * bit definitions for CR2
 */
#define USART_CR2_AADR_3_0                0
#define USART_CR2_LBDL                    5
#define USART_CR2_LBDIE                   6
#define USART_CR2_LBCL                    8
#define USART_CR2_CPHA                    9
#define USART_CR2_CPOL                    10
#define USART_CR2_CLKEN                   11
#define USART_CR2_STOP_1_0                12
#define USART_CR2_LINEN                   14

/*
 * bit definitions for CR3
 */
#define USART_CR3_EIE                     0
#define USART_CR3_IREN                    1
#define USART_CR3_IRLP                    2
#define USART_CR3_HDSEL                   3
#define USART_CR3_NACK                    4
#define USART_CR3_SCEN                    5
#define USART_CR3_DMAR                    6
#define USART_CR3_DMAT                    7
#define USART_CR3_RTSE                    8
#define USART_CR3_CTSE                    9
#define USART_CR3_CTSIE                   10
#define USART_CR3_ONEBIT                  11
/*
 * bit definitions for GTPR
 */
#define USART_GTPR_PSC_7_0                0
#define USART_GTPR_GT_7_0                 8

/*********************************************************************************************
 *                               Peripherals HEADER FILES
 *********************************************************************************************/
#include "gpio_driver.h"
#include "spi_driver.h"
#include "i2c_driver.h"
#include "usart_driver.h"



#endif /* INC_STM32F407XX_H_ */
