#ifndef __USART_H
#define	__USART_H


#include "stm32f10x.h"
#include <stdio.h>

/** 
  * 串口宏定义，不同的串口挂载的总线和IO不一样，移植时需要修改这几个宏
	* 1-修改总线时钟的宏，uart1挂载到apb2总线，其他uart挂载到apb1总线
	* 2-修改GPIO的宏
  */
	
// 串口1-USART1
#define  DEBUG_USARTx                   USART1
#define  DEBUG_USART_CLK                RCC_APB2Periph_USART1
#define  DEBUG_USART_APBxClkCmd         RCC_APB2PeriphClockCmd
#define  DEBUG_USART_BAUDRATE           115200

// USART GPIO 引脚宏定义
#define  DEBUG_USART_GPIO_CLK           (RCC_APB2Periph_GPIOA)
#define  DEBUG_USART_GPIO_APBxClkCmd    RCC_APB2PeriphClockCmd
    
#define  DEBUG_USART_TX_GPIO_PORT         GPIOA   
#define  DEBUG_USART_TX_GPIO_PIN          GPIO_Pin_9
#define  DEBUG_USART_RX_GPIO_PORT         GPIOA
#define  DEBUG_USART_RX_GPIO_PIN          GPIO_Pin_10

#define  DEBUG_USART_IRQ                USART1_IRQn
#define  DEBUG_USART_IRQHandler         USART1_IRQHandler
// 串口2-USART2

#define  DEBUG_USART2                    USART2
#define  DEBUG_USART2_CLK                RCC_APB2Periph_USART1
#define  DEBUG_USART2_APBxClkCmd         RCC_APB2PeriphClockCmd
#define  DEBUG_USART2_BAUDRATE           115200

// USART2 GPIO 引脚宏定义
#define  DEBUG_USART2_GPIO_CLK           (RCC_APB2Periph_GPIOA)
#define  DEBUG_USART2_GPIO_APBxClkCmd    RCC_APB2PeriphClockCmd
    
#define  DEBUG_USART2_TX_GPIO_PORT         GPIOA   
#define  DEBUG_USART2_TX_GPIO_PIN          GPIO_Pin_3
#define  DEBUG_USART2_RX_GPIO_PORT         GPIOA
#define  DEBUG_USART2_RX_GPIO_PIN          GPIO_Pin_2

#define  DEBUG_USART2_IRQ                USART2_IRQn
#define  DEBUG_USART2_IRQHandler         USART2_IRQHandler
void USART_Config(void);
void Usart1_Init(unsigned int baud);
void Usart2_Init(unsigned int baud);
void Usart_SendString(USART_TypeDef *USARTx, unsigned char *str, unsigned short len);
void UsartPrintf(USART_TypeDef *USARTx, char *fmt,...);

#endif /* __USART_H */
