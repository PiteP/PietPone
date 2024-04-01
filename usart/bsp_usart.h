#ifndef __USART_H
#define	__USART_H


#include "stm32f10x.h"
#include <stdio.h>

/** 
  * ���ں궨�壬��ͬ�Ĵ��ڹ��ص����ߺ�IO��һ������ֲʱ��Ҫ�޸��⼸����
	* 1-�޸�����ʱ�ӵĺ꣬uart1���ص�apb2���ߣ�����uart���ص�apb1����
	* 2-�޸�GPIO�ĺ�
  */
	
// ����1-USART1
#define  DEBUG_USARTx                   USART1
#define  DEBUG_USART_CLK                RCC_APB2Periph_USART1
#define  DEBUG_USART_APBxClkCmd         RCC_APB2PeriphClockCmd
#define  DEBUG_USART_BAUDRATE           115200

// USART GPIO ���ź궨��
#define  DEBUG_USART_GPIO_CLK           (RCC_APB2Periph_GPIOA)
#define  DEBUG_USART_GPIO_APBxClkCmd    RCC_APB2PeriphClockCmd
    
#define  DEBUG_USART_TX_GPIO_PORT         GPIOA   
#define  DEBUG_USART_TX_GPIO_PIN          GPIO_Pin_9
#define  DEBUG_USART_RX_GPIO_PORT         GPIOA
#define  DEBUG_USART_RX_GPIO_PIN          GPIO_Pin_10

#define  DEBUG_USART_IRQ                USART1_IRQn
#define  DEBUG_USART_IRQHandler         USART1_IRQHandler
// ����2-USART2

#define  DEBUG_USART2                    USART2
#define  DEBUG_USART2_CLK                RCC_APB2Periph_USART1
#define  DEBUG_USART2_APBxClkCmd         RCC_APB2PeriphClockCmd
#define  DEBUG_USART2_BAUDRATE           115200

// USART2 GPIO ���ź궨��
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
