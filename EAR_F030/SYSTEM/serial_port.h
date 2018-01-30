/*******************************************************************************
* ��Ȩ���� :  
* �汾��   :  1.0
* �ļ���   :  serial_port.h
* �������� : 
* ����     :  
* ����˵�� :  ���������

*******************************************************************************/
#ifndef  __SERIAL_PORT_H__
#define  __SERIAL_PORT_H__
/*******************************************************************************
*                                 ͷ�ļ�����
*******************************************************************************/
#include "stm32f0xx.h"

/**********************************
*�궨��
***********************************/
#define UART    USART1
#define UART_CLKSRC   RCC_APB2Periph_USART1

#define UART_IO_PORT			GPIOA
#define	UART_IO_CLKSRC		RCC_AHBPeriph_GPIOA
#define UART_RX_PIN				GPIO_Pin_3
#define UART_TX_PIN				GPIO_Pin_2
#define UART_RX_AF_PIN_SOURCE GPIO_PinSource3
#define UART_TX_AF_PIN_SOURCE GPIO_PinSource2

#define UART_BAUDRATE   115200 

#define DMA_CLKSRC RCC_AHBPeriph_DMA1
#define UART_DMA_RX_CHANNEL   DMA1_Channel3
#define UART_DMA_TX_CHANNEL   DMA1_Channel2

/***********************************
* ȫ�ֱ���
***********************************/

/***********************************
* ��Ͷ��x
***********************************/

/***********************************
* �ⲿ����
***********************************/
void UARTInit(uint8_t* p_rec_buf, uint32_t rec_num);
void UartSendNBytes (uint8_t *p_buf, uint32_t num);
int32_t GetUartReceiverResidualCnt(void);
#endif