#include "stm32f0xx_conf.h"
#include "common.h"
#include "delay.h"

INT16U g_uwDelayTime_ms = 0;
//static INT8U  g_ubMultiple_us = 0;
//static INT16U g_uwMultiple_ms = 0;


void Delay(INT16U mTime)
{
	g_uwDelayTime_ms = mTime;
	while(g_uwDelayTime_ms != 0);
}


//void Delay_Init(INT8U sysclk)
//{
//	SysTick->CTRL &= 0xFFFFFFFB;                                     // ѡ��Tickʱ��Դ����HCLK/8
//	g_ubMultiple_us = sysclk / 8;
//	g_uwMultiple_ms = (INT16U)sysclk / 8 * 1000;
//	//SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk;                   // ѡ��Tickʱ��Դ����ϵͳʱ��
//	//g_ubMultiple_us = sysclk;
//	//g_uwMultiple_ms = (INT16U)sysclk * 1000;
//}


// ΢�붨ʱ
void Delay_us(INT16U time)
{
	delay_us(time);
	#if 0
//	INT32U ulTemp = 0;
//	
//	ulTemp = (INT32U)time * g_ubMultiple_us;
//	SysTick->LOAD = ulTemp - 1;
//	SysTick->VAL = 0;                                             // ��ֵ�����SysTick_CTRL_COUNTFLAG_Msk λ
//	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;                     // ������ʱ��
//	do
//	{
//		ulTemp = SysTick->CTRL;
//	}
//	while((ulTemp & SysTick_CTRL_COUNTFLAG_Msk) != SysTick_CTRL_COUNTFLAG_Msk);       // �ȵ��������
//	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;                                        // ֹͣ��ʱ��
//	SysTick->VAL = 0;	
	#endif
}


// ���붨ʱ
void Delay_ms(INT16U time)
{
	delay_ms(time);
//	INT32U ulTemp = 0;
//	
//	ulTemp = (INT32U)time * g_uwMultiple_ms;
//	SysTick->LOAD = ulTemp - 1;
//	SysTick->VAL = 0;                                              // ��ֵ�����SysTick_CTRL_COUNTFLAG_Msk λ
//	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;                      // ������ʱ��
//	do
//	{
//		ulTemp = SysTick->CTRL;
//	}
//	while((ulTemp & SysTick_CTRL_COUNTFLAG_Msk) != SysTick_CTRL_COUNTFLAG_Msk);       // �ȵ��������
//	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;                                        // ֹͣ��ʱ��
//	SysTick->VAL = 0;
}




