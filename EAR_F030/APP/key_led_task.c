

/**
********************************************************************************
* ��ࣺ
* ģ�����ƣ�key_led_task.c
* ģ�鹦�ܣ�
* �������ڣ�
* �� �� �ߣ�
* ��    ����
********************************************************************************
**/

/***********************************
* ͷ�ļ�
***********************************/

#include "app.h"
#include "datatype.h"
#include "hardware.h"
#include "fifo.h"
#include "key_led_task.h"
#include "protocol_module.h"

#include "i2c.h"
#include "Motor_pwm.h"
/**********************************
*�궨��
***********************************/

/***********************************
* ȫ�ֱ���
***********************************/

/***********************************
* �ֲ�����
***********************************/
//static uint8_t cur_status = FALSE;
//static uint8_t pre_status = FALSE;


//KEYֵ������㰴Ϊȷ����������
typedef enum {
	NO_KEY,
	BLUE_CHECK
}KEY_VAL;
//static uint8_t key_val = NO_KEY;

//typedef enum
//{
//	POWER_ON,
//	POWER_OFF
//}MCU_STATE;

MCU_STATE mcu_state=POWER_OFF;
//mcu_state=POWER_OFF;


volatile KEY_STATE key_state=KEY_UPING;
/***********************************
* �ֲ�����
***********************************/
//
void key_led_task(void)
{
//	 uint8_t LED_cnt;
//	 uint16_t res;
	static uint8_t key_down_cnt = 0;
	static uint16_t key_wakeup_value;

	//static uint8_t high_level_cnt=0;
	//static uint8_t low_level_cnt=0;
	//static uint8_t cycle_cnt=0;
	static uint8_t motor_shake_cnt=0;
	#if 0
//	cur_status = get_key_status();
//	
//	////�������
//	if(!pre_status && cur_status)//01
//	{
//		//����		
//	}
//	if(pre_status && !cur_status)//10
//	{
//		//����
//		key_down_cnt = 0;
//		
//		if(key_down_cnt <= ONE_SEC_KEY_TIME)
//		{
//			//��⵽һ�ε㰴
//			key_val = BLUE_CHECK;
//		}
//	}
//	if(pre_status && cur_status)//11
//	{
//		//��������
//		key_down_cnt ++;
//		if(key_down_cnt > ONE_SEC_KEY_TIME)
//		{
//			//1s��ʱ			
//		}
//	}
//	
//	pre_status = cur_status;
//	
//	////LED��ָʾ
//	if(key_val == BLUE_CHECK)
//	{
//		key_val = NO_KEY;
//		
//		//��Ӧȷ���������ӵ�ָʾ
//	}
	
	//�����������״̬����������˸
	#endif
	

	

//	
//	//���ݰ���ADCֵ,�жϰ����Ƿ񱻰���
	if(key_down_cnt == 10)
	{
		key_state=KEY_DOWNING;
		key_down_cnt=0;
//		low_level_cnt=0;
//		high_level_cnt=0;
//		cycle_cnt=0;
	}
//	
	if(key_state==KEY_DOWNING)
	{
		#if 0
//		if(key_down_cnt==200) //����ʱ�������ǿ�ƽ�������ΪUPING״̬
//		{
//			key_down_cnt=0;
//			key_state=KEY_UPING;
//		}
//		else
//		{
//			key_wakeup_value=Adc_Switch(ADC_Channel_0);
//			if(key_wakeup_value>=1365)
//			{
//				key_state=KEY_DOWN_UP;
//				key_down_cnt=0;				
//			}
//			key_down_cnt++;
//		}	
		//Delay_ms(3000);
		#endif
		key_wakeup_value=Adc_Switch(ADC_Channel_0);
		if(key_wakeup_value>=1365)
		{
			key_state=KEY_DOWN_UP;
			//key_down_cnt=0;				
		}
	}
	
	if(key_state==KEY_UPING)
	{
		//ADC_ChannelConfig(ADC1,ADC_Channel_0,ADC_SampleTime_239_5Cycles);  //PA0
		key_wakeup_value=Adc_Switch(ADC_Channel_0);
		if(key_wakeup_value<=500)
		{
			key_down_cnt++;	
		}
		else
		{
			key_state=KEY_UPING; 
			key_down_cnt = 0;
		}
	}
		//���������£�����ص�ѹ�Ƿ����2.2V
	if(key_state==KEY_DOWN_UP)
	{
			//ADC_ChannelConfig(ADC1,ADC_Channel_0,ADC_SampleTime_239_5Cycles);  //PA0
			key_wakeup_value=Adc_Switch(ADC_Channel_0);
			if(key_wakeup_value>=1365)
			{
				//����
				if(motor_shake_cnt==25)
				{
					key_state=KEY_UPING;
					Motor_PWM_Freq_Dudy_Set(1,100,0);
					Motor_PWM_Freq_Dudy_Set(2,100,0);
					motor_shake_cnt=0;
					mcu_state=POWER_ON;
				}
				if(key_state==KEY_DOWN_UP)
				{
					Motor_PWM_Freq_Dudy_Set(1,100,30);
					Motor_PWM_Freq_Dudy_Set(2,100,30);
					motor_shake_cnt++;
				}
				
			}
			else	
			{
				for(int i=0;i<3;i++)
				{
					set_led(LED_RED);
					Delay_ms(500);
					set_led(LED_CLOSE);
					Delay_ms(500);
				}
				key_state=KEY_UPING;
				//��ɫLED��3s���ػ�
				#if 0
				mcu_state=POWER_OFF;
				if(cycle_cnt==3)
				{
					//���3�����ڣ�key_pressed_downΪfalse���ú���ĳ��������ⰴ���Ƿ���
					low_level_cnt=255;
					high_level_cnt=255;
					cycle_cnt=0;
					key_state=KEY_UPING;
				}
				if(high_level_cnt==25)
				{
					//2.����������25���ص�,�صƼ���
						set_led(LED_CLOSE);
						low_level_cnt++;
				}
				if(low_level_cnt==25)
				{
					//3.�صƼ�����25�����һ�����ڣ�cnt��һ
					if(cycle_cnt<3)
					{
						low_level_cnt=0;
						high_level_cnt=0;
					}
					cycle_cnt++;
				}
				if(low_level_cnt==0)
				{
					//1.���������������
					if(cycle_cnt<3)
					{
						set_led(LED_RED);
						high_level_cnt++;
					}
				}	
				#endif
			}
	}
	
#if 0
//	LED_cnt++;
//	if(LED_cnt >= 5)
//	{
//		LED_cnt = 0;
//		set_led(LED_GREEN);
//	}
//	else if(LED_cnt == 2)
//	{
//		set_led(LED_RED);		
//	}	
#endif 
//	//debug
//	static uint16_t res;
//	res=GetModeSelected();
////	
//	//��ʼ��ADS115,I2C
//	ADS115_Init();
//	
//	static uint16_t buffer;
//	buffer=ADS115_readByte(0x90);

	os_delay_ms(KEY_LED_TASK_ID, KEY_LED_PERIOD);
}
