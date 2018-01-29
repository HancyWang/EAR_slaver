

/**
********************************************************************************
* 版權：
* 模块名称：key_led_task.c
* 模块功能：
* 创建日期：
* 创 建 者：
* 声    明：
********************************************************************************
**/

/***********************************
* 头文件
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
*宏定义
***********************************/

/***********************************
* 全局变量
***********************************/

/***********************************
* 局部变量
***********************************/
//static uint8_t cur_status = FALSE;
//static uint8_t pre_status = FALSE;


//KEY值，这里点按为确认蓝牙连接
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
* 局部函数
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
//	////按键检测
//	if(!pre_status && cur_status)//01
//	{
//		//按下		
//	}
//	if(pre_status && !cur_status)//10
//	{
//		//弹起
//		key_down_cnt = 0;
//		
//		if(key_down_cnt <= ONE_SEC_KEY_TIME)
//		{
//			//检测到一次点按
//			key_val = BLUE_CHECK;
//		}
//	}
//	if(pre_status && cur_status)//11
//	{
//		//连续按下
//		key_down_cnt ++;
//		if(key_down_cnt > ONE_SEC_KEY_TIME)
//		{
//			//1s延时			
//		}
//	}
//	
//	pre_status = cur_status;
//	
//	////LED灯指示
//	if(key_val == BLUE_CHECK)
//	{
//		key_val = NO_KEY;
//		
//		//对应确认蓝牙连接的指示
//	}
	
	//获得蓝牙连接状态，连接中闪烁
	#endif
	

	

//	
//	//根据按键ADC值,判断按键是否被按下
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
//		if(key_down_cnt==200) //按键时间过长，强制将按键置为UPING状态
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
		//按键被按下，检查电池电压是否大于2.2V
	if(key_state==KEY_DOWN_UP)
	{
			//ADC_ChannelConfig(ADC1,ADC_Channel_0,ADC_SampleTime_239_5Cycles);  //PA0
			key_wakeup_value=Adc_Switch(ADC_Channel_0);
			if(key_wakeup_value>=1365)
			{
				//开机
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
				//橙色LED闪3s，关机
				#if 0
				mcu_state=POWER_OFF;
				if(cycle_cnt==3)
				{
					//完成3个周期，key_pressed_down为false，让后面的程序继续检测按键是否按下
					low_level_cnt=255;
					high_level_cnt=255;
					cycle_cnt=0;
					key_state=KEY_UPING;
				}
				if(high_level_cnt==25)
				{
					//2.灯亮计数到25，关灯,关灯计数
						set_led(LED_CLOSE);
						low_level_cnt++;
				}
				if(low_level_cnt==25)
				{
					//3.关灯计数到25，完成一个周期，cnt加一
					if(cycle_cnt<3)
					{
						low_level_cnt=0;
						high_level_cnt=0;
					}
					cycle_cnt++;
				}
				if(low_level_cnt==0)
				{
					//1.红灯亮，灯亮计数
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
//	//初始化ADS115,I2C
//	ADS115_Init();
//	
//	static uint16_t buffer;
//	buffer=ADS115_readByte(0x90);

	os_delay_ms(KEY_LED_TASK_ID, KEY_LED_PERIOD);
}
