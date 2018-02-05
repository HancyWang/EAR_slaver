

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

//extern uint8_t OUTPUT_FINISH;

volatile KEY_STATE key_state=KEY_UPING;

extern uint16_t RegularConvData_Tab[2];
/***********************************
* 局部函数
***********************************/
//
void key_led_task(void)
{
	
	//set_led(LED_GREEN);
//	 uint8_t LED_cnt;
//	 uint16_t res;
	static uint8_t key_down_cnt = 0;
	static uint8_t key_up_cnt=0;
//	static uint16_t key_wakeup_value;

	//static uint8_t motor_shake_cnt=0;
	
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
	

	//根据按键ADC值,判断按键是否被按下
	if(key_down_cnt == 10)
	{
		key_state=KEY_DOWNING;
		key_down_cnt=0;
	}

	if(key_state==KEY_DOWNING)
	{
		//key_wakeup_value=Adc_Switch(ADC_Channel_0);
		//if(key_wakeup_value>=2730)
		if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)==1)
		{
			key_up_cnt++;
//			key_state=KEY_DOWN_UP;
//			key_down_cnt=0;				
		}
		if(key_up_cnt==10)
		{
			key_state=KEY_DOWN_UP;
			key_up_cnt=0;
		}
	}
	
	if(key_state==KEY_UPING)
	{
		//key_wakeup_value=Adc_Switch(ADC_Channel_0);
		//if(key_wakeup_value<=500)
		if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)==0)
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
//		for(uint8_t i=0;i<3;i++)
//		{
//			key_wakeup_value=Adc_Switch(ADC_Channel_1);
//		}
		
		//if(key_wakeup_value>=2730)
		if(RegularConvData_Tab[0]>=2730)
		//	if(TRUE)
		{
			//开机
			
			Motor_PWM_Freq_Dudy_Set(1,100,80);
			Motor_PWM_Freq_Dudy_Set(2,100,80);
			Motor_PWM_Freq_Dudy_Set(3,100,80);
			Delay_ms(500);
			Motor_PWM_Freq_Dudy_Set(1,100,0);
			Motor_PWM_Freq_Dudy_Set(2,100,0);
			Motor_PWM_Freq_Dudy_Set(3,100,0);
			
			key_state=KEY_UPING;
			set_led(LED_GREEN);
			
			mcu_state=POWER_ON;
		}
		else	
		{
			//橙色LED闪3s，关机
			for(int i=0;i<3;i++)
			{
				set_led(LED_RED);
				Delay_ms(500);
				set_led(LED_CLOSE);
				Delay_ms(500);
			}
			key_state=KEY_UPING;
			mcu_state=POWER_OFF;
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
//	static uint16_t result;
//	result=GetModeSelected();
////	
//	//初始化ADS115,I2C
//	ADS115_Init();
//	
//	static uint16_t buffer;
//	buffer=ADS115_readByte(0x90);
//		static uint16_t result1;
//		static uint16_t result2;
//		 result1=Adc_Switch(ADC_Channel_1);
//		 
//		 result2=Adc_Switch(ADC_Channel_4);
	os_delay_ms(KEY_LED_TASK_ID, KEY_LED_PERIOD);
}
