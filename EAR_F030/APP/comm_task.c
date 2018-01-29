//////////////////////////////////////////////////////////////////////////////////	 
			  
//////////////////////////////////////////////////////////////////////////////////
#include "comm_task.h"
#include "fifo.h"
#include "CMD_Receive.h"
#include "os_cfg.h"
#include "stdio.h"
#include "delay.h"
#include "string.h"
#include "app.h"
#include "serial_port.h"
#include "protocol_module.h"
#include "key_led_task.h"
#include "Motor_pwm.h"
#include "i2c.h"
#include "key_led_task.h"
#include "hardware.h"

//全局变量
CMD_Receive g_CmdReceive;  // 命令接收控制对象
FIFO_TYPE send_fifo;//發送數據FIFO
UINT8 send_buf[SEND_BUF_LEN];

UINT8 parameter_buf[PARAMETER_BUF_LEN]; //用来接收上位机传过来的参数
UINT16 check_sum;
extern MCU_STATE mcu_state;
extern BOOL rcvParameters_from_PC;
extern KEY_STATE key_state;
//局部变量
typedef enum
{
	LOAD_PARA,  //加载参数
	GET_MODE,
	CHECK_PRESSURE,
	CHECK_PRESSURE_AGAIN,
	PREV_OUTPUT_PWM,
	CPY_PARA_TO_BUFFER,
	OUTPUT_PWM,
	CHECK_BAT_VOL,
	LED_RED_BLINK
}CHCKMODE_OUTPUT_PWM;


#define	LOAD_PARA 							0x01
#define	GET_MODE  							0x02
#define	CHECK_PRESSURE 					0x03
#define	CHECK_PRESSURE_AGAIN 		0x04
#define	PREV_OUTPUT_PWM 				0x05
#define	CPY_PARA_TO_BUFFER      0x06
#define	OUTPUT_PWM							0x07
#define	CHECK_BAT_VOL						0x08
#define	LED_RED_BLINK						0x09

typedef enum
{
	PWM_START,
	PWM_PERIOD,
	PWM_WAIT_BETWEEN,
	PWM_WAIT_AFTER,
	PWM_OUTPUT_FINISH
}PWM_STATE;

static PWM_STATE pwm1_state=PWM_START;
static PWM_STATE pwm2_state=PWM_START;
static PWM_STATE pwm3_state=PWM_START;


static PWM_STATE* p_pwm_state;
static uint8_t* p_PWM_period_cnt;
static uint8_t* p_PWM_waitBetween_cnt;
static uint8_t* p_PWM_waitAfter_cnt;
static uint8_t* p_PWM_numOfCycle;
static uint8_t* p_PWM_serial_cnt;




//typedef enum
//{
//	PWM1_START,
//	PWM1_PERIOD,
//	PWM1_WAIT_BETWEEN,
//	PWM1_WAIT_AFTER,
//	PWM1_OUTPUT_FINISH
//}PWM1_STATE;

//typedef enum
//{
//	PWM2_START,
//	PWM2_PERIOD,
//	PWM2_WAIT_BETWEEN,
//	PWM2_WAIT_AFTER,
//	PWM2_OUTPUT_FINISH
//}PWM2_STATE;

//typedef enum
//{
//	PWM3_START,
//	PWM3_PERIOD,
//	PWM3_WAIT_BETWEEN,
//	PWM3_WAIT_AFTER,
//	PWM3_OUTPUT_FINISH
//}PWM3_STATE;

// PWM1_STATE pwm1_state=PWM1_START;
// PWM2_STATE pwm2_state=PWM2_START;
// PWM3_STATE pwm3_state=PWM3_START;

 uint8_t PWM1_period_cnt=0;
 uint8_t PWM2_period_cnt=0;
 uint8_t PWM3_period_cnt=0;

 uint8_t PWM1_waitBetween_cnt=0;
 uint8_t PWM2_waitBetween_cnt=0;
 uint8_t PWM3_waitBetween_cnt=0;

 uint8_t PWM1_waitAfter_cnt=0;
 uint8_t PWM2_waitAfter_cnt=0;
 uint8_t PWM3_waitAfter_cnt=0;

 uint8_t PWM1_numOfCycle=0;
 uint8_t PWM2_numOfCycle=0;
 uint8_t PWM3_numOfCycle=0;

 uint8_t PWM1_serial_cnt=0;
 uint8_t PWM2_serial_cnt=0;
 uint8_t PWM3_serial_cnt=0;

//volatile CHCKMODE_OUTPUT_PWM state=LOAD_PARA;
 uint8_t state=LOAD_PARA;
 uint16_t mode;                      

 uint8_t buffer[PARAMETER_BUF_LEN]={0x05,0x03,
	0x11,	1,80,70,2,1,1,1,
	0x12,	1,100,50,1,2,1,2,
	0x13,	1,100,50,1,3,1,3,
	0x14,	1,100,30,1,4,1,4,
	0x15,	1,100,80,1,5,1,5,
	0x16,	1,100,90,1,6,1,6,
};
 uint8_t pwm_buffer[144]={0};
 uint16_t	mode;

 uint16_t checkPressAgain_cnt=0;
 uint8_t wait_cnt=0;
/*******************************************************************************
*                                内部函数声明
*******************************************************************************/
static BOOL ModuleUnPackFrame(void);
static BOOL ModuleProcessPacket(UINT8 *pData);
static UINT8 CheckCheckSum(UINT8* pData, UINT8 nLen);

void CalcCheckSum(UINT8* pPacket)
{
	UINT16 dataLen = pPacket[1];
	UINT16 checkSum = 0;
	UINT16 i;

	for (i = 1; i < dataLen; i++)
	{
		checkSum += pPacket[i];
	}

	pPacket[dataLen] = checkSum >> 8;
	pPacket[dataLen+1] = checkSum&0xFF;
}

/*******************************************************************************
** 函数名称: ModuleUnPackFrame
** 功能描述: 命令接收处理
** 输　  入: 无
** 输　  出: 无
** 全局变量: 无
** 调用模块: 无
*******************************************************************************/
 UINT8 sDataBuff[CMD_BUFFER_LENGTH] = {0};	
 UINT8 sBackBuff[CMD_BUFFER_LENGTH] = {0};
BOOL ModuleUnPackFrame(void)
{
	static BOOL sPacketHeadFlag = FALSE;
	static BOOL sPacketLenFlag = FALSE;
	static UINT8 sCurPacketLen = 0;
	static UINT8 sResetByte = 0;
	//static UINT8 sDataBuff[CMD_BUFFER_LENGTH] = {0};	
	//static UINT8 sBackBuff[CMD_BUFFER_LENGTH] = {0};

	UINT8 *pBuff = (UINT8 *)sDataBuff;
	UINT16 dwLen = 0;
	UINT8 byCurChar;

	// 从串口缓冲区中读取接收到的数据
	dwLen = GetBuf2Length(&g_CmdReceive);

	// 对数据进行解析
	while(0 < dwLen)
	{
		byCurChar = Buf2Read(&g_CmdReceive);

		if (sPacketHeadFlag)
		{
			// 解析到包头
			if(sPacketLenFlag)
			{
				// 解析到包长度
				pBuff[sCurPacketLen] = byCurChar;
				sCurPacketLen ++;
				sResetByte --;

				if (0 >= sResetByte)
				{
					// 接收完毕
					// 进行校验和比较
					if (CheckCheckSum(pBuff, pBuff[1]))
					{
						// 解析到一个有效数据包
						memcpy(sBackBuff, sDataBuff, CMD_BUFFER_LENGTH);
						ModuleProcessPacket(sBackBuff);//命令解包*********************************
						//防止连续接收多条命令时，命令应答信号出故障
						delay_ms(2);
					}

					sPacketHeadFlag = FALSE;
					sPacketLenFlag = FALSE;
					memset(&sDataBuff, 0x00, CMD_BUFFER_LENGTH);
				}													
			}
			else
			{
				if((CMD_BUFFER_LENGTH-1 > byCurChar) && (0 < byCurChar ))// 容错处理，防止数据包长为49或0时溢出 并且X5最长的主机发送包为15
				{
					// 解析到模块的长度
					sDataBuff[sCurPacketLen] = byCurChar;
					sResetByte = byCurChar;			
					sPacketLenFlag = TRUE;
					sCurPacketLen ++;
				}
				else
				{
					//没有解析到模块的长度, 重新解析
					sPacketHeadFlag = FALSE;
					sPacketLenFlag = FALSE;					
				}
			}
		}
		
		else if (PACK_HEAD_BYTE == byCurChar)		
		{
			// 解析到包头
			sDataBuff[0] = byCurChar;
			sPacketHeadFlag = TRUE;
			sPacketLenFlag = FALSE;			
			sCurPacketLen = 1;
			sResetByte = 0;
		}

		//pData ++;
		dwLen --;
	}
	return TRUE;
}


/*******************************************************************************
** 函数名称: CheckCheckSum
** 功能描述: 包校验
** 输　  入: pData 数据 nLen长度
** 输　  出: 无
** 全局变量: 无
** 调用模块: 无
*******************************************************************************/
UINT8 CheckCheckSum(UINT8* pData, UINT8 nLen)
{
	UINT16 bySum = 0;
	int i;
	// 计算数据的校验和	
	for(i = 1; i < nLen; i++)
	{
		bySum += pData[i];
	}		

	if (bySum == (pData[nLen] << 8)+ pData[nLen + 1])
	{
		return TRUE;
	}
	else
	{
		return FALSE;	
	}
}

/*******************************************************************************
** 函数名称: ModuleProcessPacket
** 功能描述: 命令解包
** 输　  入: pData 命令
** 输　  出: 无
** 全局变量: 无
** 调用模块: 无
*******************************************************************************/
BOOL ModuleProcessPacket(UINT8 *pData)
{	
	protocol_module_process(pData);

	return TRUE;     	
}


/*******************************************************************************
* 函数名称 : TaskDataSend
* 功能描述 : 数据发送任务，5ms执行一次
* 输入参数 : arg  创建任务时传递的参数
* 输出参数 : 无
* 返回参数 : 无
*******************************************************************************/
void TaskDataSend (void)
{
    UINT8 send_data_buf[SEND_DATA_BUF_LENGTH] = {0};
    UINT16  len;
		
		//protocol_module_send_exp_flag(1);

		//循環
		len = fifoReadData(&send_fifo, send_data_buf, SEND_DATA_BUF_LENGTH);
		if(len)
		{
				UartSendNBytes(send_data_buf, len);
		}
		
		os_delay_ms(SEND_TASK_ID, 23);
}

//封装有问题
void PaintPWM(unsigned char num)
{
	switch(num)
	{
		case 1:
			p_pwm_state=&pwm1_state;
			p_PWM_period_cnt=&PWM1_period_cnt;
			p_PWM_waitBetween_cnt=&PWM1_waitBetween_cnt;
			p_PWM_waitAfter_cnt=&PWM1_waitAfter_cnt;
			p_PWM_numOfCycle=&PWM1_numOfCycle;
			p_PWM_serial_cnt=&PWM1_serial_cnt;
			break;
		case 2:
			p_pwm_state=&pwm2_state;
			p_PWM_period_cnt=&PWM2_period_cnt;
			p_PWM_waitBetween_cnt=&PWM2_waitBetween_cnt;
			p_PWM_waitAfter_cnt=&PWM2_waitAfter_cnt;
			p_PWM_numOfCycle=&PWM2_numOfCycle;
			p_PWM_serial_cnt=&PWM2_serial_cnt;
			break;
		case 3:
			p_pwm_state=&pwm3_state;
			p_PWM_period_cnt=&PWM3_period_cnt;
			p_PWM_waitBetween_cnt=&PWM3_waitBetween_cnt;
			p_PWM_waitAfter_cnt=&PWM3_waitAfter_cnt;
			p_PWM_numOfCycle=&PWM3_numOfCycle;
			p_PWM_serial_cnt=&PWM3_serial_cnt;
			break;
		default:
			break;
	}

	if(*p_pwm_state==PWM_START)
	{
		if(pwm_buffer[(num-1)*48+8*(*p_PWM_serial_cnt)+1]==1) //如果是enable
		{
			Motor_PWM_Freq_Dudy_Set(num,pwm_buffer[(num-1)*48+8*(*p_PWM_serial_cnt)+2],pwm_buffer[(num-1)*48+8*(*p_PWM_serial_cnt)+3]);
			*p_pwm_state=PWM_PERIOD;
		}
		else
		{
			++(*p_PWM_serial_cnt);   //如果不是enable，查看下一个
			//*p_PWM_serial_cnt=*p_PWM_serial_cnt+1;
			*p_pwm_state=PWM_START;
		}
	}
	if(*p_pwm_state==PWM_PERIOD)
	{
		if((*p_PWM_period_cnt)*50==pwm_buffer[(num-1)*48+8*(*p_PWM_serial_cnt)+4]*1000)
		{
			++(*p_PWM_numOfCycle);
			//*p_PWM_numOfCycle=*p_PWM_numOfCycle+1;
			*p_PWM_period_cnt=0;
			*p_pwm_state=PWM_WAIT_BETWEEN;
			Motor_PWM_Freq_Dudy_Set(num,pwm_buffer[(num-1)*48+8*(*p_PWM_serial_cnt)+2],0);   //关闭输出PWM
		}
		++(*p_PWM_period_cnt);
	}
	
	if(*p_pwm_state==PWM_WAIT_BETWEEN)
	{
		if(*p_PWM_numOfCycle==pwm_buffer[(num-1)*48+8*(*p_PWM_serial_cnt)+5])
		{
			*p_pwm_state=PWM_WAIT_AFTER;
			*p_PWM_numOfCycle=0;
		}
		if((*p_PWM_waitBetween_cnt)*50==pwm_buffer[(num-1)*48+8*(*p_PWM_serial_cnt)+6]*1000)
		{ 
			Motor_PWM_Freq_Dudy_Set(num,pwm_buffer[(num-1)*48+8*(*p_PWM_serial_cnt)+2],pwm_buffer[(num-1)*48+8*(*p_PWM_serial_cnt)+3]); //打开输出PWM
			*p_PWM_waitBetween_cnt=0;
			*p_pwm_state=PWM_PERIOD;
		}
		++(*p_PWM_waitBetween_cnt);
	}
	
	if(*p_pwm_state==PWM_WAIT_AFTER)
	{
		if(*p_PWM_serial_cnt==6)
		{
			*p_pwm_state=PWM_OUTPUT_FINISH;
			*p_PWM_serial_cnt=0;
		}
		if((*p_PWM_waitAfter_cnt)*50==pwm_buffer[(num-1)*48+8*(*p_PWM_serial_cnt)+7]*1000)
		{
			*p_PWM_numOfCycle=0;
			Motor_PWM_Freq_Dudy_Set(num,pwm_buffer[(num-1)*48+8*(*p_PWM_serial_cnt)+2],0);   //关闭输出PWM
			++(*p_PWM_serial_cnt);
			*p_pwm_state=PWM_START;
			*p_PWM_waitAfter_cnt=0;
		}
		++(*p_PWM_waitAfter_cnt);
	}
	
}


/*******************************************************************************
** 函数名称: check_selectedMode_ouputPWM
** 功能描述: 检查模式，并对应的输出PWM波形
** 输　  入: 无
** 输　  出: 无
** 全局变量: 无
** 调用模块: 无
*******************************************************************************/

void check_selectedMode_ouputPWM()
{
	static uint16_t result=0; 
	//if(mcu_state==POWER_ON&&rcvParameters_from_PC==TRUE) //上电了并且上位机发送的参数处理ok了
	if(TRUE) 
	{
		
		if(state==LOAD_PARA)      //从flash中加载参数到内存
		{
			//先暂时屏蔽
//			uint8_t len=PARAMETER_BUF_LEN/4;  
//			uint32_t tmp[PARAMETER_BUF_LEN/4]={0};                       
//			//读取flash数据到buffer中
//			FlashRead(FLASH_WRITE_START_ADDR,tmp,len);
//			memcpy(buffer,tmp,PARAMETER_BUF_LEN);
			state=GET_MODE;
		}
		if(state==GET_MODE)    //flash参数加载内存之后，获取开关对应的模式
		{
			mode=GetModeSelected();  //得到模式
			Delay_ms(10);
			result=ADS115_readByte(0x90); //0x90,ADS115器件地址 ,得到I2C转换的值，用于对比压力是否达到threshold
			Delay_ms(10);
			state=CHECK_PRESSURE;
		}
		if(state==CHECK_PRESSURE) //检测压力
		{
			if(result>=buffer[0])  //压力达到threshold，进入输出PWM模式
			//if(result>=0)
			{
				state=PREV_OUTPUT_PWM;
			}
			else
			{
				state=CHECK_PRESSURE_AGAIN;
				
			}
		}

//		if(state==CHECK_PRESSURE_AGAIN) //再次检测压力
//		{
//			if(50*checkPressAgain_cnt==60*1000)   //连续60s检测不到，进入POWER_OFF
//			{
//				checkPressAgain_cnt=0;
//				mcu_state=POWER_OFF;
//			}
//			if(result<pwm_buffer[0])
//			{
//				checkPressAgain_cnt++;
//			}
//			else	
//			{
//				checkPressAgain_cnt=0;
//				rcvParameters_from_PC=FALSE; //重新接收参数设置
//			}
//		}

		if(state==PREV_OUTPUT_PWM)  //开始预备输出PWM波形
		{
				Delay_ms(buffer[1]*1000);
				state=CPY_PARA_TO_BUFFER;
		}
		if(state==CPY_PARA_TO_BUFFER)  //根据选择的模式，将para填充到pwm_buffer中
		{
			switch(mode)
			{
				case 1:
					memcpy(pwm_buffer,buffer+2,144);            
					break;
				case 2:
					memcpy(pwm_buffer,buffer+146,144);
					break;
				case 3:
					memcpy(pwm_buffer,buffer+290,144);
					break;
				default:
					break;
			}
			state=OUTPUT_PWM;
		}
		
		if(state==OUTPUT_PWM) //按照设定的参数，输出PWM1,PWM2,PWM3
		{
//			if(pwm1_state==PWM1_OUTPUT_FINISH&&pwm2_state==PWM2_OUTPUT_FINISH&&pwm3_state==PWM3_OUTPUT_FINISH)
//			{
//				state=CHECK_BAT_VOL;
//			}

			//**********输出PWM1,请不要删除,重要程序
			#if 0
			if(pwm1_state==PWM1_START)
			{
				if(pwm_buffer[8*PWM1_serial_cnt+1]==1) //如果是enable
				{
					Motor_PWM_Freq_Dudy_Set(1,pwm_buffer[8*PWM1_serial_cnt+2],pwm_buffer[8*PWM1_serial_cnt+3]);
					pwm1_state=PWM1_PERIOD;
				}
				else
				{
					PWM1_serial_cnt++;   //如果不是enable，查看下一个
					pwm1_state=PWM1_START;
				}
			}
			if(pwm1_state==PWM1_PERIOD)
			{
				if(PWM1_period_cnt*50==pwm_buffer[8*PWM1_serial_cnt+4]*1000)
				{
					PWM1_numOfCycle++;
					PWM1_period_cnt=0;
					pwm1_state=PWM1_WAIT_BETWEEN;
					Motor_PWM_Freq_Dudy_Set(1,pwm_buffer[8*PWM1_serial_cnt+2],0);   //关闭输出PWM
				}
				PWM1_period_cnt++;
			}
			
			if(pwm1_state==PWM1_WAIT_BETWEEN)
			{
				if(PWM1_numOfCycle==pwm_buffer[8*PWM1_serial_cnt+5])
				{
					pwm1_state=PWM1_WAIT_AFTER;
					PWM1_numOfCycle=0;
				}
				if(PWM1_waitBetween_cnt*50==pwm_buffer[8*PWM1_serial_cnt+6]*1000)
				{ 
					Motor_PWM_Freq_Dudy_Set(1,pwm_buffer[8*PWM1_serial_cnt+2],pwm_buffer[8*PWM1_serial_cnt+3]); //打开输出PWM
					PWM1_waitBetween_cnt=0;
					pwm1_state=PWM1_PERIOD;
				}
				PWM1_waitBetween_cnt++;
			}
			
			if(pwm1_state==PWM1_WAIT_AFTER)
			{
				if(PWM1_serial_cnt==6)
				{
					pwm1_state=PWM1_OUTPUT_FINISH;
					PWM1_serial_cnt=0;
				}
				if(PWM1_waitAfter_cnt*50==pwm_buffer[8*PWM1_serial_cnt+7]*1000)
				{
					PWM1_numOfCycle=0;
					Motor_PWM_Freq_Dudy_Set(1,pwm_buffer[8*PWM1_serial_cnt+2],0);   //关闭输出PWM
					PWM1_serial_cnt++;
					pwm1_state=PWM1_START;
					PWM1_waitAfter_cnt=0;
				}
				PWM1_waitAfter_cnt++;
			}
			#endif
			
			PaintPWM(1);
			PaintPWM(2);
			PaintPWM(3);

	
		#if 0
//		
//		if(state==CHECK_BAT_VOL) 
//		{
//			uint16_t result=Adc_Switch(ADC_Channel_0);
//			if(result<1365) //如果电压小于2.2v
//			{
//				//闪灯，进入POWER_OFF
//				state=LED_RED_BLINK;
//			}
//			else
//			{
//				rcvParameters_from_PC=FALSE; //重新接收参数设置
//			}
//		}
		
//		if(state==LED_RED_BLINK)
//		{
//			//橙色LED闪3s，关机
//			#if 0
//						//static uint8_t key_down_cnt = 0;
//			//static uint16_t key_wakeup_value;

////			static uint8_t high_level_cnt=0;
////			static uint8_t low_level_cnt=0;
////			static uint8_t cycle_cnt=0;
////			if(cycle_cnt==3)
////			{
////				//完成3个周期，key_pressed_down为false，让后面的程序继续检测按键是否按下
////				low_level_cnt=255;
////				high_level_cnt=255;
////				cycle_cnt=0;
////				//key_state=KEY_UPING;
////				mcu_state=POWER_OFF;
////			}
////			if(high_level_cnt==25)
////			{
////				//2.灯亮计数到25，关灯,关灯计数
////					set_led(LED_CLOSE);
////					low_level_cnt++;
////			}
////			if(low_level_cnt==25)
////			{
////				//3.关灯计数到25，完成一个周期，cnt加一
////				if(cycle_cnt<3)
////				{
////					low_level_cnt=0;
////					high_level_cnt=0;
////				}
////				cycle_cnt++;
////			}
////			if(low_level_cnt==0)
////			{
////				//1.红灯亮，灯亮计数
////				if(cycle_cnt<3)
////				{
////					set_led(LED_RED);
////					high_level_cnt++;
////				}
////			}	
//			#endif
//			for(int i=0;i<3;i++)
//			{
//				set_led(LED_RED);
//				Delay_ms(500);
//				set_led(LED_CLOSE);
//				Delay_ms(500);
//			}
//			mcu_state=POWER_OFF;
#endif
		}
		
	}
	os_delay_ms(TASK_OUTPUT_PWM, 50);
}





/*******************************************************************************
** 函数名称: CMD_ProcessTask
** 功能描述: 命令解析任务
** 输　  入: arg  创建任务时传递的参数
** 输　  出: 无
** 全局变量: 无
** 调用模块: 无
*******************************************************************************/
void CMD_ProcessTask (void)
{
	//循環
	ReceiveData(&g_CmdReceive);//接收数据到缓冲区
	ModuleUnPackFrame();//命令处理
	os_delay_ms(RECEIVE_TASK_ID, 120);
}
