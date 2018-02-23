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



//用来接收上位机传过来的参数
UINT8 parameter_buf[PARAMETER_BUF_LEN]; 

UINT8 buffer[PARAMETER_BUF_LEN];

UINT16 check_sum;
extern BOOL b_Is_PCB_PowerOn;
extern MCU_STATE mcu_state;
extern BOOL rcvParameters_from_PC;
extern KEY_STATE key_state;
extern const uint8_t default_parameter_buf[PARAMETER_BUF_LEN];

extern uint16_t RegularConvData_Tab[2];

//uint16_t prev_cnt;
//uint16_t cnt;

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
static uint16_t* p_PWM_period_cnt;
static uint16_t* p_PWM_waitBetween_cnt;
static uint16_t* p_PWM_waitAfter_cnt;
static uint8_t* p_PWM_numOfCycle;
static uint8_t* p_PWM_serial_cnt;

uint16_t PWM_waitBeforeStart_cnt=0;

uint16_t PWM1_period_cnt=0;
uint16_t PWM2_period_cnt=0;
uint16_t PWM3_period_cnt=0;

uint16_t PWM1_waitBetween_cnt=0;
uint16_t PWM2_waitBetween_cnt=0;
uint16_t PWM3_waitBetween_cnt=0;

uint16_t PWM1_waitAfter_cnt=0;
uint16_t PWM2_waitAfter_cnt=0;
uint16_t PWM3_waitAfter_cnt=0;

uint8_t PWM1_numOfCycle=0;
uint8_t PWM2_numOfCycle=0;
uint8_t PWM3_numOfCycle=0;

uint8_t PWM1_serial_cnt=0;
uint8_t PWM2_serial_cnt=0;
uint8_t PWM3_serial_cnt=0;

//volatile CHCKMODE_OUTPUT_PWM state=LOAD_PARA;
CHCKMODE_OUTPUT_PWM state=LOAD_PARA;
uint16_t mode;                      

//uint8_t pwm_buffer[144]={0};
//uint16_t	mode;
static uint8_t pwm1_buffer[49];
static uint8_t pwm2_buffer[49];
static uint8_t pwm3_buffer[49];
uint8_t pressure;
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
		
		os_delay_ms(SEND_TASK_ID, 24);  //mark一下
}

//void ResetAllState()
//{
//		mcu_state=POWER_OFF;
//		state=LOAD_PARA;
//		*p_pwm_state=PWM_START;
//		*p_PWM_period_cnt=0;
//		*p_PWM_waitBetween_cnt=0;
//		*p_PWM_waitAfter_cnt=0;
//		*p_PWM_numOfCycle=0;
//		*p_PWM_serial_cnt=0;
//		//PWM_waitBeforeStart_cnt=0;
//		Motor_PWM_Freq_Dudy_Set(1,100,0);
//		Motor_PWM_Freq_Dudy_Set(2,100,0);
//		Motor_PWM_Freq_Dudy_Set(3,100,0);
//}

/*******************************************************************************
* 函数名称 : TaskDataSend
* 功能描述 : 数据发送任务，5ms执行一次
* 输入参数 : arg  创建任务时传递的参数
* 输出参数 : 无
* 返回参数 : 无
*******************************************************************************/
void PaintPWM(unsigned char num,unsigned char* buffer)
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
	if(b_Is_PCB_PowerOn==FALSE)
	{
//		ResetAllState();
		mcu_state=POWER_OFF;
		state=LOAD_PARA;
		*p_pwm_state=PWM_START;
		*p_PWM_period_cnt=0;
		*p_PWM_waitBetween_cnt=0;
		*p_PWM_waitAfter_cnt=0;
		*p_PWM_numOfCycle=0;
		*p_PWM_serial_cnt=0;
		//PWM_waitBeforeStart_cnt=0;
		Motor_PWM_Freq_Dudy_Set(num,100,0);
	}
	else
	{
		if(*p_pwm_state==PWM_START)
		{
			if(*p_PWM_serial_cnt>buffer[0]-1)
			{
				Motor_PWM_Freq_Dudy_Set(num,buffer[1+8*(*p_PWM_serial_cnt)+2],0);
				*p_pwm_state=PWM_OUTPUT_FINISH;
				*p_PWM_serial_cnt=0;
			}
			else
			{
				Motor_PWM_Freq_Dudy_Set(num,buffer[1+8*(*p_PWM_serial_cnt)+2],buffer[1+8*(*p_PWM_serial_cnt)+3]);
				*p_pwm_state=PWM_PERIOD;
			}
		}
		
		if(*p_pwm_state==PWM_PERIOD)
		{
			if((*p_PWM_period_cnt)*CHECK_MODE_OUTPUT_PWM==buffer[1+8*(*p_PWM_serial_cnt)+4]*1000)
			{
				++(*p_PWM_numOfCycle);
				*p_PWM_period_cnt=0;
				*p_pwm_state=PWM_WAIT_BETWEEN;
				Motor_PWM_Freq_Dudy_Set(num,buffer[1+8*(*p_PWM_serial_cnt)+2],0);
			}
			else
			{
				++(*p_PWM_period_cnt);
			}
		}
		
		if(*p_pwm_state==PWM_WAIT_BETWEEN)
		{
			if(*p_PWM_numOfCycle==buffer[1+8*(*p_PWM_serial_cnt)+5])
			{
				*p_pwm_state=PWM_WAIT_AFTER;
				*p_PWM_numOfCycle=0;
			}
			else
			{
				if((*p_PWM_waitBetween_cnt)*CHECK_MODE_OUTPUT_PWM==buffer[1+8*(*p_PWM_serial_cnt)+6]*1000)
				{ 
					Motor_PWM_Freq_Dudy_Set(num,buffer[1+8*(*p_PWM_serial_cnt)+2],buffer[1+8*(*p_PWM_serial_cnt)+3]); 
					*p_PWM_waitBetween_cnt=0;
					*p_pwm_state=PWM_PERIOD;
				}
				else
				{
					++(*p_PWM_waitBetween_cnt);
				}
			}
		}
		
		if(*p_pwm_state==PWM_WAIT_AFTER)
		{
			if((*p_PWM_waitAfter_cnt)*CHECK_MODE_OUTPUT_PWM==buffer[1+8*(*p_PWM_serial_cnt)+7]*1000)
			{
				*p_PWM_numOfCycle=0;
				Motor_PWM_Freq_Dudy_Set(num,buffer[1+8*(*p_PWM_serial_cnt)+2],0);
				++(*p_PWM_serial_cnt);
				*p_pwm_state=PWM_START;
				*p_PWM_waitAfter_cnt=0;
			}
			else	
			{
				++(*p_PWM_waitAfter_cnt);
			}
		}
	}

}

void ResetParameter(unsigned char* buffer)
{
	//将代码段中flash数据拷贝到buffer中
	for(int i=0;i<PARAMETER_BUF_LEN;i++)
	{
		buffer[i]=default_parameter_buf[i];
	}
	//更新flash
	FlashWrite(FLASH_WRITE_START_ADDR,buffer,PARAMETER_BUF_LEN/4);
}

void CheckFlashData(unsigned char* buffer)
{
	uint16_t j=0;
	//如果数据出错就用默认的数据
	if(buffer[0]<1||buffer[0]>50)
	{
		ResetParameter(buffer);
		return;
	}
	for(int i=0;i<54;i++)
	{
		j++;                 //1.跳过第一个
		if(buffer[2+j++]>1) //2.enable
		{
			ResetParameter(buffer);
			return;
		}
		if(buffer[2+j++]==0)  //3.freq
		{
			ResetParameter(buffer);
			return;
		}
		if(buffer[2+j]<5||buffer[2+j]>99) //4.duty cycle
		{
			ResetParameter(buffer);
			return;
		}
		j++;
		if(buffer[2+j++]==0)            //5.period
		{
			ResetParameter(buffer);
			return;
		}
		if(buffer[2+j]<1||buffer[2+j]>250)            //6.number of cycle
		{
			ResetParameter(buffer);
			return;
		}
		j++;
		
		j++;                                  //7.wait between
		j++;																	//8.wait after
	}
}

/*******************************************************************************
** 函数名称: FillUpPWMbuffer
** 功能描述: 按照serial的有和无来填充pwm1_buffer,pwm2_buffer,pwm3_buffer
** 输　  入: 无
** 输　  出: 无
** 全局变量: 无
** 调用模块: 无
*******************************************************************************/
void FillUpPWMbuffer(uint8_t* dest,uint8_t* src)
{
	uint8_t serial_cnt=0;
	uint8_t j=1;
	for(int i=0;i<6;i++)
	{
		if(src[8*i+1]==0x01)
		{
			uint8_t k;
			for(k=0;k<8;k++)
			{
				dest[j++]=src[8*i+k];
			}
			serial_cnt++;
		}
	}
	dest[0]=serial_cnt;
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
	static uint16_t pressure_result; 
	if(mcu_state==POWER_ON)
	{
		//1.从flash中加载参数到内存
		if(state==LOAD_PARA)      
		{
			uint8_t len=PARAMETER_BUF_LEN/4;  
			uint32_t tmp[PARAMETER_BUF_LEN/4]={0};   		
			
			//读取flash数据到buffer中
			FlashRead(FLASH_WRITE_START_ADDR,tmp,len);
			memcpy(buffer,tmp,PARAMETER_BUF_LEN);
			CheckFlashData(buffer);
			state=GET_MODE;
		}
		//2.获得开关对应的模式
		if(state==GET_MODE)    //flash参数加载内存之后，获取开关对应的模式
		{
			mode=GetModeSelected();  //得到模式
			//mode=1;
			//Delay_ms(10);
			//pressure_result=ADS115_readByte(0x90); //0x90,ADS115器件地址 ,得到I2C转换的值，用于对比压力是否达到threshold
			//Delay_ms(10);
			state=CPY_PARA_TO_BUFFER;
		}
		//3.根据选择的模式将数据拷贝到pwm_buffer
		if(state==CPY_PARA_TO_BUFFER)  //根据选择的模式，将para填充到pwm_buffer中
		{
			uint8_t pwm_buffer[144];
			
			memset(pwm1_buffer,0,49);
			memset(pwm2_buffer,0,49);
			memset(pwm3_buffer,0,49);
			
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
			FillUpPWMbuffer(pwm1_buffer,pwm_buffer);
			FillUpPWMbuffer(pwm2_buffer,pwm_buffer+48);
			FillUpPWMbuffer(pwm3_buffer,pwm_buffer+96);
			state=CHECK_PRESSURE;
		}
		
		//4.检测压力
		if(state==CHECK_PRESSURE) //检测压力
		{
			pressure_result=ADS115_readByte(0x90);
			if(pressure_result>=buffer[0]*70)  //压力达到threshold，进入输出PWM模式,其中75为斜率，5mmgH对应5*70+700
			{
				state=PREV_OUTPUT_PWM;
			}
			else
			{
				state=CHECK_PRESSURE_AGAIN;
			}
		}
		
		//5.检测压力Ok,则预备输出波形，先定时waitBeforeStart这么长时间
		if(state==PREV_OUTPUT_PWM)  //开始预备输出PWM波形
		{
			//如果不加if(b_Is_PCB_PowerOn==FALSE)会导致开关重新开机waitbeforestart定时不到想要的秒数
			if(b_Is_PCB_PowerOn==FALSE)
			{
				PWM_waitBeforeStart_cnt=0;
			}
			else
			{
				if((PWM_waitBeforeStart_cnt)*CHECK_MODE_OUTPUT_PWM==buffer[1]*1000)
				{
					PWM_waitBeforeStart_cnt=0;
					//state=CPY_PARA_TO_BUFFER;
					state=OUTPUT_PWM;
				}
				else
				{
					PWM_waitBeforeStart_cnt++;
				}
			}
			  
		}
		
		//6.开始输出波形
		if(state==OUTPUT_PWM) //按照设定的参数，输出PWM1,PWM2,PWM3
		{			
			if(pwm1_state==PWM_OUTPUT_FINISH&&pwm2_state==PWM_OUTPUT_FINISH&&pwm3_state==PWM_OUTPUT_FINISH)
			{
				PWM1_serial_cnt=0;
				PWM2_serial_cnt=0;
				PWM3_serial_cnt=0;
				state=CHECK_BAT_VOL;
			}		
			else
			{
				PaintPWM(1,pwm1_buffer); 
				PaintPWM(2,pwm2_buffer);
				PaintPWM(3,pwm3_buffer);
			}
		}
		
		//7.波形输出完毕，检测电池电压
		if(state==CHECK_BAT_VOL) 
		{
			uint16_t result;
			result=RegularConvData_Tab[0];
			if(result<2730) //如果电压小于2.2v,（基准3.3v）
			{
				//闪灯，进入POWER_OFF
				state=LED_RED_BLINK;
			}
			else
			{
				state=LOAD_PARA;
				pwm1_state=PWM_START;
				pwm2_state=PWM_START;
				pwm3_state=PWM_START;
			}
		}
		
		//对应4，压力检测，如果检测压力不ok，则再次检测压力
		if(state==CHECK_PRESSURE_AGAIN) //再次检测压力
		{
			if(CHECK_MODE_OUTPUT_PWM*checkPressAgain_cnt==60*1000)   //连续60s检测不到，进入POWER_OFF
			{
				checkPressAgain_cnt=0;
				mcu_state=POWER_OFF;
				state=LOAD_PARA;
				set_led(LED_CLOSE);
			}
			else
			{
				pressure_result=ADS115_readByte(0x90);
				//特别注意，这里不能用全局变量buffer,而应该用parameter_buf
				//理由：如果进入60s倒计时状态，此时的buffer的值在CHECK_PRESSURE_AGAIN状态已经固定了
				//如果此时上位机更新了参数，parameter_buf[0]会改变，应该用这个变化了的值来判断
				if(pressure_result<parameter_buf[0]*70) 
				{
					checkPressAgain_cnt++;
				}
				else	
				{
					checkPressAgain_cnt=0;
					state=LOAD_PARA;
				}
			}
			
		}

		//对应7，如果检测电池电压小于2.2V，则闪灯
		if(state==LED_RED_BLINK)
		{
			//橙色LED闪3s
			for(int i=0;i<3;i++)
			{
				set_led(LED_RED);
				Delay_ms(500);
				set_led(LED_CLOSE);
				Delay_ms(500);
			}
			//Delay_ms(10);
			state=LOAD_PARA;
			pwm1_state=PWM_START;
			pwm2_state=PWM_START;
			pwm3_state=PWM_START;
			mcu_state=POWER_OFF;
		}
	}
	os_delay_ms(TASK_OUTPUT_PWM, CHECK_MODE_OUTPUT_PWM);
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
	os_delay_ms(RECEIVE_TASK_ID, 100);
}
