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

//ȫ�ֱ���
CMD_Receive g_CmdReceive;  // ������տ��ƶ���
FIFO_TYPE send_fifo;//�l�͔���FIFO
UINT8 send_buf[SEND_BUF_LEN];

UINT8 parameter_buf[PARAMETER_BUF_LEN]; //����������λ���������Ĳ���
UINT16 check_sum;
extern MCU_STATE mcu_state;
extern BOOL rcvParameters_from_PC;
extern KEY_STATE key_state;
//�ֲ�����
typedef enum
{
	LOAD_PARA,  //���ز���
	GET_MODE,
	CHECK_PRESSURE,
	CHECK_PRESSURE_AGAIN,
	PREV_OUTPUT_PWM,
	CPY_PARA_TO_BUFFER,
	OUTPUT_PWM,
	CHECK_BAT_VOL,
	LED_RED_BLINK
}CHCKMODE_OUTPUT_PWM;


//#define	LOAD_PARA 							0x01
//#define	GET_MODE  							0x02
//#define	CHECK_PRESSURE 					0x03
//#define	CHECK_PRESSURE_AGAIN 		0x04
//#define	PREV_OUTPUT_PWM 				0x05
//#define	CPY_PARA_TO_BUFFER      0x06
//#define	OUTPUT_PWM							0x07
//#define	CHECK_BAT_VOL						0x08
//#define	LED_RED_BLINK						0x09

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

volatile CHCKMODE_OUTPUT_PWM state=LOAD_PARA;
 //uint8_t state=LOAD_PARA;
 uint16_t mode;                      

 uint8_t buffer[PARAMETER_BUF_LEN]={
	 0
//	 0x05,0x03,
//	 //PWM1
//	0x11,	1,80,30,2,2,1,1,
//	0x12,	0,100,50,1,2,1,2,
//	0x13,	0,100,50,1,3,1,3,
//	0x14,	0,100,30,1,4,1,4,
//	0x15,	0,100,80,1,5,1,5,
//	0x16,	0,100,90,1,6,1,6,
//	 
//	0x21,	0,80,30,2,1,1,1,
//	0x22,	0,100,50,1,2,1,2,
//	0x23,	0,100,50,1,3,1,3,
//	0x24,	0,100,30,1,4,1,4,
//	0x25,	0,100,80,1,5,1,5,
//	0x26,	0,100,90,1,6,1,6,
//	 
//	0x31,	0,80,30,2,1,1,1,
//	0x32,	0,100,50,1,2,1,2,
//	0x33,	0,100,50,1,3,1,3,
//	0x34,	0,100,30,1,4,1,4,
//	0x35,	0,100,80,1,5,1,5,
//	0x36,	0,100,90,1,6,1,6,
//	  
//	//PWM2
//	0x11,	1,80,30,2,1,1,1,
//	0x12,	1,100,50,1,2,1,2,
//	0x13,	1,100,50,1,3,1,3,
//	0x14,	1,100,30,1,4,1,4,
//	0x15,	1,100,80,1,5,1,5,
//	0x16,	1,100,90,1,6,1,6,
//	 
//	0x21,	1,80,30,2,1,1,1,
//	0x22,	1,100,50,1,2,1,2,
//	0x23,	1,100,50,1,3,1,3,
//	0x24,	1,100,30,1,4,1,4,
//	0x25,	1,100,80,1,5,1,5,
//	0x26,	1,100,90,1,6,1,6,
//	 
//	0x31,	1,80,30,2,1,1,1,
//	0x32,	1,100,50,1,2,1,2,
//	0x33,	1,100,50,1,3,1,3,
//	0x34,	1,100,30,1,4,1,4,
//	0x35,	1,100,80,1,5,1,5,
//	0x36,	1,100,90,1,6,1,6,
//	
//	//PWM3
//	0x11,	1,80,30,2,1,1,1,
//	0x12,	1,100,50,1,2,1,2,
//	0x13,	1,100,50,1,3,1,3,
//	0x14,	1,100,30,1,4,1,4,
//	0x15,	1,100,80,1,5,1,5,
//	0x16,	1,100,90,1,6,1,6,
//	 
//	0x21,	1,80,30,2,1,1,1,
//	0x22,	1,100,50,1,2,1,2,
//	0x23,	1,100,50,1,3,1,3,
//	0x24,	1,100,30,1,4,1,4,
//	0x25,	1,100,80,1,5,1,5,
//	0x26,	1,100,90,1,6,1,6,
//	 
//	0x31,	1,80,30,2,1,1,1,
//	0x32,	1,100,50,1,2,1,2,
//	0x33,	1,100,50,1,3,1,3,
//	0x34,	1,100,30,1,4,1,4,
//	0x35,	1,100,80,1,5,1,5,
//	0x36,	1,100,90,1,6,1,6
};
 uint8_t pwm_buffer[144]={0};
 uint16_t	mode;

 uint16_t checkPressAgain_cnt=0;
 uint8_t wait_cnt=0;
/*******************************************************************************
*                                �ڲ���������
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
** ��������: ModuleUnPackFrame
** ��������: ������մ���
** �䡡  ��: ��
** �䡡  ��: ��
** ȫ�ֱ���: ��
** ����ģ��: ��
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

	// �Ӵ��ڻ������ж�ȡ���յ�������
	dwLen = GetBuf2Length(&g_CmdReceive);

	// �����ݽ��н���
	while(0 < dwLen)
	{
		byCurChar = Buf2Read(&g_CmdReceive);

		if (sPacketHeadFlag)
		{
			// ��������ͷ
			if(sPacketLenFlag)
			{
				// ������������
				pBuff[sCurPacketLen] = byCurChar;
				sCurPacketLen ++;
				sResetByte --;

				if (0 >= sResetByte)
				{
					// �������
					// ����У��ͱȽ�
					if (CheckCheckSum(pBuff, pBuff[1]))
					{
						// ������һ����Ч���ݰ�
						memcpy(sBackBuff, sDataBuff, CMD_BUFFER_LENGTH);
						ModuleProcessPacket(sBackBuff);//������*********************************
						//��ֹ�������ն�������ʱ������Ӧ���źų�����
						delay_ms(2);
					}

					sPacketHeadFlag = FALSE;
					sPacketLenFlag = FALSE;
					memset(&sDataBuff, 0x00, CMD_BUFFER_LENGTH);
				}													
			}
			else
			{
				if((CMD_BUFFER_LENGTH-1 > byCurChar) && (0 < byCurChar ))// �ݴ�����ֹ���ݰ���Ϊ49��0ʱ��� ����X5����������Ͱ�Ϊ15
				{
					// ������ģ��ĳ���
					sDataBuff[sCurPacketLen] = byCurChar;
					sResetByte = byCurChar;			
					sPacketLenFlag = TRUE;
					sCurPacketLen ++;
				}
				else
				{
					//û�н�����ģ��ĳ���, ���½���
					sPacketHeadFlag = FALSE;
					sPacketLenFlag = FALSE;					
				}
			}
		}
		
		else if (PACK_HEAD_BYTE == byCurChar)		
		{
			// ��������ͷ
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
** ��������: CheckCheckSum
** ��������: ��У��
** �䡡  ��: pData ���� nLen����
** �䡡  ��: ��
** ȫ�ֱ���: ��
** ����ģ��: ��
*******************************************************************************/
UINT8 CheckCheckSum(UINT8* pData, UINT8 nLen)
{
	UINT16 bySum = 0;
	int i;
	// �������ݵ�У���	
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
** ��������: ModuleProcessPacket
** ��������: ������
** �䡡  ��: pData ����
** �䡡  ��: ��
** ȫ�ֱ���: ��
** ����ģ��: ��
*******************************************************************************/
BOOL ModuleProcessPacket(UINT8 *pData)
{	
	protocol_module_process(pData);

	return TRUE;     	
}


/*******************************************************************************
* �������� : TaskDataSend
* �������� : ���ݷ�������5msִ��һ��
* ������� : arg  ��������ʱ���ݵĲ���
* ������� : ��
* ���ز��� : ��
*******************************************************************************/
void TaskDataSend (void)
{
    UINT8 send_data_buf[SEND_DATA_BUF_LENGTH] = {0};
    UINT16  len;
		
		//protocol_module_send_exp_flag(1);

		//ѭ�h
		len = fifoReadData(&send_fifo, send_data_buf, SEND_DATA_BUF_LENGTH);
		if(len)
		{
				UartSendNBytes(send_data_buf, len);
		}
		
		os_delay_ms(SEND_TASK_ID, 23);
}

/*******************************************************************************
* �������� : TaskDataSend
* �������� : ���ݷ�������5msִ��һ��
* ������� : arg  ��������ʱ���ݵĲ���
* ������� : ��
* ���ز��� : ��
*******************************************************************************/
void PaintPWM(unsigned char num,unsigned char* pwm_buffer)
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
		if(pwm_buffer[(num-1)*48+8*(*p_PWM_serial_cnt)+1]==1) //�����enable
		{
			Motor_PWM_Freq_Dudy_Set(num,pwm_buffer[(num-1)*48+8*(*p_PWM_serial_cnt)+2],pwm_buffer[(num-1)*48+8*(*p_PWM_serial_cnt)+3]);
			*p_pwm_state=PWM_PERIOD;
		}
		else
		{
			++(*p_PWM_serial_cnt);   //�������enable���鿴��һ��
			//*p_PWM_serial_cnt=*p_PWM_serial_cnt+1;
			//*p_pwm_state=PWM_START;
		}
		if(*p_PWM_serial_cnt>5)
		{
			Motor_PWM_Freq_Dudy_Set(num,pwm_buffer[(num-1)*48+8*(*p_PWM_serial_cnt)+2],0);
			*p_pwm_state=PWM_OUTPUT_FINISH;
			*p_PWM_serial_cnt=0;
		}
	}
	
	if(*p_pwm_state==PWM_PERIOD)
	{
		if((*p_PWM_period_cnt)*50==pwm_buffer[(num-1)*48+8*(*p_PWM_serial_cnt)+4]*1000)
		{
			++(*p_PWM_numOfCycle);
			*p_PWM_period_cnt=0;
			*p_pwm_state=PWM_WAIT_BETWEEN;
			Motor_PWM_Freq_Dudy_Set(num,pwm_buffer[(num-1)*48+8*(*p_PWM_serial_cnt)+2],0);   //�ر����PWM
		}
		else
		{
			++(*p_PWM_period_cnt);
		}
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
			Motor_PWM_Freq_Dudy_Set(num,pwm_buffer[(num-1)*48+8*(*p_PWM_serial_cnt)+2],pwm_buffer[(num-1)*48+8*(*p_PWM_serial_cnt)+3]); //�����PWM
			*p_PWM_waitBetween_cnt=0;
			*p_pwm_state=PWM_PERIOD;
		}
		else
		{
			++(*p_PWM_waitBetween_cnt);
		}
	}
	
	if(*p_pwm_state==PWM_WAIT_AFTER)
	{
//		if(*p_PWM_serial_cnt>=5)
//		{
//			*p_pwm_state=PWM_OUTPUT_FINISH;
//			//*p_PWM_serial_cnt=0;
//		}
		if((*p_PWM_waitAfter_cnt)*50==pwm_buffer[(num-1)*48+8*(*p_PWM_serial_cnt)+7]*1000)
		{
			*p_PWM_numOfCycle=0;
			Motor_PWM_Freq_Dudy_Set(num,pwm_buffer[(num-1)*48+8*(*p_PWM_serial_cnt)+2],0);   //�ر����PWM
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

/*******************************************************************************
** ��������: check_selectedMode_ouputPWM
** ��������: ���ģʽ������Ӧ�����PWM����
** �䡡  ��: ��
** �䡡  ��: ��
** ȫ�ֱ���: ��
** ����ģ��: ��
*******************************************************************************/

void check_selectedMode_ouputPWM()
{
	static uint16_t result=0; 
	if(mcu_state==POWER_ON&&rcvParameters_from_PC==TRUE) //�ϵ��˲�����λ�����͵Ĳ�������ok��
	//if(TRUE) 
	{
		if(state==LOAD_PARA)      //��flash�м��ز������ڴ�
		{
			uint8_t len=PARAMETER_BUF_LEN/4;  
			uint32_t tmp[PARAMETER_BUF_LEN/4]={0};                       
			//��ȡflash���ݵ�buffer��
			FlashRead(FLASH_WRITE_START_ADDR,tmp,len);
			memcpy(buffer,tmp,PARAMETER_BUF_LEN);
			state=GET_MODE;
		}
		if(state==GET_MODE)    //flash���������ڴ�֮�󣬻�ȡ���ض�Ӧ��ģʽ
		{
			mode=GetModeSelected();  //�õ�ģʽ
			Delay_ms(10);
			result=ADS115_readByte(0x90); //0x90,ADS115������ַ ,�õ�I2Cת����ֵ�����ڶԱ�ѹ���Ƿ�ﵽthreshold
			Delay_ms(10);
			state=CPY_PARA_TO_BUFFER;
			//state=CHECK_PRESSURE;
		}
		
		if(state==CPY_PARA_TO_BUFFER)  //����ѡ���ģʽ����para��䵽pwm_buffer��
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
			//state=OUTPUT_PWM;
			state=CHECK_PRESSURE;
		}
		
		if(state==CHECK_PRESSURE) //���ѹ��
		{
			if(result>=buffer[0])  //ѹ���ﵽthreshold���������PWMģʽ
			//if(result>=0)
			{
				//state=PREV_OUTPUT_PWM;
				state=PREV_OUTPUT_PWM;
			}
			else
			{
				state=CHECK_PRESSURE_AGAIN;
				
			}
		}

		if(state==CHECK_PRESSURE_AGAIN) //�ٴμ��ѹ��
		{
			if(50*checkPressAgain_cnt==60*1000)   //����60s��ⲻ��������POWER_OFF
			{
				checkPressAgain_cnt=0;
				mcu_state=POWER_OFF;
			}
			if(result<pwm_buffer[0])
			{
				checkPressAgain_cnt++;
			}
			else	
			{
				checkPressAgain_cnt=0;
				rcvParameters_from_PC=FALSE; //���½��ղ�������
			}
		}

		if(state==PREV_OUTPUT_PWM)  //��ʼԤ�����PWM����
		{
				//Delay_ms(buffer[1]*1000);//�����ʱ��ඨʱ2s��3s�ͳ�������
			  if(PWM_waitBeforeStart_cnt*50==buffer[1]*1000)
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
		
		if(state==OUTPUT_PWM) //�����趨�Ĳ��������PWM1,PWM2,PWM3
		{
			if(pwm1_state==PWM_OUTPUT_FINISH&&pwm2_state==PWM_OUTPUT_FINISH&&pwm3_state==PWM_OUTPUT_FINISH)
			{
				PWM1_serial_cnt=0;
				PWM2_serial_cnt=0;
				PWM3_serial_cnt=0;
				state=CHECK_BAT_VOL;
				rcvParameters_from_PC=FALSE;
			}			
			PaintPWM(1,pwm_buffer);
			PaintPWM(2,pwm_buffer);
			PaintPWM(3,pwm_buffer);
		}
	
		if(state==CHECK_BAT_VOL) 
		{
			//Delay_ms(10);
			//ADC1_Init();
			uint16_t result=Adc_Switch(ADC_Channel_1);
			if(result<2730) //�����ѹС��2.2v,����׼3.3v��
			//if(result>1365)
			{
				//���ƣ�����POWER_OFF
				state=LED_RED_BLINK;
			}
			else
			{
				rcvParameters_from_PC=FALSE; //���½��ղ�������
				state=LOAD_PARA;
				pwm1_state=PWM_START;
				pwm2_state=PWM_START;
				pwm3_state=PWM_START;
			}
		}
		
		if(state==LED_RED_BLINK)
		{
			//��ɫLED��3s
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
			//mcu_state=POWER_OFF;
		}
	}
	os_delay_ms(TASK_OUTPUT_PWM, 50);
}





/*******************************************************************************
** ��������: CMD_ProcessTask
** ��������: �����������
** �䡡  ��: arg  ��������ʱ���ݵĲ���
** �䡡  ��: ��
** ȫ�ֱ���: ��
** ����ģ��: ��
*******************************************************************************/
void CMD_ProcessTask (void)
{
	//ѭ�h
	ReceiveData(&g_CmdReceive);//�������ݵ�������
	ModuleUnPackFrame();//�����
	os_delay_ms(RECEIVE_TASK_ID, 120);
}
