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



//����������λ���������Ĳ���
UINT8 parameter_buf[PARAMETER_BUF_LEN]; 

UINT16 check_sum;
extern MCU_STATE mcu_state;
extern BOOL rcvParameters_from_PC;
extern KEY_STATE key_state;
extern const uint8_t default_parameter_buf[PARAMETER_BUF_LEN];

extern uint16_t RegularConvData_Tab[2];

uint16_t prev_cnt;
uint16_t cnt;

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

volatile CHCKMODE_OUTPUT_PWM state=LOAD_PARA;
uint16_t mode;                      

uint8_t pwm_buffer[144]={0};
//uint16_t	mode;

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
		if(*p_PWM_serial_cnt>5)
		{
			Motor_PWM_Freq_Dudy_Set(num,pwm_buffer[(num-1)*48+8*(*p_PWM_serial_cnt)+2],0);
			*p_pwm_state=PWM_OUTPUT_FINISH;
			*p_PWM_serial_cnt=0;
		}
		else
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
		}
	}
	
	if(*p_pwm_state==PWM_PERIOD)
	{
		static uint16_t cnt;
		if(p_PWM_waitBetween_cnt!=0)
		{
			cnt=(*p_PWM_period_cnt+1)*CHECK_MODE_OUTPUT_PWM;
		}
		else
		{
			cnt=(*p_PWM_period_cnt)*CHECK_MODE_OUTPUT_PWM;
		}
		//if((*p_PWM_period_cnt)*CHECK_MODE_OUTPUT_PWM==pwm_buffer[(num-1)*48+8*(*p_PWM_serial_cnt)+4]*1000)
		if(cnt==pwm_buffer[(num-1)*48+8*(*p_PWM_serial_cnt)+4]*1000)
		{
			cnt=0;
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
		else
		{
			if((*p_PWM_waitBetween_cnt+1)*CHECK_MODE_OUTPUT_PWM==pwm_buffer[(num-1)*48+8*(*p_PWM_serial_cnt)+6]*1000)
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
	}
	
	if(*p_pwm_state==PWM_WAIT_AFTER)
	{
//		if(*p_PWM_serial_cnt>=5)
//		{
//			*p_pwm_state=PWM_OUTPUT_FINISH;
//			//*p_PWM_serial_cnt=0;
//		}
		if((*p_PWM_waitAfter_cnt+2)*CHECK_MODE_OUTPUT_PWM==pwm_buffer[(num-1)*48+8*(*p_PWM_serial_cnt)+7]*1000)
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

void ResetParameter(uint8_t* buffer)
{
	//���������flash���ݿ�����buffer��
	for(int i=0;i<PARAMETER_BUF_LEN;i++)
	{
		buffer[i]=default_parameter_buf[i];
	}
	//����flash
	FlashWrite(FLASH_WRITE_START_ADDR,buffer,PARAMETER_BUF_LEN/4);
}

void CheckFlashData(unsigned char* buffer)
{
	uint16_t j=0;
	//������ݳ������Ĭ�ϵ�����
	if(buffer[0]<1||buffer[0]>50)
	{
		ResetParameter(buffer);
		return;
	}
	for(int i=0;i<54;i++)
	{
		j++;                 //1.������һ��
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
		if(buffer[2+j]<10||buffer[2+j]>90) //4.duty cycle
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
** ��������: check_selectedMode_ouputPWM
** ��������: ���ģʽ������Ӧ�����PWM����
** �䡡  ��: ��
** �䡡  ��: ��
** ȫ�ֱ���: ��
** ����ģ��: ��
*******************************************************************************/
void check_selectedMode_ouputPWM()
{
//			static uint16_t result3;
//		static uint16_t result4;
//		 result3=Adc_Switch(ADC_Channel_1);
//		 
//		 result4=Adc_Switch(ADC_Channel_4);
	
	uint16_t result=0; 
	if(mcu_state==POWER_ON)
	//if(TRUE)
		//if(FALSE)
	{
		//1.��flash�м��ز������ڴ�
		if(state==LOAD_PARA)      //��flash�м��ز������ڴ�
		{
			uint8_t len=PARAMETER_BUF_LEN/4;  
			uint32_t tmp[PARAMETER_BUF_LEN/4]={0};   		
			
			//��ȡflash���ݵ�buffer��
			FlashRead(FLASH_WRITE_START_ADDR,tmp,len);
			memcpy(parameter_buf,tmp,PARAMETER_BUF_LEN);
			CheckFlashData(parameter_buf);
			state=GET_MODE;
		}
		//2.��ÿ��ض�Ӧ��ģʽ
		if(state==GET_MODE)    //flash���������ڴ�֮�󣬻�ȡ���ض�Ӧ��ģʽ
		{
			mode=GetModeSelected();  //�õ�ģʽ
			//mode=1;
			Delay_ms(10);
			result=ADS115_readByte(0x90); //0x90,ADS115������ַ ,�õ�I2Cת����ֵ�����ڶԱ�ѹ���Ƿ�ﵽthreshold
			//Delay_ms(10);
			state=CPY_PARA_TO_BUFFER;
		}
		//3.����ѡ���ģʽ�����ݿ�����pwm_buffer
		if(state==CPY_PARA_TO_BUFFER)  //����ѡ���ģʽ����para��䵽pwm_buffer��
		{
			switch(mode)
			{
				case 1:
					memcpy(pwm_buffer,parameter_buf+2,144);            
					break;
				case 2:
					memcpy(pwm_buffer,parameter_buf+146,144);
					break;
				case 3:
					memcpy(pwm_buffer,parameter_buf+290,144);
					break;
				default:
					break;
			}
			state=CHECK_PRESSURE;
		}
		//4.���ѹ��
		if(state==CHECK_PRESSURE) //���ѹ��
		{
			if(result>=parameter_buf[0]*70)  //ѹ���ﵽthreshold���������PWMģʽ,����75Ϊб�ʣ�5mmgH��Ӧ5*70+700
			//if(result>=0)
			{
				state=PREV_OUTPUT_PWM;
			}
			else
			{
				state=CHECK_PRESSURE_AGAIN;
			}
		}
		
		//5.���ѹ��Ok,��Ԥ��������Σ��ȶ�ʱwaitBeforeStart��ô��ʱ��
		if(state==PREV_OUTPUT_PWM)  //��ʼԤ�����PWM����
		{
				//Delay_ms(buffer[1]*1000);//�����ʱ��ඨʱ2s��3s�ͳ�������
			  if(PWM_waitBeforeStart_cnt*CHECK_MODE_OUTPUT_PWM==parameter_buf[1]*1000)
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
		
		//6.��ʼ�������
		if(state==OUTPUT_PWM) //�����趨�Ĳ��������PWM1,PWM2,PWM3
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
				PaintPWM(1,pwm_buffer);
				PaintPWM(2,pwm_buffer);
				PaintPWM(3,pwm_buffer);
				
					//���»�PWM����
			}
		}
		
		
		//7.���������ϣ�����ص�ѹ
		if(state==CHECK_BAT_VOL) 
		{
			uint16_t result;
//			for(uint8_t i=0;i<3;i++)
//			{
//				result=Adc_Switch(ADC_Channel_1);
//			}
			result=RegularConvData_Tab[0];
			//uint16_t result=3000;
			if(result<2730) //�����ѹС��2.2v,����׼3.3v��
			//if(result>2730)
			{
				//���ƣ�����POWER_OFF
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
		
		//��Ӧ4��ѹ����⣬������ѹ����ok�����ٴμ��ѹ��
		if(state==CHECK_PRESSURE_AGAIN) //�ٴμ��ѹ��
		{
			if(CHECK_MODE_OUTPUT_PWM*checkPressAgain_cnt==60*1000)   //����60s��ⲻ��������POWER_OFF
			{
				checkPressAgain_cnt=0;
				mcu_state=POWER_OFF;
			}
			if(result<pwm_buffer[0]*70)
			{
				checkPressAgain_cnt++;
			}
			else	
			{
				checkPressAgain_cnt=0;
			}
		}

		//��Ӧ7���������ص�ѹС��2.2V��������
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
			mcu_state=POWER_OFF;
		}
	}
	os_delay_ms(TASK_OUTPUT_PWM, CHECK_MODE_OUTPUT_PWM);
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
	os_delay_ms(RECEIVE_TASK_ID, 100);
}
