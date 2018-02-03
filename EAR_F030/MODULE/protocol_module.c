/**
********************************************************************************
* ��ࣺ
* ģ�����ƣ�protocol.c
* ģ�鹦�ܣ�����λ�C�M��ͨ��
* �������ڣ�
* �� �� �ߣ�
* ��    ����
********************************************************************************
**/

/***********************************
* ͷ�ļ�
***********************************/
#include "stm32f0xx_usart.h"
#include "stm32f0xx.h"
#include "datatype.h"
#include "serial_port.h"
#include "hardware.h"
#include "fifo.h"
#include "protocol_module.h"
#include "comm_task.h"
#include "os_core.h"
#include "app.h"
#include "string.h"
#include "stdlib.h"
#include "stm32f0xx_flash.h"
#include "key_led_task.h"

#include "common.h"
/**********************************
*�궨��
***********************************/

/***********************************
* ȫ�ֱ���
***********************************/
//BOOL rcvParameters_from_PC=FALSE;

//�l�͔���FIFO
extern FIFO_TYPE send_fifo;
extern uint8_t send_buf[SEND_BUF_LEN];

extern UINT8 parameter_buf[PARAMETER_BUF_LEN];  //����Ϊ434+2������������λ���������Ĳ�������
extern UINT16 check_sum;
//�����յ���λ��������������ʱ���ñ�����ΪTRUE,���������ΪFALSE
//extern uint8_t send_exp_train_data_status;s
extern MCU_STATE mcu_state;
extern uint16_t RegularConvData_Tab[2];
/***********************************
* �ֲ�����
***********************************/

/***********************************
* �ֲ�����
***********************************/
////�l����Ч�������
//void protocol_module_send_exp_flag(uint8_t flag)
//{
//	uint8_t buffer[CMD_BUFFER_LENGTH];
//	
//	buffer[0] = PACK_HEAD_BYTE;
//	buffer[1] = 0x05;
//	buffer[2] = MODULE_CMD_TYPE;
//	buffer[3] = SEND_VOILD_EXP_FLAG_ID;
//	buffer[4] = flag;
//	
//	CalcCheckSum(buffer);
//	
//	fifoWriteData(&send_fifo, buffer, buffer[1]+2);
//}

////�l����Ч�������
//void protocol_module_send_train_data_one_page(uint8_t* buf, uint8_t len)
//{
//	uint8_t buffer[CMD_BUFFER_LENGTH];
//	uint8_t cnt,i = 0;
//	
//	buffer[0] = PACK_HEAD_BYTE;
//	buffer[1] = 0x05;
//	buffer[2] = MODULE_CMD_TYPE;
//	buffer[3] = SEND_EXP_TRAIN_DATA_ID;
//	for(cnt = 4; cnt < len+4; cnt ++)
//	{
//		buffer[cnt] = buf[i++];
//	}
//	
//	CalcCheckSum(buffer);
//	
//	fifoWriteData(&send_fifo, buffer, buffer[1]+2);
//}

//�l������У���
void protocol_module_send_train_data_check_sum(uint32_t check_sum)
{
	uint8_t buffer[CMD_BUFFER_LENGTH];
	
	buffer[0] = PACK_HEAD_BYTE;
	buffer[1] = 0x08;
	buffer[2] = MODULE_CMD_TYPE;
	buffer[3] = SEND_EXP_TRAIN_DATA_CHECK_SUM_ID;
	buffer[4] = check_sum & 0xff;
	buffer[5] = (check_sum >> 8) & 0xff;
	buffer[6] = (check_sum >> 16) & 0xff;
	buffer[7] = (check_sum >> 24) & 0xff;
	
	CalcCheckSum(buffer);
	
	fifoWriteData(&send_fifo, buffer, buffer[1]+2);
}

////�l������У���
//void protocol_module_send_bat_per(uint8_t bat_per)
//{
//	uint8_t buffer[CMD_BUFFER_LENGTH];
//	
//	buffer[0] = PACK_HEAD_BYTE;
//	buffer[1] = 0x08;
//	buffer[2] = MODULE_CMD_TYPE;
//	buffer[3] = SEND_BAT_PER_ID;
//	buffer[4] = bat_per;
//	
//	CalcCheckSum(buffer);
//	
//	fifoWriteData(&send_fifo, buffer, buffer[1]+2);
//}

void get_comm_para_to_buf(uint8_t* pdata)
{
	memset(parameter_buf,0,PARAMETER_BUF_LEN);
	check_sum=0;
	UINT8* pPos=(UINT8*)&parameter_buf;
	memset(&parameter_buf,0,PARAMETER_BUF_LEN);
	memcpy(pPos,pdata+4,1);
	memcpy(pPos+1,pdata+5,1);
	check_sum+=*pPos+*(pPos+1);
//	//debug
//	uint8_t buffer[2]={0};
//	buffer[0]=0x11;	
//	buffer[1]=0x22;	
//	fifoWriteData(&send_fifo, (UINT8*)&parameter_buf, 1);
//	fifoWriteData(&send_fifo, (UINT8*)(&parameter_buf+1), 1);
}

void get_parameter_to_buf_by_frameId(uint8_t* pdata,char frameId)
{
	int pos_mode1_pwm1=2;
	int pos_mode1_pwm2=50;
	int pos_mode1_pwm3=98;
	int pos_mode2_pwm1=146;
	int pos_mode2_pwm2=194;
	int pos_mode2_pwm3=242;
	int pos_mode3_pwm1=290;
	int pos_mode3_pwm2=338;
	int pos_mode3_pwm3=386;
	if(0x11==frameId)
	{
		uint8_t* pstart=pdata+4;
		uint8_t* pDest=(uint8_t*)(&parameter_buf)+pos_mode1_pwm1;
		char buf[48]={0};
		for(int i=0;i<48;i++)
		{
			buf[i]=*pstart++;
			check_sum+=buf[i];
		}
		//memcpy(pDest,buf,sizeof(buf));
		memcpy(pDest,buf,48);
	}
	else if(0x12==frameId)
	{
		uint8_t* pstart=pdata+4;
		uint8_t* pDest=(uint8_t*)(&parameter_buf)+pos_mode1_pwm2;
		char buf[48]={0};
		for(int i=0;i<48;i++)
		{
			buf[i]=*pstart++;
			check_sum+=buf[i];
		}
		//memcpy(pDest,buf,sizeof(buf));
		memcpy(pDest,buf,48);
	}
	else if(0x13==frameId)
	{
		uint8_t* pstart=pdata+4;
		uint8_t* pDest=(uint8_t*)(&parameter_buf)+pos_mode1_pwm3;
		char buf[48]={0};
		for(int i=0;i<48;i++)
		{
			buf[i]=*pstart++;
			check_sum+=buf[i];
		}
		//memcpy(pDest,buf,sizeof(buf));
		memcpy(pDest,buf,48);
	}
	else if(0x21==frameId)
	{
		uint8_t* pstart=pdata+4;
		uint8_t* pDest=(uint8_t*)(&parameter_buf)+pos_mode2_pwm1;
		char buf[48]={0};
		for(int i=0;i<48;i++)
		{
			buf[i]=*pstart++;
			check_sum+=buf[i];
		}
		//memcpy(pDest,buf,sizeof(buf));
		memcpy(pDest,buf,48);
	}
	else if(0x22==frameId)
	{
		uint8_t* pstart=pdata+4;
		uint8_t* pDest=(uint8_t*)(&parameter_buf)+pos_mode2_pwm2;
		char buf[48]={0};
		for(int i=0;i<48;i++)
		{
			buf[i]=*pstart++;
			check_sum+=buf[i];
		}
		//memcpy(pDest,buf,sizeof(buf));
		memcpy(pDest,buf,48);
	}
	else if(0x23==frameId)
	{
		uint8_t* pstart=pdata+4;
		uint8_t* pDest=(uint8_t*)(&parameter_buf)+pos_mode2_pwm3;
		char buf[48]={0};
		for(int i=0;i<48;i++)
		{
			buf[i]=*pstart++;
			check_sum+=buf[i];
		}
		//memcpy(pDest,buf,sizeof(buf));
		memcpy(pDest,buf,48);
	}
	else if(0x31==frameId)
	{
		uint8_t* pstart=pdata+4;
		uint8_t* pDest=(uint8_t*)(&parameter_buf)+pos_mode3_pwm1;
		char buf[48]={0};
		for(int i=0;i<48;i++)
		{
			buf[i]=*pstart++;
			check_sum+=buf[i];
		}
		//memcpy(pDest,buf,sizeof(buf));
		memcpy(pDest,buf,48);
	}
	else if(0x32==frameId)
	{
		uint8_t* pstart=pdata+4;
		uint8_t* pDest=(uint8_t*)(&parameter_buf)+pos_mode3_pwm2;
		char buf[48]={0};
		for(int i=0;i<48;i++)
		{
			buf[i]=*pstart++;
			check_sum+=buf[i];
		}
		//memcpy(pDest,buf,sizeof(buf));
		memcpy(pDest,buf,48);
	}
	else if(0x33==frameId)
	{
		uint8_t* pstart=pdata+4;
		uint8_t* pDest=(uint8_t*)(&parameter_buf)+pos_mode3_pwm3;
		char buf[48]={0};
		for(int i=0;i<48;i++)
		{
			buf[i]=*pstart++;
			check_sum+=buf[i];
		}
		//memcpy(pDest,buf,sizeof(buf));
		memcpy(pDest,buf,48);
		
		//�յ����һ֡������ɺ�д��flash
		//���check_sum
		uint8_t tmp1=(uint8_t)(check_sum>>8);
		uint8_t tmp2=(uint8_t)(check_sum&0xFF);
		*(parameter_buf+pos_mode3_pwm3+48)=tmp1;
		*(parameter_buf+pos_mode3_pwm3+48+1)=tmp2;
		//��parameter_buf�е�����д��flash��
		
		//FlashWrite(FLASH_WRITE_START_ADDR,(uint32_t*)&parameter_buf,PARAMETER_BUF_LEN/4);
		FlashWrite(FLASH_WRITE_START_ADDR,parameter_buf,PARAMETER_BUF_LEN/4);
		//rcvParameters_from_PC=TRUE;
		
		//��������ʾ�����������
		for(int i=0;i<10;i++)
		{
			set_led(LED_GREEN);
			Delay_ms(50);
			set_led(LED_CLOSE);
			Delay_ms(50);
		}
		if(mcu_state==POWER_ON)
		{
			set_led(LED_GREEN);
		}
	}
	else
	{
		//do nothing
	}
}

void send_prameter_fram1_to_PC()
{
	uint8_t buffer[CMD_BUFFER_LENGTH];
	
	memset(parameter_buf,0,PARAMETER_BUF_LEN);  //���parameter_buf
	//���parameter_buf
	uint8_t len=PARAMETER_BUF_LEN/4;                          
	uint32_t tmp[PARAMETER_BUF_LEN/4]={0};
	FlashRead(FLASH_WRITE_START_ADDR,tmp,len);
	memcpy(parameter_buf,tmp,len*4);
	
	//���͵�һ֡
	//������Ϣ2Bytes, (Mode1-PWM1, Mode1-PWM2, Mode1-PWM3),Mode2-PWM1,Mode2-PWM2
	buffer[0] = PACK_HEAD_BYTE;       //0xFF
	buffer[1] = 0x04+0xF2;            //0xF2=242,���ݳ���
	buffer[2] = MODULE_CMD_TYPE;      //0x00
	buffer[3] = SEND_FLASH_DATA_1_ID; //0x06
	//��乫����Ϣ
	buffer[4] = *parameter_buf;       //exhalation threshold
	buffer[5] = *(parameter_buf+1);   //wait before after
	
	unsigned char* pstart=parameter_buf+2;
	for(int i=2;i<242;i++)
	{
		buffer[i+4]=*pstart++;
	}
	CalcCheckSum(buffer);
	fifoWriteData(&send_fifo, buffer, buffer[1]+2);
}

void send_prameter_fram2_to_PC()
{
	//���͵ڶ�֡
	//Mode2-PWM3, (Mode3-PWM1,MODE3-PWM2,MODE3-PWM3)
	uint8_t buffer1[CMD_BUFFER_LENGTH];
	
	buffer1[0] = PACK_HEAD_BYTE;       //0xFF
	buffer1[1] = 0x04+0xC0;            //0xC0=192�����ݳ���
	buffer1[2] = MODULE_CMD_TYPE;      //0x00
	buffer1[3] = SEND_FLASH_DATA_2_ID; //0x07
	
	unsigned char* pstart=parameter_buf+242; //��ָ�벥���ڶ�֡��λ��
	for(int i=242;i<434;i++)
	{
		buffer1[i-238]=*pstart++;
	}
	CalcCheckSum(buffer1);
	fifoWriteData(&send_fifo, buffer1, buffer1[1]+2);
	
	//��������ʾ���շ������
	for(int i=0;i<5;i++)
	{
		set_led(LED_GREEN);
		Delay_ms(30);
		set_led(LED_CLOSE);
		Delay_ms(30);
	}
	if(mcu_state==POWER_ON)
	{
		set_led(LED_GREEN);
	}
}

uint16_t FlashWrite(uint32_t addr, uint8_t *p_data, uint16_t len)
{
	uint16_t i = 0;
	//uint32_t tmp	= 0;
	uint32_t address = addr;
	
	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);

	FLASH_ErasePage(address);
	
	if(len<1024/4)
	{
		for (i=0;i<len;i++)
		{
			//С��
			uint32_t ndata=p_data[4*i]+p_data[4*i+1]*256+p_data[4*i+2]*256*256+p_data[4*i+3]*256*256*256;
			FLASH_ProgramWord(address,ndata);
			//FLASH_Status st=FLASH_ProgramWord(address, 0x11223344);				
			address += 4;
		}
//		for (i=0;i<1024/4-len;i++)
//		{
//			FLASH_ProgramWord(address, 0);	
//			address += 4;
//		}
	}

	FLASH_Lock();
	#if 0
	//��������֤
//	address = addr;
//	for (i=0;i<len;i++)
//	{
//		tmp	= FlashReadWord(address);
//		address	+= 4;
//		if (tmp != p_data[i])
//			break;
//	}
	
	//�Ȳ���У��
//	uint16_t sum=0;
//	for(int j=0;j<4*len-2;j++)
//	{
//		sum+=FlashReadByte(address);
//		address++;
//	}
//	//�����������ݺͲ��ԣ�����-1,��ʾд��ʧ��
//	if(sum!=(*(char*)addr)*256+*(char*)(addr+1))
//	{
//		return -1;
//	}
#endif 
	return i;
}

void FlashRead(uint32_t addr, uint32_t *p_data, uint16_t len)
{
	UINT16 i = 0;
	UINT32 address = addr;
	
	if(p_data == NULL)
		return;
	
	for(i = 0; i < len; i ++)
	{
		p_data[i] = FlashReadWord(address);
		address += 4;
	}
}

uint32_t FlashReadWord(uint32_t addr)
{
	uint32_t data = 0;
	uint32_t address = addr;

	data = *(uint32_t*)address;
	return data;
}

uint8_t FlashReadByte(uint32_t addr)
{
	return (uint8_t)(*(uint8_t*)addr);
}

//�õ�����ģʽ
uint16_t GetModeSelected(void)
{
	uint16_t res;
	res=RegularConvData_Tab[1];
//	for(uint8_t i=0;i<3;i++)
//	{
//		res=Adc_Switch(ADC_Channel_4);
//	}
	
	if(res>=1500)
	{
		return 1;  //����ģʽ1
	}
	else if(res>=700&&res<1500)
	//else if(res>=mod2_base_vol-200&&res<=mod2_base_vol+200)
	{
		return 2;	//����ģʽ2
	}
	//else if(res>=138&&res<=538)
	else
	{
		return 3; //����ģʽ3
	}
}


//������λ������
void protocol_module_process(uint8_t* pdata)
{
	uint8_t *pCmdPacketData = (uint8_t *)pdata;
	uint8_t byFrameID = pCmdPacketData[3];

//	uint8_t bat_per;//��ص���
	
//	//���û���ϵ磬ֱ�ӷ���
//	if(mcu_state!=POWER_ON)
//	{
//		return;
//	}
	
	//pCmdPacketData = pdata;
	//byFrameID = pCmdPacketData[3];
	//byFrameID = *(pdata+3);

	//byFrameID = GET_BAT_PER_ID;
	switch(byFrameID)
	{

//	case GET_EXP_TRAIN_DATA_ID:
//			//���ʹ洢����
//			send_exp_train_data_status = TRUE;//�������ݷ���
//			
//			//��������
//			os_pend_task(KEY_LED_TASK_ID);
//			os_pend_task(EXP_DETECT_SAVE_TASK_ID);
//			break;

//	case GET_BAT_PER_ID:
//		//�õ���ص�ѹ
//		bat_per = get_bat_vol_per();
//		//���͸���λ��
//		protocol_module_send_bat_per(bat_per);
//		break;
//	
//	case PWM_VALUE_SET_ID:
//		//�õ���ص�ѹ
//		bat_per = get_bat_vol_per();
//		//���͸���λ��
//		protocol_module_send_bat_per(bat_per);
//		break;
	case COMM_PARAMETER_ID:
		get_comm_para_to_buf(pdata);
		break;
	case MODE1_PWM1_ID:
		get_parameter_to_buf_by_frameId(pdata,MODE1_PWM1_ID);
		break;
	case MODE1_PWM2_ID:
		get_parameter_to_buf_by_frameId(pdata,MODE1_PWM2_ID);
		break;
	case MODE1_PWM3_ID:
		get_parameter_to_buf_by_frameId(pdata,MODE1_PWM3_ID);
		break;
	case MODE2_PWM1_ID:
		get_parameter_to_buf_by_frameId(pdata,MODE2_PWM1_ID);
		break;
	case MODE2_PWM2_ID:
		get_parameter_to_buf_by_frameId(pdata,MODE2_PWM2_ID);
		break;
	case MODE2_PWM3_ID:
		get_parameter_to_buf_by_frameId(pdata,MODE2_PWM3_ID);
		break;
	case MODE3_PWM1_ID:
		get_parameter_to_buf_by_frameId(pdata,MODE3_PWM1_ID);
		break;
	case MODE3_PWM2_ID:
		get_parameter_to_buf_by_frameId(pdata,MODE3_PWM2_ID);
		break;
	case MODE3_PWM3_ID:
		get_parameter_to_buf_by_frameId(pdata,MODE3_PWM3_ID);
		break;
	case GET_FLASH_DATA_1_ID:
		send_prameter_fram1_to_PC();
		break;
	case GET_FLASH_DATA_2_ID:
		send_prameter_fram2_to_PC();
		break;
	default:
		break;
	}
}
