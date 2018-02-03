#ifndef __COMM_TASK_H
#define __COMM_TASK_H	    
//////////////////////////////////////////////////////////////////////////////////	 							  
//////////////////////////////////////////////////////////////////////////////////

#include "datatype.h"

//#define SEND_BUF_LEN  255
#define SEND_BUF_LEN  248

//434是数据，2是两个校验位
#define PARAMETER_BUF_LEN 436


void TaskDataSend (void);
void CMD_ProcessTask (void);
void CalcCheckSum(UINT8* pPacket);
void check_selectedMode_ouputPWM(void);
void PaintPWM(unsigned char num,unsigned char* pwm_buffer);
#endif
