

/**
********************************************************************************
* 版權：
* 模块名称：app.c
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
#include "comm_task.h"
#include "CMD_Receive.h"
#include "serial_port.h"
#include "hardware.h"
#include "Motor_pwm.h"
#include "fifo.h"
#include "protocol_module.h"
#include "stm32f0xx_rtc.h"
#include "key_led_task.h"
//#include "exp_task.h"
#include "stm32f0xx_usart.h"
#include "store_fifo.h"
/**********************************
*宏定义
***********************************/

/***********************************
* 全局变量
***********************************/
 // 命令接收控制对象
extern CMD_Receive g_CmdReceive; 

//發送數據FIFO
extern FIFO_TYPE send_fifo;
extern uint8_t send_buf[SEND_BUF_LEN];

//保存數據
//extern FIFO_TYPE train_store_fifo;
//extern STORE_HEAD exp_store_head;
//extern TRAIN_STORE_DATA_PAGE train_store_data;
/***********************************
* 局部变量
***********************************/

/***********************************
* 函数声明
***********************************/
void test_task(void);
/***********************************
* 函数定义
***********************************/
//初始化任务
void init_task(void)
{
	//初始化硬件
	init_hardware();	
	Motor_PWM_Init();
	
	//set_led(LED_GREEN);
	//set_led(LED_RED);
	
//	while(1);
	
	//初始化RTC
	//init_rtc();
	
	//初始化数据保存相关
//	fifoInit(&train_store_fifo,(uint8_t*)train_store_data.train_date_buf, sizeof(train_store_data.train_date_buf));
//	init_store_head(&exp_store_head, STORE_HEAD_OFFSET, STORE_DATA_OFFSET,STORE_DATA_PAGE_NUM,STORE_DATA_PAGE_SIZE);

	//初始化通信相关
	fifoInit(&send_fifo,send_buf,SEND_BUF_LEN);
	UARTInit(g_CmdReceive.m_Buf1, BUF1_LENGTH);	
	Init_Receive(&g_CmdReceive);
	
	//init_hardware();	
	
	os_create_task(TaskDataSend, OS_TRUE, SEND_TASK_ID);
	os_create_task(CMD_ProcessTask, OS_TRUE, RECEIVE_TASK_ID);
	//os_create_task(key_led_task, OS_TRUE, KEY_LED_TASK_ID);
	os_create_task(check_selectedMode_ouputPWM,OS_TRUE,TASK_OUTPUT_PWM);
	
	//os_create_task(exp_detect_save_task, OS_TRUE, EXP_DETECT_SAVE_TASK_ID);
	//os_create_task(exp_read_send_task, OS_TRUE, EXP_READ_SEND_TASK_ID);
	//os_create_task(test_task, OS_TRUE, TEST_TASK_ID);
	
	os_pend_task(INIT_TASK_ID);
}

//
void test_task(void)
{
//	static uint32_t tmr;
	
	//tmr = get_rtc();
	
	//檢測呼吸檢測脚狀態

	
	os_delay_ms(TEST_TASK_ID, 50);
}
