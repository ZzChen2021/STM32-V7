/*
*********************************************************************************************************
*
*	模块名称 : 主程序模块
*	文件名称 : main.c
*	版    本 : V1.0
*	说    明 : 在RTX5中实现正逆运动学计算
*              实验目的：
*                1. 正逆运动学计算。
*              实验内容：
*                1. K1键按下，正运动学计算。
*                2. K2键按下，逆运动学计算。
*                3. 各个任务实现的功能如下：
*                   AppTaskUserIF任务   : 按键消息处理。
*                   AppTaskLED任务      : LED闪烁。
*                   AppTaskMsgPro任务   : 消息处理，暂未使用。
*                   AppTaskStart任务    : 启动任务，也是最高优先级任务，这里实现按键扫描。
*                   osRtxTimerThread任务: 定时器任务，暂未使用。
*                   AppTaskUart1Rev     : 串口1收发信息
*                   AppTaskKineMatics   : 运动学计算
*              注意事项：
*
*	修改记录 :
*		版本号   日期         作者        说明
*		V1.0    2023-05-12    ZCH         1. CMSIS软包版本 V5.7.0
*                                         2. HAL库版本 V1.9.0
*                                         3. RTX5版本5.5.2
*
*********************************************************************************************************
*/

#include "includes.h"

#define MQ_SIZE 16
uint8_t put_Uart = 5;
uint8_t put_Uart2 = 8;
uint8_t get_Uart;
/*
**********************************************************************************************************
											函数声明
**********************************************************************************************************
*/
static void AppTaskCreate (void);
void AppTaskUserIF(void *argument);
void AppTaskLED(void *argument);
void AppTaskMsgPro(void *argument);
void AppTaskStart(void *argument);
/********************************************by ZCH******************************************************/
void AppTaskUart1Rev(void *argument);
void AppTaskKineMatics(void *argument);
/********************************************by ZCH******************************************************/


/*
**********************************************************************************************************
											 变量
**********************************************************************************************************
*/
/* 任务的属性设置 */
const osThreadAttr_t ThreadStart_Attr = 
{
	/* 未使用 */
//	.cb_mem = &worker_thread_tcb_1,
//	.cb_size = sizeof(worker_thread_tcb_1),
//	.stack_mem = &worker_thread_stk_1[0],
//	.stack_size = sizeof(worker_thread_stk_1),
//	.priority = osPriorityAboveNormal,
//	.tz_module = 0
	
	.name = "osRtxStartThread",
	.attr_bits = osThreadDetached, 
	.priority = osPriorityHigh4,
	.stack_size = 2048,
};

const osThreadAttr_t ThreadMsgPro_Attr = 
{
	.name = "osRtxMsgProThread",
	.attr_bits = osThreadDetached, 
	.priority = osPriorityHigh3,
	.stack_size = 1024,
};

const osThreadAttr_t ThreadLED_Attr = 
{
	.name = "osRtxLEDThread",
	.attr_bits = osThreadDetached, 
	.priority = osPriorityHigh2,
	.stack_size = 512,
};

const osThreadAttr_t ThreadUserIF_Attr = 
{
	.name = "osRtxThreadUserIF",
	.attr_bits = osThreadDetached, 
	.priority = osPriorityHigh1,
	.stack_size = 1024,
};
/********************************************by ZCH******************************************************/
const osThreadAttr_t ThreadUart1Rev_Attr = 
{
	.name = "osRtxThreadUart1Rev", /* 名字 */
	.attr_bits = osThreadDetached, /* 类型 */
	.priority = osPriorityHigh5,   /* 优先级 */
	.stack_size = 1024,            /* 线程堆栈大小 */
};
const osThreadAttr_t ThreadKineMatics_Attr = 
{
	.name = "osRtxThreadKineMatics",
	.attr_bits = osThreadDetached, 
	.priority = osPriorityHigh1,
	.stack_size = 1024,
};
const osMessageQueueAttr_t msgQueue_CAN1_Attr = 
{
	.name = "Message_Queue_CAN1",
};
/********************************************by ZCH******************************************************/

/* 任务句柄 */
osThreadId_t ThreadIdTaskUserIF = NULL;
osThreadId_t ThreadIdTaskMsgPro = NULL;
osThreadId_t ThreadIdTaskLED = NULL;
osThreadId_t ThreadIdStart = NULL;
/********************************************by ZCH******************************************************/
osThreadId_t ThreadIdTaskUart1Rev = NULL;
osThreadId_t ThreadIdKineMatics = NULL;
osMessageQueueId_t msgQueue_ID_CAN1;
/********************************************by ZCH******************************************************/

/*
*********************************************************************************************************
*	函 数 名: main
*	功能说明: 标准c程序入口。
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
int main (void) 
{	
	/* HAL库，MPU，Cache，时钟等系统初始化 */
	System_Init();

	/* 内核开启前关闭HAL的时间基准 */
	HAL_SuspendTick();
	
	/* 内核初始化 */
	osKernelInitialize();                                  

	/* 创建启动任务 */
	ThreadIdStart = osThreadNew(AppTaskStart, NULL, &ThreadStart_Attr);  

	/* 开启多任务 */
	osKernelStart();
	
	while(1);
}

/********************************************by ZCH******************************************************/
/*
*********************************************************************************************************
*	函 数 名: AppTaskUart1Rev
*	功能说明: 串口1的接收客户端消息并返回		
*	形    参: 无
*	返 回 值: 无
*   优 先 级: osPriorityHigh5 (数值越小优先级越低，这个跟uCOS相反)
*********************************************************************************************************
*/
void AppTaskUart1Rev(void *argument)
{
    uint8_t data;
    while(1)
    {
        while(comGetChar(COM1,&data))
        {
            osMessageQueuePut(msgQueue_ID_CAN1,&data,NULL,NULL);
        }
        osDelay(20); /* 任务如果有循环，要延时一段时间（阻塞），让出处理器的占用 */
    }
}

/*
*********************************************************************************************************
*	函 数 名: AppTaskKineMatics
*	功能说明: 运动学计算		
*	形    参: 无
*	返 回 值: 无
*   优 先 级: osPriorityHigh1 (数值越小优先级越低，这个跟uCOS相反)
*********************************************************************************************************
*/
void AppTaskKineMatics(void *argument)
{
//    float Tfk[5][5];
//    uint8_t p;
//    KM(Tfk,&p);
//    con_KM(Tfk,&p);
    osStatus_t os_Status;
    uint8_t ch;
    while(1)
    {
        os_Status = osMessageQueueGet(msgQueue_ID_CAN1,&ch,NULL,osWaitForever);
        if(os_Status == osOK)
        {
            printf("Get the message is %c\n",ch);
        }
    }

}
/********************************************by ZCH******************************************************/

/*
*********************************************************************************************************
*	函 数 名: AppTaskUserIF
*	功能说明: 按键消息处理		
*	形    参: 无
*	返 回 值: 无
*   优 先 级: osPriorityHigh1 (数值越小优先级越低，这个跟uCOS相反)
*********************************************************************************************************
*/
void AppTaskUserIF(void *argument)
{
	uint8_t ucKeyCode;
    float32_t q[6],Tfk[5][5];
    float32_t c1,c2,c4,c5,s1,s2,s4,s5,c23,s23;
    uint8_t p;
    uint8_t i,j;
    while(1)
    {
		ucKeyCode = bsp_GetKey();
		
		if (ucKeyCode != KEY_NONE)
		{
			switch (ucKeyCode)
			{
                /* K1键按下，随机生成弧度值 */
				case KEY_DOWN_K1:
                    for(i=1; i<6; i++)
                    {
                        srand(bsp_GetRunTime());
                        q[i] = 0+1.0*rand()/RAND_MAX *(pi);
                        if(((uint8_t)(q[i] * 100) % 10) < 5)
                        {
                            q[i] = -q[i];
                        }
                        bsp_DelayMS(5);
                    }
                    printf("\n弧度取值：\n");
                    for(i = 1;i < 6;i++)
                    {
                        printf("%11.6f ",q[i]);
                    }
                    break;
                
                /* K2键按下，正运动学计算 */
				case KEY_DOWN_K2:
					printf("正运动学计算\r\n");
                    p = 0;
                    KM(q,Tfk,&p);
					break;
                    
				/* K2键按下，逆运动学计算并验证 */
                case KEY_DOWN_K3:
                    printf("逆运动学计算\r\n");               
                    con_KM(Tfk,&p);
                    break;
                
				default:                     
					break;
			}
		}
		
		osDelay(20);
	}
}

/*
*********************************************************************************************************
*	函 数 名: AppTaskLED
*	功能说明: LED闪烁。
*	形    参: 无
*	返 回 值: 无
*   优 先 级: osPriorityHigh2 
*********************************************************************************************************
*/
void AppTaskLED(void *argument)
{
	const uint16_t usFrequency = 200; /* 延迟周期 */
	uint32_t tick;

	/* 获取当前时间 */
	tick = osKernelGetTickCount(); 
	
    while(1)
    {
		bsp_LedToggle(2);
		/* 相对延迟 */
		tick += usFrequency;                          
		osDelayUntil(tick);
    }
}

/*
*********************************************************************************************************
*	函 数 名: AppTaskMsgPro
*	功能说明: 消息处理，暂未使用。
*	形    参: 无
*	返 回 值: 无
*   优 先 级: osPriorityHigh3  
*********************************************************************************************************
*/
void AppTaskMsgPro(void *argument)
{
	while(1)
	{
		osDelay(10);
	}	
}

/*
*********************************************************************************************************
*	函 数 名: AppTaskStart
*	功能说明: 启动任务，这里用作BSP驱动包处理。
*	形    参: 无
*	返 回 值: 无
*   优 先 级: osPriorityHigh4  
*********************************************************************************************************
*/
void AppTaskStart(void *argument)
{
	const uint16_t usFrequency = 1; /* 延迟周期 */
	uint32_t tick;
	
	/* 初始化外设 */
	HAL_ResumeTick();
	bsp_Init();

	/* 创建任务 */
	AppTaskCreate();

	/* 获取当前时间 */
	tick = osKernelGetTickCount(); 
	
    while(1)
    {
		/* 需要周期性处理的程序，对应裸机工程调用的SysTick_ISR */
		bsp_ProPer1ms();
		
		/* 相对延迟 */
		tick += usFrequency;                          
		osDelayUntil(tick);
    }
}

/*
*********************************************************************************************************
*	函 数 名: AppTaskCreate
*	功能说明: 创建应用任务（普通函数，不是线程）
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void AppTaskCreate (void)
{
	ThreadIdTaskMsgPro = osThreadNew(AppTaskMsgPro, NULL, &ThreadMsgPro_Attr);  
	ThreadIdTaskLED = osThreadNew(AppTaskLED, NULL, &ThreadLED_Attr);  
	ThreadIdTaskUserIF = osThreadNew(AppTaskUserIF, NULL, &ThreadUserIF_Attr);
    /**********************by ZCH***************************/
    ThreadIdTaskUart1Rev = osThreadNew(AppTaskUart1Rev,NULL,&ThreadUart1Rev_Attr);
    ThreadIdKineMatics = osThreadNew(AppTaskKineMatics,NULL,&ThreadKineMatics_Attr);
    msgQueue_ID_CAN1 = osMessageQueueNew(MQ_SIZE,sizeof(put_Uart),&msgQueue_CAN1_Attr);
    /**********************by ZCH***************************/
}
