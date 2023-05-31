/*
*********************************************************************************************************
*
*	ģ������ : ������ģ��
*	�ļ����� : main.c
*	��    �� : V1.0
*	˵    �� : ��RTX5��ʵ�������˶�ѧ����
*              ʵ��Ŀ�ģ�
*                1. �����˶�ѧ���㡣
*              ʵ�����ݣ�
*                1. K1�����£����˶�ѧ���㡣
*                2. K2�����£����˶�ѧ���㡣
*                3. ��������ʵ�ֵĹ������£�
*                   AppTaskUserIF����   : ������Ϣ����
*                   AppTaskLED����      : LED��˸��
*                   AppTaskMsgPro����   : ��Ϣ������δʹ�á�
*                   AppTaskStart����    : ��������Ҳ��������ȼ���������ʵ�ְ���ɨ�衣
*                   osRtxTimerThread����: ��ʱ��������δʹ�á�
*                   AppTaskUart1Rev     : ����1�շ���Ϣ
*                   AppTaskKineMatics   : �˶�ѧ����
*              ע�����
*
*	�޸ļ�¼ :
*		�汾��   ����         ����        ˵��
*		V1.0    2023-05-12    ZCH         1. CMSIS����汾 V5.7.0
*                                         2. HAL��汾 V1.9.0
*                                         3. RTX5�汾5.5.2
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
											��������
**********************************************************************************************************
*/
static void AppTaskCreate(void);
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
											 ����
**********************************************************************************************************
*/
/* ������������� */
const osThreadAttr_t ThreadStart_Attr =
	{
		// /* δʹ�� */
		// .cb_mem = &worker_thread_tcb_1,
		// .cb_size = sizeof(worker_thread_tcb_1),
		// .stack_mem = &worker_thread_stk_1[0],
		// .stack_size = sizeof(worker_thread_stk_1),
		// .priority = osPriorityAboveNormal,
		// .tz_module = 0

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
		.stack_size = 2048,
};
/********************************************by ZCH******************************************************/
const osThreadAttr_t ThreadUart1Rev_Attr =
	{
		.name = "osRtxThreadUart1Rev", /* ���� */
		.attr_bits = osThreadDetached, /* ���� */
		.priority = osPriorityHigh5,   /* ���ȼ� */
		.stack_size = 1024,			   /* �̶߳�ջ��С */
};
const osThreadAttr_t ThreadKineMatics_Attr =
	{
		.name = "osRtxThreadKineMatics",
		.attr_bits = osThreadDetached,
		.priority = osPriorityHigh1,
		.stack_size = 4096,
};
const osMessageQueueAttr_t msgQueue_CAN1_Attr =
	{
		.name = "Message_Queue_CAN1",
};
/********************************************by ZCH******************************************************/

/* ������ */
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
*	�� �� ��: main
*	����˵��: ��׼c������ڡ�
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
int main(void)
{
	/* HAL�⣬MPU��Cache��ʱ�ӵ�ϵͳ��ʼ�� */
	System_Init();

	/* �ں˿���ǰ�ر�HAL��ʱ���׼ */
	HAL_SuspendTick();

	/* �ں˳�ʼ�� */
	osKernelInitialize();

	/* ������������ */
	ThreadIdStart = osThreadNew(AppTaskStart, NULL, &ThreadStart_Attr);

	/* ���������� */
	osKernelStart();

	while (1)
		;
}

/********************************************by ZCH******************************************************/
/*
*********************************************************************************************************
*	�� �� ��: AppTaskUart1Rev
*	����˵��: ����1�Ľ��տͻ�����Ϣ������
*	��    ��: ��
*	�� �� ֵ: ��
*   �� �� ��: osPriorityHigh5 (��ֵԽС���ȼ�Խ�ͣ������uCOS�෴)
*********************************************************************************************************
*/
void AppTaskUart1Rev(void *argument)
{
	uint8_t data;
	while (1)
	{
		while (comGetChar(COM1, &data))
		{
			osMessageQueuePut(msgQueue_ID_CAN1, &data, NULL, NULL);
		}
		osDelay(20); /* ����г������е��̣߳�Ҫ��ʱһ��ʱ�䣨���������ó���������ռ�� */
	}
}

/*
*********************************************************************************************************
*	�� �� ��: AppTaskKineMatics
*	����˵��: �˶�ѧ����
*	��    ��: ��
*	�� �� ֵ: ��
*   �� �� ��: osPriorityHigh1 (��ֵԽС���ȼ�Խ�ͣ������uCOS�෴)
*********************************************************************************************************
*/
void AppTaskKineMatics(void *argument)
{
	float32_t RAD[TFK_NUM + 1], Tfk[5][5];
	uint8_t p;
	uint16_t i, j, k;
	uint16_t count = 0; /*ͳ����ȷ�����*/
	EulerAngle eulerangle;
	float32_t getRAD[TFK_NUM * 100];
	uint32_t start, finish;

	/*��ȡ500���������ֵ*/
	for (i = 0; i < (TFK_NUM * 100); i++)
	{
		srand(bsp_GetRunTime());
		getRAD[i] = 0 + 1.0 * rand() / RAND_MAX * (pi);
		if (((uint8_t)(getRAD[i] * 100) % 10) < 5)
		{
			getRAD[i] = -getRAD[i];
		}
		bsp_DelayMS(5);
	}
	j = 0;
	start = bsp_GetRunTime();
	for (k = 0; k < 100; k++)
	{
		j++;
		if (k == 99)
		{
			k = 0;
		}
		/*���λ�ȡ5������ֵ*/
		for (i = 1; i < (TFK_NUM + 1); i++)
		{
			RAD[i] = getRAD[(k * TFK_NUM) + (i - 1)];
		}
		/*���˶�ѧ����*/
		solveForwardKinematics(RAD, Tfk, &p);

		/*���������ŷ���ǺͿռ�����*/
		eulerangle = convertToEulerAngle(Tfk);

		/*��ŷ���ǺͿռ������������*/
		convertToHomogeneousMatrix(eulerangle, Tfk);

		/*���˶�ѧ�������֤����������ֵ������·�����ȣ�������󷵻�0*/
		if (solveInverseKinematics(Tfk, &p, RAD) != 0)
		{
			/*����ȷ*/
			count++;
            
            /*�������*/
			if (count == 10000)
			{
				k = 100;
			}
		}
	}
	finish = bsp_GetRunTime();
	printf("\r\n���%d�μ��㲢��֤����ȷ�����Ϊ��%d\n��ʱ:%d ms\n", j, count, finish - start);
}
/********************************************by ZCH******************************************************/

/*
*********************************************************************************************************
*	�� �� ��: AppTaskUserIF
*	����˵��: ������Ϣ����
*	��    ��: ��
*	�� �� ֵ: ��
*   �� �� ��: osPriorityHigh1 (��ֵԽС���ȼ�Խ�ͣ������uCOS�෴)
*********************************************************************************************************
*/
void AppTaskUserIF(void *argument)
{
	uint8_t ucKeyCode;
	while (1)
	{
		ucKeyCode = bsp_GetKey();
		if (ucKeyCode != KEY_NONE)
		{
			switch (ucKeyCode)
			{
                
			/* K1�����£�ͬ���߳�AppTaskKineMatics */
			case KEY_DOWN_K1:
				osThreadFlagsSet(ThreadIdKineMatics, 0x01U);
				break;

			/* K2������ */
			case KEY_DOWN_K2:
				break;

			/* K3������ */
			case KEY_DOWN_K3:
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
*	�� �� ��: AppTaskLED
*	����˵��: LED��˸��
*	��    ��: ��
*	�� �� ֵ: ��
*   �� �� ��: osPriorityHigh2
*********************************************************************************************************
*/
void AppTaskLED(void *argument)
{
	const uint16_t usFrequency = 200; /* �ӳ����� */
	uint32_t tick;
	osStatus_t os_Status;

	/* ��ȡ��ǰʱ�� */
	tick = osKernelGetTickCount();

	while (1)
	{
		bsp_LedToggle(2);
		/* ����ӳ� */
		tick += usFrequency;
		osDelayUntil(tick);
	}
}

/*
*********************************************************************************************************
*	�� �� ��: AppTaskMsgPro
*	����˵��: ��Ϣ������δʹ�á�
*	��    ��: ��
*	�� �� ֵ: ��
*   �� �� ��: osPriorityHigh3
*********************************************************************************************************
*/
void AppTaskMsgPro(void *argument)
{
	osStatus_t os_Status;
	uint8_t ch;
	while (1)
	{
		os_Status = osMessageQueueGet(msgQueue_ID_CAN1, &ch, NULL, osWaitForever);
		if (os_Status == osOK)
		{
			printf("Get the message is %c\n", ch);
		}
		osDelay(10);
	}
}

/*
*********************************************************************************************************
*	�� �� ��: AppTaskStart
*	����˵��: ����������������BSP����������
*	��    ��: ��
*	�� �� ֵ: ��
*   �� �� ��: osPriorityHigh4
*********************************************************************************************************
*/
void AppTaskStart(void *argument)
{
	const uint16_t usFrequency = 1; /* �ӳ����� */
	uint32_t tick;

	/* ��ʼ������ */
	HAL_ResumeTick();
	bsp_Init();

	/* �������� */
	AppTaskCreate();

	/* ��ȡ��ǰʱ�� */
	tick = osKernelGetTickCount();

	while (1)
	{
		/* ��Ҫ�����Դ���ĳ��򣬶�Ӧ������̵��õ�SysTick_ISR */
		bsp_ProPer1ms();

		/* ����ӳ� */
		tick += usFrequency;
		osDelayUntil(tick);
	}
}

/*
*********************************************************************************************************
*	�� �� ��: AppTaskCreate
*	����˵��: ����Ӧ��������ͨ�����������̣߳�
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void AppTaskCreate(void)
{
	// ThreadIdTaskMsgPro = osThreadNew(AppTaskMsgPro, NULL, &ThreadMsgPro_Attr);
	ThreadIdTaskLED = osThreadNew(AppTaskLED, NULL, &ThreadLED_Attr);
	//ThreadIdTaskUserIF = osThreadNew(AppTaskUserIF, NULL, &ThreadUserIF_Attr);
	/**********************by ZCH***************************/
	// ThreadIdTaskUart1Rev = osThreadNew(AppTaskUart1Rev, NULL, &ThreadUart1Rev_Attr);
	ThreadIdKineMatics = osThreadNew(AppTaskKineMatics, NULL, &ThreadKineMatics_Attr);
	// msgQueue_ID_CAN1 = osMessageQueueNew(MQ_SIZE, sizeof(put_Uart), &msgQueue_CAN1_Attr);
	/**********************by ZCH***************************/
}

#include "rtx_os.h"
// OS Error Callback function
uint32_t osRtxErrorNotify(uint32_t code, void *object_id)
{
	(void)object_id;

	switch (code)
	{
	case osRtxErrorStackOverflow:
		printf(" Stack overflow detected for thread (thread_id=object_id)");
		break;
	case osRtxErrorISRQueueOverflow:
		printf(" ISR Queue overflow detected when inserting object (object_id)");
		break;
	case osRtxErrorTimerQueueOverflow:
		printf(" User Timer Callback Queue overflow detected for timer (timer_id=object_id)");
		break;
	case osRtxErrorClibSpace:
		printf(" Standard C/C++ library libspace not available: increase OS_THREAD_LIBSPACE_NUM");
		break;
	case osRtxErrorClibMutex:
		printf(" Standard C/C++ library mutex initialization failed");
		break;
	default:
		printf(" Reserved");
		break;
	}
	for (;;)
	{
	}
	// return 0U;
}
