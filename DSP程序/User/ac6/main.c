/*
*********************************************************************************************************
*
*	模块名称 : 主程序模块
*	文件名称 : main.c
*	版    本 : V1.0
*	说    明 : ARM官方DSP库移植
*              实验目的：
*                1. 学习官方DSP库的移植。
*              实验内容：
*                1. 按下按键K1, 串口打印函数arm_abs_f32的输出结果。
*                2. 按下按键K2, 串口打印函数arm_abs_q31的输出结果。
*                3. 按下按键K3, 串口打印函数arm_abs_q15的输出结果。
*              注意事项：
*                1. 本实验推荐使用串口软件SecureCRT查看打印信息，波特率115200，数据位8，奇偶校验位无，停止位1。
*                2. 务必将编辑器的缩进参数和TAB设置为4来阅读本文件，要不代码显示不整齐。
*
*	修改记录 :
*		版本号   日期         作者        说明
*		V1.0    2019-07-31   Eric2013     1. CMSIS软包版本 V5.6.0
*                                         2. HAL库版本 V1.3.0
*
*	Copyright (C), 2018-2030, 安富莱电子 www.armfly.com
*
*********************************************************************************************************
*/	
#include "bsp.h"			/* 底层硬件驱动 */
#include "arm_math.h"
#include "math.h"


/* 定义例程名和例程发布日期 */
#define EXAMPLE_NAME	"V7-ARM的DSP移植模板（源码方式）"
#define EXAMPLE_DATE	"2019-07-31"
#define DEMO_VER		"1.0"

//static void PrintfLogo(void);
void PrintfHelp(float32_t q1,float32_t q2,float32_t q3,float32_t q4,float32_t q5);
void PrintfTkf(float32_t Tfk[5][5]);
/*
*********************************************************************************************************
*	函 数 名: main
*	功能说明: c程序入口
*	形    参: 无
*	返 回 值: 错误代码(无需处理)
*********************************************************************************************************
*/

/*****************耗时*********************
0.22583ms

0.22792ms

0.67804ms

0.80334ms
0.92859ms
1.05265ms
******************************************/
int main(void)
{
	uint8_t ucKeyCode;		/* 按键代码 */
	float32_t pSrc;
    float32_t pDst;
    uint8_t flag = 0;
	q31_t pSrc1;
	q31_t pDst1;
	q15_t pSrc2;
	q15_t pDst2;
    float32_t q1,q2,q3,q4,q5,c1,c2,c4,c5,s1,s2,s4,s5,c23,s23;
    float32_t Q1,Q2,Q3,Q4,Q5,Q23,Sqart,cq1,sq1,sq4,cq23,sq23;
    float32_t Tfk[5][5];
    q1 = 2.0323;
    q2 = 1.2241;
    q3 = -1.1492;
    q4 = 2.8288;
    q5 = -2.9252;
    c1 = arm_cos_f32(q1);
    c2 = arm_cos_f32(q2);
    c4 = arm_cos_f32(q4);
    c5 = arm_cos_f32(q5);
    c23 = arm_cos_f32(q2+q3);
    s1 = arm_sin_f32(q1);
    s2 = arm_sin_f32(q2);
    s4 = arm_sin_f32(q4);
    s5 = arm_sin_f32(q5);
    s23 = arm_sin_f32(q2+q3);
    
	bsp_Init();		/* 硬件初始化 */
	
	//PrintfLogo();	/* 打印例程名称和版本等信息 */
	PrintfHelp(q1,q2,q3,q4,q5);	/* 打印操作提示 */

	bsp_StartAutoTimer(0, 100);	/* 启动1个100ms的自动重装的定时器 */
	
	/* 主程序大循环 */
	while (1)
	{
		/* CPU空闲时执行的函数，在 bsp.c */
		bsp_Idle();		
		
		/* 判断定时器超时时间 */
		if (bsp_CheckTimer(0))	
		{
			/* 每隔100ms 进来一次 */
			/* 翻转LED2的状态 */
			bsp_LedToggle(2);	
		}

		/* 处理按键事件 */
		ucKeyCode = bsp_GetKey();
		if (ucKeyCode > 0)
		{
			/* 有键按下 */
			switch (ucKeyCode)
			{
               case KEY_DOWN_K1:
                    Tfk[1][1] = c5*(s1*s4 + c23*c1*c4) - s23*c1*s5;
                    Tfk[2][1] = -c5*(c1*s4 - c23*c4*s1) - s23*s1*s5;
                    Tfk[3][1] = -c23*s5 - s23*c4*c5;
                    Tfk[4][1] = 0;
                    Tfk[1][2] = -s5*(s1*s4 + c23*c1*c4) - s23*c1*c5;
                    Tfk[2][2] = s5*(c1*s4 - c23*c4*s1) - s23*c5*s1;
                    Tfk[3][2] = s23*c4*s5 - c23*c5;
                    Tfk[4][2] = 0;
                    Tfk[1][3] = c23*c1*s4 - c4*s1;
                    Tfk[2][3] = c1*c4 + c23*s1*s4;
                    Tfk[3][3] = -s23*s4;
                    Tfk[4][3] = 0;
                    Tfk[1][4] = 25*c1*(11*s23 + 10*c2);
                    Tfk[2][4] = 25*s1*(11*s23 + 10*c2);
                    Tfk[3][4] = 275*c23 - 250*s2;
                    Tfk[4][4] = 1;
                    PrintfTkf(Tfk);
                    flag = 1;
					break;
					
				case KEY_DOWN_K2:
                    if(flag == 1)
                    {
                        for(;flag<5;flag++)
                        {
                            printf("\nResult%d: ",flag);
                            if((flag == 1)||(flag == 3)){
                                Q1 = atan2(0,-1) - atan2(Tfk[2][4],-Tfk[1][4]);
                            }
                            else{
                                Q1 = atan2(0,1) - atan2(Tfk[2][4],-Tfk[1][4]);
                            }
                            cq1 = arm_cos_f32(Q1);
                            sq1 = arm_sin_f32(Q1);
                            arm_sqrt_f32((Tfk[2][1]*cq1 - Tfk[1][1]*sq1)*(Tfk[2][1]*cq1 - Tfk[1][1]*sq1) + (Tfk[2][2]*cq1 - Tfk[1][2]*sq1)*(Tfk[2][2]*cq1 - Tfk[1][2]*sq1),&Sqart);
                            if((flag == 1)||(flag == 4))
                            {
                                Q4 = atan2(Sqart,Tfk[2][3]*cq1 - Tfk[1][3]*sq1);
                            }
                            else
                            {
                                Q4 = atan2((-Sqart),Tfk[2][3]*cq1 - Tfk[1][3]*sq1);
                            }
                            sq4 = arm_sin_f32(Q4);
                            Q5 = atan2((Tfk[2][2]*cq1 - Tfk[1][2]*sq1)/sq4,(Tfk[1][1]*sq1 - Tfk[2][1]*cq1)/sq4);
                            Q23 = atan2((-Tfk[3][3])/sq4,(Tfk[1][3]*cq1 + Tfk[2][3]*sq1)/sq4);
                            cq23 = arm_cos_f32(Q23);
                            sq23 = arm_sin_f32(Q23);
                            Q2 = atan2(((275*cq23) - Tfk[3][4])/250,(Tfk[1][4]*cq1 + Tfk[2][4]*sq1 - 275*sq23)/250);
                            Q3 = Q23 -Q2;
                            printf("%f  %f  %f  %f  %f\r\n",Q1,Q2,Q3,Q4,Q5);
                        }
                        flag = 1;
                    }
					break;

				case KEY_DOWN_K3:		

					break;
				
				default:
					break;
			}
		}
	}
}

/*
*********************************************************************************************************
*	函 数 名: PrintfHelp
*	功能说明: 打印操作提示
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void PrintfHelp(float32_t q1,float32_t q2,float32_t q3,float32_t q4,float32_t q5)
{
    printf("\r\n");
	printf("q1 = %f\r\n",q1);
    printf("q2 = %f\r\n",q2);
    printf("q3 = %f\r\n",q3);
    printf("q4 = %f\r\n",q4);
    printf("q5 = %f\r\n",q5);
}
void PrintfTkf(float32_t Tfk[5][5])
{
    uint8_t i,j;
    for(i = 1;i < 5;i++)
    {
        for(j = 1;j < 5;j++)
        {
            printf("Tfk[%d][%d] = %f\r\n",i,j,Tfk[i][j]);
        }
    }
}