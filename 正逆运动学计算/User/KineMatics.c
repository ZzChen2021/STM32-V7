/*
*********************************************************************************************************
*
*	ģ������ : �˶�ѧ����
*	�ļ����� : KineMatics.c
*	��    �� : V1.0
*	˵    �� : ���������˶�ѧ
*
*	�޸ļ�¼ :
*		�汾��   ����         ����        ˵��
*		V1.0    2023-5-12     ZCH         1.ģ�鴴��
*
*********************************************************************************************************
*/

#include <bsp.h>
#include <arm_math.h>
#include <stdlib.h>
#include <math.h>
#include "KineMatics.h"

/*
*********************************************************************************************************
*	�� �� ��: MatrixMul
*	����˵��: �������
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void MatrixMul(float32_t Mat_a[MAT_NUM][MAT_NUM], float32_t Mat_b[MAT_NUM][MAT_NUM], float32_t Mat_c[MAT_NUM][MAT_NUM])
{
    uint8_t i, j, k;
    for (i = 1; i < MAT_NUM; i++)
    {
        for (j = 1; j < MAT_NUM; j++)
        {
            Mat_c[i][j] = 0;
            for (k = 1; k < MAT_NUM; k++)
            {
                Mat_c[i][j] += Mat_a[i][k] * Mat_b[k][j];
            }
        }
    }
}

/*
*********************************************************************************************************
*	�� �� ��: getEulerAngle
*	����˵��: ����α任����ת��Ϊŷ���ǺͿռ�����
*	��    �Σ�
*               float32_t Tfk[TFK_NUM][TFK_NUM]�������˶�ѧ��ʽ������ĽǶ�ֵ
*	�� �� ֵ:   EulerAngle ea ������ŷ���ǺͿռ�����
*********************************************************************************************************
*/
EulerAngle getEulerAngle(float32_t Tfk[TFK_NUM][TFK_NUM])
{
    EulerAngle ea;
    float32_t Sqrt;
    /* ���� */
    arm_sqrt_f32((pow(Tfk[3][2], 2) + pow(Tfk[3][3], 2)), &Sqrt);
    ea.bate = atan2(-Tfk[3][1], Sqrt);
    ea.gamma = atan2(Tfk[2][1], Tfk[1][1]);
    ea.alpha = atan2(Tfk[3][2], Tfk[3][3]);
    /* ����ת�Ƕ� */
    ea.alpha = ((ea.alpha) / pi) * 180.0;
    ea.bate = ((ea.bate) / pi) * 180.0;
    ea.gamma = ((ea.gamma) / pi) * 180.0;
    ea.x = Tfk[1][4];
    ea.y = Tfk[2][4];
    ea.z = Tfk[3][4];
    
    return ea;
}
/*
*********************************************************************************************************
*	�� �� ��: KM
*	����˵��: ���˶�ѧ����
*	��    �Σ�
*               float32_t q[6]�����뻡��ֵ
*               float32_t Tfk[TFK_NUM][TFK_NUM]�������˶�ѧ��ʽ������ĽǶ�ֵ
*               unsigned char *p�����ڱ�ʶ�Ƿ�������Tfk
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void KM(float32_t q[TFK_NUM + 1], float32_t Tfk[TFK_NUM][TFK_NUM], uint8_t *p)
{
    uint8_t i, j;
    float32_t c1, c2, c4, c5, s1, s2, s4, s5, c23, s23;
    float32_t t[TFK_NUM][TFK_NUM]; // ������֤��������֤ʱ�ı�ԭTfk�����е�ֵ
    c1 = arm_cos_f32(q[1]);
    c2 = arm_cos_f32(q[2]);
    c4 = arm_cos_f32(q[4]);
    c5 = arm_cos_f32(q[5]);
    c23 = arm_cos_f32(q[2] + q[3]);
    s1 = arm_sin_f32(q[1]);
    s2 = arm_sin_f32(q[2]);
    s4 = arm_sin_f32(q[4]);
    s5 = arm_sin_f32(q[5]);
    s23 = arm_sin_f32(q[2] + q[3]);
    if ((*p) > 0) // ��֤
    {
        /******************���˶�ѧ��ʽ******************/
        t[1][1] = c5 * (s1 * s4 + c23 * c1 * c4) - s23 * c1 * s5;
        t[2][1] = -c5 * (c1 * s4 - c23 * c4 * s1) - s23 * s1 * s5;
        t[3][1] = -c23 * s5 - s23 * c4 * c5;
        t[4][1] = 0;
        t[1][2] = -s5 * (s1 * s4 + c23 * c1 * c4) - s23 * c1 * c5;
        t[2][2] = s5 * (c1 * s4 - c23 * c4 * s1) - s23 * c5 * s1;
        t[3][2] = s23 * c4 * s5 - c23 * c5;
        t[4][2] = 0;
        t[1][3] = c23 * c1 * s4 - c4 * s1;
        t[2][3] = c1 * c4 + c23 * s1 * s4;
        t[3][3] = -s23 * s4;
        t[4][3] = 0;
        t[1][4] = 25 * c1 * (11 * s23 + 10 * c2);
        t[2][4] = 25 * s1 * (11 * s23 + 10 * c2);
        t[3][4] = 275 * c23 - 250 * s2;
        t[4][4] = 1;
        /******************���˶�ѧ��ʽ******************/
    }
    else // ���
    {
        Tfk[1][1] = c5 * (s1 * s4 + c23 * c1 * c4) - s23 * c1 * s5;
        Tfk[2][1] = -c5 * (c1 * s4 - c23 * c4 * s1) - s23 * s1 * s5;
        Tfk[3][1] = -c23 * s5 - s23 * c4 * c5;
        Tfk[4][1] = 0;
        Tfk[1][2] = -s5 * (s1 * s4 + c23 * c1 * c4) - s23 * c1 * c5;
        Tfk[2][2] = s5 * (c1 * s4 - c23 * c4 * s1) - s23 * c5 * s1;
        Tfk[3][2] = s23 * c4 * s5 - c23 * c5;
        Tfk[4][2] = 0;
        Tfk[1][3] = c23 * c1 * s4 - c4 * s1;
        Tfk[2][3] = c1 * c4 + c23 * s1 * s4;
        Tfk[3][3] = -s23 * s4;
        Tfk[4][3] = 0;
        Tfk[1][4] = 25 * c1 * (11 * s23 + 10 * c2);
        Tfk[2][4] = 25 * s1 * (11 * s23 + 10 * c2);
        Tfk[3][4] = 275 * c23 - 250 * s2;
        Tfk[4][4] = 1;
    }
    for (i = 1; i < TFK_NUM; i++)
    {
        for (j = 1; j < TFK_NUM; j++)
        {
            if ((*p) > 0)
            {
                printf("%11.6f ", t[i][j]);
            }
            else
            {
                printf("%11.6f ", Tfk[i][j]);
            }
        }
        printf("\r\n");
    }
    *p = 1;
}

/*
*********************************************************************************************************
*	�� �� ��: con_KM
*	����˵��: ���˶�ѧ����
*	��    �Σ�
*               float Tfk[TFK_NUM][TFK_NUM]�������˶�ѧ��ʽ������ĽǶ�ֵ
*               unsigned char *p�����ڱ�ʶ�Ƿ�������Tfk
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void con_KM(float32_t Tfk[TFK_NUM][TFK_NUM], uint8_t *p,float32_t *bestQ)
{
    uint8_t flag;
    float32_t Q[6], Q23, Sqart, cq1, sq1, sq4, cq23, sq23;
    float32_t qx, qy; // �����ж����˶�ѧ�����ȷ��
    flag = *p;        // ���ڱ�ʶ�Ƿ�������������
    uint8_t i, j;

//	float t[TV];
//	float qt[TV * 5];
//	float q0[5] = {0, 0, 0, 0, 0};
//	float qd0[5];
//	float qd1[5];
//    float length = 100000;
//    linspace(t, 0, 1, TV);
//    zeros(qd0, 5);
//    zeros(qd1, 5);
    if (flag == 1)
    {
        for (; flag < TFK_NUM; flag++) // �������˶�ѧ�����ֽ��
        {
            /******************���˶�ѧ��ʽ******************/
            if ((flag == 1) || (flag == 3))
            {
                Q[1] = atan2(0, -1) - atan2(Tfk[2][4], -Tfk[1][4]);
            }
            else
            {
                Q[1] = atan2(0, 1) - atan2(Tfk[2][4], -Tfk[1][4]);
            }
            cq1 = arm_cos_f32(Q[1]);
            sq1 = arm_sin_f32(Q[1]);
            arm_sqrt_f32((Tfk[2][1] * cq1 - Tfk[1][1] * sq1) * (Tfk[2][1] * cq1 - Tfk[1][1] * sq1) + (Tfk[2][2] * cq1 - Tfk[1][2] * sq1) * (Tfk[2][2] * cq1 - Tfk[1][2] * sq1), &Sqart);
            if ((flag == 1) || (flag == 4))
            {
                Q[4] = atan2(Sqart, Tfk[2][3] * cq1 - Tfk[1][3] * sq1);
            }
            else
            {
                Q[4] = atan2(-Sqart, Tfk[2][3] * cq1 - Tfk[1][3] * sq1);
            }
            sq4 = arm_sin_f32(Q[4]);
            Q[5] = atan2((Tfk[2][2] * cq1 - Tfk[1][2] * sq1) / sq4, (Tfk[1][1] * sq1 - Tfk[2][1] * cq1) / sq4);
            Q23 = atan2((-Tfk[3][3]) / sq4, (Tfk[1][3] * cq1 + Tfk[2][3] * sq1) / sq4);
            cq23 = arm_cos_f32(Q23);
            sq23 = arm_sin_f32(Q23);
            qx = ((275 * cq23) - Tfk[3][4]) / 250;
            qy = (Tfk[1][4] * cq1 + Tfk[2][4] * sq1 - 275 * sq23) / 250;
            Q[2] = atan2(((275 * cq23) - Tfk[3][4]) / 250, (Tfk[1][4] * cq1 + Tfk[2][4] * sq1 - 275 * sq23) / 250);
            Q[3] = Q23 - Q[2];
            /******************���˶�ѧ��ʽ******************/

            if ((qx > 1) || (qx < (-1)) || (qy > 1) || (qy < (-1)))
            {
                printf("��%d������\r\n", flag);
            }
            else
            {
                printf("��%d���:\n%11.6f %11.6f %11.6f %11.6f %11.6f\r\n", flag, Q[1], Q[2], Q[3], Q[4], Q[5]);
                //printf("��֤��%d���:\r\n", flag);
                //KM(Q, Tfk, p);
                //compute_qt(t, qt, q0, Q + 1, qd0, qd1);
//                if((compute_path(qt)) < length)
//                {
//                    length = compute_path(qt);
//                    for(i = 1; i < 6; i++)
//                    {
//                        bestQ[i] = Q[i];
//                    }
//                }
            }
        }
        //printf("���Ž⣺\n%11.6f %11.6f %11.6f %11.6f %11.6f\r\n",bestQ[1], bestQ[2], bestQ[3], bestQ[4], bestQ[5]);
        //printf("���Ž��·������Ϊ��\n%11.6f \n",length);
    }
}

/*
*********************************************************************************************************
*	�� �� ��: getTFK
*	����˵��: ��ŷ���ǺͿռ�����ת��Ϊ��α任����
*	��    �Σ�
*               EulerAngle ea ������ŷ���ǺͿռ�����
*               float Tfk[TFK_NUM][TFK_NUM]�������˶�ѧ��ʽ������ĽǶ�ֵ
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void getTFK(EulerAngle ea, float32_t Tfk[TFK_NUM][TFK_NUM])
{
    ea.alpha = ((ea.alpha) / 180.0) * pi;
    ea.bate = ((ea.bate) / 180.0) * pi;
    ea.gamma = ((ea.gamma) / 180.0) * pi;
    float32_t Rox[MAT_NUM][MAT_NUM], Roy[MAT_NUM][MAT_NUM], Roz[MAT_NUM][MAT_NUM];
    float32_t mat[MAT_NUM][MAT_NUM], Mat[MAT_NUM][MAT_NUM];
    Rox[1][1] = 1;
    Rox[1][2] = 0;
    Rox[1][3] = 0;
    Rox[2][1] = 0;
    Rox[2][2] = arm_cos_f32(ea.alpha);
    Rox[2][3] = -arm_sin_f32(ea.alpha);
    Rox[3][1] = 0;
    Rox[3][2] = arm_sin_f32(ea.alpha);
    Rox[3][3] = arm_cos_f32(ea.alpha);

    Roy[1][1] = arm_cos_f32(ea.bate);
    Roy[1][2] = 0;
    Roy[1][3] = arm_sin_f32(ea.bate);
    Roy[2][1] = 0;
    Roy[2][2] = 1;
    Roy[2][3] = 0;
    Roy[3][1] = -arm_sin_f32(ea.bate);
    Roy[3][2] = 0;
    Roy[3][3] = arm_cos_f32(ea.bate);
    
    Roz[1][1] = arm_cos_f32(ea.gamma);
    Roz[1][2] = -arm_sin_f32(ea.gamma);
    Roz[1][3] = 0;
    Roz[2][1] = arm_sin_f32(ea.gamma);
    Roz[2][2] = arm_cos_f32(ea.gamma);
    Roz[2][3] = 0;
    Roz[3][1] = 0;
    Roz[3][2] = 0;
    Roz[3][3] = 1;
    
    MatrixMul(Roz, Roy, mat);
    MatrixMul(mat, Rox, Mat);

    Tfk[1][1] = Mat[1][1];
    Tfk[2][1] = Mat[2][1];
    Tfk[3][1] = Mat[3][1];
    Tfk[4][1] = 0;
    Tfk[1][2] = Mat[1][2];
    Tfk[2][2] = Mat[2][2];
    Tfk[3][2] = Mat[3][2];
    Tfk[4][2] = 0;
    Tfk[1][3] = Mat[1][3];
    Tfk[2][3] = Mat[2][3];
    Tfk[3][3] = Mat[3][3];
    Tfk[4][3] = 0;
    Tfk[1][4] = ea.x;
    Tfk[2][4] = ea.y;
    Tfk[3][4] = ea.z;
    Tfk[4][4] = 1;
}

void linspace(float *output, float start, float end, int num)
{
    float step = (end - start) / (num - 1);
    for (int i = 0; i < num; i++)
    {
        output[i] = start + i * step;
    }
}

/*
*********************************************************************************************************
*	�� �� ��: zeros
*	����˵��: ����������
*********************************************************************************************************
*/
void zeros(float *output, int size)
{
    for (int i = 0; i < size; i++)
    {
        output[i] = 0.0f;
    }
}

/*
*********************************************************************************************************
*	�� �� ��: compute_qt
*	����˵��: ������ζ���ʽ���ɹؽڿռ�Ĺ켣
*	��    �Σ�
*               float *qt ���켣
*	�� �� ֵ:��
*********************************************************************************************************
*/
void compute_qt(float *t, float *qt, float *q0, float *q1, float *qd0, float *qd1)
{
    float A[5], B[5], C[5], E[5], F[5];
    float tt[6];
    for (int i = 0; i < 5; i++)
    {
        A[i] = 6 * (q1[i] - q0[i]) - 3 * (qd1[i] + qd0[i]);
        B[i] = -15 * (q1[i] - q0[i]) + (8 * qd0[i] + 7 * qd1[i]);
        C[i] = 10 * (q1[i] - q0[i]) - (6 * qd0[i] + 4 * qd1[i]);
        E[i] = qd0[i];
        F[i] = q0[i];
    }

    for (int i = 0; i < TV; i++)
    {
        float t_pow[5];
        for (int j = 0; j < 5; j++)
        {
            t_pow[j] = powf(t[i], 5 - j);
        }

        for (int j = 0; j < 6; j++)
        {
            tt[j] = t_pow[j];
        }

        for (int j = 0; j < 5; j++)
        {
            qt[i * 5 + j] = tt[0] * A[j] + tt[1] * B[j] + tt[2] * C[j] + tt[4] * E[j] + tt[5] * F[j];
        }
    }
}

/*
*********************************************************************************************************
*	�� �� ��: compute_path
*	����˵��: ����·������
*	��    �Σ�
*               float *qt ���켣
*	�� �� ֵ:   float length ��·������
*********************************************************************************************************
*/
float compute_path(float *qt)
{
    uint8_t i,j;
    float sqrt,sum,length;
    length = 0.0;
    for(i = 0;i < 9;i++)
    {
        sum = 0.0;
        for(j = (i*5);j < ((i*5) + 5);j++)
        {
            sum += pow((qt[j+5] - qt[j]),2);
        }
        arm_sqrt_f32(sum,&sqrt);
        length += sqrt;
    }
    return length;
}