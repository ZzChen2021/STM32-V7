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
*	�� �� ��: convertToEulerAngle
*	����˵��: ����α任����ת��Ϊŷ���ǺͿռ�����
*	��    �Σ�
*               float32_t Tfk[TFK_NUM][TFK_NUM]�������˶�ѧ��ʽ������ĽǶ�ֵ
*	�� �� ֵ:   EulerAngle ea ������ŷ���ǺͿռ�����
*********************************************************************************************************
*/
EulerAngle convertToEulerAngle(float32_t Tfk[TFK_NUM][TFK_NUM])
{
    EulerAngle ea;
    float32_t Sqrt;
    
    /* ���� */
    arm_sqrt_f32((pow(Tfk[3][2], 2) + pow(Tfk[3][3], 2)), &Sqrt);
    arm_atan2_f32(-Tfk[3][1], Sqrt, &ea.bate);
    arm_atan2_f32(Tfk[2][1], Tfk[1][1], &ea.gamma);
    arm_atan2_f32(Tfk[3][2], Tfk[3][3], &ea.alpha);

    /* ����ת�Ƕ� */
    ea.alpha = (ea.alpha / pi) * 180.0;
    ea.bate  = (ea.bate / pi) * 180.0;
    ea.gamma = (ea.gamma / pi) * 180.0;
    ea.x = Tfk[1][4];
    ea.y = Tfk[2][4];
    ea.z = Tfk[3][4];

    return ea;
}

/*
*********************************************************************************************************
*   �� �� ��: solveForwardKinematics
*   ����˵��: ���˶�ѧ����
*   ��    �Σ�
*               float32_t q[6]�����뻡��ֵ
*               float32_t Tfk[TFK_NUM][TFK_NUM]�������˶�ѧ��ʽ������ĽǶ�ֵ
*               unsigned char *p�����ڱ�ʶ�Ƿ�������Tfk
*   �� �� ֵ: ��
*********************************************************************************************************
*/
void solveForwardKinematics(float32_t rad[TFK_NUM + 1], float32_t Tfk[TFK_NUM][TFK_NUM], uint8_t *p)
{
    uint8_t i, j;
    float32_t c1, c2, c4, c5, s1, s2, s4, s5, c23, s23;
    c1 = arm_cos_f32(rad[1]);
    c2 = arm_cos_f32(rad[2]);
    c4 = arm_cos_f32(rad[4]);
    c5 = arm_cos_f32(rad[5]);
    c23 = arm_cos_f32(rad[2] + rad[3]);
    s1 = arm_sin_f32(rad[1]);
    s2 = arm_sin_f32(rad[2]);
    s4 = arm_sin_f32(rad[4]);
    s5 = arm_sin_f32(rad[5]);
    s23 = arm_sin_f32(rad[2] + rad[3]);
    
    /******************���˶�ѧ��ʽ******************/
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
    /******************���˶�ѧ��ʽ******************/
    
    /*��ʾ�����˶�ѧ���������*/
    *p = 1;
}

/*
*********************************************************************************************************
*	�� �� ��: solveInverseKinematics
*	����˵��: ���˶�ѧ����
*	��    �Σ�
*               float Tfk[TFK_NUM][TFK_NUM]�������˶�ѧ��ʽ������ĽǶ�ֵ
*               unsigned char *p�����ڱ�ʶ�Ƿ�������Tfk
*	�� �� ֵ: ��
*********************************************************************************************************
*/
uint8_t solveInverseKinematics(float32_t Tfk[TFK_NUM][TFK_NUM], uint8_t *p, float32_t *bestRAD)
{
    uint8_t flag = 0;
    float32_t Q[6], Q23, Sqart, cq1, sq1, sq4, cq23, sq23;
    float32_t qx, qy; // �����ж����˶�ѧ�����ȷ��
    flag = *p;        // ���ڱ�ʶ�Ƿ�������������
    uint8_t i;

    float t[TV];
    float qt[TV * 5];
    float q0[5] = {0, 0, 0, 0, 0};
    float qd0[5];
    float qd1[5];
    float length;
    float minlength = 100;
    linspace(t, 0, 1, TV);
    zeros(qd0, 5);
    zeros(qd1, 5);
    if (flag == 1)
    {
        /*�������˶�ѧ�����ֽ��*/
        for (; flag < TFK_NUM; flag++)
        {
            /******************���˶�ѧ��ʽ******************/
            if ((flag == 1) || (flag == 3))
            {
                arm_atan2_f32(0, -1, &qx);
                arm_atan2_f32(Tfk[2][4], -Tfk[1][4], &qy);
                Q[1] = qx - qy;
            }
            else
            {
                arm_atan2_f32(0, 1, &qx);
                arm_atan2_f32(Tfk[2][4], -Tfk[1][4], &qy);
                Q[1] = qx - qy;
            }
            cq1 = arm_cos_f32(Q[1]);
            sq1 = arm_sin_f32(Q[1]);
            arm_sqrt_f32((Tfk[2][1] * cq1 - Tfk[1][1] * sq1) * (Tfk[2][1] * cq1 - Tfk[1][1] * sq1) + (Tfk[2][2] * cq1 - Tfk[1][2] * sq1) * (Tfk[2][2] * cq1 - Tfk[1][2] * sq1), &Sqart);
            if ((flag == 1) || (flag == 4))
            {
                arm_atan2_f32(Sqart, Tfk[2][3] * cq1 - Tfk[1][3] * sq1, &Q[4]);
            }
            else
            {
                arm_atan2_f32(-Sqart, Tfk[2][3] * cq1 - Tfk[1][3] * sq1, &Q[4]);
            }
            sq4 = arm_sin_f32(Q[4]);
            arm_atan2_f32((Tfk[2][2] * cq1 - Tfk[1][2] * sq1) / sq4, (Tfk[1][1] * sq1 - Tfk[2][1] * cq1) / sq4, &Q[5]);
            arm_atan2_f32((-Tfk[3][3]) / sq4, (Tfk[1][3] * cq1 + Tfk[2][3] * sq1) / sq4, &Q23);
            cq23 = arm_cos_f32(Q23);
            sq23 = arm_sin_f32(Q23);
            qx = ((275 * cq23) - Tfk[3][4]) / 250;
            qy = (Tfk[1][4] * cq1 + Tfk[2][4] * sq1 - 275 * sq23) / 250;
            arm_atan2_f32(((275 * cq23) - Tfk[3][4]) / 250, (Tfk[1][4] * cq1 + Tfk[2][4] * sq1 - 275 * sq23) / 250, &Q[2]);
            Q[3] = Q23 - Q[2];
            /******************���˶�ѧ��ʽ******************/
            
            /*��֤*/
            if ((qx > 1) || (qx < (-1)) || (qy > 1) || (qy < (-1)))
            {
//                printf("��%d������\r\n", flag);
            }
            else
            {
                /*���ɹؽڿռ�Ĺ켣*/
                compute_qt(t, qt, q0, Q + 1, qd0, qd1);
                
                /*����·������*/
                length = compute_path(qt);
                if (length < minlength)
                {
                    minlength = length;
                }
            }
        }
    }
    if(minlength < 100)
    {
        /*����ȷ*/
        for (i = 1; i < 6; i++)
        {
            if(Q[i] > pi)
            {
                Q[i] -= pi;
            }
            else if(Q[i] < (-pi))
            {
                Q[i] += pi;
            }
            bestRAD[i] = Q[i];
        }
        return minlength;
    }
    return 0;
}

/*
*********************************************************************************************************
*	�� �� ��: convertToHomogeneousMatrix
*	����˵��: ��ŷ���ǺͿռ�����ת��Ϊ��α任����
*	��    �Σ�
*               EulerAngle ea ������ŷ���ǺͿռ�����
*               float Tfk[TFK_NUM][TFK_NUM]�������˶�ѧ��ʽ������ĽǶ�ֵ
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void convertToHomogeneousMatrix(EulerAngle ea, float32_t Tfk[TFK_NUM][TFK_NUM])
{
    ea.alpha = (ea.alpha / 180.0) * pi;
    ea.bate = (ea.bate / 180.0) * pi;
    ea.gamma = (ea.gamma / 180.0) * pi;
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

/*
*********************************************************************************************************
*	�� �� ��: linspace
*	����˵��: ��t(output)������һ��,�޶���(0,1)
*	��    �Σ�
*               float *output �������һ���������
*               float start����һ������
*               float end����һ������
*               float num������ά��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void linspace(float *output, float start, float end, uint8_t num)
{
    float step = (end - start) / (num - 1);
    uint8_t i;
    for (i = 0; i < num; i++)
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
void zeros(float *output, uint8_t size)
{
    uint8_t i;
    for (i = 0; i < size; i++)
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
    uint8_t i, j;
    for (i = 0; i < 5; i++)
    {
        A[i] = 6 * (q1[i] - q0[i]) - 3 * (qd1[i] + qd0[i]);
        B[i] = -15 * (q1[i] - q0[i]) + (8 * qd0[i] + 7 * qd1[i]);
        C[i] = 10 * (q1[i] - q0[i]) - (6 * qd0[i] + 4 * qd1[i]);
        E[i] = qd0[i];
        F[i] = q0[i];
    }
    for (i = 0; i < TV; i++)
    {
        float t_pow[5];
        for (j = 0; j < 5; j++)
        {
            t_pow[j] = powf(t[i], 5 - j);
        }
        for (j = 0; j < 6; j++)
        {
            tt[j] = t_pow[j];
        }
        for (j = 0; j < 5; j++)
        {
            qt[i * 5 + j] = tt[0] * A[j] + tt[1] * B[j] + tt[2] * C[j] + tt[4] * E[j] + tt[5] * F[j];
        }
    }
}

/*
*********************************************************************************************************
*	�� �� ��: compute_path
*	����˵��: ����·�����ȣ�ŷ�Ͼ��룩
*	��    �Σ�
*               float *qt ���켣
*	�� �� ֵ:   float length ��·������
*********************************************************************************************************
*/
float compute_path(float *qt)
{
    uint8_t i, j;
    float sqrt, sum, length;
    length = 0.0;
    
    /* qt��һ��10*5�ľ���ÿ����֮�����ŷ�Ͼ�������� */
    for (i = 0; i < 9; i++)
    {
        sum = 0.0;
        for (j = (i * 5); j < ((i * 5) + 5); j++)
        {
            sum += pow((qt[j + 5] - qt[j]), 2);
        }
        arm_sqrt_f32(sum, &sqrt);
        length += sqrt;
    }
    return length;
}