/*
*********************************************************************************************************
*
*	ģ������ : �˶�ѧ����
*	�ļ����� : KineMatics.c
*	��    �� : V1.0
*	˵    �� : ͷ�ļ�
*
*********************************************************************************************************
*/

#ifndef __KineMatics_H
#define __KineMatics_H

#include <arm_math.h>

#define pi 3.1415926535
#define TFK_NUM 5
#define MAT_NUM 4
#define TV 10

/*
*********************************************************************************************************
*	�� �� ��: struct Euler_angle
*	����˵��: ����ŷ���ǺͿռ�����
*********************************************************************************************************
*/
typedef struct EulerAngle
{
    float32_t alpha;
    float32_t bate;
    float32_t gamma;
    float32_t x;
    float32_t y;
    float32_t z;
}EulerAngle;

void MatrixMul(float32_t Mat_a[MAT_NUM][MAT_NUM],float32_t Mat_b[MAT_NUM][MAT_NUM],float32_t Mat_c[MAT_NUM][MAT_NUM]);
void KM(float32_t q[TFK_NUM+1],float32_t Tfk[TFK_NUM][TFK_NUM],uint8_t *p);
void con_KM(float32_t Tfk[TFK_NUM][TFK_NUM], uint8_t *p,float32_t *bestQ);
EulerAngle getEulerAngle(float32_t Tfk[TFK_NUM][TFK_NUM]);
void getTFK(EulerAngle ea,float32_t Tfk[TFK_NUM][TFK_NUM]);
void linspace(float *output, float start, float end, int num);
void zeros(float *output, int size);
void compute_qt(float *t, float *qt, float *q0, float *q1, float *qd0, float *qd1);
float compute_path(float *qt);

#endif

