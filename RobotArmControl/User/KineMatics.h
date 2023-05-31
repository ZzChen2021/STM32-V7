/*
*********************************************************************************************************
*
*	模块名称 : 运动学计算
*	文件名称 : KineMatics.c
*	版    本 : V1.0
*	说    明 : 头文件
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
*	结 构 体: struct Euler_angle
*	功能说明: 保存欧拉角和空间坐标
*********************************************************************************************************
*/
typedef struct
{
    float32_t alpha;
    float32_t bate;
    float32_t gamma;
    float32_t x;
    float32_t y;
    float32_t z;
}EulerAngle;

void MatrixMul(float32_t Mat_a[MAT_NUM][MAT_NUM],float32_t Mat_b[MAT_NUM][MAT_NUM],float32_t Mat_c[MAT_NUM][MAT_NUM]);
void solveForwardKinematics(float32_t rad[TFK_NUM+1],float32_t Tfk[TFK_NUM][TFK_NUM],uint8_t *p);
uint8_t solveInverseKinematics(float32_t Tfk[TFK_NUM][TFK_NUM], uint8_t *p,float32_t *bestRAD);
EulerAngle convertToEulerAngle(float32_t Tfk[TFK_NUM][TFK_NUM]);
void convertToHomogeneousMatrix(EulerAngle ea,float32_t Tfk[TFK_NUM][TFK_NUM]);
void linspace(float *output, float start, float end, uint8_t num);
void zeros(float *output, uint8_t size);
void compute_qt(float *t, float *qt, float *q0, float *q1, float *qd0, float *qd1);
float compute_path(float *qt);

#endif

