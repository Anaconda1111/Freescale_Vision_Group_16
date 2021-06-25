//
// Created by xcs on 2021-03-03.
//

#ifndef RT1064_CODE_STEER_H_
#define RT1064_CODE_STEER_H_

#include "PID.h"
#include "headfile.h"

#define SteerMAX 3600  //��ʱ������������
#define SteerMIN 2850  //��ʱ����������ұ�
#define Steer_IMAX 100
#define MiddleSteer_PWM 3200
#define SteerOutGarage_PWM 3000           //3810 //Ŀ������45��



void Steer_PIDStruct_Init(PID_Struct Steer_PID, Filter_Struct Steer_Filter);

void SteerCtrl(PID_Struct Steer_PID, Filter_Struct Steer_Filter);

#endif // RT1064_CODE_STEER_H_
