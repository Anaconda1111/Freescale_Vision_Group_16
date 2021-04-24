//
// Created by xcs on 2021-03-03.
//

#ifndef RT1064_CODE_STEER_H_
#define RT1064_CODE_STEER_H_

#include "PID.h"

#define SteerMAX 3840  //此时舵机打向最左边
#define SteerMIN 3010  //此时舵机打向最右边
#define Steer_IMAX 100
#define MiddleSteer_PWM 3430
#define SteerOutGarage_PWM 3800           //3810 //目测向左45度


void Steer_PIDStruct_Init(PID_Struct Steer_PID, Filter_Struct Steer_Filter);

void SteerCtrl(PID_Struct Steer_PID, Filter_Struct Steer_Filter);

#endif // RT1064_CODE_STEER_H_
