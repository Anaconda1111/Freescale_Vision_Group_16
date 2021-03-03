//
// Created by xcs on 2021-03-03.
//

#ifndef RT1064_CODE_STEER_H_
#define RT1064_CODE_STEER_H_
#include "PID.h"
#define SteerMAX 0
#define SteerMIN 0
#define Steer_IMAX 0
#define MiddleSteer_PWM 0
void Steer_PIDStruct_Init(PID_Struct Steer_PID, Filter_Struct Steer_Filter);
#endif // RT1064_CODE_STEER_H_
