//
// Created by xcs on 2021-03-05.
//

#ifndef RT1064_CODE_MOTOR_H_
#define RT1064_CODE_MOTOR_H_

#include "headfile.h"
#include "zf_pwm.h"
#include "zf_qtimer.h"
#include "PID.h"


#define MotorPWM_MAX 20000 //15000 //25000
#define MotorPWM_MIN 4000
#define MotorOutGarage_PWM 15000
#define MotorI_MAX 10000

extern PID_Struct Motor_GOL_PID;
extern PID_Struct Motor_GOR_PID;
extern Filter_Struct Motor_GOL_Filter;
extern Filter_Struct Motor_GOR_Filter;




void Motor_PIDStruct_Init(PID_Struct Motor_GOL_PID, PID_Struct Motor_GOR_PID,
                          Filter_Struct Motor_GOR_Filter,
                          Filter_Struct Motor_GOL_Filter);

void MotorCtrl(PID_Struct Motor_GOL_PID, PID_Struct Motor_GOR_PID,
               Filter_Struct Motor_GOL_Filter,
               Filter_Struct Motor_GOR_Filter);

void Motor_value_get();

void stop_car(uint8 time);

void UniformSpeed(int32 Speed);

void EncoderCalDistance(uint16 distance, int32 speed);

#endif // RT1064_CODE_MOTOR_H_
