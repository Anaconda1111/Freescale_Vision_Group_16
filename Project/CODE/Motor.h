//
// Created by xcs on 2021-03-05.
//

#ifndef RT1064_CODE_MOTOR_H_
#define RT1064_CODE_MOTOR_H_

#include "headfile.h"
#include "zf_pwm.h"
#include "zf_qtimer.h"


#define MotorPWM_MAX 20000 //25000
#define MotorPWM_MIN 3000
#define MotorOutGarage_PWM 15000
#define MotorI_MAX 10000
#define IsFruit  3
#define IsAnimal 4

int16 GetDifferentSpeed(int16 Angle);

void GarageOut();

void Motor_PIDStruct_Init(PID_Struct Motor_GOL_PID, PID_Struct Motor_GOR_PID,
                          Filter_Struct Motor_GOR_Filter,
                          Filter_Struct Motor_GOL_Filter);

void MotorCtrl(PID_Struct Motor_GOL_PID, PID_Struct Motor_GOR_PID,
               Filter_Struct Motor_GOL_Filter,
               Filter_Struct Motor_GOR_Filter);

void Motor_value_get();

void motorctrl_test();
void CarStop(PID_Struct Motor_GOL_PID, PID_Struct Motor_GOR_PID,
                          Filter_Struct Motor_GOR_Filter,
                          Filter_Struct Motor_GOL_Filter);

/*

电机控制：5ms 为周期
单机脉冲：13000
编码器值：80



轮一圈，脉冲数：
L:1300
R:2330




*/

#endif // RT1064_CODE_MOTOR_H_
