//
// Created by xcs on 2021-03-05.
//

#ifndef RT1064_CODE_MOTOR_H_
#define RT1064_CODE_MOTOR_H_

#include "ADC.h"
#include "zf_pwm.h"
#include "zf_qtimer.h"
#include "PID.h"

extern uint16 SteerPWMDuty;
extern float InductanceValue_Normal[InductanceNum];

#define MotorPWM_MAX 25000
#define MotorPWM_MIN 0
#define MotorOutGarage_PWM 10000
#define MotorI_MAX 0
#define MAXEncoder 0
#define Transform 0
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

#endif // RT1064_CODE_MOTOR_H_
