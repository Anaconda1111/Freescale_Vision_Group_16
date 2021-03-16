//
// Created by xcs on 2021-03-05.
//

#ifndef RT1064_CODE_MOTOR_H_
#define RT1064_CODE_MOTOR_H_

#include "ADC.h"
#include "zf_pwm.h"
#include "zf_qtimer.h"

extern uint16 SteerPWMDuty;
extern float InductanceValue_Normal[InductanceNum];

#define MotorPWM_MAX 500
#define MotorPWM_MIN 0
#define MotorOutGarage_PWM 200
#define MotorI_MAX 0
#define MAXEncoder 0
#define Transform 0

int16 GetDifferentSpeed(int16 Angle);

void GarageOut();

#endif // RT1064_CODE_MOTOR_H_
