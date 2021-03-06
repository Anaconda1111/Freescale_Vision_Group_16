//
// Created by xcs on 2021-03-04.
//

#ifndef RT1064_CODE_PWM_H_
#define RT1064_CODE_PWM_H_
#include "common.h"
// extern PWM_Type *PWMPTR[];
typedef enum {
  Motor_RT_R,
  Motor_RT_L,
  Motor_GO_R,
  Motor_GO_L,
  SteerPWM
} PWMPin_enum;
void User_PWMInit(PWMPin_enum PWMch, uint32 freq, uint32 duty);
void User_PWMDuty(PWMPin_enum PWMch, uint32 duty);
#endif // RT1064_CODE_PWM_H_
