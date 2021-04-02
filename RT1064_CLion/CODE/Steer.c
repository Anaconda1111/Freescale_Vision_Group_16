//
// Created by xcs on 2021-03-03.
//

#include "Steer.h"
#include "ADC.h"
#include "port.h"
#include "zf_pwm.h"

uint8 Trident_Flag = 0;
uint16 SteerPWMDuty = 0;

void Steer_PIDStruct_Init(PID_Struct Steer_PID, Filter_Struct Steer_Filter) {
    SteerPWMDuty = MiddleSteer_PWM;
    Steer_PID->KP = 5;
    Steer_PID->I_MAX = Steer_IMAX;
    Steer_PID->TargetValue = 0;
    Steer_Filter->Coefficient = 1.0f / 4.0f;         //RC低通滤波器滤波系数、待定
}

void SteerCtrl(PID_Struct Steer_PID, Filter_Struct Steer_Filter) {
    Steer_PID->CurrentValue = InductanceValueHandler();
    SteerPWMDuty = MiddleSteer_PWM - (int16) (PIDCalculate(Steer_PID, Steer_Filter) / 10.0f);
    if (SteerPWMDuty > SteerMAX)
        SteerPWMDuty = SteerMAX;
    else if (SteerPWMDuty < SteerMIN)
        SteerPWMDuty = SteerMIN;
    pwm_duty(SteerPWM_CH, SteerPWMDuty);//输出PWM控制舵机打角，待完善PWM部分的代码后可用
}
