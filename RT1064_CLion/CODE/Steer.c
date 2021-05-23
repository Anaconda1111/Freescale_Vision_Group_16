//
// Created by xcs on 2021-03-03.
//

#include "Steer.h"
#include "ADC.h"
#include "port.h"
#include "zf_pwm.h"
#include "PID.h"
#include "fastmath.h"

extern PID_Struct Steer_PID;
extern Filter_Struct Steer_Filter;
extern float InductanceValue_Normal[InductanceNum];
extern int16 Island_Flag;


uint16 SteerPWMDuty = MiddleSteer_PWM;

void Steer_PIDStruct_Init(PID_Struct Steer_PID, Filter_Struct Steer_Filter) {
    SteerPWMDuty = MiddleSteer_PWM;
    Steer_PID->KP = 8.95f;
    Steer_PID->KI = 0;
    Steer_PID->KD = 0;//1.32;
    Steer_PID->I_MAX =Steer_IMAX;
    Steer_PID->TargetValue = 0;
    Steer_Filter->Coefficient = 1.0f / 4.0f;         //RC低通滤波器滤波系数、待定
}


void SteerCtrl(PID_Struct Steer_PID, Filter_Struct Steer_Filter) {
    Steer_PID->CurrentValue = InductanceValueHandler();



    int16 Angle = (int16)FastABS(Steer_PID->CurrentValue);
    Steer_PID->KP = 13.0f + Angle * 0.85f;


    SteerPWMDuty = MiddleSteer_PWM +(int16)(PIDCalculate(Steer_PID, Steer_Filter));



    //舵机保护
    if (SteerPWMDuty > SteerMAX)
    {
        SteerPWMDuty = SteerMAX;
    }
    else if (SteerPWMDuty < SteerMIN)
    {
        SteerPWMDuty = SteerMIN;
    }

    pwm_duty(SteerPWM_CH, SteerPWMDuty);//输出PWM控制舵机打角，待完善PWM部分的代码后可用

}
