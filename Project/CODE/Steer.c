//
// Created by xcs on 2021-03-03.
//

#include "Steer.h"
#include "port.h"
#include "zf_pwm.h"
#include "headfile.h"
#include "PID.h"
#include "fastmath.h"
#include "image.h"


extern PID_Struct Steer_PID;
extern Filter_Struct Steer_Filter;
extern float current_value;


uint16 SteerPWMDuty = MiddleSteer_PWM;
float Angle =0;

void Steer_PIDStruct_Init(PID_Struct Steer_PID, Filter_Struct Steer_Filter) {
    SteerPWMDuty = MiddleSteer_PWM;
    Steer_PID->KP = 10.0;
    Steer_PID->KI = 0.0;
    Steer_PID->KD = 0.0;//1.32;
    Steer_PID->I_MAX =Steer_IMAX;
    Steer_PID->TargetValue = 0;
    Steer_Filter->Coefficient = 1.0f / 4.0f;         //RC低通滤波器滤波系数、待定
}


void SteerCtrl(PID_Struct Steer_PID, Filter_Struct Steer_Filter) {
  
    Steer_PID->CurrentValue = current_value;
    
    //Steer_PID->KP = 5.0f + Angle * 0.85f;//10
    Angle =my_abs_float(current_value);
    
    Steer_PID->KP = 10.0 + Angle * 0.35f; //待测
    
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
