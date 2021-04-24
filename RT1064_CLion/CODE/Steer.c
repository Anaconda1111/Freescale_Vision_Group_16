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

uint8 Trident_Flag = 0;
uint16 SteerPWMDuty = MiddleSteer_PWM;

float Rate=3.5;

void Steer_PIDStruct_Init(PID_Struct Steer_PID, Filter_Struct Steer_Filter) {
    SteerPWMDuty = MiddleSteer_PWM;
    Steer_PID->KP = 8.95;
    Steer_PID->KI = 0;
    Steer_PID->KD = 0;//1.32;
    Steer_PID->I_MAX =Steer_IMAX;
    Steer_PID->TargetValue = 0;
    Steer_Filter->Coefficient = 1.0f / 4.0f;         //RC低通滤波器滤波系数、待定
}


void SteerCtrl(PID_Struct Steer_PID, Filter_Struct Steer_Filter) {
    Steer_PID->CurrentValue = InductanceValueHandler();



    int16 Angle = (int16)FastABS(Steer_PID->CurrentValue); // -11.0 <= angle <= 11

    if(Steer_PID->CurrentValue > 0.0)   //往右转
    {
        if(Steer_PID->CurrentValue <= 3.5)
        {
            Steer_PID->KP = 4.56 + (float)Angle * 0.57;
            Steer_PID->KI = 0.1;
            Steer_PID->KD = 0;
            Rate= 1.10 + (float)Angle * 0.12;

        }
        else if(Steer_PID->CurrentValue > 3.5 && Steer_PID->CurrentValue <= 6.5)
        {
            Steer_PID->KP = (float)6.56 + (float)(Angle-3.5) * 0.60;
            Steer_PID->KI = 0.1;
            Steer_PID->KD = 0.22;
            Rate=1.50 + (float)(Angle-3.5) * 0.17;
        }
        else if(Steer_PID->CurrentValue > 6.5 && Steer_PID->CurrentValue <= 9.0)
        {
            Steer_PID->KP = 8.35 + (float)(Angle-6.5) * 0.80;
            Steer_PID->KI = 0.1;
            Steer_PID->KD = 0.55;
            Rate=2.00 + (float)(Angle-6.5) * 0.20;

        }
        else if(Steer_PID->CurrentValue > 9.0 && Steer_PID->CurrentValue <= 10.5)
        {
            Steer_PID->KP = 10.35 + (float)(Angle-9.5) * 1.43;
            Steer_PID->KI = 0.1;
            Steer_PID->KD = 1.0;
            Rate=2.50 + (float)(Angle-9.0) * 0.67;
        }
        else
        {
            Steer_PID->KP = 12.5;
            Steer_PID->KI = 0.1;
            Steer_PID->KD = 1.0;//1.32;
            Rate = 3.50;
        }
    }
    else                               //往左转
    {
        if(Steer_PID->CurrentValue > -3.5)
        {
            Steer_PID->KP = 4.83 + (float)Angle * 0.57;
            Steer_PID->KI = 0.1;
            Steer_PID->KD = 0;
            Rate=1.10 + (float)Angle * 0.12;
        }
        else if(Steer_PID->CurrentValue < -3.5 && Steer_PID->CurrentValue > -6.5)
        {
            Steer_PID->KP = 6.83 + (float)(Angle-3.5) * 0.48;
            Steer_PID->KI = 0.1;
            Steer_PID->KD = 0.26;
            Rate=1.50 + (float)(Angle-3.5) * 0.17;
        }
        else if(Steer_PID->CurrentValue < -6.5 && Steer_PID->CurrentValue > -9.5)
        {
            Steer_PID->KP = 8.26 + (float)(Angle-6.5) * 0.67;
            Steer_PID->KI = 0.1;
            Steer_PID->KD = 0.56;
            Rate=2.00 + (float)(Angle-6.5) * 0.17;
        }
        else if(Steer_PID->CurrentValue < -9.5 && Steer_PID->CurrentValue > -12.5)
        {
            Steer_PID->KP = 10.26 + (float)(Angle-9.5) * 0.75;
            Steer_PID->KI = 0.1;
            Steer_PID->KD = 1.00;
            Rate=2.50 + (float)(Angle-9.5) * 0.33;
        }
        else
        {
            Steer_PID->KP = 12.5;
            Steer_PID->KI = 0.1;
            Steer_PID->KD = 1.0;//1.32;
            Rate = 3.5;
        }
    }

/*
    //动态P 控制
    int16 Angle = (int16)FastABS(Steer_PID->CurrentValue); // -11.0 <= angle <= 11
    Steer_PID->KP = 8.56 +  Angle/;
    Steer_PID->KI = 0.16;
    Steer_PID->KD = 1.50 + Angle ;
    Rate = (float)Angle / 2.5;
 */



    SteerPWMDuty = MiddleSteer_PWM +(int16)(PIDCalculate(Steer_PID, Steer_Filter)* Rate);

    /*   //环岛辅助打角
       if(Island_Flag)
       {
          if(InductanceValue_Normal[0] > InductanceValue_Normal[5])//往右
         {
             SteerPWMDuty = MiddleSteer_PWM -(int16)200;
         }
         else if(InductanceValue_Normal[0] < InductanceValue_Normal[5])//往左
         {
             SteerPWMDuty = MiddleSteer_PWM +(int16)200;
         }
       }
   */

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
