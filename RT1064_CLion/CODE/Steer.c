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

float Rate=3.5f;

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



    int16 Angle = (int16)FastABS(Steer_PID->CurrentValue); // -13.0 <= angle <= 13

    if(Island_Flag)
    {
        if(InductanceValue_Normal[0] > InductanceValue_Normal[5] && InductanceValue_Normal[2] > InductanceValue_Normal[3])//往右转,右环岛
        {
            Steer_PID->CurrentValue = Steer_PID->CurrentValue * 0.283f;
        }
        else if(InductanceValue_Normal[0] < InductanceValue_Normal[5] && InductanceValue_Normal[2] < InductanceValue_Normal[3])//往左转，左环岛
        {
            Steer_PID->CurrentValue = Steer_PID->CurrentValue * 0.283f;
        }
    }



    if(Steer_PID->CurrentValue > 0.0)   //往右转
    {
        if(Steer_PID->CurrentValue <= 3.5)
        {
            Steer_PID->KP = 5.56f + (float)Angle * 0.57f;
            Steer_PID->KI = 0.1f;
            Steer_PID->KD = 1.32f;
            Rate=1.10f + (float)Angle * 0.12f;

        }
        else if(Steer_PID->CurrentValue > 3.5 && Steer_PID->CurrentValue <= 6.5)
        {
            Steer_PID->KP = 7.56f + (float)(Angle-3.5) * 0.60f;
            Steer_PID->KI = 0.1f;
            Steer_PID->KD = 1.32f;
            Rate=1.50f + (float)(Angle-3.5) * 0.17f;
        }
        else if(Steer_PID->CurrentValue > 6.5 && Steer_PID->CurrentValue <= 9.0)
        {
            Steer_PID->KP = 9.35f + (float)(Angle-6.5) * 0.80f;
            Steer_PID->KI = 0.1f;
            Steer_PID->KD = 1.32f;
            Rate=2.00f + (float)(Angle-6.5) * 0.20f;

        }
        else if(Steer_PID->CurrentValue > 9.0 && Steer_PID->CurrentValue <= 10.5)
        {
            Steer_PID->KP = 11.35f + (float)(Angle-9.5) * 1.43f;
            Steer_PID->KI = 0.1f;
            Steer_PID->KD = 1.32f;
            Rate=2.50f + (float)(Angle-9.0) * 0.67f;
        }
        else
        {
            Steer_PID->KP = 13.5f;
            Steer_PID->KI = 0.1f;
            Steer_PID->KD = 1.32f;//1.32;
            Rate = 3.50f;
        }
    }
    else                               //往左转
    {
        if(Steer_PID->CurrentValue > -3.5)
        {
            Steer_PID->KP = 5.83f + (float)Angle * 0.57f;
            Steer_PID->KI = 0.1f;
            Steer_PID->KD = 1.32f;
            Rate=1.10f + (float)Angle * 0.12f;
        }
        else if(Steer_PID->CurrentValue < -3.5 && Steer_PID->CurrentValue > -6.5)
        {
            Steer_PID->KP = 7.83f + (float)(Angle-3.5) * 0.48f;
            Steer_PID->KI = 0.1f;
            Steer_PID->KD = 1.32f;
            Rate=1.50f + (float)(Angle-3.5) * 0.17f;
        }
        else if(Steer_PID->CurrentValue < -6.5 && Steer_PID->CurrentValue > -9.5)
        {
            Steer_PID->KP = 9.26f + (float)(Angle-6.5) * 0.67f;
            Steer_PID->KI = 0.1f;
            Steer_PID->KD = 1.32f;
            Rate=2.00f + (float)(Angle-6.5) * 0.17f;
        }
        else if(Steer_PID->CurrentValue < -9.5 && Steer_PID->CurrentValue > -12.5)
        {
            Steer_PID->KP = 11.26f + (float)(Angle-9.5) * 0.75f;
            Steer_PID->KI = 0.1f;
            Steer_PID->KD = 1.32f;
            Rate=2.50f + (float)(Angle-9.5) * 0.33f;
        }
        else
        {
            Steer_PID->KP = 13.5f;
            Steer_PID->KI = 0.1f;
            Steer_PID->KD = 1.32f;//1.32;
            Rate = 3.5f;
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
