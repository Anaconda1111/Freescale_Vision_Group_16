//
// Created by xcs on 2021-03-05.
//

#include "Motor.h"
#include "Steer.h"
#include "port.h"
#include "zf_systick.h"
#include "fastmath.h"


extern uint8 Camera;
extern PID_Struct Steer_PID;
extern int16 Island_Flag;

uint16 Motor_GO_L_PWM=9000;
uint16 Motor_GO_R_PWM=9000;

int16 encoder_value_L =0;
int16 encoder_value_R =0;

int16 garageout_flag = 1;

void Motor_PIDStruct_Init(PID_Struct Motor_GOL_PID, PID_Struct Motor_GOR_PID,
                          Filter_Struct Motor_GOR_Filter,
                          Filter_Struct Motor_GOL_Filter) {
    Motor_GOL_PID->I_MAX = MotorI_MAX;
    Motor_GOR_PID->I_MAX = MotorI_MAX;
    Motor_GOL_Filter->Coefficient = 1.0f / 16.0f;
    Motor_GOR_Filter->Coefficient = 1.0f / 16.0f;

    Motor_GOL_PID->KP = 1.30;
    Motor_GOL_PID->KI = 0.10;
    Motor_GOL_PID->KD = 0.50;
    Motor_GOL_Filter->Coefficient =  1.0f / 4.0f;

    Motor_GOR_PID->KP = 1.15;
    Motor_GOR_PID->KI = 0.10;
    Motor_GOR_PID->KD = 0.50;
    Motor_GOR_Filter->Coefficient =  1.0f / 4.0f;


}

int16 GetDifferentSpeed(int16 Angle) {
    if (Angle <= 100)
        return 0;
    else
        return Angle * Transform;
}

void MotorCtrl(PID_Struct Motor_GOL_PID, PID_Struct Motor_GOR_PID,
               Filter_Struct Motor_GOL_Filter,
               Filter_Struct Motor_GOR_Filter) {
    /*
    if (InductanceValue_Normal[0] < 0.01 && InductanceValue_Normal[1] < 0.01 &&
        InductanceValue_Normal[2] < 0.01 && InductanceValue_Normal[3] < 0.01) {
        pwm_duty(MotorPWM_Go_L_CH, 0);
        pwm_duty(MotorPWM_Go_R_CH, 0);
        return;
    }

    if (Camera == IsAnimal) {   //停车
        pwm_duty(MotorPWM_Go_L_CH, 0);
        pwm_duty(MotorPWM_Go_R_CH, 0);
        systick_delay_ms(3050);
    }
*/

    int16 target_speed_L,target_speed_R;
    //int16 different_speed;

    int16 Angle = (int16)FastABS(Steer_PID->CurrentValue); // -11.0 <= angle <= 11
    if(Angle >= 9.0)
    {
        target_speed_L = (int16)750;
        target_speed_R = (int16)750;
    }
    else if (Angle < 9.0 && Angle >= 6.0)
    {
        target_speed_L = (int16)800;
        target_speed_R = (int16)800;
    }
    else if(Angle < 6.0 && Angle >= 3.0)
    {
        target_speed_L = (int16)850;
        target_speed_R = (int16)850;
    }
    else
    {
        target_speed_L = (int16)900;
        target_speed_R = (int16)900;
    }



    //different_speed = GetDifferentSpeed(Angle);
    //Motor_GOL_PID->TargetValue = (float) (target_speed_L - different_speed);
    //Motor_GOR_PID->TargetValue = (float) (target_speed_R + different_speed);

    if(Angle >= 9.0)
    {

        if(Steer_PID->CurrentValue > 0.0)  //往右转
        {
            Motor_GOR_PID->TargetValue = (float) (target_speed_R - Angle*20); //系数待调整
            Motor_GOL_PID->TargetValue = (float) (target_speed_L + Angle*20); //系数待调整
        }
        else if(Steer_PID->CurrentValue < 0.0)                          //往左转
        {
            Motor_GOR_PID->TargetValue = (float) (target_speed_R + Angle*20); //系数待调整
            Motor_GOL_PID->TargetValue = (float) (target_speed_L - Angle*20); //系数待调整
        }

    }
    else if(Angle < 9.0 && Angle <= 6.0)
    {
        if(Steer_PID->CurrentValue > 0.0)  //往右转
        {
            Motor_GOR_PID->TargetValue = (float) (target_speed_R - Angle*10); //系数待调整
            Motor_GOL_PID->TargetValue = (float) (target_speed_L + Angle*10); //系数待调整
        }
        else if(Steer_PID->CurrentValue < 0.0)                          //往左转
        {
            Motor_GOR_PID->TargetValue = (float) (target_speed_R + Angle*10); //系数待调整
            Motor_GOL_PID->TargetValue = (float) (target_speed_L - Angle*10); //系数待调整
        }
    }
    else if(Angle < 6.0 && Angle >= 3.0)
    {
        if(Steer_PID->CurrentValue > 0.0)  //往右转
        {
            Motor_GOR_PID->TargetValue = (float) (target_speed_R - Angle*5); //系数待调整
            Motor_GOL_PID->TargetValue = (float) (target_speed_L + Angle*5); //系数待调整
        }
        else if(Steer_PID->CurrentValue < 0.0)                          //往左转
        {
            Motor_GOR_PID->TargetValue = (float) (target_speed_R + Angle*5); //系数待调整
            Motor_GOL_PID->TargetValue = (float) (target_speed_L - Angle*5); //系数待调整
        }
    }
    else
    {
        Motor_GOR_PID->TargetValue = (float) target_speed_R; //系数待调整
        Motor_GOL_PID->TargetValue = (float) target_speed_L;
    }


    Motor_GOL_PID->CurrentValue = encoder_value_L;
    Motor_GOR_PID->CurrentValue = encoder_value_R;


    Motor_GO_L_PWM += (int16)(PIDCalculate(Motor_GOL_PID, Motor_GOL_Filter))*1;/*除以 转换系数*/
    Motor_GO_R_PWM += (int16)(PIDCalculate(Motor_GOR_PID, Motor_GOR_Filter))*1;/*除以 转换系数*/

    //电机保护
    if (Motor_GO_L_PWM > MotorPWM_MAX)
    {
        Motor_GO_L_PWM = MotorPWM_MAX;
    }
    if (Motor_GO_R_PWM < MotorPWM_MIN)
    {
        Motor_GO_L_PWM = MotorPWM_MIN;
    }

    if (Motor_GO_R_PWM > MotorPWM_MAX)
    {
        Motor_GO_R_PWM = MotorPWM_MAX;
    }
    if (Motor_GO_R_PWM < MotorPWM_MIN)
    {
        Motor_GO_R_PWM = MotorPWM_MIN;
    }

    //出轨保护
    if(InductanceValue_Normal[1] < 3.0 && InductanceValue_Normal[2] <1.0 && InductanceValue_Normal[3] <1.0  && InductanceValue_Normal[4] <3.0
       && InductanceValue_Normal[0] <0.5 && InductanceValue_Normal[5] <0.5)
    {
        pwm_duty(MotorPWM_Go_L_CH, 0);
        pwm_duty(MotorPWM_Go_R_CH, 0);

    }
    else
    {

        pwm_duty(MotorPWM_Go_L_CH, Motor_GO_L_PWM);
        pwm_duty(MotorPWM_Go_R_CH, Motor_GO_R_PWM);
        garageout_flag = 0;

    }
}


void GarageOut() {

    pwm_duty(MotorPWM_Go_L_CH, MotorOutGarage_PWM);
    pwm_duty(MotorPWM_Go_R_CH, MotorOutGarage_PWM);

    pwm_duty(SteerPWM_CH, SteerOutGarage_PWM);
/*
        float garageout_weight[6]={100,100,100,30,30,30};

        for(int i=0;i<6;i++)
        {
          InductanceValue_Normal[i] *  garageout_weight[i] /100.0f;
        }
*/

}




void Motor_value_get()
{


    encoder_value_L = qtimer_quad_get(QTIMER_3, Qtimer2_LSB);
    encoder_value_R =-qtimer_quad_get(QTIMER_1, Qtimer1_LSB);

    //整合，归一
    encoder_value_L =(int16) (encoder_value_L / 1.300);
    encoder_value_R =(int16) (encoder_value_R / 1.900);



    qtimer_quad_clear(QTIMER_1, Qtimer1_LSB);
    qtimer_quad_clear(QTIMER_3, Qtimer2_LSB);

    /*
     motor_speed_L = encoder_value_L * 62.8 * R / encoder_unit_L;
     motor_speed_R = encoder_value_R * 62.8 * R / encoder_unit_R;
    */



}


void motorctrl()
{
    int16 Angle = (int16)FastABS(Steer_PID->CurrentValue); // -13.0 <= angle <= 13

    if(Angle <= 3.5)
    {
        if(Steer_PID->CurrentValue > 0.0) //往右转
        {
            Motor_GO_L_PWM =(uint16)(12000-Angle*57.1);
            Motor_GO_R_PWM =(uint16)(12000-Angle*285.7);
        }
        else if(Steer_PID->CurrentValue < 0.0) //往左转
        {
            Motor_GO_L_PWM =(uint16)(12000 - Angle*285.7);
            Motor_GO_R_PWM =(uint16)(12000 - Angle*57.1);
        }
    }
    else if(Angle > 3.5 && Angle <= 6.5)
    {
        if(Steer_PID->CurrentValue > 0.0) //往右转
        {
            Motor_GO_L_PWM =(uint16)(11800 - (Angle-3.5) * 66.6);
            Motor_GO_R_PWM =(uint16)(11000 - (Angle-3.5) * 333.3);
        }
        else if(Steer_PID->CurrentValue < 0.0) //往左转
        {
            Motor_GO_L_PWM =(uint16)(11000 - (Angle-3.5) * 333.3);
            Motor_GO_R_PWM =(uint16)(11800 - (Angle-3.5) * 66.6);
        }
    }
    else if(Angle > 6.5 && Angle <= 9.5)
    {
        if(Steer_PID->CurrentValue > 0.0) //往右转
        {
            Motor_GO_L_PWM =(uint16)(11600 - (Angle-6.5) * 66.6);
            Motor_GO_R_PWM =(uint16)(10000 - (Angle-6.5) * 333.3);
        }
        else if(Steer_PID->CurrentValue < 0.0) //往左转
        {
            Motor_GO_L_PWM =(uint16)(10000 - (Angle-6.5) * 333.3);
            Motor_GO_R_PWM =(uint16)(11600 - (Angle-6.5) * 66.6);
        }
    }
    else if(Angle > 9.5 && Angle <= 13.0)
    {
        if(Steer_PID->CurrentValue > 0.0) //往右转
        {
            Motor_GO_L_PWM =(uint16)(11400 - (Angle-9.5) * 57.1);
            Motor_GO_R_PWM =(uint16)(9000 - (Angle-9.5) * 285.7);
        }
        else if(Steer_PID->CurrentValue < 0.0) //往左转
        {
            Motor_GO_L_PWM =(uint16)(9000 - (Angle-9.5) * 285.7);
            Motor_GO_R_PWM =(uint16)(11400 - (Angle-9.5) * 57.1);
        }
    }
    else
    {
        if(Steer_PID->CurrentValue > 0.0) //往右转
        {
            Motor_GO_L_PWM =(uint16)12000;
            Motor_GO_R_PWM =(uint16)8000;
        }
        else if(Steer_PID->CurrentValue < 0.0) //往左转
        {
            Motor_GO_L_PWM =(uint16)8000;
            Motor_GO_R_PWM =(uint16)12000;
        }
    }

    //速度预测,弯道降速，急弯减速
    if(FastABS(Steer_PID->Differential) >= 0.0 && FastABS(Steer_PID->Differential) < 0.5)
    {
        Motor_GO_L_PWM -=0;
        Motor_GO_R_PWM -=0;
    }
    else if(FastABS(Steer_PID->Differential) >= 0.5 && FastABS(Steer_PID->Differential) < 1.0)
    {
        Motor_GO_L_PWM -=1000;
        Motor_GO_R_PWM -=1000;
    }
    else if(FastABS(Steer_PID->Differential) >= 1.0 && FastABS(Steer_PID->Differential) < 2.0)
    {
        Motor_GO_L_PWM -=2000;
        Motor_GO_R_PWM -=2000;
    }
    else if(FastABS(Steer_PID->Differential) >= 2.0 && FastABS(Steer_PID->Differential) < 3.0)
    {
        Motor_GO_L_PWM -=3000;
        Motor_GO_R_PWM -=3000;
    }
    else
    {
        Motor_GO_L_PWM -=4000;
        Motor_GO_R_PWM -=4000;
    }


    //环岛减速
    if(Island_Flag)
    {
        Motor_GO_L_PWM -=2000;
        Motor_GO_R_PWM -=2000;
    }


    //电机保护
    if (Motor_GO_L_PWM > MotorPWM_MAX)
    {
        Motor_GO_L_PWM = MotorPWM_MAX;
    }
    if (Motor_GO_R_PWM < MotorPWM_MIN)
    {
        Motor_GO_L_PWM = MotorPWM_MIN;
    }

    if (Motor_GO_R_PWM > MotorPWM_MAX)
    {
        Motor_GO_R_PWM = MotorPWM_MAX;
    }
    if (Motor_GO_R_PWM < MotorPWM_MIN)
    {
        Motor_GO_R_PWM = MotorPWM_MIN;
    }

    //出轨保护
    if(InductanceValue_Normal[1] < 3.0 && InductanceValue_Normal[2] <1.0 && InductanceValue_Normal[3] <1.0  && InductanceValue_Normal[4] <3.0
       && InductanceValue_Normal[0] <0.5 && InductanceValue_Normal[5] <0.5)
    {
        pwm_duty(MotorPWM_Go_L_CH, 0);
        pwm_duty(MotorPWM_Go_R_CH, 0);

    }
    else
    {

        pwm_duty(MotorPWM_Go_L_CH, Motor_GO_L_PWM);
        pwm_duty(MotorPWM_Go_R_CH, Motor_GO_R_PWM);
        garageout_flag = 0;

    }
}



void motorctrl_Faster()
{
    int16 Angle = (int16)FastABS(Steer_PID->CurrentValue); // -13.0 <= angle <= 13

    if(Angle <= 3.5)
    {
        if(Steer_PID->CurrentValue > 0.0) //往右转
        {
            Motor_GO_L_PWM =(uint16)(15000-Angle*114.3);
            Motor_GO_R_PWM =(uint16)(15000-Angle*400.0);
        }
        else if(Steer_PID->CurrentValue < 0.0) //往左转
        {
            Motor_GO_L_PWM =(uint16)(15000 - Angle*400.0);
            Motor_GO_R_PWM =(uint16)(15000 - Angle*114.3);
        }
    }
    else if(Angle > 3.5 && Angle <= 6.5)
    {
        if(Steer_PID->CurrentValue > 0.0) //往右转
        {
            Motor_GO_L_PWM =(uint16)(14600 - (Angle-3.5) * 133.3);
            Motor_GO_R_PWM =(uint16)(13600 - (Angle-3.5) * 466.6);
        }
        else if(Steer_PID->CurrentValue < 0.0) //往左转
        {
            Motor_GO_L_PWM =(uint16)(13600 - (Angle-3.5) * 466.6);
            Motor_GO_R_PWM =(uint16)(14600 - (Angle-3.5) * 133.3);
        }
    }
    else if(Angle > 6.5 && Angle <= 9.5)
    {
        if(Steer_PID->CurrentValue > 0.0) //往右转
        {
            Motor_GO_L_PWM =(uint16)(14200 - (Angle-6.5) * 133.3);
            Motor_GO_R_PWM =(uint16)(12200 - (Angle-6.5) * 466.6);
        }
        else if(Steer_PID->CurrentValue < 0.0) //往左转
        {
            Motor_GO_L_PWM =(uint16)(12200 - (Angle-6.5) * 466.6);
            Motor_GO_R_PWM =(uint16)(14200 - (Angle-6.5) * 133.3);
        }
    }
    else if(Angle > 9.5 && Angle <= 13.0)
    {
        if(Steer_PID->CurrentValue > 0.0) //往右转
        {
            Motor_GO_L_PWM =(uint16)(13800 - (Angle-9.5) * 114.3);
            Motor_GO_R_PWM =(uint16)(10800 - (Angle-9.5) * 400.0);
        }
        else if(Steer_PID->CurrentValue < 0.0) //往左转
        {
            Motor_GO_L_PWM =(uint16)(10800 - (Angle-9.5) * 400.0);
            Motor_GO_R_PWM =(uint16)(13800 - (Angle-9.5) * 114.3);
        }
    }
    else
    {
        if(Steer_PID->CurrentValue > 0.0) //往右转
        {
            Motor_GO_L_PWM =(uint16)13400;
            Motor_GO_R_PWM =(uint16)9400;
        }
        else if(Steer_PID->CurrentValue < 0.0) //往左转
        {
            Motor_GO_L_PWM =(uint16)9400;
            Motor_GO_R_PWM =(uint16)13400;
        }
    }

    //速度预测,弯道降速，急弯减速
    if(FastABS(Steer_PID->Differential) >= 0.0 && FastABS(Steer_PID->Differential) < 0.5)
    {
        Motor_GO_L_PWM -=0;
        Motor_GO_R_PWM -=0;
    }
    else if(FastABS(Steer_PID->Differential) >= 0.5 && FastABS(Steer_PID->Differential) < 1.0)
    {
        Motor_GO_L_PWM -=1000;
        Motor_GO_R_PWM -=1000;
    }
    else if(FastABS(Steer_PID->Differential) >= 1.0 && FastABS(Steer_PID->Differential) < 2.0)
    {
        Motor_GO_L_PWM -=2000;
        Motor_GO_R_PWM -=2000;
    }
    else if(FastABS(Steer_PID->Differential) >= 2.0 && FastABS(Steer_PID->Differential) < 3.0)
    {
        Motor_GO_L_PWM -=3000;
        Motor_GO_R_PWM -=3000;
    }
    else
    {
        Motor_GO_L_PWM -=4000;
        Motor_GO_R_PWM -=4000;
    }

    //环岛减速
    if(Island_Flag)
    {
        Motor_GO_L_PWM -=2000;
        Motor_GO_R_PWM -=2000;
    }


    //电机保护
    if (Motor_GO_L_PWM > MotorPWM_MAX)
    {
        Motor_GO_L_PWM = MotorPWM_MAX;
    }
    if (Motor_GO_R_PWM < MotorPWM_MIN)
    {
        Motor_GO_L_PWM = MotorPWM_MIN;
    }

    if (Motor_GO_R_PWM > MotorPWM_MAX)
    {
        Motor_GO_R_PWM = MotorPWM_MAX;
    }
    if (Motor_GO_R_PWM < MotorPWM_MIN)
    {
        Motor_GO_R_PWM = MotorPWM_MIN;
    }

    //出轨保护
    if(InductanceValue_Normal[1] < 3.0 && InductanceValue_Normal[2] <1.0 && InductanceValue_Normal[3] <1.0  && InductanceValue_Normal[4] <3.0
       && InductanceValue_Normal[0] <0.5 && InductanceValue_Normal[5] <0.5)
    {
        pwm_duty(MotorPWM_Go_L_CH, 0);
        pwm_duty(MotorPWM_Go_R_CH, 0);

    }
    else
    {

        pwm_duty(MotorPWM_Go_L_CH, Motor_GO_L_PWM);
        pwm_duty(MotorPWM_Go_R_CH, Motor_GO_R_PWM);
        garageout_flag = 0;

    }
}



void motorctrl_test()
{
    int16 Angle = (int16)FastABS(Steer_PID->CurrentValue); // -13.0 <= angle <= 13

    //主要输出电机差速
    if(Angle <= 3.5)
    {
        if(Steer_PID->CurrentValue > 0.0) //往右转
        {
            Motor_GO_L_PWM =(uint16)(13000-Angle*57.1);
            Motor_GO_R_PWM =(uint16)(13000-Angle*342.9);
        }
        else if(Steer_PID->CurrentValue < 0.0) //往左转
        {
            Motor_GO_L_PWM =(uint16)(13000 - Angle*342.9);
            Motor_GO_R_PWM =(uint16)(13000 - Angle*57.1);
        }
    }
    else if(Angle > 3.5 && Angle <= 6.5)
    {
        if(Steer_PID->CurrentValue > 0.0) //往右转
        {
            Motor_GO_L_PWM =(uint16)(12800 - (Angle-3.5) * 66.6);
            Motor_GO_R_PWM =(uint16)(11800 - (Angle-3.5) * 400.0);
        }
        else if(Steer_PID->CurrentValue < 0.0) //往左转
        {
            Motor_GO_L_PWM =(uint16)(11800 - (Angle-3.5) * 400.0);
            Motor_GO_R_PWM =(uint16)(12800 - (Angle-3.5) * 66.6);
        }
    }
    else if(Angle > 6.5 && Angle <= 9.5)
    {
        if(Steer_PID->CurrentValue > 0.0) //往右转
        {
            Motor_GO_L_PWM =(uint16)(12600 - (Angle-6.5) * 66.6);
            Motor_GO_R_PWM =(uint16)(10600 - (Angle-6.5) * 400.0);
        }
        else if(Steer_PID->CurrentValue < 0.0) //往左转
        {
            Motor_GO_L_PWM =(uint16)(10600 - (Angle-6.5) * 400.0);
            Motor_GO_R_PWM =(uint16)(12600 - (Angle-6.5) * 66.6);
        }
    }
    else if(Angle > 9.5 && Angle <= 13.0)
    {
        if(Steer_PID->CurrentValue > 0.0) //往右转
        {
            Motor_GO_L_PWM =(uint16)(12400 - (Angle-9.5) * 57.1);
            Motor_GO_R_PWM =(uint16)(9400 - (Angle-9.5) * 342.9);
        }
        else if(Steer_PID->CurrentValue < 0.0) //往左转
        {
            Motor_GO_L_PWM =(uint16)(9400 - (Angle-9.5) * 342.9);
            Motor_GO_R_PWM =(uint16)(12400 - (Angle-9.5) * 57.1);
        }
    }
    else
    {
        if(Steer_PID->CurrentValue > 0.0) //往右转
        {
            Motor_GO_L_PWM =(uint16)12200;
            Motor_GO_R_PWM =(uint16)82000;
        }
        else if(Steer_PID->CurrentValue < 0.0) //往左转
        {
            Motor_GO_L_PWM =(uint16)8200;
            Motor_GO_R_PWM =(uint16)12200;
        }
    }


    //速度预测,弯道降速，急弯减速
    if(FastABS(Steer_PID->Differential) >= 0.0 && FastABS(Steer_PID->Differential) < 0.5)
    {
        Motor_GO_L_PWM -=0;
        Motor_GO_R_PWM -=0;
    }
    else if(FastABS(Steer_PID->Differential) >= 0.5 && FastABS(Steer_PID->Differential) < 1.0)
    {
        Motor_GO_L_PWM -=1000;
        Motor_GO_R_PWM -=1000;
    }
    else if(FastABS(Steer_PID->Differential) >= 1.0 && FastABS(Steer_PID->Differential) < 2.0)
    {
        Motor_GO_L_PWM -=2000;
        Motor_GO_R_PWM -=2000;
    }
    else if(FastABS(Steer_PID->Differential) >= 2.0 && FastABS(Steer_PID->Differential) < 3.0)
    {
        Motor_GO_L_PWM -=3000;
        Motor_GO_R_PWM -=3000;
    }
    else
    {
        Motor_GO_L_PWM -=4000;
        Motor_GO_R_PWM -=4000;
    }


    //环岛减速
    if(Island_Flag)
    {
        Motor_GO_L_PWM -=2000;
        Motor_GO_R_PWM -=2000;
    }


    //电机保护
    if (Motor_GO_L_PWM > MotorPWM_MAX)
    {
        Motor_GO_L_PWM = MotorPWM_MAX;
    }
    if (Motor_GO_R_PWM < MotorPWM_MIN)
    {
        Motor_GO_L_PWM = MotorPWM_MIN;
    }

    if (Motor_GO_R_PWM > MotorPWM_MAX)
    {
        Motor_GO_R_PWM = MotorPWM_MAX;
    }
    if (Motor_GO_R_PWM < MotorPWM_MIN)
    {
        Motor_GO_R_PWM = MotorPWM_MIN;
    }

    //出轨保护
    if(InductanceValue_Normal[1] < 3.0 && InductanceValue_Normal[2] <1.0 && InductanceValue_Normal[3] <1.0  && InductanceValue_Normal[4] <3.0
       && InductanceValue_Normal[0] <0.5 && InductanceValue_Normal[5] <0.5)
    {
        pwm_duty(MotorPWM_Go_L_CH, 0);
        pwm_duty(MotorPWM_Go_R_CH, 0);

    }
    else
    {

        pwm_duty(MotorPWM_Go_L_CH, Motor_GO_L_PWM);
        pwm_duty(MotorPWM_Go_R_CH, Motor_GO_R_PWM);
        garageout_flag = 0;

    }
}

