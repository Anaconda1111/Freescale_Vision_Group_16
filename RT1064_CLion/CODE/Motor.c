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

extern Filter_Struct Encoder_L_Filter;
extern Filter_Struct Encoder_R_Filter;


uint16 Motor_GO_L_PWM=9000;
uint16 Motor_GO_R_PWM=9000;

int16 encoder_value_L =0;
int16 encoder_value_R =0;
uint16 speed_R = 60;  //70还可以跑，但是要擦轮胎
uint16 speed_L = 60;
int16 garageout_flag = 1;




void Motor_PIDStruct_Init(PID_Struct Motor_GOL_PID, PID_Struct Motor_GOR_PID,
                          Filter_Struct Motor_GOR_Filter,
                          Filter_Struct Motor_GOL_Filter) {
    Motor_GOL_PID->I_MAX = MotorI_MAX;
    Motor_GOR_PID->I_MAX = MotorI_MAX;
    Motor_GOL_Filter->Coefficient = 1.0f / 32.0f;
    Motor_GOR_Filter->Coefficient = 1.0f / 32.0f;

    Motor_GOL_PID->KP = 1.50f;
    Motor_GOL_PID->KI = 0.0f;
    Motor_GOL_PID->KD = 2.5f;

    Motor_GOR_PID->KP = 1.50f;
    Motor_GOR_PID->KI = 0.0f;
    Motor_GOR_PID->KD = 2.5f;


    Motor_GOL_PID->TargetValue =(float)(1000);
    Motor_GOR_PID->TargetValue =(float)(1000);
}


/*
int16 GetDifferentSpeed(int16 Angle) {
    if (Angle <= 100)
        return 0;
    else
        return Angle * Transform;
}
*/


void MotorCtrl(PID_Struct Motor_GOL_PID, PID_Struct Motor_GOR_PID,
               Filter_Struct Motor_GOL_Filter,
               Filter_Struct Motor_GOR_Filter) {


    float Speed_error = FastABS(Motor_GOL_PID->CurrentValue);
    float Angle = FastABS(Steer_PID->CurrentValue);

    if (Camera == IsAnimal) {   //停车
        pwm_duty(MotorPWM_Go_L_CH, 0);
        pwm_duty(MotorPWM_Go_R_CH, 0);
        systick_delay_ms(3050);
    }



//
    //电机PID
    if(Speed_error <= 50)
    {
        Motor_GOL_PID->KP = 3.50f + Speed_error*0.14;
        Motor_GOL_PID->KI = 0.0f;
        Motor_GOL_PID->KD = 0.0f;

        Motor_GOR_PID->KP = 3.50f + Speed_error*0.14;
        Motor_GOR_PID->KI = 0.0f;
        Motor_GOR_PID->KD = 0.0f;

    }
    else                                    //往左转
    {

        Motor_GOL_PID->KP = 10.50f;
        Motor_GOL_PID->KI = 0.0f;
        Motor_GOL_PID->KD = 0.0f;

        Motor_GOR_PID->KP = 10.50f;
        Motor_GOR_PID->KI = 0.0f;
        Motor_GOR_PID->KD = 0.0f;
    }



    if(Angle <= 2.5)
    {

        if(Steer_PID->CurrentValue > 0.0) //往右转
        {
            Motor_GOL_PID->TargetValue =(uint16)speed_L;
            Motor_GOR_PID->TargetValue =(uint16)speed_R;
        }
        else if(Steer_PID->CurrentValue < 0.0) //往左转
        {
            Motor_GOL_PID->TargetValue =(uint16)speed_L;
            Motor_GOR_PID->TargetValue =(uint16)speed_R;
        }
    }
    else if(Angle > 2.5 && Angle <= 6.5)
    {

        if(Steer_PID->CurrentValue > 0.0) //往右转
        {
            Motor_GOL_PID->TargetValue =(uint16)speed_L - 10;
            Motor_GOR_PID->TargetValue =(uint16)speed_R - 20;
        }
        else if(Steer_PID->CurrentValue < 0.0) //往左转
        {
            Motor_GOL_PID->TargetValue =(uint16)speed_L - 20;
            Motor_GOR_PID->TargetValue =(uint16)speed_R - 10;
        }
    }
    else if(Angle > 6.5 && Angle <= 9.5)
    {


        if(Steer_PID->CurrentValue > 0.0) //往右转
        {
            Motor_GOL_PID->TargetValue =(uint16)speed_L - 10;
            Motor_GOR_PID->TargetValue =(uint16)speed_R - 30;
        }
        else if(Steer_PID->CurrentValue < 0.0) //往左转
        {
            Motor_GOL_PID->TargetValue =(uint16)speed_L - 30;
            Motor_GOR_PID->TargetValue =(uint16)speed_R - 10;
        }
    }
    else if(Angle > 9.5 && Angle <= 12.5)
    {

        if(Steer_PID->CurrentValue > 0.0) //往右转
        {
            Motor_GOL_PID->TargetValue =(uint16)speed_L + 10;
            Motor_GOR_PID->TargetValue =(uint16)speed_R - 20;
        }
        else if(Steer_PID->CurrentValue < 0.0) //往左转
        {
            Motor_GOL_PID->TargetValue =(uint16)speed_L - 20;
            Motor_GOR_PID->TargetValue =(uint16)speed_R + 10;
        }
    }
    else if(Angle > 12.5 && Angle <= 15.5)
    {

        if(Steer_PID->CurrentValue > 0.0) //往右转
        {
            Motor_GOL_PID->TargetValue =(uint16)speed_L + 20;
            Motor_GOR_PID->TargetValue =(uint16)speed_R - 20;
        }
        else if(Steer_PID->CurrentValue < 0.0) //往左转
        {
            Motor_GOL_PID->TargetValue =(uint16)speed_L - 20;
            Motor_GOR_PID->TargetValue =(uint16)speed_R + 20;
        }
    }
    else if(Angle > 15.5 && Angle <= 18.5)
    {

        if(Steer_PID->CurrentValue > 0.0) //往右转
        {
            Motor_GOL_PID->TargetValue =(uint16)speed_L + 30;
            Motor_GOR_PID->TargetValue =(uint16)speed_R - 20;
        }
        else if(Steer_PID->CurrentValue < 0.0) //往左转
        {
            Motor_GOL_PID->TargetValue =(uint16)speed_L - 20;
            Motor_GOR_PID->TargetValue =(uint16)speed_R + 30;
        }
    }
    else if(Angle > 18.5 && Angle <= 21.5)
    {
        if(Steer_PID->CurrentValue > 0.0) //往右转
        {
            Motor_GOL_PID->TargetValue =(uint16)speed_L + 30;
            Motor_GOR_PID->TargetValue =(uint16)speed_R - 30;
        }
        else if(Steer_PID->CurrentValue < 0.0) //往左转
        {
            Motor_GOL_PID->TargetValue =(uint16)speed_L - 30;
            Motor_GOR_PID->TargetValue =(uint16)speed_R + 30;
        }
    }
    else if(Angle < 21.5 && Angle <= 24.5)
    {
        if(Steer_PID->CurrentValue > 0.0) //往右转
        {
            Motor_GOL_PID->TargetValue =(uint16)speed_L + 40;
            Motor_GOR_PID->TargetValue =(uint16)speed_R - 30;
        }
        else if(Steer_PID->CurrentValue < 0.0) //往左转
        {
            Motor_GOL_PID->TargetValue =(uint16)speed_L - 30;
            Motor_GOR_PID->TargetValue =(uint16)speed_R + 40;
        }
    }
    else
    {
        if(Steer_PID->CurrentValue > 0.0) //往右转
        {
            Motor_GOL_PID->TargetValue =(uint16)speed_L + 40;
            Motor_GOR_PID->TargetValue =(uint16)speed_R - 40;
        }
        else if(Steer_PID->CurrentValue < 0.0) //往左转
        {
            Motor_GOL_PID->TargetValue =(uint16)speed_L - 40;
            Motor_GOR_PID->TargetValue =(uint16)speed_R + 40;
        }
    }

    if(Island_Flag)
    {


        Motor_GOL_PID->TargetValue =(uint16)speed_L - 30;
        Motor_GOR_PID->TargetValue =(uint16)speed_R + 30;

    }



    //电机当前值
    Motor_GOL_PID->CurrentValue =(float)encoder_value_L;
    Motor_GOR_PID->CurrentValue =(float)encoder_value_R;

    //输出
    Motor_GO_L_PWM +=(int16)(PIDCalculate(Motor_GOL_PID, Motor_GOL_Filter));
    Motor_GO_R_PWM +=(int16)(PIDCalculate(Motor_GOR_PID, Motor_GOR_Filter));
    //电机保护
    if (Motor_GO_L_PWM > MotorPWM_MAX)
    {
        Motor_GO_L_PWM = MotorPWM_MAX;
    }
    if (Motor_GO_L_PWM < MotorPWM_MIN)
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
    if(InductanceValue_Normal[1] < 3.0 && InductanceValue_Normal[2] < 3.0 && InductanceValue_Normal[3] < 3.0  && InductanceValue_Normal[4] < 3.0
       && InductanceValue_Normal[0] < 3.0 && InductanceValue_Normal[5] < 3.0)
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
    //低通滤波系数
    Encoder_L_Filter->Coefficient = 1.0f/4.0f;
    Encoder_R_Filter->Coefficient = 1.0f/8.0f;


    Encoder_L_Filter->SampleValue =(float)qtimer_quad_get(QTIMER_3, Qtimer2_LSB);
    Encoder_R_Filter->SampleValue =(float)-qtimer_quad_get(QTIMER_1, Qtimer1_LSB);
    qtimer_quad_clear(QTIMER_1, Qtimer1_LSB);
    qtimer_quad_clear(QTIMER_3, Qtimer2_LSB);

    encoder_value_L =(int16)RCFilter(Encoder_L_Filter);
    encoder_value_R =(int16)(RCFilter(Encoder_R_Filter)* 0.558f);



}

/*


void motorctrl_Faster()
{
    int16 Angle = (int16)FastABS(Steer_PID->CurrentValue); // -13.0 <= angle <= 13
    //主要输出电机差速
    if(Angle <= 3.5)
    {
        if(Steer_PID->CurrentValue > 0.0) //往右转
        {
            Motor_GO_L_PWM =(uint16)(15000-Angle*228.6);
            Motor_GO_R_PWM =(uint16)(15000-Angle*342.9);
        }
        else if(Steer_PID->CurrentValue < 0.0) //往左转
        {
            Motor_GO_L_PWM =(uint16)(15000 - Angle*342.9);
            Motor_GO_R_PWM =(uint16)(15000 - Angle*228.6);
        }
    }
    else if(Angle > 3.5 && Angle <= 6.5)
    {
        if(Steer_PID->CurrentValue > 0.0) //往右转
        {
            Motor_GO_L_PWM =(uint16)(14200 - (Angle-3.5) * 266.7);
            Motor_GO_R_PWM =(uint16)(13800 - (Angle-3.5) * 400.0);
        }
        else if(Steer_PID->CurrentValue < 0.0) //往左转
        {
            Motor_GO_L_PWM =(uint16)(13800 - (Angle-3.5) * 400.0);
            Motor_GO_R_PWM =(uint16)(14200 - (Angle-3.5) * 266.7);
        }
    }
    else if(Angle > 6.5 && Angle <= 9.5)
    {
        if(Steer_PID->CurrentValue > 0.0) //往右转
        {
            Motor_GO_L_PWM =(uint16)(13400 - (Angle-6.5) * 266.7);
            Motor_GO_R_PWM =(uint16)(12600 - (Angle-6.5) * 400.0);
        }
        else if(Steer_PID->CurrentValue < 0.0) //往左转
        {
            Motor_GO_L_PWM =(uint16)(12600 - (Angle-6.5) * 400.0);
            Motor_GO_R_PWM =(uint16)(13400 - (Angle-6.5) * 266.7);
        }
    }
    else if(Angle > 9.5 && Angle <= 13.0)
    {
        if(Steer_PID->CurrentValue > 0.0) //往右转
        {
            Motor_GO_L_PWM =(uint16)(12600 - (Angle-9.5) * 228.6);
            Motor_GO_R_PWM =(uint16)(11400 - (Angle-9.5) * 342.9);
        }
        else if(Steer_PID->CurrentValue < 0.0) //往左转
        {
            Motor_GO_L_PWM =(uint16)(12600 - (Angle-9.5) * 342.9);
            Motor_GO_R_PWM =(uint16)(11400 - (Angle-9.5) * 228.6);
        }
    }
    else if(Angle > 13.0 && Angle <= 16.0)
    {
        if(Steer_PID->CurrentValue > 0.0) //往右转
        {
            Motor_GO_L_PWM =(uint16)(11800 - (Angle-13.0)*266.7);
            Motor_GO_R_PWM =(uint16)(10200 - (Angle-13.0)*400.0);
        }
        else if(Steer_PID->CurrentValue < 0.0) //往左转
        {
            Motor_GO_L_PWM =(uint16)(10200 - (Angle-13.0)*400.0);
            Motor_GO_R_PWM =(uint16)(11800 - (Angle-13.0)*266.7);
        }
    }
    else
    {
        if(Steer_PID->CurrentValue > 0.0) //往右转
        {
            Motor_GO_L_PWM =(uint16)11000;
            Motor_GO_R_PWM =(uint16)9000;
        }
        else if(Steer_PID->CurrentValue < 0.0) //往左转
        {
            Motor_GO_L_PWM =(uint16)9000;
            Motor_GO_R_PWM =(uint16)11000;
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
        Motor_GO_L_PWM -=3000;
        Motor_GO_R_PWM -=3000;
    }

    //动物停止30秒
    if (Camera == IsAnimal) {   //停车
        pwm_duty(MotorPWM_Go_L_CH, 0);
        pwm_duty(MotorPWM_Go_R_CH, 0);
        systick_delay_ms(3050);
    }

    //摄像头识别到二维码，先停车
    if(Camera)
    {
        pwm_duty(MotorPWM_Go_L_CH, 0);
        pwm_duty(MotorPWM_Go_R_CH, 0);

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
            Motor_GO_L_PWM =(uint16)(15000-Angle*228.6);
            Motor_GO_R_PWM =(uint16)(15000-Angle*342.9);
        }
        else if(Steer_PID->CurrentValue < 0.0) //往左转
        {
            Motor_GO_L_PWM =(uint16)(15000 - Angle*342.9);
            Motor_GO_R_PWM =(uint16)(15000 - Angle*228.6);
        }
    }
    else if(Angle > 3.5 && Angle <= 6.5)
    {
        if(Steer_PID->CurrentValue > 0.0) //往右转
        {
            Motor_GO_L_PWM =(uint16)(14200 - (Angle-3.5) * 266.7);
            Motor_GO_R_PWM =(uint16)(13800 - (Angle-3.5) * 400.0);
        }
        else if(Steer_PID->CurrentValue < 0.0) //往左转
        {
            Motor_GO_L_PWM =(uint16)(13800 - (Angle-3.5) * 400.0);
            Motor_GO_R_PWM =(uint16)(14200 - (Angle-3.5) * 266.7);
        }
    }
    else if(Angle > 6.5 && Angle <= 9.5)
    {
        if(Steer_PID->CurrentValue > 0.0) //往右转
        {
            Motor_GO_L_PWM =(uint16)(13400 - (Angle-6.5) * 266.7);
            Motor_GO_R_PWM =(uint16)(12600 - (Angle-6.5) * 400.0);
        }
        else if(Steer_PID->CurrentValue < 0.0) //往左转
        {
            Motor_GO_L_PWM =(uint16)(12600 - (Angle-6.5) * 400.0);
            Motor_GO_R_PWM =(uint16)(13400 - (Angle-6.5) * 266.7);
        }
    }
    else if(Angle > 9.5 && Angle <= 13.0)
    {
        if(Steer_PID->CurrentValue > 0.0) //往右转
        {
            Motor_GO_L_PWM =(uint16)(12600 - (Angle-9.5) * 228.6);
            Motor_GO_R_PWM =(uint16)(11400 - (Angle-9.5) * 342.9);
        }
        else if(Steer_PID->CurrentValue < 0.0) //往左转
        {
            Motor_GO_L_PWM =(uint16)(12600 - (Angle-9.5) * 342.9);
            Motor_GO_R_PWM =(uint16)(11400 - (Angle-9.5) * 228.6);
        }
    }
    else if(Angle > 13.0 && Angle <= 16.0)
    {
        if(Steer_PID->CurrentValue > 0.0) //往右转
        {
            Motor_GO_L_PWM =(uint16)(11800 - (Angle-13.0)*266.7);
            Motor_GO_R_PWM =(uint16)(10200 - (Angle-13.0)*400.0);
        }
        else if(Steer_PID->CurrentValue < 0.0) //往左转
        {
            Motor_GO_L_PWM =(uint16)(10200 - (Angle-13.0)*400.0);
            Motor_GO_R_PWM =(uint16)(11800 - (Angle-13.0)*266.7);
        }
    }
    else
    {
        if(Steer_PID->CurrentValue > 0.0) //往右转
        {
            Motor_GO_L_PWM =(uint16)11000;
            Motor_GO_R_PWM =(uint16)9000;
        }
        else if(Steer_PID->CurrentValue < 0.0) //往左转
        {
            Motor_GO_L_PWM =(uint16)9000;
            Motor_GO_R_PWM =(uint16)11000;
        }
    }


    //速度预测,急弯减速（提前作用）,作用不可代替弯道减速
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
        Motor_GO_L_PWM -=4000;
        Motor_GO_R_PWM -=4000;
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

*/