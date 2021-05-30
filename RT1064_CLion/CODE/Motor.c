//
// Created by xcs on 2021-03-05.
//

#include "Motor.h"
#include "Steer.h"
#include "port.h"
#include "zf_systick.h"
#include "fastmath.h"
#include "ART.h"

extern uint8 Camera;
extern PID_Struct Steer_PID;
extern int16 Island_Flag;
extern uint32 TridentFlag;
extern uint8 FirstTrident;
uint16 Motor_GO_L_PWM = 9000;
uint16 Motor_GO_R_PWM = 9000;

int16 encoder_value_L = 0;
int16 encoder_value_R = 0;
extern Filter_Struct Encoder_L_Filter;
extern Filter_Struct Encoder_R_Filter;

int16 garageout_flag = 1;

void Motor_PIDStruct_Init(PID_Struct Motor_GOL_PID, PID_Struct Motor_GOR_PID,
                          Filter_Struct Motor_GOR_Filter,
                          Filter_Struct Motor_GOL_Filter) {
    Motor_GOL_PID->I_MAX = MotorI_MAX;
    Motor_GOR_PID->I_MAX = MotorI_MAX;

    Motor_GOL_PID->KP = 200.0f;
    Motor_GOL_PID->KI = 0.0f;
    Motor_GOL_PID->KD = 0.0f;
    Motor_GOL_Filter->Coefficient = 1.0f / 16.0f;

    Motor_GOR_PID->KP = 160.0f;
    Motor_GOR_PID->KI = 10.0f;
    Motor_GOR_PID->KD = 0.0f;
    Motor_GOR_Filter->Coefficient = 1.0f / 16.0f;


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

    float Angle = Steer_PID->CurrentValue;
    float Rate = 300;
    float Rate2 = 0;
    float differentSpeed = 0;
    float baseSpeed = 13000;
    if (TridentFlag)
        baseSpeed = 16000;
    else if (FirstTrident >= 2)
        baseSpeed = 12000;
    if (FastABS(Angle) > 3.5) {
        Rate2 = (FastABS(Angle) / 10.0f) + 0.8f;
        differentSpeed = Rate * Angle;
        differentSpeed *= Rate2;
        baseSpeed -= FastABS(Angle) * 70 * Rate2;
    }
    Motor_GO_L_PWM = baseSpeed + differentSpeed;
    Motor_GO_R_PWM = baseSpeed - differentSpeed;
    // Motor_GOL_PID->TargetValue += (baseSpeed + differentSpeed);
    // Motor_GOR_PID->TargetValue -= ((differentSpeed + baseSpeed) * Rate);
    // Motor_GOL_PID->CurrentValue = encoder_value_L;
    // Motor_GOR_PID->CurrentValue = encoder_value_R;
    //
    //

    // Motor_GO_L_PWM = (int16) (PIDCalculate(Motor_GOL_PID, Motor_GOL_Filter));/*除以 转换系数*/
    // Motor_GO_R_PWM = (int16) (PIDCalculate(Motor_GOR_PID, Motor_GOR_Filter));/*除以 转换系数*/

    //电机保护
    if (Motor_GO_L_PWM > MotorPWM_MAX) {
        Motor_GO_L_PWM = MotorPWM_MAX;
    }
    if (Motor_GO_R_PWM < MotorPWM_MIN) {
        Motor_GO_L_PWM = MotorPWM_MIN;
    }

    if (Motor_GO_R_PWM > MotorPWM_MAX) {
        Motor_GO_R_PWM = MotorPWM_MAX;
    }
    if (Motor_GO_R_PWM < MotorPWM_MIN) {
        Motor_GO_R_PWM = MotorPWM_MIN;
    }

    //出轨保护
    // if (InductanceValue_Normal[1] < 3.0 && InductanceValue_Normal[2] < 1.0 && InductanceValue_Normal[3] < 1.0 && InductanceValue_Normal[4] < 3.0
    //     && InductanceValue_Normal[0] < 0.5 && InductanceValue_Normal[5] < 0.5) {
    //     pwm_duty(MotorPWM_Go_L_CH, 0);
    //     pwm_duty(MotorPWM_Go_R_CH, 0);
    //     return;
    // }
    if (InductanceValue_Normal[1] < 0.5 && InductanceValue_Normal[2] < 1.0 && InductanceValue_Normal[3] < 1.0 && InductanceValue_Normal[4] < 3.0
        && InductanceValue_Normal[0] < 0.5 && InductanceValue_Normal[5] < 0.5) {
        // pwm_duty(MotorPWM_Go_L_CH, 0);
        // pwm_duty(MotorPWM_Go_R_CH, 0);
        Motor_GO_R_PWM = 0;
        Motor_GO_R_PWM = 0;
    }

    pwm_duty(MotorPWM_Go_L_CH, Motor_GO_L_PWM);
    pwm_duty(MotorPWM_Go_R_CH, Motor_GO_R_PWM);
    /*else {

            pwm_duty(MotorPWM_Go_L_CH, Motor_GO_L_PWM);
            pwm_duty(MotorPWM_Go_R_CH, Motor_GO_R_PWM);
            garageout_flag = 0;

        }*/
}


void GarageOut() {

    pwm_duty(MotorPWM_Go_L_CH, MotorOutGarage_PWM);
    pwm_duty(MotorPWM_Go_R_CH, MotorOutGarage_PWM);
    while (InductanceValue_Normal[0] < 28)
        InductanceValueFilter();
    pwm_duty(SteerPWM_CH, SteerOutGarage_PWM);
    systick_delay_ms(100);


}


void Motor_value_get() {

    Encoder_L_Filter->Coefficient = 1.0f / 4.0f;
    Encoder_R_Filter->Coefficient = 1.0f / 8.0f;


    Encoder_L_Filter->SampleValue = (float) qtimer_quad_get(QTIMER_3, Qtimer2_LSB);
    Encoder_R_Filter->SampleValue = (float) -qtimer_quad_get(QTIMER_1, Qtimer1_LSB);


    qtimer_quad_clear(QTIMER_1, Qtimer1_LSB);
    qtimer_quad_clear(QTIMER_3, Qtimer2_LSB);

    encoder_value_L = (int16) RCFilter(Encoder_L_Filter);
    encoder_value_R = (int16) RCFilter(Encoder_R_Filter);
}


void motorctrl() {
    int16 Angle = (int16) FastABS(Steer_PID->CurrentValue); // -13.0 <= angle <= 13

    if (Angle <= 3.5) {
        if (Steer_PID->CurrentValue > 0.0) //往右转
        {
            Motor_GO_L_PWM = (uint16) (15000 - Angle * 57.1);
            Motor_GO_R_PWM = (uint16) (15000 - Angle * 285.7);
        } else if (Steer_PID->CurrentValue < 0.0) //往左转
        {
            Motor_GO_L_PWM = (uint16) (15000 - Angle * 285.7);
            Motor_GO_R_PWM = (uint16) (15000 - Angle * 57.1);
        }
    } else if (Angle > 3.5 && Angle <= 6.5) {
        if (Steer_PID->CurrentValue > 0.0) //往右转
        {
            Motor_GO_L_PWM = (uint16) (14800 - (Angle - 3.5) * 66.6);
            Motor_GO_R_PWM = (uint16) (14800 - (Angle - 3.5) * 333.3);
        } else if (Steer_PID->CurrentValue < 0.0) //往左转
        {
            Motor_GO_L_PWM = (uint16) (14000 - (Angle - 3.5) * 333.3);
            Motor_GO_R_PWM = (uint16) (14800 - (Angle - 3.5) * 66.6);
        }
    } else if (Angle > 6.5 && Angle <= 9.5) {
        if (Steer_PID->CurrentValue > 0.0) //往右转
        {
            Motor_GO_L_PWM = (uint16) (14600 - (Angle - 6.5) * 66.6);
            Motor_GO_R_PWM = (uint16) (14000 - (Angle - 6.5) * 333.3);
        } else if (Steer_PID->CurrentValue < 0.0) //往左转
        {
            Motor_GO_L_PWM = (uint16) (14000 - (Angle - 6.5) * 333.3);
            Motor_GO_R_PWM = (uint16) (14600 - (Angle - 6.5) * 66.6);
        }
    } else if (Angle > 9.5 && Angle <= 13.0) {
        if (Steer_PID->CurrentValue > 0.0) //往右转
        {
            Motor_GO_L_PWM = (uint16) (14400 - (Angle - 9.5) * 57.1);
            Motor_GO_R_PWM = (uint16) (12000 - (Angle - 9.5) * 285.7);
        } else if (Steer_PID->CurrentValue < 0.0) //往左转
        {
            Motor_GO_L_PWM = (uint16) (12000 - (Angle - 9.5) * 285.7);
            Motor_GO_R_PWM = (uint16) (14400 - (Angle - 9.5) * 57.1);
        }
    } else {
        if (Steer_PID->CurrentValue > 0.0) //往右转
        {
            Motor_GO_L_PWM = (uint16) 15000;
            Motor_GO_R_PWM = (uint16) 8000;
        } else if (Steer_PID->CurrentValue < 0.0) //往左转
        {
            Motor_GO_L_PWM = (uint16) 8000;
            Motor_GO_R_PWM = (uint16) 12000;
        }
    }

    //速度预测,弯道降速，急弯减速
    if (FastABS(Steer_PID->Differential) >= 0.0 && FastABS(Steer_PID->Differential) < 0.5) {
        Motor_GO_L_PWM -= 0;
        Motor_GO_R_PWM -= 0;
    } else if (FastABS(Steer_PID->Differential) >= 0.5 && FastABS(Steer_PID->Differential) < 1.0) {
        Motor_GO_L_PWM -= 1000;
        Motor_GO_R_PWM -= 1000;
    } else if (FastABS(Steer_PID->Differential) >= 1.0 && FastABS(Steer_PID->Differential) < 2.0) {
        Motor_GO_L_PWM -= 2000;
        Motor_GO_R_PWM -= 2000;
    } else if (FastABS(Steer_PID->Differential) >= 2.0 && FastABS(Steer_PID->Differential) < 3.0) {
        Motor_GO_L_PWM -= 3000;
        Motor_GO_R_PWM -= 3000;
    } else {
        Motor_GO_L_PWM -= 4000;
        Motor_GO_R_PWM -= 4000;
    }


    //环岛减速
    if (Island_Flag) {
        Motor_GO_L_PWM -= 2000;
        Motor_GO_R_PWM -= 2000;
    }


    //电机保护
    if (Motor_GO_L_PWM > MotorPWM_MAX) {
        Motor_GO_L_PWM = MotorPWM_MAX;
    }
    if (Motor_GO_R_PWM < MotorPWM_MIN) {
        Motor_GO_L_PWM = MotorPWM_MIN;
    }

    if (Motor_GO_R_PWM > MotorPWM_MAX) {
        Motor_GO_R_PWM = MotorPWM_MAX;
    }
    if (Motor_GO_R_PWM < MotorPWM_MIN) {
        Motor_GO_R_PWM = MotorPWM_MIN;
    }

    //出轨保护
    if (InductanceValue_Normal[1] < 3.0 && InductanceValue_Normal[2] < 1.0 && InductanceValue_Normal[3] < 1.0 && InductanceValue_Normal[4] < 3.0
        && InductanceValue_Normal[0] < 0.5 && InductanceValue_Normal[5] < 0.5) {
        pwm_duty(MotorPWM_Go_L_CH, 0);
        pwm_duty(MotorPWM_Go_R_CH, 0);

    } else {

        pwm_duty(MotorPWM_Go_L_CH, Motor_GO_L_PWM);
        pwm_duty(MotorPWM_Go_R_CH, Motor_GO_R_PWM);
        garageout_flag = 0;

    }
}


void motorctrl_Faster() {
    int16 Angle = (int16) FastABS(Steer_PID->CurrentValue); // -13.0 <= angle <= 13

    if (Angle <= 3.5) {
        if (Steer_PID->CurrentValue > 0.0) //往右转
        {
            Motor_GO_L_PWM = (uint16) (16000 - Angle * 114.3);
            Motor_GO_R_PWM = (uint16) (16000 - Angle * 400.0);
        } else if (Steer_PID->CurrentValue < 0.0) //往左转
        {
            Motor_GO_L_PWM = (uint16) (16000 - Angle * 400.0);
            Motor_GO_R_PWM = (uint16) (16000 - Angle * 114.3);
        }
    } else if (Angle > 3.5 && Angle <= 6.5) {
        if (Steer_PID->CurrentValue > 0.0) //往右转
        {
            Motor_GO_L_PWM = (uint16) (15600 - (Angle - 3.5) * 133.3);
            Motor_GO_R_PWM = (uint16) (14600 - (Angle - 3.5) * 466.6);
        } else if (Steer_PID->CurrentValue < 0.0) //往左转
        {
            Motor_GO_L_PWM = (uint16) (14600 - (Angle - 3.5) * 466.6);
            Motor_GO_R_PWM = (uint16) (15600 - (Angle - 3.5) * 133.3);
        }
    } else if (Angle > 6.5 && Angle <= 9.5) {
        if (Steer_PID->CurrentValue > 0.0) //往右转
        {
            Motor_GO_L_PWM = (uint16) (15200 - (Angle - 6.5) * 133.3);
            Motor_GO_R_PWM = (uint16) (13200 - (Angle - 6.5) * 466.6);
        } else if (Steer_PID->CurrentValue < 0.0) //往左转
        {
            Motor_GO_L_PWM = (uint16) (13200 - (Angle - 6.5) * 466.6);
            Motor_GO_R_PWM = (uint16) (15200 - (Angle - 6.5) * 133.3);
        }
    } else if (Angle > 9.5 && Angle <= 13.0) {
        if (Steer_PID->CurrentValue > 0.0) //往右转
        {
            Motor_GO_L_PWM = (uint16) (14800 - (Angle - 9.5) * 114.3);
            Motor_GO_R_PWM = (uint16) (11800 - (Angle - 9.5) * 400.0);
        } else if (Steer_PID->CurrentValue < 0.0) //往左转
        {
            Motor_GO_L_PWM = (uint16) (11800 - (Angle - 9.5) * 400.0);
            Motor_GO_R_PWM = (uint16) (14800 - (Angle - 9.5) * 114.3);
        }
    } else if (Angle > 13.0 && Angle <= 16.0) {
        if (Steer_PID->CurrentValue > 0.0) //往右转
        {
            Motor_GO_L_PWM = (uint16) (14400 - (Angle - 13.0) * 133.3);
            Motor_GO_R_PWM = (uint16) (10400 - (Angle - 13.0) * 466.7);
        } else if (Steer_PID->CurrentValue < 0.0) //往左转
        {
            Motor_GO_L_PWM = (uint16) (10400 - (Angle - 13.0) * 466.7);
            Motor_GO_R_PWM = (uint16) (14400 - (Angle - 13.0) * 133.3);
        }
    } else {
        if (Steer_PID->CurrentValue > 0.0) //往右转
        {
            Motor_GO_L_PWM = (uint16) 14000;
            Motor_GO_R_PWM = (uint16) 9000;
        } else if (Steer_PID->CurrentValue < 0.0) //往左转
        {
            Motor_GO_L_PWM = (uint16) 9000;
            Motor_GO_R_PWM = (uint16) 14000;
        }
    }

    //速度预测,弯道降速，急弯减速
    if (FastABS(Steer_PID->Differential) >= 0.0 && FastABS(Steer_PID->Differential) < 0.5) {
        Motor_GO_L_PWM -= 0;
        Motor_GO_R_PWM -= 0;
    } else if (FastABS(Steer_PID->Differential) >= 0.5 && FastABS(Steer_PID->Differential) < 1.0) {
        Motor_GO_L_PWM -= 2000;
        Motor_GO_R_PWM -= 2000;
    } else if (FastABS(Steer_PID->Differential) >= 1.0 && FastABS(Steer_PID->Differential) < 2.0) {
        Motor_GO_L_PWM -= 3000;
        Motor_GO_R_PWM -= 3000;
    } else if (FastABS(Steer_PID->Differential) >= 2.0 && FastABS(Steer_PID->Differential) < 3.0) {
        Motor_GO_L_PWM -= 4000;
        Motor_GO_R_PWM -= 4000;
    } else {
        Motor_GO_L_PWM -= 5000;
        Motor_GO_R_PWM -= 5000;
    }


    //环岛减速
    if (Island_Flag) {
        Motor_GO_L_PWM -= 3000;
        Motor_GO_R_PWM -= 3000;
    }

    //动物停止30秒
    if (Camera == IsAnimal) {   //停车
        pwm_duty(MotorPWM_Go_L_CH, 0);
        pwm_duty(MotorPWM_Go_R_CH, 0);
        systick_delay_ms(3050);
    }

    //摄像头识别到二维码，先停车
    if (Camera == InLeft || Camera == InRight) {
        pwm_duty(MotorPWM_Go_L_CH, 0);
        pwm_duty(MotorPWM_Go_R_CH, 0);
    }
}


void motorctrl_test() {
    int16 Angle = (int16) FastABS(Steer_PID->CurrentValue); // -13.0 <= angle <= 13
    float Rate = 7.50f;
    //主要输出电机差速
    if (Angle <= 3.5) {
        if (Steer_PID->CurrentValue > 0.0) //往右转
        {
            Motor_GO_L_PWM = (uint16) (16000 - Angle * 63.1 * (Rate - 2.5));//63.1
            Motor_GO_R_PWM = (uint16) (16000 - Angle * 347.9 * (Rate - 2.5));//342.9
        } else if (Steer_PID->CurrentValue < 0.0) //往左转
        {
            Motor_GO_L_PWM = (uint16) (16000 - Angle * 347.9 * (Rate - 2.5));
            Motor_GO_R_PWM = (uint16) (16000 - Angle * 63.1 * (Rate - 2.5));
        }
    } else if (Angle > 3.5 && Angle <= 6.5) {
        if (Steer_PID->CurrentValue > 0.0) //往右转
        {
            Motor_GO_L_PWM = (uint16) (14800 - (Angle - 3.5) * 76.6 * (Rate - 1.0));//66.6
            Motor_GO_R_PWM = (uint16) (13800 - (Angle - 3.5) * 410.0 * (Rate - 1.0));//400
        } else if (Steer_PID->CurrentValue < 0.0) //往左转
        {
            Motor_GO_L_PWM = (uint16) (13800 - (Angle - 3.5) * 410.0 * (Rate - 1.0));
            Motor_GO_R_PWM = (uint16) (14800 - (Angle - 3.5) * 76.6 * (Rate - 1.0));
        }
    } else if (Angle > 6.5 && Angle <= 9.5) {
        if (Steer_PID->CurrentValue > 0.0) //往右转
        {
            Motor_GO_L_PWM = (uint16) (14600 - (Angle - 6.5) * 76.6 * (Rate));
            Motor_GO_R_PWM = (uint16) (12600 - (Angle - 6.5) * 410.0 * (Rate));
        } else if (Steer_PID->CurrentValue < 0.0) //往左转
        {
            Motor_GO_L_PWM = (uint16) (12600 - (Angle - 6.5) * 410.0 * Rate);
            Motor_GO_R_PWM = (uint16) (14600 - (Angle - 6.5) * 76.6 * Rate);
        }
    } else if (Angle > 9.5 && Angle <= 13.0) {
        if (Steer_PID->CurrentValue > 0.0) //往右转
        {
            Motor_GO_L_PWM = (uint16) (14400 - (Angle - 9.5) * 62.1 * (Rate + 1));
            Motor_GO_R_PWM = (uint16) (11400 - (Angle - 9.5) * 347.9 * (Rate + 1));
        } else if (Steer_PID->CurrentValue < 0.0) //往左转
        {
            Motor_GO_L_PWM = (uint16) (11400 - (Angle - 9.5) * 347.9 * (Rate + 2));
            Motor_GO_R_PWM = (uint16) (14400 - (Angle - 9.5) * 63.1 * (Rate + 2));
        }
    }// else {
    //     if (Steer_PID->CurrentValue > 0.0) //往右转
    //     {
    //         Motor_GO_L_PWM = (uint16) 12200;
    //         Motor_GO_R_PWM = (uint16) 8200;
    //     } else if (Steer_PID->CurrentValue < 0.0) //往左转
    //     {
    //         Motor_GO_L_PWM = (uint16) 8200;
    //         Motor_GO_R_PWM = (uint16) 12200;
    //     }
    // }


    //速度预测,弯道降速，急弯减速
    if (FastABS(Steer_PID->Differential) >= 0.0 && FastABS(Steer_PID->Differential) < 0.5) {
        Motor_GO_L_PWM -= 0;
        Motor_GO_R_PWM -= 0;
    } else if (FastABS(Steer_PID->Differential) >= 0.5 && FastABS(Steer_PID->Differential) < 1.0) {
        Motor_GO_L_PWM -= 200;
        Motor_GO_R_PWM -= 200;
    } else if (FastABS(Steer_PID->Differential) >= 1.0 && FastABS(Steer_PID->Differential) < 2.0) {
        Motor_GO_L_PWM -= 500;
        Motor_GO_R_PWM -= 500;
    } else if (FastABS(Steer_PID->Differential) >= 2.0 && FastABS(Steer_PID->Differential) < 3.0) {
        Motor_GO_L_PWM -= 1000;
        Motor_GO_R_PWM -= 1000;
    } else {
        Motor_GO_L_PWM -= 1000;
        Motor_GO_R_PWM -= 1000;
    }


    //环岛减速
    if (Island_Flag) {
        Motor_GO_L_PWM -= 2000;
        Motor_GO_R_PWM -= 2000;
    }
    //
    // Motor_GO_L_PWM += 1000;
    // Motor_GO_R_PWM += 1000;
    //电机保护
    if (Motor_GO_L_PWM > MotorPWM_MAX) {
        Motor_GO_L_PWM = MotorPWM_MAX;
    }
    if (Motor_GO_R_PWM < MotorPWM_MIN) {
        Motor_GO_L_PWM = MotorPWM_MIN;
    }

    if (Motor_GO_R_PWM > MotorPWM_MAX) {
        Motor_GO_R_PWM = MotorPWM_MAX;
    }
    if (Motor_GO_R_PWM < MotorPWM_MIN) {
        Motor_GO_R_PWM = MotorPWM_MIN;
    }

    //出轨保护
    if (InductanceValue_Normal[1] < 3.0 && InductanceValue_Normal[2] < 1.0 && InductanceValue_Normal[3] < 1.0 && InductanceValue_Normal[4] < 3.0
        && InductanceValue_Normal[0] < 0.5 && InductanceValue_Normal[5] < 0.5) {
        pwm_duty(MotorPWM_Go_L_CH, 0);
        pwm_duty(MotorPWM_Go_R_CH, 0);

    } else {

        pwm_duty(MotorPWM_Go_L_CH, Motor_GO_L_PWM);
        pwm_duty(MotorPWM_Go_R_CH, Motor_GO_R_PWM);
        garageout_flag = 0;

    }
}

