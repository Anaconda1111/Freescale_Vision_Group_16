//
// Created by xcs on 2021-03-05.
//

#include "Motor.h"
#include "PID.h"
#include "Steer.h"
#include "zf_qtimer.h"
#include "port.h"

struct PID_Parameter Motor_GOL_PID_Parameter = {0};
struct PID_Parameter Motor_GOR_PID_Parameter = {0};
struct Filter_Parameter Motor_GOL_Filter_Parameter = {0};
struct Filter_Parameter Motor_GOR_Filter_Parameter = {0};

uint16 Motor_GO_L_PWM;
uint16 Motor_GO_R_PWM;

void Motor_PIDStruct_Init(PID_Struct Motor_GOL_PID, PID_Struct Motor_GOR_PID,
                          Filter_Struct Motor_GOR_Filter,
                          Filter_Struct Motor_GOL_Filter) {
    Motor_GOL_PID->I_MAX = MotorI_MAX;
    Motor_GOR_PID->I_MAX = MotorI_MAX;
    // Motor_GOL_Filter->Coefficient=
    // Motor_GOR_Filter->Coefficient=
}

int16 GetDifferentSpeed(int16 Angle) {
    if (Angle <= 3)
        return 0;
    else
        return Angle * Transform;
}

void MotorCtrl(PID_Struct Motor_GOL_PID, PID_Struct Motor_GOR_PID, Filter_Struct Motor_GOL_Filter, Filter_Struct Motor_GOR_Filter) {
    if (InductanceValue_Normal[0] < 0.01 && InductanceValue_Normal[1] < 0.01 &&
        InductanceValue_Normal[2] < 0.01 && InductanceValue_Normal[3] < 0.01) {
        pwm_duty(MotorPWM_Go_L_CH, 0);
        pwm_duty(MotorPWM_Go_R_CH, 0);
        return;
    }
    int16 base_speed, different_speed;
    int16 Angle = MiddleSteer_PWM - SteerPWMDuty;
    if (Angle <= 3)
        base_speed = MAXEncoder;
    else
        base_speed = MAXEncoder / 2;
    different_speed = GetDifferentSpeed(Angle);
    Motor_GOL_PID->TargetValue = (float) (base_speed - different_speed);
    Motor_GOR_PID->TargetValue = (float) (base_speed + different_speed);
    Motor_GOL_PID->CurrentValue = qtimer_quad_get(QTIMER_1, Qtimer1_LSB);
    qtimer_quad_clear(QTIMER_1, Qtimer1_LSB);
    Motor_GOR_PID->CurrentValue = qtimer_quad_get(QTIMER_3, Qtimer2_LSB);
    qtimer_quad_clear(QTIMER_3, Qtimer2_LSB);
    Motor_GO_L_PWM += (uint16) (PIDCalculate(Motor_GOL_PID, Motor_GOL_Filter)/*/*转换系数*/);
    Motor_GO_R_PWM += (uint16) (PIDCalculate(Motor_GOR_PID, Motor_GOR_Filter)/*/*转换系数*/);
    pwm_duty(MotorPWM_Go_L_CH, Motor_GO_L_PWM);
    pwm_duty(MotorPWM_Go_R_CH, Motor_GO_R_PWM);
}