//
// Created by xcs on 2021-03-05.
//

#include "Motor.h"
#include "PID.h"
#include "Steer.h"
#include "zf_qtimer.h"

struct PID_Parameter Motor_GO_L_PWM = {0};
struct PID_Parameter Motor_GO_R_PWM = {0};
struct Filter_Parameter Motor_GO_L_Filter = {0};
struct Filter_Parameter Motor_GO_R_Filter = {0};

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

void MotorCtrl(PID_Struct Motor_GOL_PID, PID_Struct Motor_GOR_PID,
               Filter_Struct Motor_GOL_Filter, Filter_Struct Motor_GOR_Filter) {
  if (InductanceValue_Normal[0] < 0.01 && InductanceValue_Normal[1] < 0.01 &&
      InductanceValue_Normal[2] < 0.01 && InductanceValue_Normal[3] < 0.01) {
    //    pwm_duty(,0);
    //    pwm_duty(,0);
    //    return;
  }
  int16 base_speed, different_speed;
  int16 Angle = MiddleSteer_PWM - SteerPWMDuty;
  if (Angle <= 3)
    base_speed = MAXEncoder;
  else
    base_speed = MAXEncoder / 2;
  different_speed = GetDifferentSpeed(Angle);
  Motor_GOL_PID->TargetValue = (float)(base_speed - different_speed);
  Motor_GOR_PID->TargetValue = (float)(base_speed + different_speed);
  //  Motor_GOL_PID->CurrentValue=qtimer_quad_get();
  //  qtimer_quad_clear();
  //  Motor_GOR_PID->CurrentValue=qtimer_quad_get();
  //  qtimer_quad_clear();
  //  Motor_GO_L_PWM+=(uint16)(PIDCalculate(Motor_GOL_PID,Motor_GOL_Filter)*/*转换系数*/);
  //  Motor_GO_R_PWM+=(uint16)(PIDCalculate(Motor_GOR_PID,Motor_GOR_Filter)*/*转换系数*/);
  //  pwm_duty(,Motor_GO_L_PWM);
  //  pwm_duty(,Motor_GO_R_PWM);
}