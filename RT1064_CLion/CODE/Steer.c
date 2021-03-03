//
// Created by xcs on 2021-03-03.
//

#include "Steer.h"
#include "zf_pwm.h"
struct PID_Parameter Steer_PID_Parameter = {0};
struct Filter_Parameter Steer_Filter_Parameter = {0};
uint16 SteerPWM = MiddleSteer_PWM;
void Steer_PIDStruct_Init(PID_Struct Steer_PID, Filter_Struct Steer_Filter) {
  Steer_PID->ResultMax = SteerMAX;
  Steer_PID->ResultMin = SteerMIN;
  Steer_PID->I_MAX = Steer_IMAX;
  Steer_PID->TargetValue = 0;
  // Steer_Filter->Coefficient=         RC��ͨ�˲����˲�ϵ��������
}

void SteerCtrl(PID_Struct Steer_PID, Filter_Struct Steer_Filter) {
  // Steer_PID->CurrentValue=
  SteerPWM =
      MiddleSteer_PWM + ((int16)PIDCalculate(Steer_PID, Steer_Filter) /
                         50); //�ο�ʦ�ֵĴ��룬�����ת��ΪPWM��50���ܺ���Ҫ��
  // pwm_duty()    //���PWM���ƶ����ǣ�������PWM���ֵĴ�������
}