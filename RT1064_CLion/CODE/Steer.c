//
// Created by xcs on 2021-03-03.
//

#include "Steer.h"
#include "ADC.h"
#include "port.h"
#include "zf_pwm.h"

struct PID_Parameter Steer_PID_Parameter = {0};
struct Filter_Parameter Steer_Filter_Parameter = {0};
uint8 Trident_Flag = 0;
uint16 SteerPWMDuty = MiddleSteer_PWM;

void Steer_PIDStruct_Init(PID_Struct Steer_PID, Filter_Struct Steer_Filter) {
    Steer_PID->ResultMax = (SteerMAX - MiddleSteer_PWM) * 50;
    Steer_PID->ResultMin = (MiddleSteer_PWM - SteerMIN) * 50;
    Steer_PID->I_MAX = Steer_IMAX;
    Steer_PID->TargetValue = 0;
    // Steer_Filter->Coefficient=         RC��ͨ�˲����˲�ϵ��������
}

void SteerCtrl(PID_Struct Steer_PID, Filter_Struct Steer_Filter) {
    Steer_PID->CurrentValue = InductanceValueHandler();
    SteerPWMDuty = MiddleSteer_PWM + ((int16) PIDCalculate(Steer_PID, Steer_Filter) / 50);
    //�ο�ʦ�ֵĴ��룬�����ת��ΪPWM��50���ܺ���Ҫ��
    pwm_duty(SteerPWM_CH, SteerPWMDuty);//���PWM���ƶ����ǣ�������PWM���ֵĴ�������
}
