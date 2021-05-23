//
// Created by xcs on 2021-03-06.
//

#include "isr.h"
#include "ADC.h"
#include "Uart.h"
#include "Text.h"


extern PID_Struct Motor_GOL_PID;
extern PID_Struct Motor_GOR_PID;
extern int16 encoder_value_L;
extern int16 encoder_value_R;
extern uint16 Motor_GO_L_PWM;
extern uint16 Motor_GO_R_PWM;

extern PID_Struct Steer_PID;
extern Filter_Struct Steer_Filter;

extern Filter_Struct Motor_GOL_Filter;
extern Filter_Struct Motor_GOR_Filter;


void CSI_IRQHandler(void) {
    CSI_DriverIRQHandler();
    __DSB();
}

void PIT_IRQHandler(void) {
    if (PIT_FLAG_GET(PIT_CH0)) {

        Get_InductanceValue();
        SteerCtrl(Steer_PID, Steer_Filter);
        Motor_value_get();
        MotorCtrl(Motor_GOL_PID, Motor_GOR_PID,Motor_GOL_Filter,Motor_GOR_Filter);

        //GetCameraMessage();
        PIT_FLAG_CLEAR(PIT_CH0);
    }

    if (PIT_FLAG_GET(PIT_CH1)) {
        PIT_FLAG_CLEAR(PIT_CH1);
    }

    if (PIT_FLAG_GET(PIT_CH2)) {
        PIT_FLAG_CLEAR(PIT_CH2);
    }

    if (PIT_FLAG_GET(PIT_CH3)) {
        PIT_FLAG_CLEAR(PIT_CH3);
    }

    __DSB();
}