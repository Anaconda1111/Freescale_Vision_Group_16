//
// Created by xcs on 2021-03-06.
//

#include "isr.h"
#include "ADC.h"
#include "Uart.h"
#include "Text.h"
#include "Interactive.h"
#include "ART.h"

extern PID_Struct Steer_PID;
extern Filter_Struct Steer_Filter;

extern PID_Struct Motor_GOL_PID;
extern PID_Struct Motor_GOR_PID;
extern Filter_Struct Motor_GOL_Filter;
extern Filter_Struct Motor_GOR_Filter;

void CSI_IRQHandler(void) {
    CSI_DriverIRQHandler();
    __DSB();
}

void PIT_IRQHandler(void) {
    if (PIT_FLAG_GET(PIT_CH0)) {

        InductanceValueFilter();
        GetCameraMessage();
        TridentMessageHandle();
        // AprilTagMessageHandle();
        GarageIn();
        SteerCtrl(Steer_PID, Steer_Filter);
        //InductanceValueRCFilter();
        PIT_FLAG_CLEAR(PIT_CH0);
    }

    if (PIT_FLAG_GET(PIT_CH1)) {
        Motor_value_get();
        Interactive();
        PIT_FLAG_CLEAR(PIT_CH1);
    }

    if (PIT_FLAG_GET(PIT_CH2)) {
        MotorCtrl(Motor_GOL_PID, Motor_GOR_PID, Motor_GOL_Filter, Motor_GOR_Filter);
        PIT_FLAG_CLEAR(PIT_CH2);
    }

    if (PIT_FLAG_GET(PIT_CH3)) {
        PIT_FLAG_CLEAR(PIT_CH3);
    }

    __DSB();
}