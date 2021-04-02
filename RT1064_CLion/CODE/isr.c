//
// Created by xcs on 2021-03-06.
//

#include "isr.h"
#include "ADC.h"
#include "Uart.h"
#include "Text.h"

void CSI_IRQHandler(void) {
    CSI_DriverIRQHandler();
    __DSB();
}

void PIT_IRQHandler(void) {
    if (PIT_FLAG_GET(PIT_CH0)) {
        Get_InductanceValue();

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