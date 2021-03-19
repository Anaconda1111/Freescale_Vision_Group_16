//
// Created by xcs on 2021-03-09.
//
#include "zf_gpio.h"
#include "zf_systick.h"
#include "Text.h"
#include "Motor.h"
#include "port.h"
#include "Steer.h"
#include "Uart.h"

void LED() {
    gpio_set(B9, 1);
    systick_delay_ms(100);
    gpio_set(B9, 0);
    systick_delay_ms(100);
}

void MotorText() {
    pwm_init(MotorPWM_Go_L_CH, 13 * 1000, 0);
    pwm_init(MotorPWM_Go_R_CH, 13 * 1000, 0);
    pwm_init(MotorPWM_Return_L_CH, 13 * 1000, 0);
    pwm_init(MotorPWM_Return_R_CH, 13 * 1000, 0);
    pwm_duty(MotorPWM_Go_L_CH, 12500);
    pwm_duty(MotorPWM_Return_L_CH, 0);
    pwm_duty(MotorPWM_Go_R_CH, 12500);
    pwm_duty(MotorPWM_Return_R_CH, 0);
}

void SteerText() {
    pwm_init(SteerPWM_CH, 50, MiddleSteer_PWM);
    systick_delay_ms(2000);
    pwm_duty(SteerPWM_CH, SteerOutGarage_PWM);
}

void MatlabText() {
    for (int i = 0; i < 100; ++i) {
        uint8 TV = i;
        uint8 CV = i + 1;
        SendDataPackage(CV, TV);
    }
}


void DRAWCURVE(PID_Struct T_PID, Filter_Struct T_Filter) {
    uint16 i = 10000;
    while (i > 0) {
        if (i % 2 == 0) {
            if (T_PID->TargetValue == 0)
                T_PID->TargetValue = 100;
            else
                T_PID->TargetValue = 0;
        }
        int16 TargetValue = (int16) (T_PID->TargetValue);
        int16 CurrentValue = (int16) (T_PID->CurrentValue);
        int16 Data[4] = {CurrentValue, TargetValue, 0, 0};
        SendDataPackage_int16(Data, 4);
        i--;
    }
}

void Sendint16Data() {
    int16 Dtat[4] = {100, -100, 200, -200};
    SendDataPackage_int16(Dtat, 4);
}