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
#include "SEEKFREE_OLED.h"
#include "ANO_DT.h"
#include "ADC.h"

extern uint16 Inductance_FilterValue[InductanceNum];
extern uint16 Inductance_SampleValue[InductanceNum];
extern uint16 InductanceValue_Average[InductanceNum];

void LED() {
    gpio_set(B9, 1);
    systick_delay_ms(1000);
    gpio_set(B9, 0);
    systick_delay_ms(1000);
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


void Sendint16Data() {
    uint8 i = 0;
    int16 data1 = (int16) Inductance_SampleValue[i];
    int16 data2 = (int16) Inductance_FilterValue[i];
    int16 data3 = (int16) InductanceValue_Average[i];
    ANO_DT_send_int16(USART_8, data1, data2, data3, 0, 0, 0, 0, 0);
}

