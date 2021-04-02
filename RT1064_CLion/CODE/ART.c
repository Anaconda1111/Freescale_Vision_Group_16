//
// Created by xcs on 2021-03-24.
//

#include "ART.h"
#include "port.h"
#include "zf_pwm.h"

extern uint8 Camera;

uint16 YunTai1SteerPWMDuty = 0;
uint16 YunTai2SteerPWMDuty = 0;

void AprilTagMessageHandle() {
    uint8 Z_rotation = 0;
    if (Camera == FindAprilTagInLeft) {
        uart_getchar(USART_6, &Z_rotation);
        YunTai2SteerPWMDuty += YunTaiPWMCalculate(Z_rotation);
    } else if (Camera == FindAprilTagInRight) {
        uart_getchar(USART_6, &Z_rotation);
        YunTai2SteerPWMDuty += YunTaiPWMCalculate(Z_rotation);
    }
    pwm_duty(Yuntai2PWM_CH, YunTai2SteerPWMDuty);
}

int16 YunTaiPWMCalculate(uint8 Z_rotation) {
    return 0;
}