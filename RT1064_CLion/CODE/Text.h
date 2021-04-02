//
// Created by xcs on 2021-03-09.
//

#ifndef RT1064_TEXT_H
#define RT1064_TEXT_H

#include "PID.h"
#include "zf_gpio.h"
#include "zf_systick.h"
#include "Text.h"
#include "Motor.h"
#include "port.h"
#include "Steer.h"
#include "Uart.h"
#include "SEEKFREE_OLED.h"

void LED();

void MotorText();

void SteerText();

void MatlabText();

void DRAWCURVE(struct PID_Parameter T_PID, struct Filter_Parameter T_Filter);

void Sendint16Data();

#endif //RT1064_TEXT_H
