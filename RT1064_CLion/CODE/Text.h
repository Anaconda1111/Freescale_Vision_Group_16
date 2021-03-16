//
// Created by xcs on 2021-03-09.
//

#ifndef RT1064_TEXT_H
#define RT1064_TEXT_H

#include "PID.h"

void LED();

void MotorText();

void SteerText();

void MatlabText();

void DRAWCURVE(PID_Struct T_PID, Filter_Struct T_Filter);

#endif //RT1064_TEXT_H
