//
// Created by xcs on 2021-03-24.
//

#ifndef RT1064_ART_H
#define RT1064_ART_H

#include "Uart.h"

#define FindAprilTagInLeft   5
#define FindAprilTagInRight  6

#define MidYunTai1SteerPWM 0
#define MidYunTai2SteerPWM 0


int16 YunTaiPWMCalculate(uint8 Z_rotation);

#endif //RT1064_ART_H
