//
// Created by xcs on 2021-03-24.
//

#ifndef RT1064_ART_H
#define RT1064_ART_H

#include "Uart.h"


#define MidYunTai2SteerPWM 3430
#define MaxYunTai2SteerPWM 6000
#define MinYunTai2SteerPWM 1270

#define Trident_Left 2
#define Trident_Right 1
#define IsFruit  3
#define IsAnimal 4
#define InLeft  5
#define InRight 6
#define Find_Trident 7
#define TridentOut 8
#define Reversing 9
#define FindFinishLine 10

void TridentMessageHandle();

void AprilTagMessageHandle();

void SlowDown();

void SlowDown2();

void GarageIn();

void Reversing_();

#endif //RT1064_ART_H
