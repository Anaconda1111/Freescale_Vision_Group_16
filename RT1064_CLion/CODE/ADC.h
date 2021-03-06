//
// Created by xcs on 2021-03-03.
//

#ifndef RT1064_CODE_ADC_H_
#define RT1064_CODE_ADC_H_

#define FastABS(x) (x > 0 ? x : x * -1.0f)

#define InductanceNum 5 //电感个数
#define SamplingNum 5   //采集多少组数据
#define Trident_Left 2
#define Trident_Right 1

#define Curve_threshold 0
#define Island_threshold 0

#include "zf_adc.h"

extern ADC_Type *ADCN[];
typedef enum {
    ADCInput1,
    ADCInput2,
    ADCInput3,
    ADCInput4,
    ADCInput5,
    ADCInput6,
    ADCInput7,
    ADCInput8,
    ADCInput9,
    ADCInput10,
} ADCPIN_enum;

typedef enum {
    Island_L1 = 10,
    Island_L2 = 35,
    Island_R1 = 10,
    Island_R2 = 35,
    Island_M = 10,


    Straight_L1 = 24,
    Straight_L2 = 14,
    Straight_R1 = 24,
    Straight_R2 = 14,
    Straight_M = 24,


    Curve_L1 = 18,
    Curve_R1 = 18,
    Curve_M = 18,

    Trident1 = 34,
    Trident2 = 24,
    TridentM = 24,
    Trident3 = 14,
    Trident4 = 04
} Weight_enum;

uint16 User_ADC_Convert(ADCN_enum ADCn, ADCPIN_enum ADCPIN);

float InductanceValueHandler();

float FastSqrt(float x);

void Trident();

#endif // RT1064_CODE_ADC_H_
