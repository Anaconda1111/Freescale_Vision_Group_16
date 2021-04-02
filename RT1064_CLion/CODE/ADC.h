//
// Created by xcs on 2021-03-03.
//

#ifndef RT1064_CODE_ADC_H_
#define RT1064_CODE_ADC_H_


#define InductanceNum 6 //电感个数
#define SamplingNum 7   //采集多少组数据
#define Trident_Left 2
#define Trident_Right 1

#define Curve_threshold 10000
#define Island_threshold 10000


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

void Get_InductanceValue();


float InductanceValueHandler();


void Setting_Weight();

#endif // RT1064_CODE_ADC_H_
