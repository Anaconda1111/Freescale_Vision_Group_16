//
// Created by xcs on 2021-03-03.
//

#ifndef RT1064_CODE_ADC_H_
#define RT1064_CODE_ADC_H_

#define InductanceNum 6 //电感个数
#define SamplingNum 10   //采集多少组数据
#define Trident_Left 2
#define Trident_Right 1

#define Curve_threshold 10
#define Island_threshold 200 //未知待定



typedef enum {
    Island_R1 = 100,
    Island_R2 = 100,
    Island_M1 = 100,
    Island_M2 = 100,
    Island_L2 = 100,
    Island_L1 = 100,


    Straight_L1 = 100,
    Straight_L2 = 45,
    Straight_M1 = 45,
    Straight_M2 = 45,
    Straight_R2 = 45,
    Straight_R1 = 100,

    Curve_L1 = 100,
    Curve_L2 = 100,
    Curve_M1 = 100,
    Curve_M2 = 100,
    Curve_R2 = 100,
    Curve_R1 = 100,


    Trident1 = 100,
    Trident2 = 100,
    Trident3 = 100,
    Trident4 = 75,
    Trident5 = 75,
    Trident6 = 100,

    Base_L1 = 100,
    Base_L2 = 100,
    Base_M1 = 100,
    Base_M2 = 100,
    Base_R2 = 100,
    Base_R1 = 100

} Weight_enum;

void Get_InductanceValue();


float InductanceValueHandler();


void Setting_Weight();

#endif // RT1064_CODE_ADC_H_
