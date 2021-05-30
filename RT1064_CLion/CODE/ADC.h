// //
// // Created by xcs on 2021-03-03.
// //

#ifndef RT1064_CODE_ADC_H_
#define RT1064_CODE_ADC_H_

#define InductanceNum 6 //电感个数
#define SamplingNum 10   //采集多少组数据

#define Curve_threshold 10
#define Island_threshold 155


typedef enum {
    LIsland1 = 300,
    LIsland2 = 300,
    LIsland3 = 250,
    LIsland4 = 25,
    LIsland5 = 25,
    LIsland6 = 0,

    RIsland1 = 300,
    RIsland2 = 300,
    RIsland3 = 300,
    RIsland4 = 10,
    RIsland5 = 10,
    RIsland6 = 10,


    Trident1 = 300,
    Trident2 = 300,
    Trident3 = 300,
    Trident4 = 0,
    Trident5 = 0,
    Trident6 = 0,

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

void InductanceValueRCFilter();

void IslandTest();

void InductanceValueFilter();

int TridentTest();

unsigned char StraightLine();

#endif // RT1064_CODE_ADC_H_
