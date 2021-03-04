//
// Created by xcs on 2021-03-03.
//

#ifndef RT1064_CODE_ADC_H_
#define RT1064_CODE_ADC_H_
#include "zf_adc.h"
#define InductanceNum 5 //电感个数
#define SamplingNum 5   //采集多少组数据
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

void User_ADC_Init(ADCN_enum ADCn, ADCPIN_enum ADCPIN, ADCRES_enum resolution);
uint16 User_ADC_Convert(ADCN_enum ADCn, ADCPIN_enum ADCPIN);
float InductanceValueHandler();
float FastSqrt(float x);
#endif // RT1064_CODE_ADC_H_
