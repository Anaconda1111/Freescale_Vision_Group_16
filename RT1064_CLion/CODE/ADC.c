//
// Created by xcs on 2021-03-03.
//

#include "ADC.h"
#include "port.h"
#include "zf_iomuxc.h"

#define ADC_PIN_CONF SPEED_100MHZ | DSE_R0_6 //配置ADC引脚默认配置

uint16 Inductance_ADCValue[InductanceNum][SamplingNum];
uint16 Inductance_MAXValue[InductanceNum] = {3600, 3600, 3600, 3600, 7200};
uint16 MiddleInductance = 0;
void User_ADC_Init(ADCN_enum ADCn, ADCPIN_enum ADCPIN, ADCRES_enum resolution) {
  switch (ADCPIN) {
  case ADCInput1:
    iomuxc_pinconf(Steer_ADCInput1, ALT5, ADC_PIN_CONF);
    break;
  case ADCInput2:
    iomuxc_pinconf(Steer_ADCInput2, ALT5, ADC_PIN_CONF);
    break;
  case ADCInput3:
    iomuxc_pinconf(Steer_ADCInput3, ALT5, ADC_PIN_CONF);
    break;
  case ADCInput4:
    iomuxc_pinconf(Steer_ADCInput4, ALT5, ADC_PIN_CONF);
    break;
  case ADCInput5:
    iomuxc_pinconf(Steer_ADCInput5, ALT5, ADC_PIN_CONF);
    break;
  case ADCInput6:
    iomuxc_pinconf(Steer_ADCInput6, ALT5, ADC_PIN_CONF);
    break;
  case ADCInput7:
    iomuxc_pinconf(Steer_ADCInput7, ALT5, ADC_PIN_CONF);
    break;
  case ADCInput8:
    iomuxc_pinconf(Steer_ADCInput8, ALT5, ADC_PIN_CONF);
    break;
  case ADCInput9:
    iomuxc_pinconf(Steer_ADCInput9, ALT5, ADC_PIN_CONF);
    break;
  case ADCInput10:
    iomuxc_pinconf(Steer_ADCInput10, ALT5, ADC_PIN_CONF);
    break;
  default:
    break;
  }
  adc_init(ADCn, resolution);
}

uint16 User_ADC_Convert(ADCN_enum ADCn, ADCPIN_enum ADCPIN) {
  adc_channel_config_t adcChannelConfigStruct;

  adcChannelConfigStruct.channelNumber = ADCPIN;
  adcChannelConfigStruct.enableInterruptOnConversionCompleted = false;

  ADC_SetChannelConfig(ADCN[ADCn], 0, &adcChannelConfigStruct);
  while (0U == ADC_GetChannelStatusFlags(ADCN[ADCn], 0))
    ;
  return ADC_GetChannelConversionValue(ADCN[ADCn], 0);
}

void Get_InductanceValue() {
  for (int i = 0; i < SamplingNum; ++i) {
    Inductance_ADCValue[0][i] = User_ADC_Convert(ADC_2, ADCInput1);
    Inductance_ADCValue[1][i] = User_ADC_Convert(ADC_2, ADCInput2);
    Inductance_ADCValue[2][i] = User_ADC_Convert(ADC_2, ADCInput3);
    Inductance_ADCValue[3][i] = User_ADC_Convert(ADC_2, ADCInput4);
    Inductance_ADCValue[4][i] = User_ADC_Convert(ADC_2, ADCInput5);
  }
}

float InductanceValueHandler() {
  Get_InductanceValue();

  //将电感值最大最小值去掉
  for (int i = 0; i < InductanceNum; ++i) {
    uint8 MAX_index = 0, MIN_index = 0;
    uint16 MAX_Value = 0, MIN_Value = 3600;
    for (int j = 0; j < SamplingNum; ++j) {
      if (Inductance_ADCValue[i][j] > MAX_Value) {
        MAX_Value = Inductance_ADCValue[i][j];
        MAX_index = j;
      }
      if (Inductance_ADCValue[i][j] < MIN_Value) {
        MIN_Value = Inductance_ADCValue[i][j];
        MIN_index = j;
      }
    }
    Inductance_ADCValue[i][MAX_index] = 0;
    Inductance_ADCValue[i][MIN_index] = 0;
  }

  //电感值求和：掐头去尾
  uint16 InductanceValue_Sum[InductanceNum] = {0};
  for (int i = 0; i < InductanceNum; ++i) {
    for (int j = 0; j < InductanceNum; ++j) {
      InductanceValue_Sum[i] += Inductance_ADCValue[i][j];
    }
  }

  //电感值均值
  uint16 InductanceValue_Average[InductanceNum] = {0};
  for (int i = 0; i < InductanceNum; ++i) {
    InductanceValue_Average[i] = InductanceValue_Sum[i] / (SamplingNum - 2);
  }

  //电感值归一化：
  float InductanceValue_Normal[InductanceNum] = {0};
  for (int i = 0; i < InductanceNum; ++i) {
    InductanceValue_Normal[i] =
        (float)InductanceValue_Average[i] / (float)Inductance_MAXValue[i];
  }
  MiddleInductance = InductanceValue_Normal[4]; //中间电感值做检测环岛用

  float InductanceV1, InductanceV2;

  InductanceV1 =
      FastSqrt(InductanceValue_Normal[0] * InductanceValue_Normal[0] +
               InductanceValue_Normal[1] * InductanceValue_Normal[1]);

  InductanceV2 =
      FastSqrt(InductanceValue_Normal[2] * InductanceValue_Normal[2] +
               InductanceValue_Normal[3] * InductanceValue_Normal[3]);

  float Current_Value = ((FastSqrt(InductanceV1) - FastSqrt(InductanceV2)) /
                         (InductanceV1 + InductanceV2) * 100);

  return Current_Value;
}

float FastSqrt(float x) {
  float a = x;
  unsigned int i = *(unsigned int *)&x;
  i = (i + 0x3f76cf62) >> 1UL;
  x = *(float *)&i;
  x = (x + a / x) * 0.5f;
  return x;
}