//
// Created by xcs on 2021-03-03.
//

#include "ADC.h"
#include "port.h"
#include "zf_iomuxc.h"

#define ADC_PIN_CONF SPEED_100MHZ | DSE_R0_6 //≈‰÷√ADC“˝Ω≈ƒ¨»œ≈‰÷√

uint16 Inductance_ADCValue[InductanceNum][SamplingNum];
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