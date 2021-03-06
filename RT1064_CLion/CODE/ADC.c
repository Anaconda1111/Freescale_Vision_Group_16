//
// Created by xcs on 2021-03-03.
//

#include "ADC.h"
#include "zf_iomuxc.h"

#define ADC_PIN_CONF SPEED_100MHZ | DSE_R0_6 //配置ADC引脚默认配置
uint16 Inductance_ADCValue[InductanceNum][SamplingNum];
float InductanceValue_Normal[InductanceNum] = {0};
uint16 Inductance_MAXValue[InductanceNum] = {3600, 3600, 3600, 3600, 7200};
uint16 MiddleInductance = 0;

float Inductance_Weight[5] = {0};

uint8 Camera = 0;
int8 Island_Flag = -1;

uint16 User_ADC_Convert(ADCN_enum ADCn, ADCPIN_enum ADCPIN) {
    adc_channel_config_t adcChannelConfigStruct;

    adcChannelConfigStruct.channelNumber = ADCPIN;
    adcChannelConfigStruct.enableInterruptOnConversionCompleted = false;

    ADC_SetChannelConfig(ADCN[ADCn], 0, &adcChannelConfigStruct);
    while (0U == ADC_GetChannelStatusFlags(ADCN[ADCn], 0));
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
    for (int i = 0; i < InductanceNum; ++i) {
        InductanceValue_Normal[i] =
                (float) InductanceValue_Average[i] / (float) Inductance_MAXValue[i];
    }
    MiddleInductance = InductanceValue_Normal[4]; //中间电感值做检测环岛用


    for (int i = 0; i < InductanceNum; ++i) { //给电感值加权重
        InductanceValue_Average[i] *= Inductance_Weight[i];
    }

    float InductanceV1, InductanceV2;
    InductanceV1 = FastSqrt(InductanceValue_Normal[0] * InductanceValue_Normal[0] + InductanceValue_Normal[1] * InductanceValue_Normal[1]);
    InductanceV2 = FastSqrt(InductanceValue_Normal[2] * InductanceValue_Normal[2] + InductanceValue_Normal[3] * InductanceValue_Normal[3]);
    float Current_Value = ((FastSqrt(InductanceV1) - FastSqrt(InductanceV2)) / (InductanceV1 + InductanceV2) * 100);
    return Current_Value;
}

float FastSqrt(float x) {
    float a = x;
    unsigned int i = *(unsigned int *) &x;
    i = (i + 0x3f76cf62) >> 1UL;
    x = *(float *) &i;
    x = (x + a / x) * 0.5f;
    return x;
}

void Setting_Weight() {
    if (Island_Flag > 0) {
        Inductance_Weight[0] = (float) Island_L1 / 100.0f;
        Inductance_Weight[1] = (float) Island_L2 / 100.0f;
        Inductance_Weight[2] = (float) Island_R2 / 100.0f;
        Inductance_Weight[3] = (float) Island_R1 / 100.0f;
        Inductance_Weight[4] = (float) Island_M / 100.0f;
    } else if (Camera) {
        if (Camera == Trident_Left) {
            Inductance_Weight[0] = (float) Trident1 / 100.0f;
            Inductance_Weight[1] = (float) Trident2 / 100.0f;
            Inductance_Weight[2] = (float) Trident4 / 100.0f;
            Inductance_Weight[3] = (float) Trident3 / 100.0f;
            Inductance_Weight[4] = (float) TridentM / 100.0f;
        } else if (Camera == Trident_Right) {
            Inductance_Weight[0] = (float) Trident3 / 100.0f;
            Inductance_Weight[1] = (float) Trident4 / 100.0f;
            Inductance_Weight[2] = (float) Trident2 / 100.0f;
            Inductance_Weight[3] = (float) Trident1 / 100.0f;
            Inductance_Weight[4] = (float) TridentM / 100.0f;
        }
    } else if (FastABS(InductanceValue_Normal[1] - InductanceValue_Normal[2]) >= Curve_threshold) {
        Inductance_Weight[0] = (float) Curve_L1 / 100.0f;
        Inductance_Weight[3] = (float) Curve_R1 / 100.0f;
        Inductance_Weight[4] = (float) Curve_M / 100.0f;
        float i = (float) (100 - Curve_L1 - Curve_R1 - Curve_M) / 100.f;
        Inductance_Weight[1] = (InductanceValue_Normal[1] / (InductanceValue_Normal[1] + InductanceValue_Normal[2])) * i;
        Inductance_Weight[2] = (InductanceValue_Normal[2] / (InductanceValue_Normal[1] + InductanceValue_Normal[2])) * i;
    } else {
        Inductance_Weight[0] = (float) Straight_L1 / 100.0f;
        Inductance_Weight[1] = (float) Straight_L2 / 100.0f;
        Inductance_Weight[2] = (float) Straight_R2 / 100.0f;
        Inductance_Weight[3] = (float) Straight_R1 / 100.0f;
        Inductance_Weight[4] = (float) Straight_M / 100.0f;
    }
}