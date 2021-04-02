//
// Created by xcs on 2021-03-03.
//

#include "ADC.h"
#include "fastmath.h"
#include "port.h"

uint16 Inductance_ADCValue[InductanceNum][SamplingNum] = {0};
float InductanceValue_Normal[InductanceNum] = {0};
uint16 Inductance_MAXValue[InductanceNum] = {1, 1, 1, 1, 1, 1};
uint16 InductanceValue_Average[InductanceNum] = {0};
float Inductance_Weight[5] = {0};


extern uint8 Camera;
int8 Island_Flag = -1;


void Get_InductanceValue() {
    for (int i = 0; i < SamplingNum; ++i) {
        // Inductance_ADCValue[0][i] = adc_convert(ADC_1, Steer_ADCInput1_CH);
        // Inductance_ADCValue[1][i] = adc_convert(ADC_1, Steer_ADCInput2_CH);
        // Inductance_ADCValue[2][i] = adc_convert(ADC_1, Steer_ADCInput3_CH);
        // Inductance_ADCValue[3][i] = adc_convert(ADC_1, Steer_ADCInput4_CH);
        // Inductance_ADCValue[4][i] = adc_convert(ADC_1, Steer_ADCInput5_CH);
        Inductance_ADCValue[0][i] = adc_convert(ADC_1, Steer_ADCInput2_CH);//右1（竖）
        Inductance_ADCValue[1][i] = adc_convert(ADC_1, Steer_ADCInput4_CH);//右2（横）
        Inductance_ADCValue[2][i] = adc_convert(ADC_1, Steer_ADCInput5_CH);//右3（内八）
        Inductance_ADCValue[3][i] = adc_convert(ADC_1, Steer_ADCInput6_CH);//左3（内八）
        Inductance_ADCValue[4][i] = adc_convert(ADC_1, Steer_ADCInput8_CH);//左2（横）
        Inductance_ADCValue[5][i] = adc_convert(ADC_1, Steer_ADCInput9_CH);//左1（竖）
    }
    //将电感值最大最小值去掉
    for (int i = 0; i < InductanceNum; ++i) {
        uint8 MAX_index = 0, MIN_index = 0;
        uint16 MAX_Value = 0, MIN_Value = 10000;
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

    uint16 InductanceValue_Sum[InductanceNum] = {0};
    //电感值求和：掐头去尾
    for (int i = 0; i < InductanceNum; ++i) {
        for (int j = 0; j < SamplingNum; ++j) {
            InductanceValue_Sum[i] += Inductance_ADCValue[i][j];
        }
    }

    //电感值均值
    for (int i = 0; i < InductanceNum; ++i) {
        InductanceValue_Average[i] = InductanceValue_Sum[i] / (SamplingNum - 2);
    }

    //电感值归一化：
    for (int i = 0; i < InductanceNum; ++i) {
        InductanceValue_Normal[i] = ((float) InductanceValue_Average[i] / (float) Inductance_MAXValue[i]);//放大;
    }

    //检测环岛
    // float MidInductanceValue = InductanceValue_Normal[4];
    // if ((MidInductanceValue > Island_threshold) || InductanceValue_Normal[0] > InductanceValue_Normal[1])
    //     Island_Flag *= -1;
}

float InductanceValueHandler() {
    // Setting_Weight();
    // for (int i = 0; i < InductanceNum; ++i) { //给电感值加权重
    //     InductanceValue_Normal[i] *= Inductance_Weight[i];
    // }

    float InductanceV1, InductanceV2, Current_Value;
    // InductanceV1 = FastSqrt(InductanceValue_Normal[0] * InductanceValue_Normal[0] + InductanceValue_Normal[1] * InductanceValue_Normal[1]);
    // InductanceV2 = FastSqrt(InductanceValue_Normal[2] * InductanceValue_Normal[2] + InductanceValue_Normal[3] * InductanceValue_Normal[3]);
    // float Current_Value = ((FastSqrt(InductanceV1) - FastSqrt(InductanceV2)) / (InductanceV1 + InductanceV2));
    InductanceV1 = InductanceValue_Normal[0] + InductanceValue_Normal[1] + InductanceValue_Normal[2];
    InductanceV2 = InductanceValue_Normal[3] + InductanceValue_Normal[4] + InductanceValue_Normal[5];
    Current_Value = (InductanceV2 - InductanceV1) /*/ (InductanceV1 + InductanceV2)*/;
    // if (FastABS(InductanceV1 - InductanceV2) < 0.001)
    //     Current_Value /= 100.0f;
    // else
    //     Current_Value *= 100;
    return Current_Value;
}


void Setting_Weight() {
    if (Island_Flag > 0) {
        Inductance_Weight[0] = (float) Island_L1 / 10.0f;
        Inductance_Weight[1] = (float) Island_L2 / 10.0f;
        Inductance_Weight[2] = (float) Island_R2 / 10.0f;
        Inductance_Weight[3] = (float) Island_R1 / 10.0f;
        Inductance_Weight[4] = (float) Island_M / 10.0f;
    } else if (Camera) {
        if (Camera == Trident_Left) {
            Inductance_Weight[0] = (float) Trident1 / 10.0f;
            Inductance_Weight[1] = (float) Trident2 / 10.0f;
            Inductance_Weight[2] = (float) Trident4 / 10.0f;
            Inductance_Weight[3] = (float) Trident3 / 10.0f;
            Inductance_Weight[4] = (float) TridentM / 10.0f;
        } else if (Camera == Trident_Right) {
            Inductance_Weight[0] = (float) Trident3 / 10.0f;
            Inductance_Weight[1] = (float) Trident4 / 10.0f;
            Inductance_Weight[2] = (float) Trident2 / 10.0f;
            Inductance_Weight[3] = (float) Trident1 / 10.0f;
            Inductance_Weight[4] = (float) TridentM / 10.0f;
        }
    } else if (FastABS(InductanceValue_Normal[1] - InductanceValue_Normal[2]) >= Curve_threshold) {
        Inductance_Weight[0] = (float) Curve_L1 / 10.0f;
        Inductance_Weight[3] = (float) Curve_R1 / 10.0f;
        Inductance_Weight[4] = (float) Curve_M / 10.0f;
        float i = (float) (100 - Curve_L1 - Curve_R1 - Curve_M);
        Inductance_Weight[1] = (InductanceValue_Normal[1] / (InductanceValue_Normal[1] + InductanceValue_Normal[2])) * i / 10.0f;
        Inductance_Weight[2] = (InductanceValue_Normal[2] / (InductanceValue_Normal[1] + InductanceValue_Normal[2])) * i / 10.0f;
    } else {
        Inductance_Weight[0] = (float) Straight_L1 / 10.0f;
        Inductance_Weight[1] = (float) Straight_L2 / 10.0f;
        Inductance_Weight[2] = (float) Straight_R2 / 10.0f;
        Inductance_Weight[3] = (float) Straight_R1 / 10.0f;
        Inductance_Weight[4] = (float) Straight_M / 10.0f;
    }
}










