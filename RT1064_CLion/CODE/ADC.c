//
// Created by xcs on 2021-03-03.
//

#include "ADC.h"
#include "fastmath.h"
#include "port.h"
#include "PID.h"
#include "zf_systick.h"
#include "ART.h"
#include "zf_gpio.h"
#include "ANO_DT.h"

uint16 Inductance_ADCValue[InductanceNum][SamplingNum] = {0};
uint16 Inductance_FilterValue[InductanceNum] = {0};
uint16 Inductance_LastValue[InductanceNum] = {0};
uint16 Inductance_SampleValue[InductanceNum] = {0};
uint8 ADCFilterCoefficient = 1;//表示2的n次方
uint16 InductanceValue_Average[InductanceNum] = {0};
float InductanceValue_Normal[InductanceNum] = {0};

float Inductance_MAXValue[InductanceNum] = {2730, 3285, 3720, 3450, 2870, 2850};
float Inductance_InitialValue[InductanceNum] = {0, 200, 95, 170, 126, 47};

float Inductance_Weight[InductanceNum] = {0};

extern PID_Struct Steer_PID;
extern uint8 Camera;
extern uint16 TridentTurn;
int16 Island_Flag = 0;
uint8 LeftIsland = 0;
uint8 RightIsland = 0;
uint32 TridentFlag = 0;
float Current_Value;


void InductanceValueRCFilter() {
    Inductance_SampleValue[0] = adc_convert(ADC_1, Steer_ADCInput1_CH);
    Inductance_SampleValue[1] = adc_convert(ADC_1, Steer_ADCInput2_CH);
    Inductance_SampleValue[2] = adc_convert(ADC_1, Steer_ADCInput3_CH);
    Inductance_SampleValue[3] = adc_convert(ADC_1, Steer_ADCInput4_CH);
    Inductance_SampleValue[4] = adc_convert(ADC_1, Steer_ADCInput5_CH);
    Inductance_SampleValue[5] = adc_convert(ADC_1, Steer_ADCInput6_CH);
    for (int i = 0; i < InductanceNum; ++i) {
        Inductance_FilterValue[i] = (Inductance_SampleValue[i] >> ADCFilterCoefficient) +
                                    (Inductance_LastValue[i] >> ADCFilterCoefficient);
        Inductance_LastValue[i] = Inductance_SampleValue[i];
    }
}

void Get_InductanceValue() {
    for (int i = 0; i < SamplingNum; ++i) {
        Inductance_ADCValue[0][i] = adc_convert(ADC_1, Steer_ADCInput1_CH);
        Inductance_ADCValue[1][i] = adc_convert(ADC_1, Steer_ADCInput2_CH);
        Inductance_ADCValue[2][i] = adc_convert(ADC_1, Steer_ADCInput3_CH);
        Inductance_ADCValue[3][i] = adc_convert(ADC_1, Steer_ADCInput4_CH);
        Inductance_ADCValue[4][i] = adc_convert(ADC_1, Steer_ADCInput5_CH);
        Inductance_ADCValue[5][i] = adc_convert(ADC_1, Steer_ADCInput6_CH);
    }
/*
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

*/

}

void InductanceValueFilter() {
    Get_InductanceValue();
    //电感值排序，冒泡排序
    for (int i = 0; i < InductanceNum; i++) {
        for (int j = 0; j < SamplingNum; j++) {
            for (int k = 0; k < SamplingNum - j - 1; k++) {

                if (Inductance_ADCValue[i][k] > Inductance_ADCValue[i][k + 1]) {
                    int temp = Inductance_ADCValue[i][k];
                    Inductance_ADCValue[i][k] = Inductance_ADCValue[i][k + 1];
                    Inductance_ADCValue[i][k + 1] = temp;
                }
            }
        }
    }


    //电感值求和：掐头去尾
    uint16 InductanceValue_Sum[InductanceNum] = {0};
    for (int i = 0; i < InductanceNum; ++i) {
        for (int j = 2; j < SamplingNum - 2; ++j) {
            InductanceValue_Sum[i] += Inductance_ADCValue[i][j];
        }
    }

    //电感值均值

    for (int i = 0; i < InductanceNum; ++i) {
        InductanceValue_Average[i] = InductanceValue_Sum[i] / (SamplingNum - 4);
    }



    //电感值归一化：

    //法1：当前值 - 初值 / 最大值 - 初值  *100
    for (int i = 0; i < InductanceNum; ++i) {
        InductanceValue_Normal[i] = (float) InductanceValue_Average[i] - (float) Inductance_InitialValue[i];//初值调整
    }


    for (int i = 0; i < InductanceNum; ++i) {
        InductanceValue_Normal[i] = (float) InductanceValue_Normal[i] / ((float) Inductance_MAXValue[i] - (float) Inductance_InitialValue[i]);
        InductanceValue_Normal[i] *= 100;
    }
}

float InductanceValueHandler() {
    IslandTest();
    Setting_Weight();


    for (int i = 0; i < InductanceNum; ++i) { //给电感值加权重
        InductanceValue_Normal[i] *= Inductance_Weight[i];
    }
    float V1, V2, V3, V4;
    float InductanceV1, InductanceV2;

    // V1 = FastABS(InductanceValue_Normal[0] - InductanceValue_Normal[5]);
    // V2 = FastABS(InductanceValue_Normal[2] - InductanceValue_Normal[3]);
    // V3 = FastABS((InductanceValue_Normal[0] + InductanceValue_Normal[2]) - (InductanceValue_Normal[3] + InductanceValue_Normal[5]));
    // V4 = (2 * (InductanceValue_Normal[1] - InductanceValue_Normal[5])+(InductanceValue_Normal[2]-InductanceValue_Normal[3])) /
    //             (2*(InductanceValue_Normal[1]+))
    InductanceV1 = FastSqrt(InductanceValue_Normal[1] * InductanceValue_Normal[1] + InductanceValue_Normal[2] * InductanceValue_Normal[2] + InductanceValue_Normal[0] * InductanceValue_Normal[0]);
    InductanceV2 = FastSqrt(InductanceValue_Normal[3] * InductanceValue_Normal[3] + InductanceValue_Normal[4] * InductanceValue_Normal[4] + InductanceValue_Normal[5] * InductanceValue_Normal[5]);
    Current_Value = ((FastSqrt(InductanceV1) - FastSqrt(InductanceV2)) / (InductanceV1 + InductanceV2) * 100);
    // ANO_DT_send_int16(USART_8, (int16) V1, (int16) V2, (int16) V3, 0, 0, 0, 0, 0);

    // if (Island_Flag>0) {
    //     if (InductanceValue_Normal[0] > InductanceValue_Normal[5])//往右转,右环岛
    //     {
    //         if (InductanceValue_Normal[2] > InductanceValue_Normal[3]) {
    //             float InductanceV1, InductanceV2;
    //             InductanceV1 = FastSqrt(InductanceValue_Normal[2] * InductanceValue_Normal[2] + InductanceValue_Normal[0] * InductanceValue_Normal[0]);
    //             InductanceV2 = FastSqrt(InductanceValue_Normal[3] * InductanceValue_Normal[3] + InductanceValue_Normal[5] * InductanceValue_Normal[5]);
    //             Current_Value = (InductanceV1 - InductanceV2) / (InductanceV1 + InductanceV2) * 100;
    //         }
    //         /*
    //                    else
    //                    {
    //                      float InductanceV1, InductanceV2;
    //                      InductanceV1 = FastSqrt(InductanceValue_Normal[1] * InductanceValue_Normal[1] + InductanceValue_Normal[2] * InductanceValue_Normal[2] + InductanceValue_Normal[0] * InductanceValue_Normal[0]);
    //                      InductanceV2 = FastSqrt(InductanceValue_Normal[3] * InductanceValue_Normal[3] + InductanceValue_Normal[4] * InductanceValue_Normal[4] + InductanceValue_Normal[5] * InductanceValue_Normal[5]);
    //                      Current_Value = ((FastSqrt(InductanceV1) - FastSqrt(InductanceV2)) / (InductanceV1 + InductanceV2) * 100);
    //                    }
    //      */
    //     } else if (InductanceValue_Normal[0] < InductanceValue_Normal[5])//往左转，左环岛
    //     {
    //         if (InductanceValue_Normal[2] < InductanceValue_Normal[3]) {
    //             float InductanceV1, InductanceV2;
    //             InductanceV1 = FastSqrt(InductanceValue_Normal[2] * InductanceValue_Normal[2] + InductanceValue_Normal[0] * InductanceValue_Normal[0]);
    //             InductanceV2 = FastSqrt(InductanceValue_Normal[3] * InductanceValue_Normal[3] + InductanceValue_Normal[5] * InductanceValue_Normal[5]);
    //             Current_Value = (InductanceV1 - InductanceV2) / (InductanceV1 + InductanceV2) * 100;
    //         }
    //         /*
    //                      else
    //                      {
    //                        float InductanceV1, InductanceV2;
    //                        InductanceV1 = FastSqrt(InductanceValue_Normal[1] * InductanceValue_Normal[1] + InductanceValue_Normal[2] * InductanceValue_Normal[2] + InductanceValue_Normal[0] * InductanceValue_Normal[0]);
    //                        InductanceV2 = FastSqrt(InductanceValue_Normal[3] * InductanceValue_Normal[3] + InductanceValue_Normal[4] * InductanceValue_Normal[4] + InductanceValue_Normal[5] * InductanceValue_Normal[5]);
    //                        Current_Value = ((FastSqrt(InductanceV1) - FastSqrt(InductanceV2)) / (InductanceV1 + InductanceV2) * 100);
    //                      }
    //        */
    //     } else {
    //         float InductanceV1, InductanceV2;
    //         InductanceV1 = FastSqrt(InductanceValue_Normal[1] * InductanceValue_Normal[1] + InductanceValue_Normal[2] * InductanceValue_Normal[2] + InductanceValue_Normal[0] * InductanceValue_Normal[0]);
    //         InductanceV2 = FastSqrt(InductanceValue_Normal[3] * InductanceValue_Normal[3] + InductanceValue_Normal[4] * InductanceValue_Normal[4] + InductanceValue_Normal[5] * InductanceValue_Normal[5]);
    //         Current_Value = ((FastSqrt(InductanceV1) - FastSqrt(InductanceV2)) / (InductanceV1 + InductanceV2) * 100);
    //     }
    // } else  //一般情况
    // {
    //     float InductanceV1, InductanceV2;
    //     InductanceV1 = FastSqrt(InductanceValue_Normal[1] * InductanceValue_Normal[1] + InductanceValue_Normal[2] * InductanceValue_Normal[2] + InductanceValue_Normal[0] * InductanceValue_Normal[0]);
    //     InductanceV2 = FastSqrt(InductanceValue_Normal[3] * InductanceValue_Normal[3] + InductanceValue_Normal[4] * InductanceValue_Normal[4] + InductanceValue_Normal[5] * InductanceValue_Normal[5]);
    //     Current_Value = ((FastSqrt(InductanceV1) - FastSqrt(InductanceV2)) / (InductanceV1 + InductanceV2) * 100);
    // }
    // //出轨时角度维持输出
    // if (InductanceValue_Normal[1] < 3.0 && InductanceValue_Normal[2] < 1.0 && InductanceValue_Normal[3] < 1.0 && InductanceValue_Normal[4] < 3.0
    //     && InductanceValue_Normal[0] < 0.5 && InductanceValue_Normal[5] < 0.5) {
    //     Current_Value = -(Steer_PID->D_LastError);//D_LastError = 0 - Current_Value;
    //
    //     if (Current_Value > 0.0) {
    //         Current_Value = 13.0f;
    //     } else {
    //         Current_Value = -13.0f;
    //     }
    // }
    if (Current_Value > 0)
        Current_Value *= 1.5f;
    else if (Current_Value < 0)
        Current_Value *= 1.2f;

    if (TridentFlag)
        if (InductanceValue_Normal[2] + InductanceValue_Normal[3] < 30)
            Current_Value /= 2.0f;
    return Current_Value;
}


void Setting_Weight() {
    if (Island_Flag > 0) {
        if (InductanceValue_Normal[0] >= 26 && LeftIsland != 1)
            RightIsland = 1;
        else if (InductanceValue_Normal[5] >= 5 && RightIsland != 1)
            LeftIsland = 1;

        if (RightIsland) {
            Inductance_Weight[0] = (float) RIsland1 / 100.0f;
            Inductance_Weight[1] = (float) RIsland2 / 100.0f;
            Inductance_Weight[2] = (float) RIsland3 / 100.0f;
            Inductance_Weight[3] = (float) RIsland4 / 100.0f;
            Inductance_Weight[4] = (float) RIsland5 / 100.0f;
            Inductance_Weight[5] = (float) RIsland6 / 100.0f;
        } else if (LeftIsland) {
            Inductance_Weight[0] = (float) LIsland6 / 100.0f;
            Inductance_Weight[1] = (float) LIsland5 / 100.0f;
            Inductance_Weight[2] = (float) LIsland4 / 100.0f;
            Inductance_Weight[3] = (float) LIsland3 / 100.0f;
            Inductance_Weight[4] = (float) LIsland2 / 100.0f;
            Inductance_Weight[5] = (float) LIsland1 / 100.0f;
        }

    } else if (TridentTurn) {
        TridentTurn--;
        if (Camera == Trident_Left) {
            Inductance_Weight[0] = (float) Trident6 / 100.0f;
            Inductance_Weight[1] = (float) Trident5 / 100.0f;
            Inductance_Weight[2] = (float) Trident4 / 100.0f;
            Inductance_Weight[3] = (float) Trident3 / 100.0f;
            Inductance_Weight[4] = (float) Trident2 / 100.0f;
            Inductance_Weight[5] = (float) Trident1 / 100.0f;
        } else if (Camera == Trident_Right) {
            Inductance_Weight[0] = (float) Trident1 / 100.0f;
            Inductance_Weight[1] = (float) Trident2 / 100.0f;
            Inductance_Weight[2] = (float) Trident3 / 100.0f;
            Inductance_Weight[3] = (float) Trident4 / 100.0f;
            Inductance_Weight[4] = (float) Trident5 / 100.0f;
            Inductance_Weight[5] = (float) Trident6 / 100.0f;
        }
    } else {
        Inductance_Weight[0] = (float) Base_R1 / 100.0f;
        Inductance_Weight[1] = (float) Base_R2 / 100.0f;
        Inductance_Weight[2] = (float) Base_M1 / 100.0f;
        Inductance_Weight[3] = (float) Base_M2 / 100.0f;
        Inductance_Weight[4] = (float) Base_L2 / 100.0f;
        Inductance_Weight[5] = (float) Base_L1 / 100.0f;
    }

}

void IslandTest() {
    if (TridentFlag == 1)
        return;
    if (Island_Flag != 1) {
        if (FastABS(InductanceValue_Normal[2] + InductanceValue_Normal[3]) >= Island_threshold) {
            Island_Flag = 1;
            gpio_set(BEEF, 1);
        }
    } else if (FastABS(InductanceValue_Normal[2] + InductanceValue_Normal[3]) <= 80) {
        if (InductanceValue_Normal[0] < 2 && LeftIsland == 1) {
            Island_Flag = 0;
            LeftIsland = 0;
            systick_delay_ms(50);
            gpio_set(BEEF, 0);
        } else if (InductanceValue_Normal[0] < 15 && RightIsland == 1) {
            Island_Flag = 0;
            RightIsland = 0;
            gpio_set(BEEF, 0);
        }
    }

}

int TridentTest() {
    if (TridentFlag) {
        TridentFlag--;
        gpio_set(BEEF, 1);
        return 0;

    } else
        gpio_set(BEEF, 0);

    if (InductanceValue_Normal[2] <= 25 && InductanceValue_Normal[3] <= 25 &&
        (InductanceValue_Normal[1] + InductanceValue_Normal[4]) > 60 && FastABS(Steer_PID->CurrentValue) <= 2
        && FastABS(InductanceValue_Normal[2] - InductanceValue_Normal[3] < 10)) {
        return 1;
        //gpio_set(BEEF, 1);
    } else
        return 0;

}

unsigned char StraightLine() {
    if (InductanceValue_Normal[0] <= 2 && InductanceValue_Normal[5] < 2 && Steer_PID->CurrentValue <= 3)
        return 1;
    else
        return 0;
}