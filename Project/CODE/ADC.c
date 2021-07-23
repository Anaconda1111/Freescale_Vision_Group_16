//
// Created by xcs on 2021-03-03.
//

#include "ADC.h"
#include "fastmath.h"
#include "port.h"
#include "PID.h"

uint16 Inductance_ADCValue[InductanceNum][SamplingNum] = {0};
float InductanceValue_Normal[InductanceNum] = {0};

float Inductance_MAXValue[InductanceNum] ={2250,4025,3877,3738,4003,3598};
float Inductance_InitialValue[InductanceNum] ={0,175,77,148,113,43};
float Inductance_Weight[6] = {0};

extern PID_Struct Steer_PID;


void Get_InductanceValue() {
    for (int i = 0; i < SamplingNum; ++i) {
        Inductance_ADCValue[0][i] = adc_convert(ADC_1, Steer_ADCInput1_CH);
        Inductance_ADCValue[1][i] = adc_convert(ADC_1, Steer_ADCInput2_CH);
        Inductance_ADCValue[2][i] = adc_convert(ADC_1, Steer_ADCInput3_CH);
        Inductance_ADCValue[3][i] = adc_convert(ADC_1, Steer_ADCInput4_CH);
        Inductance_ADCValue[4][i] = adc_convert(ADC_1, Steer_ADCInput5_CH);
        Inductance_ADCValue[5][i] = adc_convert(ADC_1, Steer_ADCInput6_CH);
    }

    //电感值排序，冒泡排序
    for(int i=0;i < InductanceNum;i++)
    {
        for(int j=0;j < SamplingNum;j++)
        {
            for(int k=0;k< SamplingNum-j-1;k++)
            {

                if(Inductance_ADCValue[i][k]>Inductance_ADCValue[i][k+1])
                {
                    int temp=Inductance_ADCValue[i][k];
                    Inductance_ADCValue[i][k]=Inductance_ADCValue[i][k+1];
                    Inductance_ADCValue[i][k+1]=temp;
                }
            }
        }
    }


    //电感值求和：掐头去尾
    uint16 InductanceValue_Sum[InductanceNum] = {0};
    for (int i = 0; i < InductanceNum; ++i) {
        for (int j = 2; j < SamplingNum-2; ++j) {
            InductanceValue_Sum[i] += Inductance_ADCValue[i][j];
        }
    }

    //电感值均值
    uint16 InductanceValue_Average[InductanceNum] = {0};
    for (int i = 0; i < InductanceNum; ++i) {
        InductanceValue_Average[i] = InductanceValue_Sum[i] / (SamplingNum - 4);
    }



    //电感值归一化：

    //法1：当前值 - 初值 / 最大值 - 初值  *100
    for (int i = 0; i < InductanceNum; ++i) {
        InductanceValue_Normal[i] = (float) InductanceValue_Average[i] - (float) Inductance_InitialValue[i];//初值调整
    }


    for (int i = 0; i < InductanceNum; ++i) {
        InductanceValue_Normal[i] = (float) InductanceValue_Normal[i] / ((float) Inductance_MAXValue[i]-(float) Inductance_InitialValue[i]);
        InductanceValue_Normal[i] *=100;
    }




}










