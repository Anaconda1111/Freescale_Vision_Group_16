//
// Created by xcs on 2021-03-03.
//

#include "ADC.h"
#include "fastmath.h"
#include "port.h"
#include "PID.h"

uint16 Inductance_ADCValue[InductanceNum][SamplingNum] = {0};
float InductanceValue_Normal[InductanceNum] = {0};

float Inductance_MAXValue[InductanceNum] ={2730,3285,3720,3450,2870,2850};
float Inductance_InitialValue[InductanceNum] ={0,125,77,136,48,43};

float Inductance_Weight[6] = {0};

extern PID_Struct Steer_PID;
extern uint8 Camera;

int16 Island_Flag =0;
int16 Trident_Flag =0;



float Current_Value;

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

float InductanceValueHandler() {


    Setting_Weight();

    for (int i = 0; i < InductanceNum; ++i) { //给电感值加权重
        InductanceValue_Normal[i] *= Inductance_Weight[i];
    }



    if(Island_Flag)
    {
        if(InductanceValue_Normal[0] > InductanceValue_Normal[5])//往右转,右环岛
        {
            if(InductanceValue_Normal[2] > InductanceValue_Normal[3])
            {
                float InductanceV1, InductanceV2;
                InductanceV1 = FastSqrt(InductanceValue_Normal[2] * InductanceValue_Normal[2]+ InductanceValue_Normal[0] * InductanceValue_Normal[0]);
                InductanceV2 = FastSqrt(InductanceValue_Normal[3] * InductanceValue_Normal[3]+ InductanceValue_Normal[5] * InductanceValue_Normal[5]);
                Current_Value =(InductanceV1 - InductanceV2) / (InductanceV1 + InductanceV2) * 100;
            }
            /*
                       else
                       {
                         float InductanceV1, InductanceV2;
                         InductanceV1 = FastSqrt(InductanceValue_Normal[1] * InductanceValue_Normal[1] + InductanceValue_Normal[2] * InductanceValue_Normal[2] + InductanceValue_Normal[0] * InductanceValue_Normal[0]);
                         InductanceV2 = FastSqrt(InductanceValue_Normal[3] * InductanceValue_Normal[3] + InductanceValue_Normal[4] * InductanceValue_Normal[4] + InductanceValue_Normal[5] * InductanceValue_Normal[5]);
                         Current_Value = ((FastSqrt(InductanceV1) - FastSqrt(InductanceV2)) / (InductanceV1 + InductanceV2) * 100);
                       }
         */
        }
        else if(InductanceValue_Normal[0] < InductanceValue_Normal[5])//往左转，左环岛
        {
            if(InductanceValue_Normal[2] < InductanceValue_Normal[3])
            {
                float InductanceV1, InductanceV2;
                InductanceV1 = FastSqrt(InductanceValue_Normal[2] * InductanceValue_Normal[2]+ InductanceValue_Normal[0] * InductanceValue_Normal[0]);
                InductanceV2 = FastSqrt(InductanceValue_Normal[3] * InductanceValue_Normal[3]+ InductanceValue_Normal[5] * InductanceValue_Normal[5]);
                Current_Value =(InductanceV1 - InductanceV2) / (InductanceV1 + InductanceV2) * 100;
            }
            /*
                         else
                         {
                           float InductanceV1, InductanceV2;
                           InductanceV1 = FastSqrt(InductanceValue_Normal[1] * InductanceValue_Normal[1] + InductanceValue_Normal[2] * InductanceValue_Normal[2] + InductanceValue_Normal[0] * InductanceValue_Normal[0]);
                           InductanceV2 = FastSqrt(InductanceValue_Normal[3] * InductanceValue_Normal[3] + InductanceValue_Normal[4] * InductanceValue_Normal[4] + InductanceValue_Normal[5] * InductanceValue_Normal[5]);
                           Current_Value = ((FastSqrt(InductanceV1) - FastSqrt(InductanceV2)) / (InductanceV1 + InductanceV2) * 100);
                         }
           */
        }
        else
        {
            float InductanceV1, InductanceV2;
            InductanceV1 = FastSqrt(InductanceValue_Normal[1] * InductanceValue_Normal[1] + InductanceValue_Normal[2] * InductanceValue_Normal[2] + InductanceValue_Normal[0] * InductanceValue_Normal[0]);
            InductanceV2 = FastSqrt(InductanceValue_Normal[3] * InductanceValue_Normal[3] + InductanceValue_Normal[4] * InductanceValue_Normal[4] + InductanceValue_Normal[5] * InductanceValue_Normal[5]);
            Current_Value = ((FastSqrt(InductanceV1) - FastSqrt(InductanceV2)) / (InductanceV1 + InductanceV2) * 100);
        }
    }
    else  //一般情况
    {
        float InductanceV1, InductanceV2;
        InductanceV1 = FastSqrt(InductanceValue_Normal[1] * InductanceValue_Normal[1] + InductanceValue_Normal[2] * InductanceValue_Normal[2] + InductanceValue_Normal[0] * InductanceValue_Normal[0]);
        InductanceV2 = FastSqrt(InductanceValue_Normal[3] * InductanceValue_Normal[3] + InductanceValue_Normal[4] * InductanceValue_Normal[4] + InductanceValue_Normal[5] * InductanceValue_Normal[5]);
        Current_Value = ((FastSqrt(InductanceV1) - FastSqrt(InductanceV2)) / (InductanceV1 + InductanceV2) * 100);
    }
    //出轨时角度维持输出
    if(InductanceValue_Normal[1] < 3.0 && InductanceValue_Normal[2] <1.0 && InductanceValue_Normal[3] <1.0  && InductanceValue_Normal[4] <3.0
       && InductanceValue_Normal[0] <0.5 && InductanceValue_Normal[5] <0.5)
    {
        Current_Value = -(Steer_PID->D_LastError);//D_LastError = 0 - Current_Value;

        if(Current_Value > 0.0)
        {
            Current_Value = 13.0f;
        }
        else
        {
            Current_Value = -13.0f;
        }
    }

    return Current_Value;
}


void Setting_Weight() {
    if (FastABS(InductanceValue_Normal[1] + InductanceValue_Normal[4]) >= Island_threshold) {  //入环岛
        Inductance_Weight[0] = (float) Island_R1 / 100.0f;
        Inductance_Weight[1] = (float) Island_R2 / 100.0f;
        Inductance_Weight[2] = (float) Island_M1 / 100.0f;
        Inductance_Weight[3] = (float) Island_M2 / 100.0f;
        Inductance_Weight[4] = (float) Island_L2 / 100.0f;
        Inductance_Weight[5] = (float) Island_L1 / 100.0f;
        Island_Flag = 1;

    } else if (Camera) {
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
        Island_Flag=0;
        Camera = 0;
        Trident_Flag =0;
    }

}








