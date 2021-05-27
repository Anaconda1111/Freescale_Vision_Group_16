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
    //�����ֵ�����Сֵȥ��
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
    //���ֵ����ð������
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


    //���ֵ��ͣ���ͷȥβ
    uint16 InductanceValue_Sum[InductanceNum] = {0};
    for (int i = 0; i < InductanceNum; ++i) {
        for (int j = 2; j < SamplingNum-2; ++j) {
            InductanceValue_Sum[i] += Inductance_ADCValue[i][j];
        }
    }

    //���ֵ��ֵ
    uint16 InductanceValue_Average[InductanceNum] = {0};
    for (int i = 0; i < InductanceNum; ++i) {
        InductanceValue_Average[i] = InductanceValue_Sum[i] / (SamplingNum - 4);
    }



    //���ֵ��һ����

    //��1����ǰֵ - ��ֵ / ���ֵ - ��ֵ  *100
    for (int i = 0; i < InductanceNum; ++i) {
        InductanceValue_Normal[i] = (float) InductanceValue_Average[i] - (float) Inductance_InitialValue[i];//��ֵ����
    }


    for (int i = 0; i < InductanceNum; ++i) {
        InductanceValue_Normal[i] = (float) InductanceValue_Normal[i] / ((float) Inductance_MAXValue[i]-(float) Inductance_InitialValue[i]);
        InductanceValue_Normal[i] *=100;
    }




}

float InductanceValueHandler() {


    Setting_Weight();

    for (int i = 0; i < InductanceNum; ++i) { //�����ֵ��Ȩ��
        InductanceValue_Normal[i] *= Inductance_Weight[i];
    }




        float InductanceV1, InductanceV2;
        InductanceV1 = FastSqrt(InductanceValue_Normal[1] * InductanceValue_Normal[1] + InductanceValue_Normal[2] * InductanceValue_Normal[2] + InductanceValue_Normal[0] * InductanceValue_Normal[0]);
        InductanceV2 = FastSqrt(InductanceValue_Normal[3] * InductanceValue_Normal[3] + InductanceValue_Normal[4] * InductanceValue_Normal[4] + InductanceValue_Normal[5] * InductanceValue_Normal[5]);
        Current_Value = ((FastSqrt(InductanceV1) - FastSqrt(InductanceV2)) / (InductanceV1 + InductanceV2) * 100);

    //����ʱ�Ƕ�ά�����
    if(InductanceValue_Normal[1] < 3.0 && InductanceValue_Normal[2] < 3.0 && InductanceValue_Normal[3] < 3.0  && InductanceValue_Normal[4] < 3.0
       && InductanceValue_Normal[0] < 3.0 && InductanceValue_Normal[5] < 3.0)
    {
        Current_Value = -(Steer_PID->D_LastError);//D_LastError = 0 - Current_Value;

        if(Current_Value > 0.0)
        {
            Current_Value = 16.0;
        }
        else
        {
            Current_Value = -16.0;
        }
    }
/*
    if(Island_Flag)
    {


        Current_Value *= 60.0f;

    }
    else
    {
        Current_Value *= 1.5f;
    }
*/
    if(Island_Flag)
    {
        Current_Value *= 2.5f;
    }
    else
    {
        Current_Value *= 1.5f;
    }


    return Current_Value;
}


void Setting_Weight() {
    if (FastABS(InductanceValue_Normal[1] + InductanceValue_Normal[4]) >= Island_threshold) {  //�뻷��
        Inductance_Weight[0] = (float) Island_R1 / 100.0f;
        Inductance_Weight[1] = (float) Island_R2 / 100.0f;
        Inductance_Weight[2] = (float) Island_M1 / 100.0f;
        Inductance_Weight[3] = (float) Island_M2 / 100.0f;
        Inductance_Weight[4] = (float) Island_L2 / 100.0f;
        Inductance_Weight[5] = (float) Island_L1 / 100.0f;
        Island_Flag = 1;


/*
    }else if(InductanceValue_Normal[1] >= 15 &&  InductanceValue_Normal[4] >= 20 &&  InductanceValue_Normal[5] <= 1.5 &&  InductanceValue_Normal[0] >=1.5) {  //�뻷��,�һ���

        Inductance_Weight[0] = (float) Island_R1 / 100.0f;
        Inductance_Weight[1] = (float) Island_R2 / 100.0f;
        Inductance_Weight[2] = (float) Island_M1 / 100.0f;
        Inductance_Weight[3] = (float) Island_M2 / 100.0f;
        Inductance_Weight[4] = (float) Island_L2 / 100.0f;
        Inductance_Weight[5] = (float) Island_L1 / 100.0f;
        Island_Flag = 1;

    }else if(InductanceValue_Normal[1] >= 20 &&  InductanceValue_Normal[4] >= 20 &&  InductanceValue_Normal[5] >= 1.30 &&  InductanceValue_Normal[0] <=1.0){ //�뻷�����󻷵�

        Inductance_Weight[0] = (float) Island_R1 / 100.0f;
        Inductance_Weight[1] = (float) Island_R2 / 100.0f;
        Inductance_Weight[2] = (float) Island_M1 / 100.0f;
        Inductance_Weight[3] = (float) Island_M2 / 100.0f;
        Inductance_Weight[4] = (float) Island_L2 / 100.0f;
        Inductance_Weight[5] = (float) Island_L1 / 100.0f;
        Island_Flag = 1;
*/

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








