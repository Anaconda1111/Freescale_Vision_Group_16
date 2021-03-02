#include "PID.h"

float PIDCalculate(PID_Struct PID,Filter_Struct Filter){
    PID->CurrentError=PID->TargetValue-PID->CurrentValue;      //���㵱ǰ���
    PID->D_CurrentError=PID->CurrentError;

    PID->Integral+=(PID->CurrentError+PID->LastError)/2.0f;    //�������λ���

    if (FastABS(PID->Integral)>PID->I_MAX){                 //�����ֱ���
        if (PID->Integral>0)
            PID->Integral=PID->I_MAX;
        else
            PID->Integral=-1.0f*PID->I_MAX;
    }

    Filter->SampleValue=PID->D_CurrentError;                   //�����ڼ���΢�ֵ�ֵ���˲�����
    PID->D_CurrentError=RCFilter(Filter);


    PID->Differential=PID->D_CurrentError-PID->D_LastError;    //����΢��ֵ
    PID->D_LastError=PID->D_CurrentError;

    PID->Result=PID->KP*PID->CurrentError+                     //PIDλ��ʽ
                PID->KI*PID->Integral+
                PID->KD*PID->Differential;

    if (PID->Result>PID->ResultMax)
        PID->Result=PID->ResultMax;
    else if (PID->Result<PID->ResultMin)
        PID->Result=PID->ResultMin;
    return PID->Result;
}

float RCFilter(Filter_Struct Filter){
    Filter->OutputValue=Filter->Coefficient*Filter->SampleValue+
                            (1-Filter->Coefficient)*Filter->Last_OutputValue;
    Filter->Last_OutputValue=Filter->OutputValue;
    return Filter->OutputValue;
}
