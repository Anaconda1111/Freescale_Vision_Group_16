//
// Created by xcs on 2021-03-03.
//

#include "PID.h"
#include "fastmath.h"

// float PIDCalculate(PID_Struct PID, Filter_Struct Filter) {
//     PID->CurrentError = PID->TargetValue - PID->CurrentValue; //计算当前误差
//     PID->D_CurrentError = PID->CurrentError;
//
//     PID->Integral += (PID->CurrentError + PID->LastError) / 2.0f; //计算梯形积分
//
//     if (FastABS(PID->Integral) > PID->I_MAX) { //抗积分饱和
//         if (PID->Integral > 0)
//             PID->Integral = PID->I_MAX;
//         else
//             PID->Integral = -1.0f * PID->I_MAX;
//     }
//
//     Filter->SampleValue = PID->D_CurrentError; //将用于计算微分的值做滤波处理
//     PID->D_CurrentError = RCFilter(Filter);
//
//     PID->Differential = PID->D_CurrentError - PID->D_LastError; //计算微分值
//     PID->D_LastError = PID->D_CurrentError;
//
//     PID->Result = PID->KP * PID->CurrentError + // PID位置式
//                   PID->KI * PID->Integral + PID->KD * PID->Differential;
//
//     return PID->Result;
// }
//
// float RCFilter(Filter_Struct Filter) {
//     Filter->OutputValue = Filter->Coefficient * Filter->SampleValue +
//                           (1 - Filter->Coefficient) * Filter->Last_OutputValue;
//     Filter->Last_OutputValue = Filter->OutputValue;
//     return Filter->OutputValue;
// }


float PIDCalculate(PID_Struct PID, Filter_Struct Filter) {
    PID->CurrentError = PID->TargetValue - PID->CurrentValue; //计算当前误差
    PID->D_CurrentError = PID->CurrentError;

    PID->Integral += (PID->CurrentError + PID->LastError) / 2.0f; //计算梯形积分

    if (FastABS(PID->Integral) > PID->I_MAX) { //抗积分饱和
        if (PID->Integral > 0)
            PID->Integral = PID->I_MAX;
        else
            PID->Integral = -1.0f * PID->I_MAX;
    }

    Filter->SampleValue = PID->D_CurrentError; //将用于计算微分的值做滤波处理
    PID->D_CurrentError = RCFilter(Filter);

    PID->Differential = PID->D_CurrentError - PID->D_LastError; //计算微分值
    PID->D_LastError = PID->D_CurrentError;

    PID->Result = PID->KP * PID->CurrentError + // PID位置式
                  PID->KI * PID->Integral + PID->KD * PID->Differential;

    return PID->Result;
}

float RCFilter(Filter_Struct Filter) {
    Filter->OutputValue = Filter->Coefficient * Filter->SampleValue +
                          (1 - Filter->Coefficient) * Filter->Last_OutputValue;
    Filter->Last_OutputValue = Filter->OutputValue;
    return Filter->OutputValue;
}