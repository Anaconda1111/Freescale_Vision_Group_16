//
// Created by xcs on 2021-03-03.
//

#ifndef RT1064_CODE_PID_H_
#define RT1064_CODE_PID_H_
#include "headfile.h"


typedef struct PID_Parameter* PID_Struct;
typedef struct Filter_Parameter* Filter_Struct;


struct PID_Parameter {
    float KP;
    float KI;
    float KD;
    float CurrentError;
    float LastError;
    float D_CurrentError;
    float D_LastError;
    float Result;
    float CurrentValue;
    float TargetValue;
    float I_MAX;
    float Integral;
    float Differential;
};

struct Filter_Parameter {
    float OutputValue;
    float Last_OutputValue;
    float SampleValue;
    float Coefficient;
};

float PIDCalculate(PID_Struct PID, Filter_Struct Filter);

float RCFilter(Filter_Struct Filter);

#endif // RT1064_CODE_PID_H_
