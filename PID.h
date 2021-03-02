#ifndef AI__PID_H_
#define AI__PID_H_
#define FastABS(x) (x>0?x:x*-1.0)
typedef struct PID_Parameter *PID_Struct;
typedef struct Filter_Parameter *Filter_Struct;
struct PID_Parameter{
    float KP;
    float KI;
    float KD;
    float CurrentError;
    float LastError;
    float D_CurrentError;
    float D_LastError;
    float Result;
    float ResultMax;
    float ResultMin;
    float CurrentValue;
    float TargetValue;
    float I_MAX;
    float Integral;
    float Differential;
};

struct Filter_Parameter{
    float OutputValue;
    float Last_OutputValue;
    float SampleValue;
    float Coefficient;
};

float PIDCalculate(PID_Struct PID,Filter_Struct Filter);
float RCFilter(Filter_Struct Filter);
#endif//AI__PID_H_

