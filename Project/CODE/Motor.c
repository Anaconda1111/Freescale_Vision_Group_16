//
// Created by xcs on 2021-03-05.
//
#include "headfile.h"
#include "Motor.h"
#include "Steer.h"
#include "port.h"
#include "zf_systick.h"
#include "fastmath.h"
#include "ADC.h"
#include "image.h"


extern PID_Struct Steer_PID;
extern Filter_Struct Encoder_L_Filter;
extern Filter_Struct Encoder_R_Filter;
extern uint8 middle_line_peak;
extern uint8 middle_line_high;
extern float Angle;
extern uint8 garageout;
extern uint8 Flag_go;
extern uint8 Flag_island_L;
extern uint8 Flag_island_R;
extern uint8 Flag_forkroad;

uint16 Motor_GO_L_PWM=8000;
uint16 Motor_GO_R_PWM=8000;

float encoder_value_L;
float encoder_value_R;
float encoder_value;



float speed =150; 
float speed_error =0;



float Kp=0.85;//220速度时，4.65
float Kd=3.5;

//电机控制周期10ms,输出差速，增量式输出，


void Motor_PIDStruct_Init(PID_Struct Motor_GOL_PID, PID_Struct Motor_GOR_PID,
                          Filter_Struct Motor_GOR_Filter,
                          Filter_Struct Motor_GOL_Filter) {
    Motor_GOL_PID->I_MAX = MotorI_MAX;
    Motor_GOR_PID->I_MAX = MotorI_MAX;
    Motor_GOL_Filter->Coefficient = 1.0f / 32.0f;
    Motor_GOR_Filter->Coefficient = 1.0f / 32.0f;
    
    Motor_GOL_PID->KP = 1.0f;
    Motor_GOL_PID->KI = 0.0f;
    Motor_GOL_PID->KD = 5.3f;
  
    Motor_GOR_PID->KP = 1.0f;
    Motor_GOR_PID->KI = 0.0f;
    Motor_GOR_PID->KD = 5.3f;
    
       //电机目标值
     Motor_GOL_PID->TargetValue = 0;
     Motor_GOR_PID->TargetValue = 0;
    
    //电机当前值
    Motor_GOL_PID->CurrentValue =0;
    Motor_GOR_PID->CurrentValue =0;
    
}



void MotorCtrl(PID_Struct Motor_GOL_PID, PID_Struct Motor_GOR_PID,
               Filter_Struct Motor_GOL_Filter,
               Filter_Struct Motor_GOR_Filter) {
   

    
    //电机当前值
    Motor_GOL_PID->CurrentValue =encoder_value_L;
    Motor_GOR_PID->CurrentValue =encoder_value_R;
    
    //电机目标值
     Motor_GOL_PID->TargetValue = speed;
     Motor_GOR_PID->TargetValue = speed;
    
    float speed_error_L = FastABS(Motor_GOL_PID->CurrentError);
    float speed_error_R = FastABS(Motor_GOR_PID->CurrentError);
    speed_error = (speed_error_L + speed_error_R)/2;
    
    Angle = FastABS(Steer_PID->CurrentError);



    Motor_GOL_PID->KP =Kp;
    Motor_GOR_PID->KP =Kp;
    
    Motor_GOL_PID->KD =Kd;
    Motor_GOR_PID->KD =Kd;    
    
    
   
    
    //速度选择，直道弯道
    if(Flag_island_L || Flag_island_R)
    {
        speed = 160;//180
    }
    else
    {  
        speed =160;//220
     

    }
 
    if(middle_line_high >= 90)
    {
        speed += 20;
    }

      
  
    
  
    if(Flag_forkroad)
    {
        speed -=20;
      
    }
    
    speed = speed - Angle* 0.84;
 
    
    //电机目标值,加上差速
    if(Steer_PID->CurrentError < 0)
    {
        Motor_GOL_PID->TargetValue = speed - Steer_PID->CurrentError * 1.5;
        Motor_GOR_PID->TargetValue = speed;
    }   
     
    if(Steer_PID->CurrentError > 0)
    {
        Motor_GOL_PID->TargetValue = speed;
        Motor_GOR_PID->TargetValue = speed + Steer_PID->CurrentError * 1.5;
    }
 
    
     //  Motor_GOL_PID->TargetValue = speed;
     //  Motor_GOR_PID->TargetValue = speed;
 
    

    
    if(Motor_GOL_PID->TargetValue < 0)
    {
      Motor_GOL_PID->TargetValue = 0;
    }
    
    if(Motor_GOR_PID->TargetValue < 0)
    {
      Motor_GOR_PID->TargetValue = 0;
    }

    
    
    //输出
    Motor_GO_L_PWM =Motor_GO_L_PWM + (int16)(PIDCalculate(Motor_GOL_PID, Motor_GOL_Filter));
    Motor_GO_R_PWM =Motor_GO_R_PWM + (int16)(PIDCalculate(Motor_GOR_PID, Motor_GOR_Filter));
    
    
 
   
    
    //电机保护
    if (Motor_GO_L_PWM > MotorPWM_MAX)
    {
        Motor_GO_L_PWM = MotorPWM_MAX;
    }
    if (Motor_GO_L_PWM < MotorPWM_MIN)
    {
        Motor_GO_L_PWM = MotorPWM_MIN;
    }

    if (Motor_GO_R_PWM > MotorPWM_MAX)
    {
        Motor_GO_R_PWM = MotorPWM_MAX;
    }
    if (Motor_GO_R_PWM < MotorPWM_MIN)
    {
        Motor_GO_R_PWM = MotorPWM_MIN;
    }
  
    
   
   
    
   if(Flag_go ==1)
   {
         pwm_duty(MotorPWM_Go_L_CH, Motor_GO_L_PWM);
         pwm_duty(MotorPWM_Go_R_CH, Motor_GO_R_PWM);
             
   }
  
  
}    




void Motor_value_get()
{
    //低通滤波系数
    Encoder_L_Filter->Coefficient = 1.0f/8.0f;  
    Encoder_R_Filter->Coefficient = 1.0f/8.0f;
 
      
    Encoder_L_Filter->SampleValue =(float)qtimer_quad_get(QTIMER_3, Qtimer2_LSB);
    Encoder_R_Filter->SampleValue =(float)-qtimer_quad_get(QTIMER_1, Qtimer1_LSB);
    qtimer_quad_clear(QTIMER_1, Qtimer1_LSB);
    qtimer_quad_clear(QTIMER_3, Qtimer2_LSB);
  
    encoder_value_L =RCFilter(Encoder_L_Filter);
    encoder_value_R =RCFilter(Encoder_R_Filter);
    
    encoder_value  =((encoder_value_L + encoder_value_R)/2);
  
}

void stop_car(uint8 time)
{
      Flag_go=0;
                  pwm_duty(MotorPWM_Go_L_CH, 0);
                  pwm_duty(MotorPWM_Go_R_CH, 0);
                  pwm_duty(MotorPWM_Return_L_CH, 16000);//20000
                  pwm_duty(MotorPWM_Return_R_CH, 16000);
                  systick_delay_ms(time);//速度220时，500ms
                  pwm_duty(MotorPWM_Return_L_CH, 0);
                  pwm_duty(MotorPWM_Return_R_CH, 0);
                 //while(1);
                 // systick_delay_ms(1000);

                     
 
}

void stop_car1()
{
    Flag_go=0;
    while(encoder_value > 0)
    {
      
      pwm_duty(MotorPWM_Go_L_CH, 0);
      pwm_duty(MotorPWM_Go_R_CH, 0);
      pwm_duty(MotorPWM_Return_L_CH, 16000);
      pwm_duty(MotorPWM_Return_R_CH, 16000);
      
    }
    
      pwm_duty(MotorPWM_Return_L_CH, 0);
      pwm_duty(MotorPWM_Return_R_CH, 0);     
       systick_delay_ms(1000);
  
}
  

void EncoderCalDistance(uint16 distance, int32 speed) 
{
    
    qtimer_quad_clear(QTIMER_1, Qtimer1_LSB);
    qtimer_quad_clear(QTIMER_3, Qtimer2_LSB);
    encoder_value_R = 0;
    encoder_value_L = 0;
    encoder_value = 0;
    UniformSpeed(speed);
    while (FastABS(encoder_value)<= distance) {
        UniformSpeed(speed);
        encoder_value_L = qtimer_quad_get(QTIMER_1, Qtimer1_LSB);
        encoder_value_R = qtimer_quad_get(QTIMER_3, Qtimer2_LSB);
        encoder_value = FastABS((encoder_value_L + encoder_value_R)/2);
    }
    qtimer_quad_clear(QTIMER_1, Qtimer1_LSB);
    qtimer_quad_clear(QTIMER_3, Qtimer2_LSB);
    encoder_value_L = 0;
    encoder_value_R = 0;
    UniformSpeed(0);
  
}


void UniformSpeed(int32 Speed)
{
    if (Speed > 0) {
        pwm_duty(MotorPWM_Return_L_CH, 0);
        pwm_duty(MotorPWM_Return_R_CH, 0);
        pwm_duty(MotorPWM_Go_L_CH, Speed);
        pwm_duty(MotorPWM_Go_R_CH, Speed);
    } else if (Speed < 0) {
        Speed = my_abs_int(Speed);
        pwm_duty(MotorPWM_Go_R_CH, 0);
        pwm_duty(MotorPWM_Go_L_CH, 0);
        pwm_duty(MotorPWM_Return_L_CH, Speed);
        pwm_duty(MotorPWM_Return_R_CH, Speed);
    } else {
        pwm_duty(MotorPWM_Go_R_CH, 0);
        pwm_duty(MotorPWM_Go_L_CH, 0);
        pwm_duty(MotorPWM_Return_R_CH, 0);
        pwm_duty(MotorPWM_Return_L_CH, 0);
    }
}