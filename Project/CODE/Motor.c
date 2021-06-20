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

extern uint8 Camera;
extern PID_Struct Steer_PID;
extern Filter_Struct Encoder_L_Filter;
extern Filter_Struct Encoder_R_Filter;


uint16 Motor_GO_L_PWM=15000;
uint16 Motor_GO_R_PWM=15000;

int16 encoder_value_L;
int16 encoder_value_R;
uint16 speed_R = 65;  //70还可以跑，但是要擦轮胎
uint16 speed_L = 65;


void Motor_PIDStruct_Init(PID_Struct Motor_GOL_PID, PID_Struct Motor_GOR_PID,
                          Filter_Struct Motor_GOR_Filter,
                          Filter_Struct Motor_GOL_Filter) {
    Motor_GOL_PID->I_MAX = MotorI_MAX;
    Motor_GOR_PID->I_MAX = MotorI_MAX;
    Motor_GOL_Filter->Coefficient = 1.0f / 32.0f;
    Motor_GOR_Filter->Coefficient = 1.0f / 32.0f;
    
    Motor_GOL_PID->KP = 1.50f;
    Motor_GOL_PID->KI = 0.0f;
    Motor_GOL_PID->KD = 2.5f;
  
    Motor_GOR_PID->KP = 1.50f;
    Motor_GOR_PID->KI = 0.0f;
    Motor_GOR_PID->KD = 2.5f;
   
    
    Motor_GOL_PID->TargetValue =(float)(1000);
    Motor_GOR_PID->TargetValue =(float)(1000);
}



void MotorCtrl(PID_Struct Motor_GOL_PID, PID_Struct Motor_GOR_PID,
               Filter_Struct Motor_GOL_Filter,
               Filter_Struct Motor_GOR_Filter) {
    
    float Speed_error = FastABS(Motor_GOL_PID->CurrentValue);
    float Angle = FastABS(Steer_PID->CurrentValue);

    if (Camera == IsAnimal) {   //停车
        pwm_duty(MotorPWM_Go_L_CH, 0);
        pwm_duty(MotorPWM_Go_R_CH, 0);
        systick_delay_ms(3050);
    }

   
 
    //电机PID
    if(Speed_error <= 80)  //50 
    {
            Motor_GOL_PID->KP = 3.00f + Speed_error*0.15;
            Motor_GOL_PID->KI = 0.0f;
            Motor_GOL_PID->KD = 0.0f;
          
            Motor_GOR_PID->KP = 3.50f + Speed_error*0.14;
            Motor_GOR_PID->KI = 0.0f;
            Motor_GOR_PID->KD = 0.0f;
   
    }
    else                                    //往左转
    {
        
            Motor_GOL_PID->KP = 15.00f;   //10.5
            Motor_GOL_PID->KI = 0.0f;
            Motor_GOL_PID->KD = 0.0f;
          
            Motor_GOR_PID->KP = 14.70f;   //10.5
            Motor_GOR_PID->KI = 0.0f;
            Motor_GOR_PID->KD = 0.0f;
     }
    
    speed_R = 50;  //70还可以跑，但是要擦轮胎
    speed_L = 50;
 
   if(Angle <= 2.5)
   {
        speed_L = speed_L;
        speed_R = speed_R;
      
   }
   else if(Angle > 2.5 && Angle <= 6.5)
   {
        speed_L = speed_L;
        speed_R = speed_R;
      
      
   }
   else if(Angle > 6.5 && Angle <= 9.5)
   {
        speed_L = speed_L;
        speed_R = speed_R;
           
        
      
   }
   else if(Angle > 9.5 && Angle <= 12.5)
   {
        speed_L = speed_L - 5;
        speed_R = speed_R - 5;
          
      
   }
   else if(Angle > 12.5 && Angle <= 15.5)
   {
        speed_L = speed_L - 10;
        speed_R = speed_R - 10;
        
      
   }
   else if(Angle > 15.5 && Angle <= 18.5)
   {
        speed_L = speed_L - 10;
        speed_R = speed_R - 10;
         
      
   }
   else if(Angle > 18.5 && Angle <= 21.5)
   {
        speed_L = speed_L - 10;
        speed_R = speed_R - 10;
       
   }
   else
   {
        speed_L = speed_L - 15;
        speed_R = speed_R - 15;
       
   }
    
    
 
   if(Angle <= 2.5)
   {
        
      if(Steer_PID->CurrentValue > 0.0) //往右转
     {
        Motor_GOL_PID->TargetValue =speed_L;
        Motor_GOR_PID->TargetValue =speed_R;
     }
     else if(Steer_PID->CurrentValue < 0.0) //往左转
     {
        Motor_GOL_PID->TargetValue =speed_L;
        Motor_GOR_PID->TargetValue =speed_R;
     }
   }
   else if(Angle > 2.5 && Angle <= 6.5)
   {
     
      if(Steer_PID->CurrentValue > 0.0) //往右转
       {
         Motor_GOL_PID->TargetValue =speed_L;
         Motor_GOR_PID->TargetValue =speed_R - 5;
       }
       else if(Steer_PID->CurrentValue < 0.0) //往左转
       {
          Motor_GOL_PID->TargetValue =speed_L - 5;
          Motor_GOR_PID->TargetValue =speed_R;
       }
   }
   else if(Angle > 6.5 && Angle <= 9.5)
   {
          
        
      if(Steer_PID->CurrentValue > 0.0) //往右转
       {
           Motor_GOL_PID->TargetValue =speed_L + 5;
          Motor_GOR_PID->TargetValue =speed_R - 5;
       }
       else if(Steer_PID->CurrentValue < 0.0) //往左转
       {
          Motor_GOL_PID->TargetValue =speed_L - 5;
          Motor_GOR_PID->TargetValue =speed_R + 5;
       }
   }
   else if(Angle > 9.5 && Angle <= 12.5)
   {
         
      if(Steer_PID->CurrentValue > 0.0) //往右转
       {
           Motor_GOL_PID->TargetValue =speed_L + 10;
          Motor_GOR_PID->TargetValue =speed_R - 5;
       }
       else if(Steer_PID->CurrentValue < 0.0) //往左转
       {
           Motor_GOL_PID->TargetValue =speed_L - 5;
          Motor_GOR_PID->TargetValue =speed_R + 10;
       }
   }
   else if(Angle > 12.5 && Angle <= 15.5)
   {
       
      if(Steer_PID->CurrentValue > 0.0) //往右转
       {
           Motor_GOL_PID->TargetValue =speed_L + 15;
          Motor_GOR_PID->TargetValue =speed_R - 10;
       }
       else if(Steer_PID->CurrentValue < 0.0) //往左转
       {
           Motor_GOL_PID->TargetValue =speed_L - 10;
          Motor_GOR_PID->TargetValue =speed_R + 15;
       }
   }
   else if(Angle > 15.5 && Angle <= 18.5)
   {
        
      if(Steer_PID->CurrentValue > 0.0) //往右转
       {
          Motor_GOL_PID->TargetValue =speed_L + 20;
          Motor_GOR_PID->TargetValue =speed_R - 15;
       }
       else if(Steer_PID->CurrentValue < 0.0) //往左转
       {
          Motor_GOL_PID->TargetValue =speed_L - 15;
          Motor_GOR_PID->TargetValue =speed_R + 20;
       }
   }
   else if(Angle > 18.5 && Angle <= 21.5)
   {
       if(Steer_PID->CurrentValue > 0.0) //往右转
       {
          Motor_GOL_PID->TargetValue =speed_L + 25;
          Motor_GOR_PID->TargetValue =speed_R - 20;
       }
       else if(Steer_PID->CurrentValue < 0.0) //往左转
       {
          Motor_GOL_PID->TargetValue =speed_L - 20;
          Motor_GOR_PID->TargetValue =speed_R + 25;
       }
   }
   else if(Angle < 21.5 && Angle <= 24.5)
   {
       if(Steer_PID->CurrentValue > 0.0) //往右转
       {
          Motor_GOL_PID->TargetValue =speed_L + 30;
          Motor_GOR_PID->TargetValue =speed_R - 20;
       }
       else if(Steer_PID->CurrentValue < 0.0) //往左转
       {
          Motor_GOL_PID->TargetValue =speed_L - 20;
          Motor_GOR_PID->TargetValue =speed_R + 30;
       }
   }
   else
   {
       if(Steer_PID->CurrentValue > 0.0) //往右转
       {
          Motor_GOL_PID->TargetValue =speed_L + 35;
          Motor_GOR_PID->TargetValue =speed_R - 20;
       }
       else if(Steer_PID->CurrentValue < 0.0) //往左转
       {
          Motor_GOL_PID->TargetValue =speed_L - 20;
          Motor_GOR_PID->TargetValue =speed_R + 35;
       }
   }
/* 
    if(Island_Flag)
    {
      
      
          Motor_GOL_PID->TargetValue =speed_L - 10;
          Motor_GOR_PID->TargetValue =speed_R - 10;
     
    }
   
 */   
    if(Motor_GOL_PID->TargetValue <= 0)
    {
      Motor_GOL_PID->TargetValue = 0;
    }
    
    if(Motor_GOR_PID->TargetValue <= 0)
    {
      Motor_GOR_PID->TargetValue = 0;
    }
    
    //电机当前值
    Motor_GOL_PID->CurrentValue =(float)encoder_value_L;
    Motor_GOR_PID->CurrentValue =(float)encoder_value_R;
    
    //输出
    Motor_GO_L_PWM +=(int16)(PIDCalculate(Motor_GOL_PID, Motor_GOL_Filter));
    Motor_GO_R_PWM +=(int16)(PIDCalculate(Motor_GOR_PID, Motor_GOR_Filter));
    
    
 /*   //滤波 处理
    Motor_GOL_Filter->SampleValue = Motor_GO_L_PWM;
    Motor_GOR_Filter->SampleValue = Motor_GO_R_PWM;
  
     Motor_GO_L_PWM =(int16)RCFilter(Motor_GOL_Filter);
     Motor_GO_R_PWM =(int16)RCFilter(Motor_GOR_Filter);
  */  
   
    
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
    
    //出轨保护
    if(InductanceValue_Normal[1] < 3.0 && InductanceValue_Normal[2] < 3.0 && InductanceValue_Normal[3] < 3.0  && InductanceValue_Normal[4] < 3.0 
         && InductanceValue_Normal[0] < 3.0 && InductanceValue_Normal[5] < 3.0)
      {
        
       
          pwm_duty(MotorPWM_Go_L_CH, 0);
          pwm_duty(MotorPWM_Go_R_CH, 0);
       
       
    
      }
      else
      {
        
        pwm_duty(MotorPWM_Go_L_CH, Motor_GO_L_PWM);
        pwm_duty(MotorPWM_Go_R_CH, Motor_GO_R_PWM);
        
       }
    
  
}    


void GarageOut() {
  
        pwm_duty(SteerPWM_CH, SteerOutGarage_PWM);//
        pwm_duty(MotorPWM_Go_L_CH, Motor_GO_L_PWM);
        pwm_duty(MotorPWM_Go_R_CH, Motor_GO_R_PWM);
    
        
}




void Motor_value_get()
{
    //低通滤波系数
    Encoder_L_Filter->Coefficient = 1.0f/4.0f;  
    Encoder_R_Filter->Coefficient = 1.0f/8.0f;
 
      
    Encoder_L_Filter->SampleValue =(float)qtimer_quad_get(QTIMER_3, Qtimer2_LSB);
    Encoder_R_Filter->SampleValue =(float)-qtimer_quad_get(QTIMER_1, Qtimer1_LSB);
    qtimer_quad_clear(QTIMER_1, Qtimer1_LSB);
    qtimer_quad_clear(QTIMER_3, Qtimer2_LSB);
  
    encoder_value_L =(int16)RCFilter(Encoder_L_Filter);
    encoder_value_R =(int16)(RCFilter(Encoder_R_Filter)* 0.558f);
    
    
  
}

void CarStop(PID_Struct Motor_GOL_PID, PID_Struct Motor_GOR_PID,
                          Filter_Struct Motor_GOR_Filter,
                          Filter_Struct Motor_GOL_Filter)
{       
  
     float Speed_error = FastABS(Motor_GOL_PID->CurrentValue);
     //电机PID
    if(Speed_error <= 80)  //50 
    {
            Motor_GOL_PID->KP = 2.80f + Speed_error*0.14;
            Motor_GOL_PID->KI = 0.0f;
            Motor_GOL_PID->KD = 0.0f;
          
            Motor_GOR_PID->KP = 3.50f + Speed_error*0.14;
            Motor_GOR_PID->KI = 0.0f;
            Motor_GOR_PID->KD = 0.0f;
   
    }
    else                                    //往左转
    {
        
            Motor_GOL_PID->KP = 14.00f;   //10.5
            Motor_GOL_PID->KI = 0.0f;
            Motor_GOL_PID->KD = 0.0f;
          
            Motor_GOR_PID->KP = 14.70f;   //10.5
            Motor_GOR_PID->KI = 0.0f;
            Motor_GOR_PID->KD = 0.0f;
     }
    
    //电机目标值
    Motor_GOL_PID->TargetValue =(float)0.0;
    Motor_GOR_PID->TargetValue =(float)0.0;
    
    //电机当前值
    Motor_GOL_PID->CurrentValue =(float)encoder_value_L;
    Motor_GOR_PID->CurrentValue =(float)encoder_value_R;
    
    //输出
    Motor_GO_L_PWM =(uint16)(PIDCalculate(Motor_GOL_PID, Motor_GOL_Filter)*1000);
    Motor_GO_R_PWM =(uint16)(PIDCalculate(Motor_GOR_PID, Motor_GOR_Filter)*1000);
   
    
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
  
    
    
    
    if((encoder_value_L + encoder_value_R) > 10.0)
      {
         pwm_duty(MotorPWM_Go_L_CH, 0);
         pwm_duty(MotorPWM_Go_R_CH, 0);
         pwm_duty(MotorPWM_Return_L_CH, Motor_GO_L_PWM);//后退
         pwm_duty(MotorPWM_Return_R_CH, Motor_GO_R_PWM);
        
        
    
      }
      else if((encoder_value_L + encoder_value_R) < -10.0)
      {
        
        pwm_duty(MotorPWM_Return_L_CH, 0);
        pwm_duty(MotorPWM_Return_R_CH, 0);
        pwm_duty(MotorPWM_Go_L_CH, Motor_GO_L_PWM);//前进
        pwm_duty(MotorPWM_Go_R_CH, Motor_GO_R_PWM);
       
        
       }
       else
       {
          pwm_duty(MotorPWM_Go_L_CH, 0);//停止
          pwm_duty(MotorPWM_Go_R_CH, 0);
          pwm_duty(MotorPWM_Return_L_CH, 0);
          pwm_duty(MotorPWM_Return_R_CH, 0);
       }
  
  
}

void motorctrl_test()
{
  // float Angle = (int16)FastABS(Steer_PID->CurrentValue); // -16.0 <= angle <= 16
  /*  
   //主要输出电机差速
   if(Angle <= 3.5)
   {
     if(Steer_PID->CurrentValue > 0.0) //往右转
     {
        Motor_GO_L_PWM =(uint16)(16000-Angle*114.3);
        Motor_GO_R_PWM =(uint16)(16000-Angle*342.9);
     }
     else if(Steer_PID->CurrentValue < 0.0) //往左转
     {
        Motor_GO_L_PWM =(uint16)(16000 - Angle*342.9);
        Motor_GO_R_PWM =(uint16)(16000 - Angle*114.3);
     }
   }
   else if(Angle > 3.5 && Angle <= 6.5)
   {
      if(Steer_PID->CurrentValue > 0.0) //往右转
      {
          Motor_GO_L_PWM =(uint16)(15600 - (Angle-3.5) * 133.3);
          Motor_GO_R_PWM =(uint16)(14800 - (Angle-3.5) * 400.0);
      }
      else if(Steer_PID->CurrentValue < 0.0) //往左转
      {
          Motor_GO_L_PWM =(uint16)(14800 - (Angle-3.5) * 400.0);
          Motor_GO_R_PWM =(uint16)(15600 - (Angle-3.5) * 133.3);
      }
   }
   else if(Angle > 6.5 && Angle <= 9.5)
   {
      if(Steer_PID->CurrentValue > 0.0) //往右转
      {
          Motor_GO_L_PWM =(uint16)(15200 - (Angle-6.5) * 133.3);
          Motor_GO_R_PWM =(uint16)(13600 - (Angle-6.5) * 400.0);
      }
      else if(Steer_PID->CurrentValue < 0.0) //往左转
      {
          Motor_GO_L_PWM =(uint16)(13600 - (Angle-6.5) * 400.0);
          Motor_GO_R_PWM =(uint16)(15200 - (Angle-6.5) * 133.3);
      }
   }
   else if(Angle > 9.5 && Angle <= 13.0)
   {
      if(Steer_PID->CurrentValue > 0.0) //往右转
      {
          Motor_GO_L_PWM =(uint16)(14800 - (Angle-9.5) * 114.3);
          Motor_GO_R_PWM =(uint16)(12400 - (Angle-9.5) * 342.9);
      }
      else if(Steer_PID->CurrentValue < 0.0) //往左转
      {
          Motor_GO_L_PWM =(uint16)(12400 - (Angle-9.5) * 342.9);
          Motor_GO_R_PWM =(uint16)(14800 - (Angle-9.5) * 114.3);
      }
   }
   else if(Angle > 13.0 && Angle <= 16.0)
   {
      if(Steer_PID->CurrentValue > 0.0) //往右转
      {
          Motor_GO_L_PWM =(uint16)(14400 - (Angle-13.0)*133.3);
          Motor_GO_R_PWM =(uint16)(11200 - (Angle-13.0)*400.0);
      }
      else if(Steer_PID->CurrentValue < 0.0) //往左转
      {
          Motor_GO_L_PWM =(uint16)(11200 - (Angle-13.0)*400.0);
          Motor_GO_R_PWM =(uint16)(14400 - (Angle-13.0)*133.3);
      }
   }
   else
   {
      if(Steer_PID->CurrentValue > 0.0) //往右转
      {
          Motor_GO_L_PWM =(uint16)(14000-(Angle-16.0)*133.3);
          Motor_GO_R_PWM =(uint16)(10000-(Angle-16.0)*400.0);
      }
      else if(Steer_PID->CurrentValue < 0.0) //往左转
      {
          Motor_GO_L_PWM =(uint16)(10000-(Angle-16.0)*400.0);
          Motor_GO_R_PWM =(uint16)(14000-(Angle-16.0)*133.3);
      }
   }
   */
   
  float Angle = (int16)FastABS(Steer_PID->CurrentValue); // -13.0 <= angle <= 13
    //主要输出电机差速
    if(Angle <= 13.5)
    {
        if(Steer_PID->CurrentValue > 0.0) //往右转
        {
            Motor_GO_L_PWM =(uint16)(Motor_GO_L_PWM - (uint16)Angle *222.3);
            Motor_GO_R_PWM =(uint16)(Motor_GO_R_PWM - (uint16)Angle *370.4);
        }
        else if(Steer_PID->CurrentValue < 0.0) //往左转
        {
            Motor_GO_L_PWM =(uint16)(Motor_GO_L_PWM - (uint16)Angle * 370.4);
            Motor_GO_R_PWM =(uint16)(Motor_GO_R_PWM - (uint16)Angle *222.3);
        }
    }
    else if(Angle > 13.0 && Angle <= 18.0)
    {
        if(Steer_PID->CurrentValue > 0.0) //往右转
        {
            Motor_GO_L_PWM =(uint16)(Motor_GO_L_PWM + (uint16)Angle * 400.0);
            Motor_GO_R_PWM =(uint16)Motor_GO_R_PWM;
        }
        else if(Steer_PID->CurrentValue < 0.0) //往左转
        {
           Motor_GO_L_PWM =(uint16)Motor_GO_L_PWM;
            Motor_GO_R_PWM =(uint16)(Motor_GO_R_PWM + (uint16)Angle *400.0);
        }
    }
    else
    {
        if(Steer_PID->CurrentValue > 0.0) //往右转
        {
            Motor_GO_L_PWM =(uint16)Motor_GO_L_PWM - 1000;
            Motor_GO_R_PWM =(uint16)Motor_GO_R_PWM - 5000;
        }
        else if(Steer_PID->CurrentValue < 0.0) //往左转
        {
            Motor_GO_L_PWM =(uint16)Motor_GO_L_PWM - 5000;
            Motor_GO_R_PWM =(uint16)Motor_GO_R_PWM - 1000;
        }
    }
 
   //速度预测,急弯减速（提前作用）,作用不可代替弯道减速
   if(FastABS(Steer_PID->Differential) >= 0.0 && FastABS(Steer_PID->Differential) < 0.5)
   {
      Motor_GO_L_PWM -=0;
      Motor_GO_R_PWM -=0;
   }
   else if(FastABS(Steer_PID->Differential) >= 0.5 && FastABS(Steer_PID->Differential) < 1.0)
   {
      Motor_GO_L_PWM -=1000;
      Motor_GO_R_PWM -=1000;
   }
   else if(FastABS(Steer_PID->Differential) >= 1.0 && FastABS(Steer_PID->Differential) < 2.0)
   {
      Motor_GO_L_PWM -=1500;
      Motor_GO_R_PWM -=1500;
   }
   else if(FastABS(Steer_PID->Differential) >= 2.0 && FastABS(Steer_PID->Differential) < 3.0)
   {
      Motor_GO_L_PWM -=2000;
      Motor_GO_R_PWM -=2000;
   }
   else
   {
      Motor_GO_L_PWM -=3000;
      Motor_GO_R_PWM -=3000;
   }
   
/*
    //环岛减速
    if(Island_Flag)
    {
      Motor_GO_L_PWM -=2000;
      Motor_GO_R_PWM -=2000;
    }
*/
   
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
    
    //出轨保护
    if(InductanceValue_Normal[1] < 3.0 && InductanceValue_Normal[2] < 3.0 && InductanceValue_Normal[3] < 3.0 && InductanceValue_Normal[4] < 3.0 
         && InductanceValue_Normal[0] < 3.0 && InductanceValue_Normal[5] < 3.0)
      {
         pwm_duty(MotorPWM_Go_L_CH, 0);
         pwm_duty(MotorPWM_Go_R_CH, 0);
      
      }
      else
      {
      
          pwm_duty(MotorPWM_Go_L_CH, Motor_GO_L_PWM);
          pwm_duty(MotorPWM_Go_R_CH, Motor_GO_R_PWM);
        
       }
   
 
}


