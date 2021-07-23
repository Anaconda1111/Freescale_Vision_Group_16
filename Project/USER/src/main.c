/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2019,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：一群：179029047(已满)  二群：244861897
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *+
 * @file       		main
 * @company	   		成都逐飞科技有限公司
 * @author     		逐飞科技(QQ3184284598)
 * @version    		查看doc内version文件 版本说明
 * @Software 		IAR 8.3 or MDK 5.24
 * @Target core		NXP RT1064DVL6A
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2019-04-30
 * @note		
					接线定义：
 					------------------------------------  
						模块管脚         	单片机管脚
						SDA(51的RX)         查看SEEKFREE_MT9V03X_CSI.h文件中的MT9V03X_CSI_COF_UART_TX	宏定义
						SCL(51的TX)         查看SEEKFREE_MT9V03X_CSI.h文件中的MT9V03X_CSI_COF_UART_RX	宏定义
						场中断(VSY)         查看SEEKFREE_MT9V03X_CSI.h文件中的MT9V03X_CSI_VSYNC_PIN		宏定义
						行中断(HREF)	    不需要使用
						像素中断(PCLK)      查看SEEKFREE_MT9V03X_CSI.h文件中的MT9V03X_CSI_PCLK_PIN		宏定义
						数据口(D0-D7)		B31-B24 B31对应摄像头接口D0
						默认分辨率是        188*120
						默认FPS           	50帧
					------------------------------------ 
 ********************************************************************************************************************/


//打开新的工程或者工程移动了位置务必执行以下操作
//第一步 关闭上面所有打开的文件
//第二步 project  clean  等待下方进度条走完


#include "headfile.h"
#include "Binarization.h"
#include "image.h"
#include "port.h"
#include "interactive.h"
#include "Steer.h"
#include "Motor.h"
#include "ANO_DT.h"
   
extern float current_value;
extern uint8 middle_line_peak;
extern uint8 zebra_time;
extern uint8 Flag_zebra_L;
extern uint8 middle_line_high;
extern uint8 Flag_zebra_R;
extern uint8 Flag_go;
extern float encoder_value;
extern float encoder_value_L;
extern float encoder_value_R;
extern float speed;
extern float Angle;
extern uint8 Flag_island_L;
extern uint8 Flag_island_R;
extern int time;

//
struct PID_Parameter Motor_GOL_PID_Parameter = {0};
struct PID_Parameter Motor_GOR_PID_Parameter = {0};
struct Filter_Parameter Motor_GOL_Filter_Parameter = {0};
struct Filter_Parameter Motor_GOR_Filter_Parameter = {0};
//
struct PID_Parameter Steer_PID_Parameter = {0};
struct Filter_Parameter Steer_Filter_Parameter = {0};


struct Filter_Parameter Encoder_Filter_Parameter_L = {0};
struct Filter_Parameter Encoder_Filter_Parameter_R = {0};

PID_Struct Steer_PID = &Steer_PID_Parameter;
PID_Struct Motor_GOL_PID = &Motor_GOL_PID_Parameter;
PID_Struct Motor_GOR_PID = &Motor_GOR_PID_Parameter;

Filter_Struct Steer_Filter = &Steer_Filter_Parameter;
Filter_Struct Motor_GOL_Filter = &Motor_GOL_Filter_Parameter;
Filter_Struct Motor_GOR_Filter = &Motor_GOR_Filter_Parameter;

Filter_Struct Encoder_L_Filter = &Encoder_Filter_Parameter_L;
Filter_Struct Encoder_R_Filter = &Encoder_Filter_Parameter_R;


uint8 garageout =1;
uint8 garageout_dir =0;
uint8 flag_garagein_L =0;
uint8 flag_garagein_R =1;

int main(void)
{
    DisableGlobalIRQ();
    board_init();   //务必保留，本函数用于初始化MPU 时钟 调试串口

    systick_delay_ms(300);	//延时300ms，等待主板其他外设上电成功
/*
    //ADC:
    adc_init(ADC_1, Steer_ADCInput1_CH, ADC_12BIT);
    adc_init(ADC_1, Steer_ADCInput2_CH, ADC_12BIT);
    adc_init(ADC_1, Steer_ADCInput3_CH, ADC_12BIT);
    adc_init(ADC_1, Steer_ADCInput4_CH, ADC_12BIT);
    adc_init(ADC_1, Steer_ADCInput5_CH, ADC_12BIT);
    adc_init(ADC_1, Steer_ADCInput6_CH, ADC_12BIT);    
*/ 
    
    //SteerPWM:
    pwm_init(SteerPWM_CH, 50, MiddleSteer_PWM); 
    pwm_init(Yuntai1PWM_CH, 333, 23000); //min:10550,max:35950
    pwm_init(JiguangPWM_CH, 125, 0); 
    
    
     //MotorPWM:
    pwm_init(MotorPWM_Go_L_CH, 13 * 1000, 0);
    pwm_init(MotorPWM_Go_R_CH, 13 * 1000, 0);
    pwm_init(MotorPWM_Return_L_CH, 13 * 1000, 0);
    pwm_init(MotorPWM_Return_R_CH, 13 * 1000, 0);
   
    //Qtimer:
    qtimer_quad_init(QTIMER_1, Qtimer1_LSB, Qtimer1_DIR);
    qtimer_quad_init(QTIMER_3, Qtimer2_LSB, Qtimer2_DIR);
    
     //DialSwitch:
    gpio_init(DialSwitch1, GPI, 0, GPIO_PIN_CONFIG);
    gpio_init(DialSwitch2, GPI, 0, GPIO_PIN_CONFIG);
    
     // KEY:
    gpio_init(KEY1, GPI, 1, GPIO_PIN_CONFIG);
    gpio_init(KEY2, GPI, 1, GPIO_PIN_CONFIG);
    gpio_init(KEY3, GPI, 1, GPIO_PIN_CONFIG);
    gpio_init(KEY4, GPI, 1, GPIO_PIN_CONFIG);    
        
    gpio_init(B9, GPO, 0, GPIO_PIN_CONFIG);
    gpio_init(BEEF, GPO, 0, GPIO_PIN_CONFIG);
    
  
    //OLED:
    oled_init();
    
    //LCD
   // lcd_init();
    

    
    uart_init(USART_4, 115200, UART4_TX_C16, UART4_RX_C17);    
    uart_init(USART_8, 115200, ART_TXDCH, ART_RXDCH);
    
    
    mt9v03x_csi_init();//初始化摄像头 使用CSI接口
    
    
    Steer_PIDStruct_Init(Steer_PID, Steer_Filter);
    Motor_PIDStruct_Init(Motor_GOL_PID, Motor_GOR_PID, Motor_GOR_Filter, Motor_GOL_Filter);
    
    
    //PIT:
    pit_init();
    pit_interrupt_ms(PIT_CH0, 10);
    pit_interrupt_ms(PIT_CH1, 1000);
    //pit_interrupt_ms(PIT_CH2, 20);
  //  pit_interrupt_ms(PIT_CH3, 10);
    
    NVIC_SetPriority(PIT_IRQn, 1);
    
    
    
    EnableGlobalIRQ(0);

    while(garageout)
    {    
     

        if(mt9v03x_csi_finish_flag)
        {
           
            ImageBinary();
            
           //  scan_line_base();
            // Interactive();
             
             
            if (gpio_get(KEY1) == 0) {
              garageout_dir =1;
              time =0;
              systick_delay_ms(3000);  
            }
            if (gpio_get(KEY2) == 0) {
              garageout_dir =2;
              time =0;
              systick_delay_ms(3000);  
            }
              
              
            
            
           //左边出库
            if (garageout_dir ==1) 
            { 
              
              pwm_duty(SteerPWM_CH, MiddleSteer_PWM);
              pwm_duty(MotorPWM_Go_L_CH, 12000);
              pwm_duty(MotorPWM_Go_R_CH, 12000);      
              
                uint8 flag=0;
                uint8 i=0;
                uint8 top;       
                uint8 time=0;
                for(i=0;i<100;i++)
                {
                  if(flag ==0 && image(i,MT9V03X_CSI_W-10)==0 && image(i+1,MT9V03X_CSI_W-10)==0)
                  {
                    time++;
                    if(time > 20)
                    {
                      flag =1;
                    }
                      
                  }        
                  if(flag ==1 && image(i,MT9V03X_CSI_W-10)==1 && image(i+1,MT9V03X_CSI_W-10)==1)
                  {
                    flag=2;
                    top = i;
                  }
                }
                
                if(flag ==2 && top < 60)
                {
                    pwm_duty(SteerPWM_CH, SteerMAX);
                    pwm_duty(MotorPWM_Go_L_CH, 12000);
                    pwm_duty(MotorPWM_Go_R_CH, 12000);  
                    systick_delay_ms(500); 
                    flag_garagein_L =1;
                    flag_garagein_R =0;
                    garageout =0;                     
                                
                }
            }
            
                    
           //右边出库
            if (garageout_dir ==2) 
            { 
              
              pwm_duty(SteerPWM_CH, MiddleSteer_PWM);
              pwm_duty(MotorPWM_Go_L_CH, 12000);
              pwm_duty(MotorPWM_Go_R_CH, 12000);      
              
                uint8 flag=0;
                uint8 i=0;
                uint8 top;             
                for(i=0;i<100;i++)
                {
                  if(flag ==0 && image(i,0)==0 && image(i+1,0)==0)
                  {
                      flag =1;
                      
                  }        
                  if(flag ==1 && image(i,0)==1 && image(i+1,0)==1)
                  {
                    flag=2;
                    top = i;
                  }
                }
                
                if(flag ==2 && top < 60)
                {
                    pwm_duty(SteerPWM_CH, SteerMIN);
                    pwm_duty(MotorPWM_Go_L_CH, 12000);
                    pwm_duty(MotorPWM_Go_R_CH, 12000);  
                    systick_delay_ms(500); 
                    flag_garagein_L =0;
                    flag_garagein_R =1;
                    garageout =0;                     
                                
                }
            } 
          
            mt9v03x_csi_finish_flag = 0;       
        }

     
    }    
 
    pwm_duty(MotorPWM_Go_L_CH, 0);
    pwm_duty(MotorPWM_Go_R_CH, 0);  
    pwm_duty(MotorPWM_Return_L_CH, 0);
    pwm_duty(MotorPWM_Return_R_CH, 0);
  
    Flag_go =1;
    
   // int16 data1=0,data2=0,data3=0,data4=0,data5=0,data6=0,data7=0,data8=0;
 
    while(1)
    {   
       
        if(mt9v03x_csi_finish_flag)
        {
              gpio_toggle(B9);
            // csi_seekfree_sendimg_03x(USART_4,mt9v03x_csi_image[0],MT9V03X_CSI_W,MT9V03X_CSI_H);
            // lcd_displayimage032_zoom(mt9v03x_csi_image[0], MT9V03X_CSI_W, MT9V03X_CSI_H, 160, 128);
          
              ImageBinary();
 
             image_ctrl(); 
            // Interactive();
              Interactive2();
              SteerCtrl(Steer_PID, Steer_Filter);
            
                   
              mt9v03x_csi_finish_flag = 0;       
        }
        
         
             //pwm_duty(JiguangPWM_CH, 25000);
 /*       
        data1 = (int16)encoder_value;
        data2 = (int16)encoder_value_L;
        data3 = (int16)encoder_value_R;
        data4 = (int16)speed;
        data5 = (int16)Angle;
        data6 = (int16)Motor_GOL_PID->TargetValue;
        data7 = (int16)Motor_GOR_PID->TargetValue;
   */     
       // ANO_DT_send_int16(USART_4, data1, data2, data3, data4, data5, data6, data7, data8);
        
 /*   
      if(Flag_island_L || Flag_island_R)
      {
          pwm_duty(MotorPWM_Go_L_CH, 11000);
          pwm_duty(MotorPWM_Go_R_CH, 11000);
                 
      }
      else
      {
          pwm_duty(MotorPWM_Go_L_CH, 12000);
          pwm_duty(MotorPWM_Go_R_CH, 12000);
          
        
      }

 */      
        
        
    
  }
  
  
  
  
  
  
  
  
  
  
  
  
  
}
   