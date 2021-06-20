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
   
   
extern float current_value;



struct PID_Parameter Steer_PID_Parameter = {0};
struct Filter_Parameter Steer_Filter_Parameter = {0};

PID_Struct Steer_PID = &Steer_PID_Parameter;
Filter_Struct Steer_Filter = &Steer_Filter_Parameter;

uint32 duojipwm = 3430;

int main(void)
{
    DisableGlobalIRQ();
    board_init();   //务必保留，本函数用于初始化MPU 时钟 调试串口

    systick_delay_ms(300);	//延时300ms，等待主板其他外设上电成功
	
    //SteerPWM:
    pwm_init(SteerPWM_CH, 50, 3550); 
    pwm_init(Yuntai1PWM_CH, 333, 14350); //min:10550,max:35950
    pwm_init(JiguangPWM_CH, 125, 25000); 
    
    
        //MotorPWM:
    pwm_init(MotorPWM_Go_L_CH, 13 * 1000, 0);
    pwm_init(MotorPWM_Go_R_CH, 13 * 1000, 0);
    pwm_init(MotorPWM_Return_L_CH, 13 * 1000, 0);
    pwm_init(MotorPWM_Return_R_CH, 13 * 1000, 0);
   

    
     //DialSwitch:
    gpio_init(DialSwitch1, GPI, 0, GPIO_PIN_CONFIG);
    gpio_init(DialSwitch2, GPI, 0, GPIO_PIN_CONFIG);
        
    gpio_init(B9, GPO, 0, GPIO_PIN_CONFIG);
    gpio_init(BEEF, GPO, 0, GPIO_PIN_CONFIG);
    
  
    //OLED:
   // oled_init();
    
    //LCD
    lcd_init();
    

    
    uart_init(USART_4, 115200, UART4_TX_C16, UART4_RX_C17);    
    
    
    
    mt9v03x_csi_init();//初始化摄像头 使用CSI接口
    
    
    Steer_PIDStruct_Init(Steer_PID, Steer_Filter);
/*    
          //PIT:
    pit_init();
    pit_interrupt_ms(PIT_CH0, 20);
   // pit_interrupt_ms(PIT_CH1, 10);
 //   pit_interrupt_ms(PIT_CH2, 15);
  //  pit_interrupt_ms(PIT_CH3, 10);
    
    NVIC_SetPriority(PIT_IRQn, 1);
    
*/

    
    EnableGlobalIRQ(0);
    while(1)
    {
      
         
    
       // pwm_duty(SteerPWM_CH, duojipwm);
      
   
        if(mt9v03x_csi_finish_flag)
        {
              mt9v03x_csi_finish_flag = 0;
            // csi_seekfree_sendimg_03x(USART_4,mt9v03x_csi_image[0],MT9V03X_CSI_W,MT9V03X_CSI_H);
            // lcd_displayimage032_zoom(mt9v03x_csi_image[0], MT9V03X_CSI_W, MT9V03X_CSI_H, 160, 128);
          
              ImageBinary();
              image_ctrl(); 
              SteerCtrl(Steer_PID, Steer_Filter);
              Interactive();
        
                 
        }
      
   
    
          pwm_duty(MotorPWM_Go_L_CH, 11000);
          pwm_duty(MotorPWM_Go_R_CH, 11000);
          
     
        
        
         // systick_delay_ms(50);	
        
       
  
        
    }
}






