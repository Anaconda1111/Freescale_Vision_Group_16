/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2019,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ��һȺ��179029047(����)  ��Ⱥ��244861897
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *+
 * @file       		main
 * @company	   		�ɶ���ɿƼ����޹�˾
 * @author     		��ɿƼ�(QQ3184284598)
 * @version    		�鿴doc��version�ļ� �汾˵��
 * @Software 		IAR 8.3 or MDK 5.24
 * @Target core		NXP RT1064DVL6A
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2019-04-30
 * @note		
					���߶��壺
 					------------------------------------  
						ģ��ܽ�         	��Ƭ���ܽ�
						SDA(51��RX)         �鿴SEEKFREE_MT9V03X_CSI.h�ļ��е�MT9V03X_CSI_COF_UART_TX	�궨��
						SCL(51��TX)         �鿴SEEKFREE_MT9V03X_CSI.h�ļ��е�MT9V03X_CSI_COF_UART_RX	�궨��
						���ж�(VSY)         �鿴SEEKFREE_MT9V03X_CSI.h�ļ��е�MT9V03X_CSI_VSYNC_PIN		�궨��
						���ж�(HREF)	    ����Ҫʹ��
						�����ж�(PCLK)      �鿴SEEKFREE_MT9V03X_CSI.h�ļ��е�MT9V03X_CSI_PCLK_PIN		�궨��
						���ݿ�(D0-D7)		B31-B24 B31��Ӧ����ͷ�ӿ�D0
						Ĭ�Ϸֱ�����        188*120
						Ĭ��FPS           	50֡
					------------------------------------ 
 ********************************************************************************************************************/


//���µĹ��̻��߹����ƶ���λ�����ִ�����²���
//��һ�� �ر��������д򿪵��ļ�
//�ڶ��� project  clean  �ȴ��·�����������


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
    board_init();   //��ر��������������ڳ�ʼ��MPU ʱ�� ���Դ���

    systick_delay_ms(300);	//��ʱ300ms���ȴ��������������ϵ�ɹ�
	
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
    
    
    
    mt9v03x_csi_init();//��ʼ������ͷ ʹ��CSI�ӿ�
    
    
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






