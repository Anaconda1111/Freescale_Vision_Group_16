/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2019,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：一群：179029047(已满)  二群：244861897
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file       		main
 * @company	   		成都逐飞科技有限公司
 * @author     		逐飞科技(QQ3184284598)
 * @version    		查看doc内version文件 版本说明
 * @Software 		IAR 8.3 or MDK 5.24
 * @Target core		NXP RT1064DVL6A
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2019-04-30
 ********************************************************************************************************************/

#ifndef _RT106X_config_h
#define _RT106X_config_h

//-------------------------------------------------------------------------------------------------------------------
//  当前版本号
//  如果此处版本号与version.txt文件版本不一致，以version.txt文件为准
//-------------------------------------------------------------------------------------------------------------------
#define SEEKFREE_RT106X_VERSION   0X100




//-------------------------------------------------------------------------------------------------------------------
//  PRINTF_ENABLE 为 0               不使用printf串口打印函数及不输出调试信息，则不会初始化相应串口
//  PRINTF_ENABLE 为 1               使用printf串口打印函数及输出调试信息，则会初始化相应串口，这里定义为1之后，无需自己手动初始化串口
//-------------------------------------------------------------------------------------------------------------------
#define PRINTF_ENABLE               1


#if(1 == PRINTF_ENABLE)

//-------------------------------------------------------------------------------------------------------------------
//printf串口打印函数所使用的串口、串口波特率、串口引脚宏定义
//-------------------------------------------------------------------------------------------------------------------
#define DEBUG_UART                  USART_1
#define DEBUG_UART_BAUD             115200
#define DEBUG_UART_TX_PIN           UART1_TX_B12
#define DEBUG_UART_RX_PIN           UART1_RX_B13

//-------------------------------------------------------------------------------------------------------------------
//  DEBUG_INFO 为 0                  不输出断言信息
//  DEBUG_INFO 为 1                  输出断言信息，断言信息包含错误所在文件及所在行  需要PRINTF宏定义为1才能有效输出
//-------------------------------------------------------------------------------------------------------------------
#define DEBUG_PRINTF                1

#endif

//控制模块电感值ADC
#define SteerADC1 ADC2_CH9_B20
#define SteerADC2 ADC2_CH13_B24
#define SteerADC3 ADC2_CH14_B25
#define SteerADC4 ADC2_CH15_B26
//舵机PWM
#define DUOJI_Channel PWM4_MODULE2_CHA_C30
//电机PWM
#define Motor_Go_L PWM2_MODULE3_CHA_D2
#define Motor_Go_R PWM1_MODULE3_CHB_D1
#define Motor_Return_L PWM1_MODULE3_CHA_D0
#define Motor_Return_R PWM2_MODULE3_CHB_D3
//C25，C26，C27，C28，C29用于OLED
//传输用于训练模型ADC
#define AI_ADC1 ADC1_CH3_B14
#define AI_ADC2 ADC1_CH4_B15
#define AI_ADC3 ADC1_CH8_B19
#define AI_ADC4 ADC1_CH6_B17
#define AI_ADC5 ADC1_CH7_B18
#define AI_ADC6 ADC1_CH0_B27
#define AI_ADC7 ADC1_CH10_B21
//串口转蓝牙
#define UART8_TX UART8_TX_D16
#define UART8_RX UART8_RX_D17
//编码器
#define Speed_A1 QTIMER1_TIMER0_C0
#define Speed_B1 QTIMER1_TIMER1_C1
#define Speed_A2 QTIMER1_TIMER2_C2
#define Speed_B2 QTIMER1_TIMER3_C24
//蜂鸣器
#define BEEP C3
//按键 
#define K1 B9
#define K2 B10
#define K3 B11
#define K4 B28
//拨码开关
#define BM1 B0
#define BM2 B1
#define BM3 B2
#define BM4 B3
//D4干簧管
#define Stop C16
//D14，D15备用管脚
   


#endif
