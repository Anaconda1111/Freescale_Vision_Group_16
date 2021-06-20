//
// Created by xcs on 2021-03-03.
//

#ifndef RT1064_CODE_PORT_H_
#define RT1064_CODE_PORT_H_

#include "zf_adc.h"
#include "headfile.h"
/***************************************ADC***************************************************************/
#define Steer_ADCInput1_CH ADC1_CH4_B15
#define Steer_ADCInput2_CH ADC1_CH2_B13
#define Steer_ADCInput3_CH ADC1_CH3_B14
#define Steer_ADCInput4_CH ADC1_CH1_B12
#define Steer_ADCInput5_CH ADC1_CH5_B16
#define Steer_ADCInput6_CH ADC1_CH6_B17

#define Steer_ADCInput7_CH ADC1_CH11_B22
#define Steer_ADCInput8_CH ADC1_CH12_B23
#define Steer_ADCInput9_CH ADC1_CH13_B24
#define Steer_ADCInput10_CH ADC1_CH14_B25
/*********************************************************************************************************/

/***************************************KYE***************************************************************/
#define KEY1 B1
#define KEY2 B0

#define KEY3 D18
#define KEY4 D19

/*********************************************************************************************************/

/****************************************TFT*************************************************************/
#define TFT_SPIN    SPI_4          
#define TFT_SCL     SPI4_SCK_C23   
#define TFT_SDA     SPI4_MOSI_C2  
#define TFT_SDA_IN  SPI4_MISO_C21  
#define TFT_CS      SPI4_CS3_C27   
#define BL_PIN		C24	            
#define REST_PIN	C25             
#define DC_PIN		C26	           
/*********************************************************************************************************/


/****************************************OLED*************************************************************/
#define OLEDCS C21
#define OLEDDC C24
#define OLEDRES C23
#define OLEDSDA C26
#define OLEDSCK C25
/*********************************************************************************************************/

/*****************************************PWM*************************************************************/
#define MotorPWM_Return_R_CH PWM1_MODULE3_CHA_D0
#define MotorPWM_Return_L_CH PWM2_MODULE3_CHB_D3
#define MotorPWM_Go_R_CH PWM1_MODULE3_CHB_D1
#define MotorPWM_Go_L_CH PWM2_MODULE3_CHA_D2
#define SteerPWM_CH PWM4_MODULE3_CHA_C31
#define Yuntai1PWM_CH PWM4_MODULE2_CHA_C30
#define JiguangPWM_CH PWM1_MODULE0_CHB_D13
/*********************************************************************************************************/

/*****************************************UART************************************************************/
#define ART_TXDCH UART6_TX_B2
#define ART_RXDCH UART6_RX_B3
#define BullTooth_TXDCH UART8_TX_D16
#define BullTooth_RXDCH UART8_RX_D17
/*********************************************************************************************************/

/*****************************************QTIMER**********************************************************/
#define Qtimer1_LSB QTIMER1_TIMER0_C0
#define Qtimer1_DIR QTIMER1_TIMER1_C1
#define Qtimer2_LSB QTIMER3_TIMER2_B18
#define Qtimer2_DIR QTIMER3_TIMER3_B19
/*********************************************************************************************************/

/*****************************************BEEF************************************************************/
#define BEEF B11
/*********************************************************************************************************/

/*****************************************DialSwitch******************************************************/
#define DialSwitch1 D26
#define DialSwitch2 D27
/*********************************************************************************************************/
#endif // RT1064_CODE_PORT_H_
