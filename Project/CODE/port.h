//
// Created by xcs on 2021-03-03.
//

#ifndef RT1064_CODE_PORT_H_
#define RT1064_CODE_PORT_H_

#include "zf_adc.h"
#include "headfile.h"
/***************************************ADC***************************************************************/
#define Steer_ADCInput1_CH ADC1_CH2_B13
#define Steer_ADCInput2_CH ADC1_CH4_B15
#define Steer_ADCInput3_CH ADC1_CH1_B12
#define Steer_ADCInput4_CH ADC1_CH3_B14
#define Steer_ADCInput5_CH ADC1_CH6_B17
#define Steer_ADCInput6_CH ADC1_CH5_B16

/*********************************************************************************************************/

/***************************************KYE***************************************************************/
#define KEY1 B1
#define KEY2 B0
#define KEY3 D15
#define KEY4 D12

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
#define OLEDCS C27
#define OLEDDC C26
#define OLEDRES C25
#define OLEDSDA C2
#define OLEDSCK C23
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
#define ART_TXDCH UART8_TX_D16
#define ART_RXDCH UART8_RX_D17
#define BullTooth_TXDCH UART4_TX_C16
#define BullTooth_RXDCH UART4_RX_C17
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
