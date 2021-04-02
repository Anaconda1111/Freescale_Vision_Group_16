//
// Created by xcs on 2021-03-03.
//

#include "main.h"
#include "Text.h"
#include "zf_systick.h"

extern uint16 Motor_GO_L_PWMDuty;
extern uint16 Motor_GO_R_PWMDuty;
extern uint16 SteerPWMDuty;

// //PID&FilterStruct:
struct PID_Parameter Steer_PID_Parameter = {0};
struct Filter_Parameter Steer_Filter_Parameter = {0};
struct PID_Parameter Motor_GOL_PID_Parameter = {0};
struct PID_Parameter Motor_GOR_PID_Parameter = {0};
struct Filter_Parameter Motor_GOL_Filter_Parameter = {0};
struct Filter_Parameter Motor_GOR_Filter_Parameter = {0};


PID_Struct Motor_GOL_PID = &Motor_GOL_PID_Parameter,
        Motor_GOR_PID = &Motor_GOR_PID_Parameter,
        Steer_PID = &Steer_PID_Parameter;


Filter_Struct Motor_GOL_Filter = &Motor_GOL_Filter_Parameter,
        Motor_GOR_Filter = &Motor_GOR_Filter_Parameter,
        Steer_Filter = &Steer_Filter_Parameter;


int main() {
    DisableGlobalIRQ();
    board_init();
    systick_delay_ms(100);

    // //ADC:
    adc_init(ADC_1, Steer_ADCInput1_CH, ADC_12BIT);
    adc_init(ADC_1, Steer_ADCInput2_CH, ADC_12BIT);
    adc_init(ADC_1, Steer_ADCInput3_CH, ADC_12BIT);
    adc_init(ADC_1, Steer_ADCInput4_CH, ADC_12BIT);
    adc_init(ADC_1, Steer_ADCInput5_CH, ADC_12BIT);
    adc_init(ADC_1, Steer_ADCInput6_CH, ADC_12BIT);
    adc_init(ADC_1, Steer_ADCInput7_CH, ADC_12BIT);
    adc_init(ADC_1, Steer_ADCInput8_CH, ADC_12BIT);
    adc_init(ADC_1, Steer_ADCInput9_CH, ADC_12BIT);
    adc_init(ADC_1, Steer_ADCInput10_CH, ADC_12BIT);

    // //SteerPWM:
    pwm_init(SteerPWM_CH, 50, MiddleSteer_PWM);

    // //MotorPWM:
    pwm_init(MotorPWM_Go_L_CH, 13 * 1000, 0);
    pwm_init(MotorPWM_Go_R_CH, 13 * 1000, 0);
    pwm_init(MotorPWM_Return_L_CH, 13 * 1000, 0);
    pwm_init(MotorPWM_Return_R_CH, 13 * 1000, 0);

    // //YuntaiPWM:
    // pwm_init(Yuntai1PWM_CH,50,Middle);
    // pwm_init(Yuntai2PWM_CH,50,Middle);

    //Qtimer:
    qtimer_quad_init(QTIMER_1, Qtimer1_LSB, Qtimer1_DIR);
    qtimer_quad_init(QTIMER_3, Qtimer2_LSB, Qtimer2_DIR);

    //KEY:
    gpio_init(KEY1, GPI, 1, GPIO_PIN_CONFIG);
    gpio_init(KEY2, GPI, 1, GPIO_PIN_CONFIG);
    gpio_init(KEY3, GPI, 1, GPIO_PIN_CONFIG);
    gpio_init(KEY4, GPI, 1, GPIO_PIN_CONFIG);

    //DialSwitch:
    gpio_init(DialSwitch1, GPI, 0, GPIO_PIN_CONFIG);
    gpio_init(DialSwitch2, GPI, 0, GPIO_PIN_CONFIG);

    // BEEF
    //gpio_init(BEEF, GPO, 0, GPIO_PIN_CONFIG);
    // //UART
    // uart_init(USART_6, 19200, ART_TXDCH, ART_RXDCH);
    uart_init(USART_8, 9600, BullTooth_TXDCH, BullTooth_RXDCH);

    // //OLED:
    oled_init();


    Motor_PIDStruct_Init(Motor_GOL_PID, Motor_GOR_PID, Motor_GOR_Filter, Motor_GOL_Filter);
    Steer_PIDStruct_Init(Steer_PID, Steer_Filter);
    // GarageOut();//³ö¿âº¯Êý
    gpio_init(B9, GPO, 1, GPIO_PIN_CONFIG);

    //PIT:
    // pit_init();
    // pit_interrupt_ms(PIT_CH0, 10);
    // NVIC_SetPriority(PIT_IRQn, 1);
    EnableGlobalIRQ(0);
    pwm_duty(MotorPWM_Go_L_CH, Motor_GO_L_PWMDuty);
    pwm_duty(MotorPWM_Go_R_CH, Motor_GO_R_PWMDuty);
    while (1) {
        Interactive();
        systick_delay_ms(50);
        Get_InductanceValue();
        SteerCtrl(Steer_PID, Steer_Filter);
        while (InductanceValue_Normal[0] < 10 && InductanceValue_Normal[1] < 10 &&
               InductanceValue_Normal[2] < 10 && InductanceValue_Normal[3] < 10) {
            pwm_duty(MotorPWM_Go_L_CH, 0);
            pwm_duty(MotorPWM_Go_R_CH, 0);
            Get_InductanceValue();
        }
        pwm_duty(MotorPWM_Go_L_CH, Motor_GO_L_PWMDuty);
        pwm_duty(MotorPWM_Go_R_CH, Motor_GO_R_PWMDuty);
        //MotorCtrl(Motor_GOL_PID, Motor_GOR_PID, Motor_GOL_Filter, Motor_GOR_Filter);
    }
}
