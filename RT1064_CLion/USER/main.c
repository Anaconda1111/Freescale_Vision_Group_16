//
// Created by xcs on 2021-03-03.
//

#include "main.h"
#include "Text.h"
#include "SEEKFREE_VIRSCO.h"
#include "Visualscope.h"
#include "zf_systick.h"

int main() {
    DisableGlobalIRQ();
    board_init();
    systick_delay_ms(100);
    // //ADC:
    // adc_init(ADC_1, Steer_ADCInput1_CH, ADC_12BIT);
    // adc_init(ADC_1, Steer_ADCInput2_CH, ADC_12BIT);
    // adc_init(ADC_1, Steer_ADCInput3_CH, ADC_12BIT);
    // adc_init(ADC_1, Steer_ADCInput4_CH, ADC_12BIT);
    // adc_init(ADC_1, Steer_ADCInput5_CH, ADC_12BIT);
    // //SteerPWM:
    // pwm_init(SteerPWM_CH, 50, 3800);
    // //MotorPWM:
    // pwm_init(MotorPWM_Go_L_CH, 13 * 1000, 0);
    // pwm_init(MotorPWM_Go_R_CH, 13 * 1000, 0);
    // pwm_init(MotorPWM_Return_L_CH, 13 * 1000, 0);
    // pwm_init(MotorPWM_Return_R_CH, 13 * 1000, 0);
    // //Qtimer:
    // qtimer_quad_init(QTIMER_1, Qtimer1_LSB, Qtimer1_DIR);
    // qtimer_quad_init(QTIMER_3, Qtimer2_LSB, Qtimer2_DIR);
    //KEY:
    // gpio_init(KEY1, GPI, 1, GPIO_PIN_CONFIG);
    // gpio_init(KEY2, GPI, 1, GPIO_PIN_CONFIG);
    // gpio_init(KEY3, GPI, 1, GPIO_PIN_CONFIG);
    // gpio_init(KEY4, GPI, 1, GPIO_PIN_CONFIG);
    // //DialSwitch:
    // gpio_init(DialSwitch1, GPI, 0, GPIO_PIN_CONFIG);
    // gpio_init(DialSwitch2, GPI, 0, GPIO_PIN_CONFIG);
    // //UART
    // uart_init(USART_6, 19200, ART_TXDCH, ART_RXDCH);
    // uart_init(USART_8, 9600, BullTooth_TXDCH, BullTooth_RXDCH);
    //gpio_init(B9, GPO, 1, GPIO_PIN_CONFIG);
    // //OLED:
    // oled_init();
    // //PIT:
    // pit_init();
    // NVIC_SetPriority(PIT_IRQn, 1);
    // GarageOut();
    gpio_init(B9, GPO, 1, GPIO_PIN_CONFIG);




    // // struct PID_Parameter TEXT_PID = {0};
    // // struct Filter_Parameter TEXT_Filter = {0};
    // // PID_Struct TPID = &TEXT_PID;
    // // Filter_Struct TFILTER = &TEXT_Filter;
    // // TPID->ResultMax = 30;
    // // TPID->ResultMin = -30;
    // // TPID->I_MAX = 20;
    // //
    // // TPID->KP = 1;
    // // TPID->KD = 10;
    // TFILTER->Coefficient = (float) 1.0f / 32.0f;
    pwm_init(SteerPWM_CH, 50, MiddleSteer_PWM);
    while (1) {
        LED();
        systick_delay_ms(200 * 5);
        pwm_duty(SteerPWM_CH, SteerMIN);
        systick_delay_ms(200 * 5);
        pwm_duty(SteerPWM_CH, MiddleSteer_PWM);
        systick_delay_ms(200 * 5);
        pwm_duty(SteerPWM_CH, SteerMAX);
        systick_delay_ms(200 * 5);
        pwm_duty(SteerPWM_CH, MiddleSteer_PWM);
        systick_delay_ms(200 * 5);
    }
}
