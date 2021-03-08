//
// Created by xcs on 2021-03-03.
//

#include "main.h"

int main() {
    DisableGlobalIRQ();
    board_init();

    //ADC:
    adc_init(ADC_1, Steer_ADCInput1_CH, ADC_12BIT);
    adc_init(ADC_1, Steer_ADCInput2_CH, ADC_12BIT);
    adc_init(ADC_1, Steer_ADCInput3_CH, ADC_12BIT);
    adc_init(ADC_1, Steer_ADCInput4_CH, ADC_12BIT);
    adc_init(ADC_1, Steer_ADCInput5_CH, ADC_12BIT);
    //SteerPWM:
    pwm_init(SteerPWM_CH, 50, MiddleSteer_PWM * 5);
    //MotorPWM:
    pwm_init(MotorPWM_Go_L_CH, 13 * 1000, 0);
    pwm_init(MotorPWM_Go_R_CH, 13 * 1000, 0);
    pwm_init(MotorPWM_Return_L_CH, 13 * 1000, 0);
    pwm_init(MotorPWM_Return_R_CH, 13 * 1000, 0);
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
    //UART
    uart_init(USART_6, 9600, ART_TXDCH, ART_RXDCH);
    uart_init(USART_8, 9600, BullTooth_TXDCH, BullTooth_RXDCH);
    //OLED:
    oled_init();
    while (1) { ;
    }
}
