//
// Created by xcs on 2021-03-03.
//

#include "main.h"
#include "Text.h"
#include "zf_systick.h"

extern float Current_Value;
extern int16 garageout_flag;
//PID&FilterStruct:
struct PID_Parameter Steer_PID_Parameter = {0};
struct Filter_Parameter Steer_Filter_Parameter = {0};
struct PID_Parameter Motor_GOL_PID_Parameter = {0};
struct PID_Parameter Motor_GOR_PID_Parameter = {0};
struct Filter_Parameter Motor_GOL_Filter_Parameter = {0};
struct Filter_Parameter Motor_GOR_Filter_Parameter = {0};
struct Filter_Parameter Encoder_Filter_Parameter_L = {0};
struct Filter_Parameter Encoder_Filter_Parameter_R = {0};

PID_Struct Motor_GOL_PID = &Motor_GOL_PID_Parameter;
PID_Struct Motor_GOR_PID = &Motor_GOR_PID_Parameter;
PID_Struct Steer_PID = &Steer_PID_Parameter;
Filter_Struct Motor_GOL_Filter = &Motor_GOL_Filter_Parameter;
Filter_Struct Motor_GOR_Filter = &Motor_GOR_Filter_Parameter;
Filter_Struct Steer_Filter = &Steer_Filter_Parameter;

Filter_Struct Encoder_L_Filter = &Encoder_Filter_Parameter_L;
Filter_Struct Encoder_R_Filter = &Encoder_Filter_Parameter_R;

extern int16 encoder_value_L;
extern int16 encoder_value_R;

int main(void) {
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
    /*
    adc_init(ADC_1, Steer_ADCInput7_CH, ADC_12BIT);
    adc_init(ADC_1, Steer_ADCInput8_CH, ADC_12BIT);
    adc_init(ADC_1, Steer_ADCInput9_CH, ADC_12BIT);
    adc_init(ADC_1, Steer_ADCInput10_CH, ADC_12BIT);
    */
    //SteerPWM:
    pwm_init(SteerPWM_CH, 50, MiddleSteer_PWM);
    pwm_init(Yuntai2PWM_CH, 50, MidYunTai2SteerPWM);

    //MotorPWM:
    pwm_init(MotorPWM_Go_L_CH, 13 * 1000, 0);
    pwm_init(MotorPWM_Go_R_CH, 13 * 1000, 0);
    pwm_init(MotorPWM_Return_L_CH, 13 * 1000, 0);
    pwm_init(MotorPWM_Return_R_CH, 13 * 1000, 0);

    //JiguangPWM
    pwm_init(JiguangPWM_CH, 125, 0);

    //Qtimer:
    qtimer_quad_init(QTIMER_1, Qtimer1_LSB, Qtimer1_DIR);
    qtimer_quad_init(QTIMER_3, Qtimer2_LSB, Qtimer2_DIR);


    // KEY:
    gpio_init(KEY1, GPI, 1, GPIO_PIN_CONFIG);
    gpio_init(KEY2, GPI, 1, GPIO_PIN_CONFIG);
    gpio_init(KEY3, GPI, 1, GPIO_PIN_CONFIG);
    gpio_init(KEY4, GPI, 1, GPIO_PIN_CONFIG);

    //DialSwitch:
    gpio_init(DialSwitch1, GPI, 0, GPIO_PIN_CONFIG);
    gpio_init(DialSwitch2, GPI, 0, GPIO_PIN_CONFIG);


    //OLED:
    oled_init();


    //
    gpio_init(B9, GPO, 0, GPIO_PIN_CONFIG);
    gpio_init(BEEF, GPO, 0, GPIO_PIN_CONFIG);

    // //UART
    uart_init(USART_6, 115200, ART_TXDCH, ART_RXDCH);
    uart_init(USART_8, 9600, BullTooth_TXDCH, BullTooth_RXDCH);


    //PIT:
    pit_init();
    pit_interrupt_ms(PIT_CH0, 1);
    pit_interrupt_ms(PIT_CH1, 20);
    pit_interrupt_ms(PIT_CH2, 10);
    NVIC_SetPriority(PIT_IRQn, 1);


    Motor_PIDStruct_Init(Motor_GOL_PID, Motor_GOR_PID, Motor_GOR_Filter, Motor_GOL_Filter);
    Steer_PIDStruct_Init(Steer_PID, Steer_Filter);
    GarageOut();
    EnableGlobalIRQ(0);
    uint16 MotorPWM = 11000;
    while (1) { ;
        // ANO_DT_send_int16(USART_8, (int16) Current_Value, SteerPWMDuty, 0, 0, 0, 0, 0, 0);
        // systick_delay_ms(10);
        // motorctrl_test();
        // oled_int16(0, 0, YunTaiPWM);
        // if (gpio_get(KEY1) == 0)
        //     YunTaiPWM += 10;
        // if (gpio_get(KEY2) == 0)
        //     YunTaiPWM -= 10;
        // pwm_duty(Yuntai2PWM_CH, YunTaiPWM);
        // for (int i = 0; i < 50; ++i) {
        //     //Interactive();
        //     Motor_GOL_PID->CurrentValue = encoder_value_L;
        //     Result = (uint16) (PIDCalculate(Motor_GOL_PID, Motor_GOL_Filter));
        //     MotorPWM = Result;
        //     if (MotorPWM > MotorPWM_MAX)
        //         MotorPWM = MotorPWM_MAX;
        //     else if (MotorPWM < MotorPWM_MIN)
        //         MotorPWM = MotorPWM_MIN;
        //pwm_duty(MotorPWM_Go_L_CH, MotorPWM);
        //ANO_DT_send_int16(USART_8, (int16) Motor_GOL_PID->TargetValue, (int16) encoder_value_L, (int16) /*MotorPWM, Result,*/ 0, 0, 0, 0, 0, 0);
        // }
        // if (Motor_GOL_PID->TargetValue == 100)
        //     Motor_GOL_PID->TargetValue = 50;
        // else
        //     Motor_GOL_PID->TargetValue = 100;
        //MotorCtrl(Motor_GOL_PID, Motor_GOR_PID, Motor_GOL_Filter, Motor_GOR_Filter);
        //TridentMessageHandle();
        // ShowInductanceValue_Average();
        // systick_delay_ms(20);
        // SteerCtrl(Steer_PID, Steer_Filter);


        // while (InductanceValue_Normal[1] < 0.5 && InductanceValue_Normal[2] < 1.0 && InductanceValue_Normal[3] < 1.0 && InductanceValue_Normal[4] < 3.0
        //        && InductanceValue_Normal[0] < 0.5 && InductanceValue_Normal[5] < 0.5) {
        //     pwm_duty(MotorPWM_Go_L_CH, 0);
        //     pwm_duty(MotorPWM_Go_R_CH, 0);
        // }
        // MotorCtrl(Motor_GOL_PID, Motor_GOR_PID, Motor_GOL_Filter, Motor_GOR_Filter);
        // systick_delay_ms(10);
        // gpio_toggle(BEEF);

        // SteerCtrl(Steer_PID, Steer_Filter);
        // motorctrl_test();

        // if (Camera == InLeft || Camera == InRight) {
        //     pwm_duty(MotorPWM_Go_L_CH, 0);
        //     pwm_duty(MotorPWM_Go_R_CH, 0);
        //     gpio_set(B9, 1);
        // }
        // motorctrl();
        //motorctrl_Faster();
        // motorctrl_test();

        //MotorCtrl(Motor_GOL_PID, Motor_GOR_PID,Motor_GOL_Filter,Motor_GOR_Filter);
        /*
             if(garageout_flag)
             {
               GarageOut();
             }
        */

        //¼¤¹âµ÷ÊÔ
        // pwm_duty(JiguangPWM_CH, 25000);


        /*
           data1 =(int16)Current_Value;
           data2 =(int16)FastABS(InductanceValue_Normal[2] - InductanceValue_Normal[3]);
           data3 =(int16)FastABS(InductanceValue_Normal[1] - InductanceValue_Normal[4]);
           data4 =(int16)FastABS(InductanceValue_Normal[0] - InductanceValue_Normal[5]);
    */
/*        data1 = (int16)FastABS(Steer_PID->Differential);


        ANO_DT_send_int16(USART_8,data1,data2,data3,data4,data5,data6,data7,data8);
*/
        /*
          data5 = (int16) Motor_GOL_PID->TargetValue;
          data6 = (int16) Motor_GOL_PID->CurrentValue;
          data7 = (int16) Motor_GOR_PID->TargetValue;
          data8 = (int16) Motor_GOR_PID->CurrentValue;
          ANO_DT_send_int16(USART_8,data1,data2,data3,data4,data5,data6,data7,data8);

      */


    }
}





