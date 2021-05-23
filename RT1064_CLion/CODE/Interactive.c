//
// Created by xcs on 2021-03-08.
//

#include "Interactive.h"
#include "port.h"
#include "SEEKFREE_OLED.h"
#include "PID.h"
#include "zf_pwm.h"
#include "zf_systick.h"
#include "ADC.h"
#include "Steer.h"
#include "fastmath.h"

extern uint16 SteerPWMDuty;
extern uint16 Motor_GO_L_PWM;
extern uint16 Motor_GO_R_PWM;
extern PID_Struct Steer_PID;
extern PID_Struct Motor_GOL_PID;
extern PID_Struct Motor_GOR_PID;
extern float InductanceValue_Normal[InductanceNum];
extern uint16 InductanceValue_Average[InductanceNum];
extern int16 encoder_value_L;
extern int16 encoder_value_R;
extern int16 garageout_flag;
extern int16 Island_Flag;


uint8 key_scan() {


    if (gpio_get(KEY1) == 0) {
        return key1press;
    } else if (gpio_get(KEY2) == 0) {
        return key2press;
    } else if (gpio_get(KEY3) == 0) {
        return key3press;
    } else if (gpio_get(KEY4) == 0) {
        return key4press;
    } else
        return 0;//无按键按下
}

uint8 bm_scan(void) {
    uint8 bm1flag;
    uint8 bm2flag;
    uint8 status = 0x00;
    bm1flag = gpio_get(DialSwitch1);
    bm2flag = gpio_get(DialSwitch2);
    if (bm2flag == 0 && bm1flag == 1) {
        status = 0x01;
    } else if (bm2flag == 1 && bm1flag == 0) {
        status = 0x10;
    } else if (bm2flag == 1 && bm1flag == 1) {
        status = 0x11;
    }
    return status;
}


void Interactive() {
    uint8 Status;
    Status = bm_scan();
    uint8 Key_State = key_scan();
    switch (Status) {
        case 0X00: {
            oled_fill(0X00);
            oled_p6x8str(0, 0, "adc1:");
            oled_printf_float(50, 0, InductanceValue_Normal[0], 5, 2);
            oled_p6x8str(0, 1, "adc2:");
            oled_printf_float(50, 1, InductanceValue_Normal[1], 5, 2);
            oled_p6x8str(0, 2, "adc3:");
            oled_printf_float(50, 2, InductanceValue_Normal[2], 5, 2);
            oled_p6x8str(0, 3, "adc4:");
            oled_printf_float(50, 3, InductanceValue_Normal[3], 5, 2);
            oled_p6x8str(0, 4, "adc5:");
            oled_printf_float(50, 4, InductanceValue_Normal[4], 5, 2);
            oled_p6x8str(0, 5, "adc6:");
            oled_printf_float(50, 5, InductanceValue_Normal[5], 5, 2);
            oled_p6x8str(0, 6, "CurrentValue:");
            oled_printf_float(50, 6, Steer_PID->CurrentValue, 5, 2);
            oled_p6x8str(0, 7, "SteerPWMDuty:");
            oled_printf_float(50, 7, SteerPWMDuty, 5, 2);

            switch (Key_State) {
                case key1press: {

                }break;
                case key2press: {

                }break;
                case key3press: {

                }break;
                case key4press: {

                }break;

            }
        }
            break;

        case 0x01: {

            oled_fill(0X00);
            oled_p6x8str(0, 0, "SteerKP:");
            oled_printf_float(50, 0, Steer_PID->KP, 5, 2);
            oled_p6x8str(0, 1, "SteerKI:");
            oled_printf_float(50, 1, Steer_PID->KI, 5, 2);
            oled_p6x8str(0, 2, "SteerKD:");
            oled_printf_float(50, 2, Steer_PID->KD, 5, 2);
            oled_p6x8str(0, 3, "CurrentValue:");
            oled_printf_float(50, 3, Steer_PID->CurrentValue, 5, 2);
            oled_p6x8str(0, 4, "Steerpwn:");
            oled_printf_float(50, 4, SteerPWMDuty, 5, 2);
            oled_p6x8str(0, 5, "Island:");
            oled_printf_float(50, 5, Island_Flag, 5, 2);
            oled_p6x8str(0, 6, "D_diffe:");
            oled_printf_float(50, 6, FastABS(Steer_PID->Differential), 5, 2);

            /*           oled_fill(0X00);
                       oled_p6x8str(0, 0, "GOR_KP:");
                       oled_printf_float(50, 0, Motor_GOR_PID->KP, 5, 2);
                       oled_p6x8str(0, 1, "GOR_KI:");
                       oled_printf_float(50, 1, Motor_GOR_PID->KI, 5, 2);
                       oled_p6x8str(0, 2, "GOR_KD:");
                       oled_printf_float(50, 2, Motor_GOR_PID->KD, 5, 2);
           */
            /*           oled_fill(0X00);
                       oled_p6x8str(0, 0, "Rate:");
                       oled_printf_float(50, 0, Rate, 5, 2);
              */
            switch (Key_State) {
                case key1press: {
                    Steer_PID->KP +=0.01f;
                    //Motor_GOR_PID->KP +=0.01;


                }
                    break;
                case key2press: {
                    Steer_PID->KP -=0.01f;
                    // Motor_GOR_PID->KP -=0.01;

                }
                    break;
                case key3press: {
                    Steer_PID->KI += 0.01f;
                    // Motor_GOR_PID->KI +=0.01;
                }
                    break;
                case key4press: {
                    Steer_PID->KI -= 0.01f;
                    //Motor_GOR_PID->KI -=0.01;
                }
                    break;
            }
        }
            break;
        case 0x10: {
            /*
               oled_fill(0X00);
               oled_p6x8str(0, 0, "GOL_KP:");
               oled_printf_float(50, 0, Motor_GOL_PID->KP, 5, 2);
               oled_p6x8str(0, 1, "GOL_KI:");
               oled_printf_float(50, 1, Motor_GOL_PID->KI, 5, 2);
               oled_p6x8str(0, 2, "GOL_KD:");
               oled_printf_float(50, 2, Motor_GOL_PID->KD, 5, 2);
           */

        


            switch (Key_State) {
                case key1press: {
                    // Motor_GOL_PID->KP +=0.01;

                    // Steer_PID->KD +=0.01;


                }
                    break;
                case key2press: {
                    // Motor_GOL_PID->KP -=0.01;

                    //Steer_PID->KD -=0.01f;


                }
                    break;
                case key3press: {
                    //  Motor_GOL_PID->KI +=0.01;

                }
                    break;
                case key4press: {
                    //  Motor_GOL_PID->KI -=0.01;

                }
                    break;
            }
        }
        break;
        case 0x11: {

            //  pwm_duty(MotorPWM_Go_L_CH, Motor_GO_L_PWM);
            //  pwm_duty(MotorPWM_Go_R_CH, Motor_GO_R_PWM);
            //oled_int16(0, 0, SteerPWMDuty);

            oled_fill(0X00);
            oled_p6x8str(0, 0, "MotorL:");
            oled_p6x8str(0, 1, "MotorR:");
            oled_int16(50, 0, Motor_GO_L_PWM);
            oled_int16(50, 1, Motor_GO_R_PWM);
            oled_p6x8str(0, 2, "EncoderL:");
            oled_p6x8str(0, 3, "EncoderR:");
            oled_int16(50, 2, encoder_value_L);//L
            oled_int16(50, 3, encoder_value_R);//R
            oled_p6x8str(0, 4, "Angle:");
            oled_printf_float(50, 4, Steer_PID->CurrentValue, 5, 2);
            oled_p6x8str(0, 5, "garage:");
            oled_printf_float(50, 5, garageout_flag, 5, 2);
            oled_p6x8str(0, 6, "D_diffe:");
            oled_printf_float(50, 6, FastABS(Steer_PID->Differential), 5, 2);

            switch (Key_State) {
                case key1press: {
                    //Motor_GO_L_PWM +=20;
                    garageout_flag =1;

                }
                    break;
                case key2press: {
                    // Motor_GO_L_PWM -=20;
                    garageout_flag = 0;

                }
                    break;
                case key3press: {
                    // Motor_GO_R_PWM +=20;
                }
                    break;
                case key4press: {
                    //Motor_GO_R_PWM -=20;
                }
                    break;
            }
        }
        break;
    }
}

void ShowInductanceValue_Normal() {
    oled_fill(0x00);
    oled_p6x8str(0, 0, "ADC1(L1):");
    oled_printf_float(50, 0, InductanceValue_Normal[0], 5, 4);
    oled_p6x8str(0, 1, "ADC2(L2):");
    oled_printf_float(50, 1, InductanceValue_Normal[1], 5, 4);
    oled_p6x8str(0, 2, "ADC3(R2):");
    oled_printf_float(50, 2, InductanceValue_Normal[2], 5, 4);
    oled_p6x8str(0, 3, "ADC4(R1):");
    oled_printf_float(50, 3, InductanceValue_Normal[3], 5, 4);
    oled_p6x8str(0, 4, "ADC5(Mid):");
    oled_printf_float(50, 4, InductanceValue_Normal[4], 5, 4);
}

void ShowInductanceValue_Average() {
    oled_fill(0x00);
    oled_p6x8str(0, 0, "ADC1(L1):");
    oled_int16(50, 0, InductanceValue_Average[0]);
    oled_p6x8str(0, 1, "ADC2(L2):");
    oled_int16(50, 1, InductanceValue_Average[1]);
    oled_p6x8str(0, 2, "ADC3(R2):");
    oled_int16(50, 2, InductanceValue_Average[2]);
    oled_p6x8str(0, 3, "ADC4(R1):");
    oled_int16(50, 3, InductanceValue_Average[3]);
    oled_p6x8str(0, 4, "ADC5(Mid):");
    oled_int16(50, 4, InductanceValue_Average[4]);
}

void ShowADCConvert() {
    oled_fill(0x00);
    // oled_p6x8str(0, 0, "Input6:");
    // oled_int16(50, 0, adc_convert(ADC_1, Steer_ADCInput6_CH));
    // oled_p6x8str(0, 1, "Input7:");
    // oled_int16(50, 1, adc_convert(ADC_1, Steer_ADCInput7_CH));
    // oled_p6x8str(0, 2, "Input8:");
    // oled_int16(50, 2, adc_convert(ADC_1, Steer_ADCInput8_CH));
    // oled_p6x8str(0, 3, "Input9:");
    // oled_int16(50, 3, adc_convert(ADC_1, Steer_ADCInput9_CH));
    // oled_p6x8str(0, 4, "Input10:");
    // oled_int16(50, 4, adc_convert(ADC_1, Steer_ADCInput10_CH));

    oled_p6x8str(0, 0, "Normal1:");
    oled_int16(50, 0, (int16) InductanceValue_Normal[0]);
    oled_p6x8str(0, 1, "Normal2:");
    oled_int16(50, 1, (int16) InductanceValue_Normal[1]);
    oled_p6x8str(0, 2, "Normal3:");
    oled_int16(50, 2, (int16) InductanceValue_Normal[2]);
    oled_p6x8str(0, 3, "Normal4:");
    oled_int16(50, 3, (int16) InductanceValue_Normal[3]);
    oled_p6x8str(0, 4, "Normal5:");
    oled_int16(50, 4, (int16) InductanceValue_Normal[4]);
    oled_p6x8str(0, 5, "Normal6:");
    oled_int16(50, 5, (int16) InductanceValue_Normal[5]);
};