//
// Created by xcs on 2021-03-24.
//

#include "ART.h"
#include "port.h"
#include "zf_pwm.h"
#include "zf_systick.h"
#include "Steer.h"
#include "Interactive.h"
#include "ADC.h"
#include "Motor.h"

extern uint8 Camera;
extern int16 encoder_value_L;
extern int16 encoder_value_R;
extern uint32 TridentFlag;
extern int16 Island_Flag;
extern PID_Struct Steer_PID;
extern Filter_Struct Steer_Filter;
uint16 YunTai2SteerPWMDuty = 0;
uint8 NumberRecFlag = 1;
uint8 NumberRecResult = 0;
uint16 TridentTurn = 0;
uint8 FirstTrident = 0;
uint16 GarageInFlag = 650;

void AprilTagMessageHandle() {
    //第一阶段，先减速，同时根据识别到的Apriltag转动云台舵机
    if (Camera == InLeft || Camera == InRight) {
        if (Camera == InLeft)
            pwm_duty(Yuntai2PWM_CH, MaxYunTai2SteerPWM);
        else
            pwm_duty(Yuntai2PWM_CH, MinYunTai2SteerPWM);
        SlowDown();
        uart_putchar(USART_6, 1);
        //第二阶段：缓慢行车搜索黑框并识别图片
        systick_delay_ms(20);
        while (Camera != IsAnimal && Camera != IsFruit)
            GetCameraMessage();
        //第三阶段：识别到图片后执行任务
        if (Camera == IsAnimal) {
            pwm_duty(MotorPWM_Go_L_CH, 7000);
            pwm_duty(MotorPWM_Go_R_CH, 7000);
            systick_delay_ms(1500);
        } else { ;//激光打靶
        }
        pwm_duty(Yuntai2PWM_CH, MidYunTai2SteerPWM);
        Camera = 0;
    }
}


void TridentMessageHandle() {
    // if (FirstTrident == 0) {
    //     if (Camera == Trident_Left || Camera == Trident_Right)
    //         NumberRecResult = Camera;
    //     if (TridentTest()) {
    //         gpio_toggle(BEEF);
    //         FirstTrident++;
    //         if (NumberRecResult == Trident_Left)
    //             pwm_duty(SteerPWM_CH, SteerMIN);
    //         else
    //             pwm_duty(SteerPWM_CH, SteerMAX);
    //         systick_delay_ms(80);
    //     }
    // } else if (FirstTrident) {
    //     if (TridentTest()) {
    //         gpio_toggle(BEEF);
    //         if (NumberRecResult == Trident_Left)
    //             pwm_duty(SteerPWM_CH, SteerMAX);
    //         else
    //             pwm_duty(SteerPWM_CH, SteerMIN);
    //         systick_delay_ms(80);
    //         FirstTrident++;
    //     }
    // }



    // if (TridentFlag != 1) {
    //     if (Island_Flag != 1 && Camera == Find_Trident && StraightLine()) {
    //         TridentFlag = 1;
    //         Camera = 0;
    //         uart_putchar(USART_6, 1);
    //     } else {
    //         Camera = 0;
    //         uart_putchar(USART_6, 0);
    //         return;
    //     }
    // }
    // if (TridentFlag == 1 && FirstTrident == 0) {
    //     SlowDown2();
    //     while (encoder_value_L > 10 || encoder_value_R > 10)
    //         Motor_value_get();
    //     uart_putchar(USART_6, 1);
    //     while (Camera != Trident_Left && Camera != Trident_Right)
    //         GetCameraMessage();
    //     NumberRecResult = Camera;
    //     // TridentTurn = 0xffff;
    //     if (Camera == Trident_Left)
    //         pwm_duty(SteerPWM_CH, 3740);
    //     else
    //         pwm_duty(SteerPWM_CH, 3000);
    //     pwm_duty(MotorPWM_Go_L_CH, 12000);
    //     pwm_duty(MotorPWM_Go_R_CH, 12000);
    //     systick_delay_ms(500);
    //     FirstTrident++;
    //     // while (TridentTurn) {
    //     //     SteerCtrl(Steer_PID, Steer_Filter);
    //     // }
    //     gpio_toggle(BEEF);
    //     uart_putchar(USART_6, 1);
    //     TridentFlag = 0;
    // } else if (FirstTrident == 1) {
    //     FirstTrident++;
    //     if (TridentTest()) {
    //         if (NumberRecResult == Trident_Left)
    //             pwm_duty(SteerPWM_CH, SteerMIN);
    //         else
    //             pwm_duty(SteerPWM_CH, SteerMAX);
    //         systick_delay_ms(100);
    //     }
    // }

    if (TridentTest()) {
        TridentFlag = 3900;
        if (FirstTrident == 0) {
            SlowDown2();
            systick_delay_ms(200);
            uart_putchar(USART_6, 0XFF);
            while (Camera != Trident_Left && Camera != Trident_Right)
                GetCameraMessage();
            NumberRecResult = Camera;
            Reversing_();
            if (Camera == Trident_Left) {
                pwm_duty(SteerPWM_CH, 3740);
                pwm_duty(MotorPWM_Go_L_CH, 8000);
                pwm_duty(MotorPWM_Go_R_CH, 14000);
            } else {
                pwm_duty(SteerPWM_CH, 3000);
                pwm_duty(MotorPWM_Go_L_CH, 14000);
                pwm_duty(MotorPWM_Go_R_CH, 8000);
            }
            systick_delay_ms(500);
            FirstTrident++;
        } else {
            FirstTrident++;
            if (NumberRecResult == Trident_Left)
                pwm_duty(SteerPWM_CH, SteerMIN);
            else
                pwm_duty(SteerPWM_CH, SteerMAX);
            systick_delay_ms(150);
        }
    }
}

void GarageIn() {
    // if (!TridentFlag && FirstTrident >= 2) {
    //     uart_putchar(USART_6, 0XFE);
    //     if (Camera == FindFinishLine) {
    //         systick_delay_ms(50);
    //         pwm_duty(SteerPWM_CH, SteerOutGarage_PWM);
    //         pwm_duty(MotorPWM_Go_L_CH, MotorOutGarage_PWM);
    //         pwm_duty(MotorPWM_Go_R_CH, MotorOutGarage_PWM);
    //         systick_delay_ms(100);
    //         pwm_duty(MotorPWM_Go_L_CH, 0);
    //         pwm_duty(MotorPWM_Go_R_CH, 0);
    //         while (1)
    //             gpio_set(BEEF, 1);
    //     }
    // }
    if (FirstTrident >= 2 && TridentFlag == 0) {
        GarageInFlag--;
    }
    if (!GarageInFlag) {
        pwm_duty(SteerPWM_CH, SteerOutGarage_PWM - 100);
        pwm_duty(MotorPWM_Go_L_CH, MotorOutGarage_PWM - 3000);
        pwm_duty(MotorPWM_Go_R_CH, MotorOutGarage_PWM - 3000);
        systick_delay_ms(150);
        pwm_duty(MotorPWM_Go_L_CH, 0);
        pwm_duty(MotorPWM_Go_R_CH, 0);
        while (1)
            gpio_set(BEEF, 0);
    }


}


void SlowDown() {
    pwm_duty(MotorPWM_Go_L_CH, 0);
    pwm_duty(MotorPWM_Go_R_CH, 0);
    for (int i = -2000; i < 6000; i += 100) {
        if (i < 0) {
            int j = i * -1;
            pwm_duty(MotorPWM_Return_L_CH, j);
            pwm_duty(MotorPWM_Return_R_CH, j);
            systick_delay_ms(5);
        } else if (i == 0) {
            pwm_duty(MotorPWM_Return_L_CH, 0);
            pwm_duty(MotorPWM_Return_R_CH, 0);
        } else {
            pwm_duty(MotorPWM_Go_L_CH, i);
            pwm_duty(MotorPWM_Go_R_CH, i);
        }
    }
}

void SlowDown2() {
    pwm_duty(MotorPWM_Go_L_CH, 0);
    pwm_duty(MotorPWM_Go_R_CH, 0);
    pwm_duty(MotorPWM_Return_L_CH, 30000);
    pwm_duty(MotorPWM_Return_R_CH, 30000);
    systick_delay_ms(420);
    pwm_duty(MotorPWM_Return_R_CH, 0);
    pwm_duty(MotorPWM_Return_L_CH, 0);
    while (encoder_value_R != 0 && encoder_value_L != 0)
        Motor_value_get();
    systick_delay_ms(50);
    // for (int i = -20000; i <= -15000; i += 100) {
    //     systick_delay_ms(5);
    //     if (i < 0) {
    //         int j = i * -1;
    //         pwm_duty(MotorPWM_Return_R_CH, j);
    //         pwm_duty(MotorPWM_Return_L_CH, j);
    //     }
    // }

}

void Reversing_() {
    pwm_duty(MotorPWM_Return_R_CH, 10000);
    pwm_duty(MotorPWM_Return_L_CH, 10000);
    systick_delay_ms(230);
    pwm_duty(MotorPWM_Return_R_CH, 0);
    pwm_duty(MotorPWM_Return_L_CH, 0);
}