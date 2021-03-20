

void Garageout_exmple(){
    static int flag=0;

    pwm_duty(SteerPWM_CH, MiddleSteer_PWM);
    pwm_duty(MotorPWM_Go_L_CH, MotorOutGarage_PWM);
    pwm_duty(MotorPWM_Go_R_CH, MotorOutGarage_PWM);

    while(1) {


        if (flag == 0 && InductanceValue_Normal[4] > 3 / 4 * Inductance_MAXValue[4]) {
            flag = 1;
            pwm_duty(SteerPWM_CH, MiddleSteer_PWM);
        }

        if (flag == 1 && InductanceValue_Normal[4] < 1 / 2 * Inductance_MAXValue[4]) {
            pwm_duty(SteerPWM_CH, SteerOutGarage_PWM);
            flag = 2;

        }
        if (flag == 2 && InductanceValue_Normal[4] > 4 / 5 * Inductance_MAXValue[4]) {
            flag = 3;
            pwm_duty(SteerPWM_CH, MiddleSteer_PWM);
        }
        if (flag == 3) {
            break;
        }
    }
}
