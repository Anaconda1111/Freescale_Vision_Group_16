//
// Created by xcs on 2021-03-04.
//

#include "PWM.h"
#include "fsl_pwm.h"
#include "port.h"
#include "zf_iomuxc.h"
#define PWM_SRC_CLK_FREQ CLOCK_GetFreq(kCLOCK_IpgClk) //定义PWM输入时钟源频率
#define PWM_PIN_CONF SPEED_100MHZ | KEEPER_EN | DSE_R0_6 //配置PWM引脚默认配置
void User_PWMInit(PWMPin_enum PWMch, uint32 freq, uint32 duty) {
  uint8 pwm_num;
  uint8 pwm_module;
  uint8 pwm_module_ch;
  uint16 temp_prsc;
  pwm_config_t pwmConfig;

  switch (PWMch) {
  case Motor_RT_R: {
    iomuxc_pinconf(MotorPWM_Return_R, ALT1, PWM_PIN_CONF);
    pwm_num = 2;
    pwm_module = 3;
    pwm_module_ch = 1;
    break;
  }
  case Motor_GO_R: {
    iomuxc_pinconf(MotorPWM_Go_R, ALT1, PWM_PIN_CONF);
    pwm_num = 1;
    pwm_module = 3;
    pwm_module_ch = 0;
    break;
  }
  case Motor_RT_L: {
    iomuxc_pinconf(MotorPWM_Return_L, ALT1, PWM_PIN_CONF);
    pwm_num = 1;
    pwm_module = 3;
    pwm_module_ch = 1;
    break;
  }
  case Motor_GO_L: {
    iomuxc_pinconf(MotorPWM_Go_L, ALT1, PWM_PIN_CONF);
    pwm_num = 1;
    pwm_module = 0;
    pwm_module_ch = 1;
    break;
  }
  case SteerPWM: {
    iomuxc_pinconf(SteerPWM_Pin, ALT1, PWM_PIN_CONF);
    pwm_num = 4;
    pwm_module = 2;
    pwm_module_ch = 1;
    break;
  }
  }

  PWM_GetDefaultConfig(&pwmConfig);

  pwmConfig.reloadLogic = kPWM_ReloadPwmFullCycle;
  pwmConfig.pairOperation = kPWM_Independent;
  pwmConfig.enableDebugMode = true;

  //计算分频系数
  temp_prsc = (PWM_SRC_CLK_FREQ / freq) >> 16;
  if (1 >= temp_prsc)
    pwmConfig.prescale = kPWM_Prescale_Divide_1;
  else if (2 >= temp_prsc)
    pwmConfig.prescale = kPWM_Prescale_Divide_2;
  else if (4 >= temp_prsc)
    pwmConfig.prescale = kPWM_Prescale_Divide_4;
  else if (8 >= temp_prsc)
    pwmConfig.prescale = kPWM_Prescale_Divide_8;
  else if (16 >= temp_prsc)
    pwmConfig.prescale = kPWM_Prescale_Divide_16;
  else if (32 >= temp_prsc)
    pwmConfig.prescale = kPWM_Prescale_Divide_32;
  else if (64 >= temp_prsc)
    pwmConfig.prescale = kPWM_Prescale_Divide_64;
  else if (128 >= temp_prsc)
    pwmConfig.prescale = kPWM_Prescale_Divide_128;
  else {
    assert(0); //频率过小 或者IPG频率过高
  }

  if (PWM_Init(PWMPTR[pwm_num], (pwm_submodule_t)pwm_module, &pwmConfig) ==
      kStatus_Fail) //第一次初始化便于打开时钟
  {
    assert(0); //初始化失败
  }
  PWM_Deinit(PWMPTR[pwm_num], (pwm_submodule_t)pwm_module);

  if (PWM_Init(PWMPTR[pwm_num], (pwm_submodule_t)pwm_module, &pwmConfig) ==
      kStatus_Fail) //重新初始化设置正确的参数
  {
    assert(0); //初始化失败
  }

  //设置频率占空比等参数
  pwm_signal_param_t pwmSignal;

  pwmSignal.pwmChannel = (pwm_channels_t)(pwm_module_ch);
  pwmSignal.level = kPWM_HighTrue;
  pwmSignal.dutyCyclePercent = duty; // PWM_DUTY_MAX
  pwmSignal.deadtimeValue =
      0; //((uint64_t)PWM_SRC_CLK_FREQ * 650) / 1000000000;

  //清除LOAD OKAY位  以设置新的参数
  PWM_SetPwmLdok(PWMPTR[pwm_num], (pwm_module_control_t)(1 << (pwm_module)),
                 false);
  PWM_SetupPwm(PWMPTR[pwm_num], (pwm_submodule_t)pwm_module, &pwmSignal, 1,
               kPWM_EdgeAligned, freq, PWM_SRC_CLK_FREQ);
  //设置LOAD OKAY位  以更新设置
  PWM_SetPwmLdok(PWMPTR[pwm_num], (pwm_module_control_t)(1 << (pwm_module)),
                 true);

  //启动定时器
  PWM_StartTimer(PWMPTR[pwm_num], (pwm_module_control_t)(1 << (pwm_module)));

  PWMPTR[pwm_num]->SM[pwm_module].DISMAP[0] = 0;
}

void User_PWMDuty(PWMPin_enum PWMch, uint32 duty) {
  uint8 pwm_num;
  uint8 pwm_module;
  uint8 pwm_module_ch;
  switch (PWMch) {
  case Motor_RT_R: {
    pwm_num = 2;
    pwm_module = 3;
    pwm_module_ch = 1;
    break;
  }
  case Motor_RT_L: {
    pwm_num = 1;
    pwm_module = 3;
    pwm_module_ch = 1;
    break;
  }
  case Motor_GO_R: {
    pwm_num = 1;
    pwm_module = 3;
    pwm_module_ch = 0;
    break;
  }
  case Motor_GO_L: {
    pwm_num = 1;
    pwm_module = 0;
    pwm_module_ch = 1;
    break;
  }
  case SteerPWM: {
    pwm_num = 4;
    pwm_module = 2;
    pwm_module_ch = 1;
  }
  }

  //清除LOAD OKAY位  以设置新的参数
  PWM_SetPwmLdok(PWMPTR[pwm_num], (pwm_module_control_t)(1 << (pwm_module)),
                 false);
  PWM_UpdatePwmDutycycle(PWMPTR[pwm_num], (pwm_submodule_t)pwm_module,
                         (pwm_channels_t)(pwm_module_ch), kPWM_EdgeAligned,
                         duty);
  //设置LOAD OKAY位  以更新设置
  PWM_SetPwmLdok(PWMPTR[pwm_num], (pwm_module_control_t)(1 << (pwm_module)),
                 true);
}