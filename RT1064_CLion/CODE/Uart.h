//
// Created by xcs on 2021-03-05.
//

#ifndef RT1064_CODE_UART_H_
#define RT1064_CODE_UART_H_

#include "zf_uart.h"

void SendDataPackage(uint8 CurrentValue, uint8 TargetValue);

void SendDataPackage_int16(int16 *Data, uint8 len);

void GetCameraMessage();

#endif // RT1064_CODE_UART_H_
