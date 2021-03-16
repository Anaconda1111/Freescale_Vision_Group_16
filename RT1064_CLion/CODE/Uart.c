//
// Created by xcs on 2021-03-05.
//

#include "Uart.h"

void SendDataPackage(uint8 CurrentValue, uint8 TargetValue) {
    uart_putchar(USART_8, 1);
    uart_putchar(USART_8, CurrentValue);
    uart_putchar(USART_8, TargetValue);
}