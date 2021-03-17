//
// Created by xcs on 2021-03-05.
//

#ifndef RT1064_CODE_UART_H_
#define RT1064_CODE_UART_H_


void example_uart_callback(LPUART_Type *base, lpuart_handle_t *handle, status_t status, void *userData);
void send_steer_data(void);
void send_motor_data(void);






#endif // RT1064_CODE_UART_H_
