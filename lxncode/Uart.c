//
// Created by xcs on 2021-03-05.
//

#include "Uart.h"
#include "zf_uart.h"



extern uint16 SteerPWMDuty; //舵机PWM占空比
extern uint16 Motor_GO_L_PWM;
extern uint16 Motor_GO_R_PWM;

uint8 steerdata =(uint8)(SteerPWMDuty - SteerMIN) * 100/(SteerMAX - SteerMIN);//将舵机的值转化为0~100°，中间值为50°

uint8 motordata_R=(uint8)(Motor_GO_R_PWM - MotorPWM_MIN) * 100/(MotorPWM_MAX - MotorPWM_MIN);//0~100
uint8 motordata_L=(uint8)(Motor_GO_L_PWM - MotorPWM_MIN) * 100/(MotorPWM_MAX - MotorPWM_MIN);//0~100



uint8 example_rx_buffer;
lpuart_transfer_t   example_receivexfer;
lpuart_handle_t     example_g_lpuartHandle;


uint8 uart_data;

void example_uart_callback(LPUART_Type *base, lpuart_handle_t *handle, status_t status, void *userData)
{
    if(kStatus_LPUART_RxIdle == status)
    {
        //数据已经被写入到了 之前设置的BUFF中
        //本例程使用的BUFF为 example_rx_buffer
        uart_data = example_rx_buffer;//将数据取出
    }

    handle->rxDataSize = example_receivexfer.dataSize;  //还原缓冲区长度
    handle->rxData = example_receivexfer.data;          //还原缓冲区地址
}



void send_steer_data(void)
{

    example_uart_callback(LPUART_Type *base, lpuart_handle_t *handle, status_t status, void *userData);

    //初始换串口   波特率为115200 TX为D16 RX为D17
    uart_init (USART_8, 115200,UART8_TX_D16,UART8_RX_D17);
    NVIC_SetPriority(LPUART8_IRQn,15);         //设置串口中断优先级 范围0-15 越小优先级越高
    uart_rx_irq(USART_8,1);

    //配置串口接收的缓冲区及缓冲区长度
    example_receivexfer.dataSize = 1;
    example_receivexfer.data = &example_rx_buffer;

    //设置中断函数及其参数
    uart_set_handle(USART_8, &example_g_lpuartHandle, example_uart_callback, NULL, 0, example_receivexfer.data, 1);





    //串口字节发送
    uart_putchar(USART_8,steerdata);
    //更多功能函数 自行查阅zf_uart文件
    systick_delay_ms(100);



}


void send_motor_data(void)
{
    example_uart_callback(LPUART_Type *base, lpuart_handle_t *handle, status_t status, void *userData);

    //初始换串口   波特率为115200 TX为D16 RX为D17
    uart_init (USART_8, 115200,UART8_TX_D16,UART8_RX_D17);
    NVIC_SetPriority(LPUART8_IRQn,15);         //设置串口中断优先级 范围0-15 越小优先级越高
    uart_rx_irq(USART_8,1);

    //配置串口接收的缓冲区及缓冲区长度
    example_receivexfer.dataSize = 1;
    example_receivexfer.data = &example_rx_buffer;

    //设置中断函数及其参数
    uart_set_handle(USART_8, &example_g_lpuartHandle, example_uart_callback, NULL, 0, example_receivexfer.data, 1);





    //串口字节发送
    uart_putchar(USART_8,motordata_L);
    uart_putchar(USART_8,motordata_R);
    uart_putchar(USART_8,0000);//分隔数据
    //更多功能函数 自行查阅zf_uart文件
    systick_delay_ms(100);

}





