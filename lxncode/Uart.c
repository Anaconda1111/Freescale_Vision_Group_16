//
// Created by xcs on 2021-03-05.
//

#include "Uart.h"
#include "zf_uart.h"



extern uint16 SteerPWMDuty; //���PWMռ�ձ�
extern uint16 Motor_GO_L_PWM;
extern uint16 Motor_GO_R_PWM;

uint8 steerdata =(uint8)(SteerPWMDuty - SteerMIN) * 100/(SteerMAX - SteerMIN);//�������ֵת��Ϊ0~100�㣬�м�ֵΪ50��

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
        //�����Ѿ���д�뵽�� ֮ǰ���õ�BUFF��
        //������ʹ�õ�BUFFΪ example_rx_buffer
        uart_data = example_rx_buffer;//������ȡ��
    }

    handle->rxDataSize = example_receivexfer.dataSize;  //��ԭ����������
    handle->rxData = example_receivexfer.data;          //��ԭ��������ַ
}



void send_steer_data(void)
{

    example_uart_callback(LPUART_Type *base, lpuart_handle_t *handle, status_t status, void *userData);

    //��ʼ������   ������Ϊ115200 TXΪD16 RXΪD17
    uart_init (USART_8, 115200,UART8_TX_D16,UART8_RX_D17);
    NVIC_SetPriority(LPUART8_IRQn,15);         //���ô����ж����ȼ� ��Χ0-15 ԽС���ȼ�Խ��
    uart_rx_irq(USART_8,1);

    //���ô��ڽ��յĻ�����������������
    example_receivexfer.dataSize = 1;
    example_receivexfer.data = &example_rx_buffer;

    //�����жϺ����������
    uart_set_handle(USART_8, &example_g_lpuartHandle, example_uart_callback, NULL, 0, example_receivexfer.data, 1);





    //�����ֽڷ���
    uart_putchar(USART_8,steerdata);
    //���๦�ܺ��� ���в���zf_uart�ļ�
    systick_delay_ms(100);



}


void send_motor_data(void)
{
    example_uart_callback(LPUART_Type *base, lpuart_handle_t *handle, status_t status, void *userData);

    //��ʼ������   ������Ϊ115200 TXΪD16 RXΪD17
    uart_init (USART_8, 115200,UART8_TX_D16,UART8_RX_D17);
    NVIC_SetPriority(LPUART8_IRQn,15);         //���ô����ж����ȼ� ��Χ0-15 ԽС���ȼ�Խ��
    uart_rx_irq(USART_8,1);

    //���ô��ڽ��յĻ�����������������
    example_receivexfer.dataSize = 1;
    example_receivexfer.data = &example_rx_buffer;

    //�����жϺ����������
    uart_set_handle(USART_8, &example_g_lpuartHandle, example_uart_callback, NULL, 0, example_receivexfer.data, 1);





    //�����ֽڷ���
    uart_putchar(USART_8,motordata_L);
    uart_putchar(USART_8,motordata_R);
    uart_putchar(USART_8,0000);//�ָ�����
    //���๦�ܺ��� ���в���zf_uart�ļ�
    systick_delay_ms(100);

}





