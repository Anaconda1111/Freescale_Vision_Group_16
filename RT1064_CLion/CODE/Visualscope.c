//
// Created by xcs on 2021-03-10.
//

#include "Visualscope.h"
// #include "SEEKFREE_VIRSCO.h"
#include "zf_uart.h"

unsigned short CRC_CHECK(unsigned char *Buf, unsigned char CRC_CNT) {
    unsigned short CRC_Temp;
    int i, j;
    CRC_Temp = 0xffff;

    for (i = 0; i < CRC_CNT; i++) {
        CRC_Temp ^= Buf[i];
        for (j = 0; j < 8; j++) {
            if (CRC_Temp & 0x01)
                CRC_Temp = (CRC_Temp >> 1) ^ 0xA001;
            else
                CRC_Temp = CRC_Temp >> 1;
        }
    }
    return (CRC_Temp);
}


void VisualScope_Output(float data1, float data2, float data3, float data4) {
    int temp[4] = {0};
    unsigned int temp1[4] = {0};
    unsigned char databuf[10] = {0};
    int i;
    unsigned short CRC16 = 0;

    temp[0] = (int) data1;
    temp[1] = (int) data2;
    temp[2] = (int) data3;
    temp[3] = (int) data4;

    temp1[0] = (unsigned int) temp[0];
    temp1[1] = (unsigned int) temp[1];
    temp1[2] = (unsigned int) temp[2];
    temp1[3] = (unsigned int) temp[3];

    for (i = 0; i < 4; i++) {
        databuf[i * 2] = (unsigned char) (temp1[i] % 256);
        databuf[i * 2 + 1] = (unsigned char) (temp1[i] / 256);
    }

    CRC16 = CRC_CHECK(databuf, 8);
    databuf[8] = CRC16 % 256;
    databuf[9] = CRC16 / 256;
    for (i = 0; i < 10; i++) {
        uart_putchar(USART_8, databuf[i]);
    }
}

void display(void) {

    float OutData[4] = {0};
    OutData[0] = 1000;
    OutData[1] = 2000;
    OutData[2] = 3000;
    OutData[3] = 4000;
    VisualScope_Output(OutData[0], OutData[1], OutData[2], OutData[3]);
}