//
// Created by xcs on 2021-06-04.
//
#include "headfile.h"
#include "Binarization.h"
#include "fastmath.h"

extern uint8 (*mt9v03x_csi_image)[MT9V03X_CSI_W];

uint8 T =0;
uint16 W = MT9V03X_CSI_W;
uint16 H = MT9V03X_CSI_H;
//uint8 LT_NewImage[120][188] = {0};
uint8 otsu_image[MT9V03X_CSI_H][MT9V03X_CSI_W]={0}; 

// ��򷨽����������ֵ T 
uint8 OtsuThreshold() {
    uint8 T = 0;

    uint32 N0 = 0;
    uint32 N1 = 0;
    uint32 N = W * H;
    double GraySum0 = 0;
    double GraySum1 = 0;
    double W0 = 0;
    double W1 = 0;
    double U0 = 0;
    double U1 = 0;
    double Tempg = -1;
    double g = -1;
    uint16 graynum[256] = {0};
    for (int i = 0; i < H; ++i) {
        for (int j = 0; j < W; ++j) {
            uint8 pix = mt9v03x_csi_image[i][j];
            graynum[pix]++;
        }
    }

    for (int i = 0; i < 256; ++i) {
        N0 += graynum[i];
        N1 = N - N0;
        GraySum0 = 0;
        GraySum1 = 0;
        if (N1 == 0 || N0 == 0)
            continue;
        W0 = (double) N0 / (double) N;
        W1 = 1 - W0;
        for (int j = 0; j <= i; ++j) {
            GraySum0 += j * graynum[j];
        }
        for (int j = i + 1; j < 256; ++j) {
            GraySum1 += j * graynum[j];
        }
        U0 = GraySum0 / (double) N0;
        U1 = GraySum1 / (double) N1;
        g = W0 * W1 * FastPow(U0 - U1, 2);
        if (Tempg < g) {
            Tempg = g;
            T = i;
        }
    }

    return T;
}

//ͼ���ֵ��,��򷨶�ֵ�� 
//�Ҷ�ֵԽ��Ϊ��ɫ
void ImageBinary() {
        T = OtsuThreshold();
    for (int i = 0; i < H; ++i) {
        for (int j = 0; j < W; ++j) {
            if (mt9v03x_csi_image[i][j] <= T)
                otsu_image[i][j] = 1;
            else
                otsu_image[i][j] = 0;
        }
    }
}

/*
//�ֲ���ֵ����ֵ��
void LocalThreshold() {
    uint32 PixSum = 0;
    uint8 PixNum = 0;
    for (int i = 0; i < H; ++i) {
        for (int j = 0; j < W; ++j) {
            PixNum = 0;
            PixSum = 0;
            if (i != 0 && j != 0 && i != H - 1 && j != W - 1) {//�����ڱ߿�����ص�
                PixSum = PixSum + mt9v03x_csi_image[i - 1][j - 1] + mt9v03x_csi_image[i - 1][j] + mt9v03x_csi_image[i - 1][j + 1] +
                         mt9v03x_csi_image[i][j - 1] + mt9v03x_csi_image[i][j + 1] +
                         mt9v03x_csi_image[i + 1][j - 1] + mt9v03x_csi_image[i + 1][j] + mt9v03x_csi_image[i + 1][j + 1];
                PixNum = 8;
                LocalBinary(PixSum, PixNum, i, j);

            } else if (i == 0 && j == 0) {//���Ͻǵ�
                PixSum = PixSum + mt9v03x_csi_image[i][j + 1] + mt9v03x_csi_image[i + 1][j] + mt9v03x_csi_image[i + 1, j + 1];
                PixNum = 3;
                LocalBinary(PixSum, PixNum, i, j);
            } else if (i == 0 && j == W - 1) {//���Ͻǵ�
                PixSum = PixSum + mt9v03x_csi_image[i][j - 1] + mt9v03x_csi_image[i + 1][j - 1] + mt9v03x_csi_image[i + 1][j];
                PixNum = 3;
                LocalBinary(PixSum, PixNum, i, j);
            } else if (i == H - 1 && j == 0) {//���½ǵ�
                PixSum = PixSum + mt9v03x_csi_image[i - 1][j] + mt9v03x_csi_image[i - 1][j + 1] + mt9v03x_csi_image[i][j + 1];
                PixNum = 3;
                LocalBinary(PixSum, PixNum, i, j);
            } else if (i == H - 1 && j == W - 1) {//���½ǵ�
                PixSum = PixSum + mt9v03x_csi_image[i - 1][j - 1] + mt9v03x_csi_image[i - 1][j] + mt9v03x_csi_image[i][j - 1];
                PixNum = 3;
                LocalBinary(PixSum, PixNum, i, j);
            } else if (i == 0) {//��һ��
                PixSum = PixSum + mt9v03x_csi_image[i][j - 1] + mt9v03x_csi_image[i][j + 1] +
                         mt9v03x_csi_image[i + 1][j - 1] + mt9v03x_csi_image[i + 1][j] + mt9v03x_csi_image[i + 1][j + 1];
                PixNum = 5;
                LocalBinary(PixSum, PixNum, i, j);
            } else if (i == H - 1) {//���һ��
                PixSum = PixSum + mt9v03x_csi_image[i - 1][j - 1] + mt9v03x_csi_image[i - 1][j] + mt9v03x_csi_image[i - 1][j + 1] +
                         mt9v03x_csi_image[i][j - 1] + mt9v03x_csi_image[i][j + 1];
                PixNum = 5;
                LocalBinary(PixSum, PixNum, i, j);
            } else if (j == 0) {//��һ��
                PixSum = PixSum + mt9v03x_csi_image[i - 1][j] + mt9v03x_csi_image[i - 1][j + 1] +
                         mt9v03x_csi_image[i][j + 1] +
                         mt9v03x_csi_image[i + 1][j] + mt9v03x_csi_image[i + 1][j + 1];
                PixNum = 5;
                LocalBinary(PixSum, PixNum, i, j);
            } else if (j == W - 1) {//���һ��
                PixSum = PixSum + mt9v03x_csi_image[i - 1][j - 1] + mt9v03x_csi_image[i - 1][j] +
                         mt9v03x_csi_image[i][j - 1] +
                         mt9v03x_csi_image[i + 1][j - 1] + mt9v03x_csi_image[i + 1][j];
                PixNum = 5;
                LocalBinary(PixSum, PixNum, i, j);
            }

        }
    }
}

//�ֲ���ֵ����ֵ��
void LocalBinary(uint32 PixSum, uint8 PixNum, uint8 i, uint8 j) {
    float Average = (float) PixSum / (float) PixNum;
    if (FastABS(mt9v03x_csi_image[i][j] - Average) > 20)
        LT_NewImage[i][j] = 0;
    else
        LT_NewImage[i][j] = 255;
}
*/