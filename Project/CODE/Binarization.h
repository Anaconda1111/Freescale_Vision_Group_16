//
// Created by xcs on 2021-06-04.
//

#ifndef RT1064_BINARIZATION_H
#define RT1064_BINARIZATION_H

#endif //RT1064_BINARIZATION_H

#include "SEEKFREE_MT9V03X.h"
#include "SEEKFREE_MT9V03X_CSI.h"
#include "fastmath.h"

void ImageBinary();

uint8 OtsuThreshold();

void LocalBinary(uint32 PixSum, uint8 PixNum, uint8 i, uint8 j);

void LocalThreshold();