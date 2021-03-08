//
// Created by xcs on 2021-03-07.
//

#include "math.h"
float FastSqrt(float x) {
    float a = x;
    unsigned int i = *(unsigned int *) &x;
    i = (i + 0x3f76cf62) >> 1UL;
    x = *(float *) &i;
    x = (x + a / x) * 0.5f;
    return x;
}