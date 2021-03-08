//
// Created by xcs on 2021-03-08.
//

#ifndef RT1064_INTERACTIVE_H
#define RT1064_INTERACTIVE_H

#include "zf_gpio.h"

#define key1press 1
#define key2press 2
#define key3press 3
#define key4press 4

uint8 key_scan();

uint8 bm_scan(void);

void Interactive();

#endif //RT1064_INTERACTIVE_H
