//
// Created by xcs on 2021-03-08.
//

#include "Interactive.h"
#include "port.h"


uint8 key_scan() {


    if (gpio_get(KEY1) == 0) {
        return key1press;
    } else if (gpio_get(KEY2) == 0) {
        return key2press;
    } else if (gpio_get(KEY3) == 0) {
        return key3press;
    } else if (gpio_get(KEY4) == 0) {
        return key4press;
    } else
        return 0;//无按键按下
}

uint8 bm_scan(void) {
    uint8 bm1flag;
    uint8 bm2flag;
    static uint8 status = 0x00;
    bm1flag = gpio_get(DialSwitch1);
    bm2flag = gpio_get(DialSwitch2);
    if (bm2flag == 0 && bm1flag == 1) {
        status = 0x01;
    } else if (bm2flag == 1 && bm1flag == 0) {
        status = 0x10;
    } else if (bm2flag == 1 && bm1flag == 1) {
        status = 0x11;
    }
    return status;
}

void Interactive() {
    uint8 Status = bm_scan();
    switch (Status) {
        case 0X00: {
            switch (key_scan()) {
                case key1press: {
                }
                    break;
                case key2press: {
                }
                    break;
                case key3press: {
                }
                    break;
                case key4press: {
                }
                    break;
            }
        }
        case 0x01: {
            switch (key_scan()) {
                case key1press: {
                }
                    break;
                case key2press: {
                }
                    break;
                case key3press: {
                }
                    break;
                case key4press: {
                }
                    break;
            }
        }
        case 0x10: {
            switch (key_scan()) {
                case key1press: {
                }
                    break;
                case key2press: {
                }
                    break;
                case key3press: {
                }
                    break;
                case key4press: {
                }
                    break;
            }
        }
        case 0x11: {
            switch (key_scan()) {
                case key1press: {
                }
                    break;
                case key2press: {
                }
                    break;
                case key3press: {
                }
                    break;
                case key4press: {
                }
                    break;
            }
        }

    }
}