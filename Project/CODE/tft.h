#ifndef _TFT_H
#define _TFT_H

#include "headfile.h"


void tft_showimage(void);
void tft_show_otsu_image(uint8 width, uint8 high, uint8 dis_width, uint8 dis_high);

void tft_show_border(uint8 width, uint8 high, uint8 dis_width, uint8 dis_high);

#endif