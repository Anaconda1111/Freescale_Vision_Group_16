#include "headfile.h"
#include "tft.h"
#include "image.h"

extern uint8 (*mt9v03x_csi_image)[MT9V03X_CSI_W];

extern uint8 otsu_image[MT9V03X_CSI_H][MT9V03X_CSI_W];


extern uint8 left_border[MT9V03X_CSI_H];  //������߽�
extern uint8 right_border[MT9V03X_CSI_H]; //�����ұ߽�
extern uint8 middle_line[MT9V03X_CSI_H];  //�����е�����

//  @param      x     	        ����x��������  ������Χ 0 -��TFT_X_MAX-1��   160-1=159
//  @param      y     	        ����y��������  ������Χ 0 -��TFT_Y_MAX/16-1�� 128/16 -1=7





//����ͷͼ�������ʾ
void tft_showimage(void)
{
  if(mt9v03x_csi_finish_flag)
  {      
    
    lcd_displayimage032_zoom(mt9v03x_csi_image[0], MT9V03X_CSI_W, MT9V03X_CSI_H, 160, 128);
      
   }
  
}

//width:ͼ���ȣ�high:ͼ��߶ȣ�dis_width:��ʾ��ȣ� dis_high:��ʾ�߶�
void tft_show_otsu_image(uint8 width, uint8 high, uint8 dis_width, uint8 dis_high)
{
  
  uint8 i,j;
  uint8 temp1,temp2;
  
  //����Һ����Ļ��ʾ��Χ
  lcd_set_region(0,0,dis_width-1,dis_high-1);
  
  for(i=0;i<dis_high;i++)
  {
    for(j=0;j<dis_width;j++)
    {
      temp1 = (uint8)(i*high/dis_high);
      temp2 = (uint8)(j*width/dis_width);
      
      if(otsu_image[temp1][temp2] ==1)
      {
        lcd_drawpoint(j,i,BLACK);
      }
      else
      {
        lcd_drawpoint(j,i,WHITE);
      }
    }
      
  }
}
  
//tft ��ʾ���ұ߽磬����,�ڶ�ֵ��ͼ����ʾ�����
void tft_show_border(uint8 width, uint8 high, uint8 dis_width, uint8 dis_high)
{
    uint8  i=0;
    uint8  temp1=0;
    uint8  temp2=0;
    
  //����Һ����Ļ��ʾ��Χ
  lcd_set_region(0,0,dis_width-1,dis_high-1);
  
     
    for(i=0;i<dis_high;i++)
    {
        temp1 = (uint8)(i*high/dis_high);
        temp2 = (uint8)(left_border[temp1]*dis_width/width);
        
        lcd_drawpoint(dis_width - temp2-1,dis_high - i-1,YELLOW);
       
    }
    
    for(i=0;i<dis_high;i++)
    {
        temp1 = (uint8)(i*high/dis_high);
        temp2 = (uint8)(right_border[temp1]*dis_width/width);
        
        lcd_drawpoint(dis_width - temp2-1,dis_high - i-1,RED);
       
    }

    for(i=0;i<dis_high;i++)
    {
        temp1 = (uint8)(i*high/dis_high);
        temp2 = (uint8)(middle_line[temp1]*dis_width/width);
        
        lcd_drawpoint(dis_width - temp2-1,dis_high - i-1,BLUE);
       
    }

      
  
}
  






