#ifndef __IMAGE_H
#define __IMAGE_H
#include "headfile.h"

//例如访问第10行 50列的点，mt9v03x_csi_image[10][50]就可以了
//图像原点为左上角
//[0,0]
extern uint8 otsu_image[MT9V03X_CSI_H][MT9V03X_CSI_W];

/*

//配置摄像头参数
//#define MT9V03X_CSI_W               120            //图像宽度  范围1-752      RT105X RT106X 采集时列宽度必须为4的倍数
//#define MT9V03X_CSI_H               80             //图像高度	范围1-480

*/


//将图像原点转化为右下角，方便扫线
//image(h, w);  //h:行; w:列
//image(h,w)=0,为白点； image(h, w)=1,为黑点；
//image(10,20);右下角为原点，弟10行，20列
//图像原点转化
uint8 image(uint8 h, uint8 w); 

void set_pixel(uint8 h, uint8 w);

float mysqrt(float x);//

float my_abs_float(float x);

void array_init(uint8 * array, uint8 num, uint8 value);

void all_array_init(); 

//uint8 pixels_difference(uint8 x1,uint8 y1,uint8 x2,uint8 y2);  //像素差值

 //基本扫线
void scan_line_base();  

//计算丟线数目、丟线起始行，丟线结束行
void count_throw_line(uint8 startline, uint8 endline); 

//曲率计算
float calculate_curvature(uint8 x1, uint8 y1, uint8 x2, uint8 y2, uint8 x3, uint8 y3); 

//拟合,生成fitting_line[]
void regression(uint8 type, uint8 startline, uint8 endline); 

//寻找拐点
void find_inflection_point(uint8 type, uint8 startline, uint8 endline);


//拉线
void stayguy(uint8 type, uint8 * inflection1, uint8 * inflection2);

//计算车相对于赛道的偏差
float calculate_differ(uint8 start_point, uint8 end_point);         

//赛道宽度突变
uint8 width_wave(uint8 startline, uint8 endline);

//判断边界是否连续，
uint8 judge_continuty(uint8 type, uint8 start_point, uint8 end_point);

//寻找断点
void find_break_point(uint8 type, uint8 startline, uint8 endline);   

//计算方差
uint8 count_variance(uint8 type, uint8 start_point, uint8 end_point); 

//重新解算出中线值
void count_middle_line(uint8 startline, uint8 endline); 

//摄像头控制
void image_ctrl();     

 //赛道元素判断
void judge_image();   

//检测环岛入口，type=1,左环岛; type=2,右环岛
uint8 check_island(uint8 type); 
uint8 check_island_status1();

uint8 check_forkroad();
uint8 check_t_road();


//图像上位机显示
void image_send();




#endif
