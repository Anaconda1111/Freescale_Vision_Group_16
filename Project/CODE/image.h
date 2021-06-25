#ifndef __IMAGE_H
#define __IMAGE_H
#include "headfile.h"

//������ʵ�10�� 50�еĵ㣬mt9v03x_csi_image[10][50]�Ϳ�����
//ͼ��ԭ��Ϊ���Ͻ�
//[0,0]
extern uint8 otsu_image[MT9V03X_CSI_H][MT9V03X_CSI_W];

/*

//��������ͷ����
//#define MT9V03X_CSI_W               120            //ͼ����  ��Χ1-752      RT105X RT106X �ɼ�ʱ�п�ȱ���Ϊ4�ı���
//#define MT9V03X_CSI_H               80             //ͼ��߶�	��Χ1-480

*/


//��ͼ��ԭ��ת��Ϊ���½ǣ�����ɨ��
//image(h, w);  //h:��; w:��
//image(h,w)=0,Ϊ�׵㣻 image(h, w)=1,Ϊ�ڵ㣻
//image(10,20);���½�Ϊԭ�㣬��10�У�20��
//ͼ��ԭ��ת��
uint8 image(uint8 h, uint8 w); 

void set_pixel(uint8 h, uint8 w);

float mysqrt(float x);//

float my_abs_float(float x);

void array_init(uint8 * array, uint8 num, uint8 value);

void all_array_init(); 

//uint8 pixels_difference(uint8 x1,uint8 y1,uint8 x2,uint8 y2);  //���ز�ֵ

 //����ɨ��
void scan_line_base();  

//����G����Ŀ���G����ʼ�У��G�߽�����
void count_throw_line(uint8 startline, uint8 endline); 

//���ʼ���
float calculate_curvature(uint8 x1, uint8 y1, uint8 x2, uint8 y2, uint8 x3, uint8 y3); 

//���,����fitting_line[]
void regression(uint8 type, uint8 startline, uint8 endline); 

//Ѱ�ҹյ�
void find_inflection_point(uint8 type, uint8 startline, uint8 endline);


//����
void stayguy(uint8 type, uint8 * inflection1, uint8 * inflection2);

//���㳵�����������ƫ��
float calculate_differ(uint8 start_point, uint8 end_point);         

//�������ͻ��
uint8 width_wave(uint8 startline, uint8 endline);

//�жϱ߽��Ƿ�������
uint8 judge_continuty(uint8 type, uint8 start_point, uint8 end_point);

//Ѱ�Ҷϵ�
void find_break_point(uint8 type, uint8 startline, uint8 endline);   

//���㷽��
uint8 count_variance(uint8 type, uint8 start_point, uint8 end_point); 

//���½��������ֵ
void count_middle_line(uint8 startline, uint8 endline); 

//����ͷ����
void image_ctrl();     

 //����Ԫ���ж�
void judge_image();   

//��⻷����ڣ�type=1,�󻷵�; type=2,�һ���
uint8 check_island(uint8 type); 
uint8 check_island_status1();

uint8 check_forkroad();
uint8 check_t_road();


//ͼ����λ����ʾ
void image_send();




#endif
