#ifndef __IMAGE_H
#define __IMAGE_H
#include "headfile.h"




#define MidYunTai2SteerPWM 23000
#define MaxYunTai2SteerPWM 35750
#define MinYunTai2SteerPWM 11500

#define Trident_Left 2
#define Trident_Right 1
#define IsFruit  3
#define IsAnimal 4
#define InLeft  5
#define InRight 6
#define TAGNOTFOUND 7
#define Go 8
#define Return 9
#define Shoot 10




//������ʵ�10�� 50�еĵ㣬mt9v03x_csi_image[10][50]�Ϳ�����
//ͼ��ԭ��Ϊ���Ͻ�
//[0,0]
extern uint8 otsu_image[MT9V03X_CSI_H][MT9V03X_CSI_W];

/*

//��������ͷ����
//#define MT9V03X_CSI_W               188           //ͼ����  ��Χ1-752      RT105X RT106X �ɼ�ʱ�п�ȱ���Ϊ4�ı���
//#define MT9V03X_CSI_H               120             //ͼ��߶�	��Χ1-480

*/


//��ͼ��ԭ��ת��Ϊ���½ǣ�����ɨ��
//image(h, w);  //h:��; w:��
//image(h,w)=0,Ϊ�׵㣻 image(h, w)=1,Ϊ�ڵ㣻
//image(10,20);���½�Ϊԭ�㣬��10�У�20��
//ͼ��ԭ��ת��
uint8 image(uint8 h, uint8 w); 

void set_pixel(uint8 h, uint8 w);

float mysqrt(float x);//

int mymiddle(int x, int y);

float my_abs_float(float x);

uint8 my_abs_int(int x);

void array_init(uint8 * array, uint8 num, uint8 value);

void all_array_init(); 

//uint8 pixels_difference(uint8 x1,uint8 y1,uint8 x2,uint8 y2);  //���ز�ֵ

 //����ɨ��
void scan_line_base();  

//��ɨ��
void scan_line_simple();

//����G����Ŀ���G����ʼ�У��G�߽�����
void count_throw_line(uint8 startline, uint8 endline); 

//���ʼ���
float calculate_curvature(uint8 x1, uint8 y1, uint8 x2, uint8 y2, uint8 x3, uint8 y3); 



//���-normal,���� fitting_line[]
void regression_normal(int type, int startline, int endline);


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


uint8 check_forkroad();

uint8 check_zebra(uint8 startline, uint8 endline);


uint8 JugeCrossload(uint8 startline, uint8 endline);
uint8 JudgeZebra_L();
uint8 JudgeZebra_R();

void clear_image(uint8 h, uint8 w);


void cover_image(uint8 h, uint8 w);

void garagein_L();
void garagein_R();

uint8 check_block(uint8 startline, uint8 endline);

void garageout1();


uint8 count_lost_time(uint8 type, uint8 startline, uint8 endline);

void SobelTest();

uint8 check_block_sobel();



//ART
void ARTGetNumber();

uint8 ARTGetAprilTagNumber();

uint8 ARTGetImageMessage();

uint8 ARTGetFruitImageCenter();

uint8 GetCameraMessage();

void ART_SendByte(uint8 byte);

#endif
