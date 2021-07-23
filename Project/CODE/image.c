#include "image.h"
#include "headfile.h"
#include "Binarization.h"
#include "Motor.h"
#include "port.h"
#include "Steer.h"

#define Middle_point_w (MT9V03X_CSI_W/2 - 1)
#define Middle_point_h (MT9V03X_CSI_H/2 - 1)


/*

//配置摄像头参数
//#define MT9V03X_CSI_W               120             //图像宽度  范围1-752      RT105X RT106X 采集时列宽度必须为4的倍数
//#define MT9V03X_CSI_H               80             //图像高度	范围1-480

*/

extern uint8 flag_garagein_L;
extern uint8 flag_garagein_R;
extern uint8 garageout_dir;
extern uint8 garageout;
extern int time;
extern uint16 Motor_GO_L_PWM;
extern uint16 Motor_GO_R_PWM;
extern uint16 encoder_value_L;
extern uint16 encoder_value_R;

//状态机写法
uint8 Flag_island_L=0;
uint8 Flag_island_R=0;
uint8 Flag_crossroad=0;
uint8 Flag_straightway=0;
uint8 Flag_curve=0;
uint8 Flag_zebra_L=0;
uint8 Flag_zebra_R=0;
uint8 Flag_rampway=0;
uint8 Flag_forkroad=0;
uint8 Flag_t_road=0;
uint8 Flag_depend_R=0;
uint8 Flag_depend_L=0;
uint8 Flag_garageout_L=0;
uint8 Flag_garageout_R=0;
uint8 Flag_block =0;

uint8 Flag_go =0;


uint8 left_border[MT9V03X_CSI_H]={0};  //赛道左边界
uint8 right_border[MT9V03X_CSI_H]={0};  //赛道右边界
uint8 middle_line[MT9V03X_CSI_H]={0};  //赛道中点坐标
uint8 track_width[MT9V03X_CSI_H]={0};   //赛道宽度

//正常赛道宽度
uint8 track_width_normal[80]={75,74,74,73,73,72,72,71,71,70,70,69,69,68,68,67,67,65,65,64,64,63,63,62,62,61,61,60,60,59,59,58,58,57,57,56,56,55,55,54,54,53,53,52,52,51,51,50,50,49,49,48,48,47,47,46,46,45,45,44,44,43,43,42,42,41,41,40,40,39,39,38,38,37,37,36,36,35,35,34}; 

uint8 flag_left_border[MT9V03X_CSI_H]={0};  //1:边界存在；0：边界未扫到
uint8 flag_right_border[MT9V03X_CSI_H]={0};

uint8 middle_line_peak=MT9V03X_CSI_H-1;              //中线顶点
uint8 middle_line_high=MT9V03X_CSI_H-1;         //图像中线高度


uint8 throw_line_num_L=0;                //左丟线数目
uint8 throw_line_num_R=0;              //右丟线数目
uint8 throw_line_start_L=0;             //丟线起始行
uint8 throw_line_end_L=0;              //丟线结束行
uint8 throw_line_start_R=0;
uint8 throw_line_end_R=0;

uint8 fitting_border_L[MT9V03X_CSI_H]={0};  //拟合的边界，用于后面计算边界的方差
uint8 fitting_border_R[MT9V03X_CSI_H]={0};
uint8 fitting_middleline[MT9V03X_CSI_H]={0};

uint8 inflection_upL[3]={0};      //左上拐点，第一个元素为拐点标志位(1:存在；0：不存在)，二三个元素为拐点的坐标(x,y)
uint8 inflection_lowerL[3]={0};   //
uint8 inflection_upR[3]={0};      //右上拐点，第一个元素为拐点标志位(1:存在；0：不存在)，二三个元素为拐点的坐标(x,y)
uint8 inflection_lowerR[3]={0};

uint8 break_point_L1[3]={0};   //边界突然性断裂的目标点
uint8 break_point_L2[3]={0};
uint8 break_point_R1[3]={0};
uint8 break_point_R2[3]={0};
uint8 break_point_M1[3]={0};
uint8 break_point_M2[3]={0};


float intercept=0; //截距
float slope=0; //斜率
float current_value=0;//车相对于赛道的偏差
float current_value_last=0;//上一次误差

//元素状态标志位
uint8 flag_element_status=0;

//赛道边界连续性
uint8 continuity_L=0;
uint8 continuity_R=0;


uint8 island_time =0;

uint8 forkroad_time=0;

uint8 forkroad_direction =0;

int64 Sobel=0;

uint8 camera=0;
uint8 block_direction=0;
uint32 theta =0;
uint8 Camera=0;

//将图像原点转化为右下角，方便扫线
//image(h, w);  //h:行; w:列
//image(0,0)=0,为白点； image(h, w)=1,为黑点；
uint8 image(uint8 h, uint8 w)
{
   return otsu_image[MT9V03X_CSI_H-h-1][MT9V03X_CSI_W-w-1];  
}

//修改二值化图，设置黑像素点
void set_pixel(uint8 h, uint8 w)
{
    if(h > MT9V03X_CSI_H-1)
    {
        h = MT9V03X_CSI_H-1;
    }

    if(h <= 0)
    {
        h=0;
    }
    
    if(w > MT9V03X_CSI_W-1)
    {
        w = MT9V03X_CSI_W-1;
    }
    
    if(w <= 0)
    {
        w=0;
    }

    otsu_image[MT9V03X_CSI_H-h-1][MT9V03X_CSI_W-w-1]=1;

}



float mysqrt(float x)
{
	float a = x;
	unsigned int i = *(unsigned int *)&x;
	i = (i + 0x3f76cf62) >> 1;
	x = *(float *)&i;
	x = (x + a / x) * 0.5;
	return x;
}

int mymax(int x, int y)
{
  return (x>=y?x:y);
}

int mymin(int x, int y)
{
  return (x>=y?y:x);
}

int mymiddle(int x, int y)
{
    int middle;
    middle =(int)((x+y)/2);
    return middle;
}


float my_abs_float(float x)
{
  x = x >= 0 ? x : x*(-1);
  return x;
}

uint8 my_abs_int(int x)
{
  x= x>= 0 ? x : x*(-1);
  return x;
}


//数组初始化,array: 数组名；num:数组大小；value:初始化成的值
void array_init(uint8 * array, uint8 num, uint8 value)
{
  uint8 i;
  for(i=0;i<num;i++)
  {
    array[i]=value;
    
  }
}

void all_array_init()
{
  array_init(left_border, MT9V03X_CSI_H, MT9V03X_CSI_W-1);
  array_init(right_border, MT9V03X_CSI_H, 0);
  array_init(middle_line, MT9V03X_CSI_H, Middle_point_w);
  array_init(track_width, MT9V03X_CSI_H, 0);
  array_init(flag_left_border, MT9V03X_CSI_H, 0);
  array_init(flag_right_border, MT9V03X_CSI_H, 0);
  array_init(fitting_border_L, MT9V03X_CSI_H, MT9V03X_CSI_W-1);
  array_init(fitting_border_R, MT9V03X_CSI_H, 0);
  array_init(fitting_middleline, MT9V03X_CSI_H,Middle_point_w);
  
  array_init(break_point_L1, 3, 0);
  array_init(break_point_L2, 3, 0);
  array_init(break_point_R1, 3, 0);
  array_init(break_point_R2, 3, 0);
  
  array_init(inflection_upL, 3, 0);
  array_init(inflection_lowerL, 3, 0);
  array_init(inflection_upR, 3, 0);
  array_init(inflection_lowerR, 3, 0);
  
}

/*
//像素点差比和计算,返回两个像素点之间的差值
uint8 pixels_difference(uint8 x1, uint8 y1, uint8 x2, uint8 y2)
{
  uint8 difference_value;
  uint8 differenceV1=0,differenceV2=0;
  
  differenceV1 = abs(image(x1,y1) - image(x2,y2));
  differenceV2 = image(x1,y1) + image(x2,y2);
  difference_value = differenceV1 / differenceV2 * 100;
  
  return difference_value;
}
*/


//基本扫线
//得出赛道边界，中线，顶点，赛道宽度，边界标志位
void scan_line_base()
{
  //i:行；j:列
  uint8 i,j;
  //
  uint8 old  = Middle_point_w;
 
  //特征量初始化
  throw_line_start_L =0;
  throw_line_end_L =MT9V03X_CSI_H-1;
  throw_line_start_R =0;
  throw_line_end_R =MT9V03X_CSI_H-1;
  throw_line_num_L =0;
  throw_line_num_R =0;
  middle_line_peak =MT9V03X_CSI_H-1;  

  //数组初始化
  all_array_init();
  
  
  //寻找第一行中点
  if(image(0,Middle_point_w)==0)
  {
      old = Middle_point_w;
  }
  else
  {
    int array[MT9V03X_CSI_W]={0};
    int num=0;
    int sum=0;
      for(j=0;j< MT9V03X_CSI_W-1;j++)
      {
          if(image(0,j)==0)
          {
              array[num]=j;
              num++;
          }
          
      }
      
      for(j=0;j<num;j++)
      {
        sum+=array[j];
      }
        
      old = (uint8)(sum/num);
  }
    
    
 //扫线
  
  for(i=0;(i+2)<MT9V03X_CSI_H-1;i++)
  {

        
      for(j=old; (j+1)<MT9V03X_CSI_W-1; j+=2)   //寻找左边界
      {
        if(image(i,j)==1 && image(i,j+1)==1)
        {
          left_border[i]=j;
          flag_left_border[i]=1; 
          break;
        }
       
      }
      
      
      for(j=old; (j-1)>0; j-=2)               //寻找右边界
      {
        if(image(i,j)==1 && image(i,j-1)==1)
        {
          right_border[i]=j;
          flag_right_border[i]=1; 
          break;
        }
       
      }
    
      //赛道宽度 中线
      if(flag_left_border[i]==1 && flag_right_border[i]==1)
      {
          middle_line[i]=(left_border[i] + right_border[i])/2;
          track_width[i]=left_border[i] - right_border[i];
      }
      else if(flag_left_border[i]==0 && flag_right_border[i]==1)
      {
          middle_line[i]=(MT9V03X_CSI_W-1 + right_border[i])/2;
          track_width[i]=MT9V03X_CSI_W-1 - right_border[i];
      }
      else if(flag_left_border[i]==1 && flag_right_border[i]==0)
      {
         middle_line[i]=(left_border[i])/2;
         track_width[i]=left_border[i];
      }
      else
      {
          middle_line[i]=Middle_point_w;
          track_width[i]=MT9V03X_CSI_W-1;
      }
   
    //上顶点
    if(i> 10)
    {
       if(i==MT9V03X_CSI_H-1)
       {
          middle_line_peak = MT9V03X_CSI_H-1;
       }
       
      if(image(i,middle_line[i])==1 && image(i+1,middle_line[i])==1)  //寻找上顶点
      {
          middle_line_peak=i;
          if(i> 20)
          {
            break;
          }
          
      }
     }
    
    old = middle_line[i];
    
  }
  
  //图像中点高度
  for(i=15;i+2<MT9V03X_CSI_H-1;i++)
  {
      if(image(i, Middle_point_w)==1 && image(i+1, Middle_point_w)==1)
      {
          middle_line_high = i;
          break;
      }
  }
  
  

}


//简单扫线
void scan_line_simple()
{
  //i:行；j:列
  uint8 i,j;
  //
  uint8 old  = Middle_point_w;
 

  
  //寻找第一行中点
  if(image(0,Middle_point_w-1)==0)
  {
      old = Middle_point_w;
  }
  else
  {
    int array[MT9V03X_CSI_W]={0};
    int num=0;
    int sum=0;
      for(j=0;j< MT9V03X_CSI_W-1;j++)
      {
          if(image(0,j)==0)
          {
              array[num]=j;
              num++;
          }
          
      }
      
      for(j=0;j<num;j++)
      {
        sum+=array[j];
      }
        
      old = (uint8)(sum/num);
  }
    
    
 //扫线
  
  for(i=0;(i+2)<MT9V03X_CSI_H-1;i++)
  {

        
      for(j=old; (j+1)<MT9V03X_CSI_W-1; j+=2)   //寻找左边界
      {
        if(image(i,j)==1 && image(i,j+1)==1)
        {
          left_border[i]=j;
          flag_left_border[i]=1; 
          break;
        }
       
      }
      
      
      for(j=old; (j-1)>0; j-=2)               //寻找右边界
      {
        if(image(i,j)==1 && image(i,j-1)==1)
        {
          right_border[i]=j;
          flag_right_border[i]=1; 
          break;
        }
       
      }
        //赛道宽度 中线
      if(flag_left_border[i]==1 && flag_right_border[i]==1)
      {
          middle_line[i]=(left_border[i] + right_border[i])/2;
          track_width[i]=left_border[i] - right_border[i];
      }
      else if(flag_left_border[i]==0 && flag_right_border[i]==1)
      {
          middle_line[i]=(MT9V03X_CSI_W-1 + right_border[i])/2;
          track_width[i]=MT9V03X_CSI_W-1 - right_border[i];
      }
      else if(flag_left_border[i]==1 && flag_right_border[i]==0)
      {
         middle_line[i]=(left_border[i])/2;
         track_width[i]=left_border[i];
      }
      else
      {
          middle_line[i]=Middle_point_w;
          track_width[i]=MT9V03X_CSI_W-1;
      }
   
    //上顶点
      
     if(i==MT9V03X_CSI_H-1)
     {
        middle_line_peak = MT9V03X_CSI_H-1;
     }
     
    if(image(i,middle_line[i])==1 && image(i+1,middle_line[i])==1)  //寻找上顶点
    {
        middle_line_peak=i;
        break;
    }
    
    old = middle_line[i];
    
  }
  
  
  
  
    
}


  


//计算丟线数目、丟线起始行，丟线结束行
void count_throw_line(uint8 startline, uint8 endline)
{
  
  if(endline > middle_line_peak)
  {
    endline = middle_line_peak;
  }
  
  uint8 time_L=0;
  uint8 time_R=0;
  throw_line_num_L=0;
  throw_line_num_R=0;
  
  uint8 i=0;

 //记录丟线数据
  for(i=startline;i<endline;i++)
  {
    if(!flag_left_border[i])
    {
      throw_line_num_L++;
      
    }
   
    
    if(!flag_right_border[i])
    {
        throw_line_num_R++;
    }
    
  }
  
  for(i=startline; i<endline-2; i++)
  {
      if(time_L==0 && flag_left_border[i]==0 && flag_left_border[i+1]==0)
      {
          throw_line_start_L=i;
          time_L=1;
      }
      
      if(time_L==1 && flag_left_border[i]==1 && flag_left_border[i+1]==1)
      {
          throw_line_end_L=i;
          time_L=2;
        
      }
      
      
      if(time_R==0 && flag_right_border[i]==0 && flag_right_border[i+1]==0)
      {
          throw_line_start_R=i;
          time_R=1;
      }
      
      if(time_R==1 && flag_right_border[i]==1 && flag_right_border[i+1]==1)
      {
          throw_line_end_R=i;
          time_R=2;
        
      }
     
        
        
      
      
  }
  
  if(time_L ==1)
  {
   
    int i=0;
    for(i=throw_line_start_L+15;i+1<middle_line_peak;i++)
    {
        if(image(i,MT9V03X_CSI_W-1)==1 && image(i+1,MT9V03X_CSI_W-1)==1
           && image(i,MT9V03X_CSI_W-2)==1 && image(i+1,MT9V03X_CSI_W-2)==1)
        {
            throw_line_end_L = i+1;
            time_L =2;
        }
    }
   
    if(time_L ==1)
    {
        throw_line_end_L = middle_line_peak;
     }
      
  }
  
  if(time_R ==1)
  {
    
     int i=0;
    for(i=throw_line_start_R+15;i+1<middle_line_peak;i++)
    {
        if(image(i,0)==1 && image(i+1,0)==1
           && image(i,1)==1 && image(i+1,1)==1)
        {
            throw_line_end_R = i+1;
            time_R =2;
        }
    }
    
    if(time_R ==1)
    {
        throw_line_end_R = middle_line_peak;
     }

  }
  
}
  

//三点计算曲率，圆形半径越大，曲率越小，弯曲程度就越小，也就越近似于一条直线
float calculate_curvature(uint8 x1, uint8 y1, uint8 x2, uint8 y2, uint8 x3, uint8 y3)
{
  float a,b,c,R;
  a=mysqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
  b=mysqrt((x1-x3)*(x1-x3)+(y1-y3)*(y1-y3));
  c=mysqrt((x2-x3)*(x2-x3)+(y2-y3)*(y2-y3));
  
  if(a*b*c == 0)
  {
    R=0;
  }
  else
  {
  
    R=a*b*c/mysqrt((a+b+c)*(a+b-c)*(a-b+c)*(b+c-a));
  }
  
  return (1/R);
  
  
  
}



void regression_normal(int type, int startline, int endline)
{
    int i = 0;
    int sumlines = endline - startline;
    int sumX = 0;
    int sumY = 0;
    float averageX = 0;
    float averageY = 0;
    float sumUp = 0;
    float sumDown = 0;
    
    float parameterA =0;//截距
    float parameterB =0;//斜率
    float value=0;
    
    if (type == 0)//拟合中线
    {
        for (i = startline; i < endline; i++)
        {
            sumX += i;
            sumY += middle_line[i];
        }
        if (sumlines == 0) sumlines = 1;
        averageX = sumX / sumlines;     //x的平均值
        averageY = sumY / sumlines;     //y的平均值
        for (i = startline; i < endline; i++)
        {
            
            sumUp += (middle_line[i] - averageY) * (i - averageX);
            sumDown += (i - averageX) * (i - averageX);
        }
        if (sumDown == 0)
        {
          parameterB = 0;
        }
        else
        {
          parameterB = sumUp / sumDown;
        }
        
        parameterA = averageY - parameterB * averageX;
        
         for(i=0; i<MT9V03X_CSI_H-1; i++)
        {
            value = parameterB * i + parameterA;  
            
            if(value<0)
            {
                value  =0;
            }
            else if(value>MT9V03X_CSI_W-1)
            {
              value = MT9V03X_CSI_W-1;
            }
          
            fitting_middleline[i]=(uint8)(value);
        }

    }  
    else if (type == 1)//拟合左线
    {
        for (i = startline; i < endline; i++)
        {
            sumX += i;
            sumY += left_border[i];
        }
        if (sumlines == 0) sumlines = 1;
        averageX = sumX / sumlines;     //x的平均值
        averageY = sumY / sumlines;     //y的平均值
        for (i = startline; i < endline; i++)
        {
            
            sumUp += (left_border[i] - averageY) * (i - averageX);
            sumDown += (i - averageX) * (i - averageX);
        }
        if (sumDown == 0)
        {
          parameterB = 0;
        }
        else
        {
          parameterB = sumUp / sumDown;
        }
        
        parameterA = averageY - parameterB * averageX;
        
         for(i=0; i<MT9V03X_CSI_H-1; i++)
        {
            value = parameterB * i + parameterA;  
            
            if(value<0)
            {
                value  =0;
            }
            else if(value>MT9V03X_CSI_W-1)
            {
              value = MT9V03X_CSI_W-1;
            }
          
            fitting_border_L[i]=(uint8)(value);
        }

    }
    else if (type == 2)//拟合右线
    {
        for (i = startline; i < endline; i++)
        {
            sumX += i;
            sumY += right_border[i];
        }
        if (sumlines == 0) sumlines = 1;
        averageX = sumX / sumlines;     //x的平均值
        averageY = sumY / sumlines;     //y的平均值
        for (i = startline; i < endline; i++)
        {
            sumUp += (right_border[i] - averageY) * (i - averageX);
            sumDown += (i - averageX) * (i - averageX);
        }
        if (sumDown == 0)
        {
          parameterB = 0;
        }
        else 
        {
          parameterB = sumUp / sumDown;
        }
        parameterA = averageY - parameterB * averageX;
        
        for(i=0; i<MT9V03X_CSI_H-1; i++)
        {
            value = parameterB * i + parameterA;
            
            if(value<0)
            {
                value = 0;
            }
            else if(value>MT9V03X_CSI_W-1)
            {
              value = MT9V03X_CSI_W-1;
            }

            fitting_border_R[i]=(uint8)(value);
        }


    }
}


//计算方差
uint8 count_variance(uint8 type, uint8 start_point, uint8 end_point)
{
        if(end_point >= MT9V03X_CSI_H-1) 
        {
          end_point = MT9V03X_CSI_H-1;
        }
        
 	uint8 variance=0;  //方差
	uint8 i;
	int sum=0;
	
        
        if(type == 1)  //计算左线方差
        {
            for(i=start_point; i<end_point; i++)
            {
                    sum+=my_abs_int(left_border[i]-fitting_border_L[i]);
                    
            }
            if(end_point == start_point)
            {
              variance = 0;
            }
            else
            {
              variance=(uint8)(sum/(end_point - start_point));
            }        
        }
        else if(type ==2)  //计算右线方差
        {
            for(i=start_point; i<end_point; i++)
            {
                    sum+=my_abs_int(right_border[i]-fitting_border_R[i]);
                    
            }
            if(end_point == start_point)
            {
              variance = 0;
            }
            else
            {
              variance=(uint8)(sum/(end_point - start_point));
            }        
        }
  
	return variance;
}



//对两个点进行拉线，得到边界数组,修改二值化图，后面要重新扫线
void stayguy(uint8 type, uint8 * inflection1, uint8 * inflection2)
{
  int slope=0;
  int intercept=0;
  int value =0;
  
  if(type == 0) //中线
  {
      slope = (inflection2[2] - inflection1[2])/(inflection2[1] - inflection1[1]);
      intercept = (inflection1[2] + inflection2[2])/2 - slope*(inflection1[1]+inflection2[1])/2;
      
      int i=0;
      for(i=inflection1[1];i<inflection2[1];i++)
      {
        value = slope * i +intercept;
        
        if(value < 0)
        {
          value =0;
        }
        else if(value > MT9V03X_CSI_W-1)
        {
          value = MT9V03X_CSI_W-1;
        }
        
        middle_line[i]=(uint8)(value);
 
      
      }
  }
  else if(type == 1) //对左边界经行拉线
  {
      slope = (inflection2[2] - inflection1[2])/(inflection2[1] - inflection1[1]);
      intercept = (inflection1[2] + inflection2[2])/2 - slope*(inflection1[1]+inflection2[1])/2;
      
      int i=0;
      for(i=inflection1[1];i<inflection2[1];i++)
      {
        value = slope * i +intercept;
        
        if(value <0)
        {
          value =0;
        }
        else if(value > MT9V03X_CSI_W-1)
        {
          value = MT9V03X_CSI_W-1;
        }
        
        left_border[i]=(uint8)(value);
        set_pixel(i, left_border[i]);
        set_pixel(i, left_border[i]+1);
        set_pixel(i, left_border[i]+2);
        
      
      }
  }
  else if(type == 2)//对右边界经行拉线
  {
      slope = (inflection2[2] - inflection1[2])/(inflection2[1] - inflection1[1]);
      intercept = (inflection1[2] + inflection2[2])/2 - slope*(inflection1[1]+inflection2[1])/2;
      
      int i=0;
      for(i=inflection1[1];i<inflection2[1];i++)
      {
        value = slope * i +intercept;
        
        if(value <0)
        {
          value =0;
        }
        else if(value > MT9V03X_CSI_W-1)
        {
          value = MT9V03X_CSI_W-1;
        }
        
        
        right_border[i]=(uint8)(value);
        set_pixel(i, right_border[i]);
        set_pixel(i, right_border[i]-1);
        set_pixel(i, right_border[i]-2);
       
      }
   
  }
  
}


//判断左右边界线时候具有连续性
uint8 judge_continuty(uint8 type, uint8 start_point, uint8 end_point)
{
  
  if(end_point > middle_line_peak)
  {
    end_point = middle_line_peak;
  }
  
  uint8 i;
  uint8 error=0;
  uint8 break_point_threshold=10;
  
 
  if(type == 1)  //判断左线的连续性
  {
      for(i=start_point; i+1<end_point; i++)
      {
            error=abs(left_border[i+1]-left_border[i]);
            
            if(error > break_point_threshold)//判断为断点
            {
                return 0;  
              
            }
             
      }

  }
  else if(type == 2)  //判断右线的连续性
  {
      for(i=start_point; i+1<end_point; i++)
      {
        
            error=abs(right_border[i+1]-right_border[i]);
            
            if(error > break_point_threshold)//判断为断点
            {
                
                return 0;  
              
            }
       
        
      }     
  }
  
  return 1;
  
}

//寻找断点,
void find_break_point(uint8 type, uint8 startline, uint8 endline)
{
  if(endline > middle_line_peak)
  {
     endline = middle_line_peak;
  }
  
  uint8 i;
  int8 error=0;
  uint8 break_point_threshold=20;
  uint8 time=1;
 
  if(type == 1)  //判断左线
  {
      for(i=startline; i+1<endline; i++)
      {
        
         error=(left_border[i+1]-left_border[i]);
       
        
        
        if(error > break_point_threshold && time==1)//判断为断点
        {
           break_point_L1[0]=1;
           break_point_L1[1]=i;       
           break_point_L1[2]=left_border[i];
           time=2;
          
        }
        else if(error < -break_point_threshold && time==1)
        {
           break_point_L1[0]=1;
           break_point_L1[1]=i+1;       
           break_point_L1[2]=left_border[i+1];
           time=2;
             
            
        }
          
          
        
        if(error > break_point_threshold && time==2)
        {
           break_point_L2[0]=1;
           break_point_L2[1]=i;
           break_point_L2[2]=left_border[i];
           break;
          
        }
        else if(error < -break_point_threshold && time==2)
        {
          
           break_point_L2[0]=1;
           break_point_L2[1]=i+1;
           break_point_L2[2]=left_border[i+1];
           break;
          
          
        }
          
      }      
      
  }
  else if(type == 2)  //判断右线
  {
      for(i=startline; i+1<endline; i++)
      {
        error=(right_border[i+1]-right_border[i]);
        
        
        if(error > break_point_threshold && time==1)//判断为断点
        {
           break_point_R1[0]=1;
           break_point_R1[1]=i;
           break_point_R1[2]=right_border[i];
           time=2;
          
        }
        else if(error < -break_point_threshold && time==1)
        {
           break_point_R1[0]=1;
           break_point_R1[1]=i+1;
           break_point_R1[2]=right_border[i+1];
           time=2;
             
          
        }
          
        
        if(error > break_point_threshold && time==2)
        {
           break_point_R2[0]=1;
           break_point_R2[1]=i;
           break_point_R2[2]=right_border[i];
           break;
          
        }
        else if(error < -break_point_threshold && time==2)
        {
           break_point_R2[0]=1;
           break_point_R2[1]=i+1;
           break_point_R2[2]=right_border[i+1];
           break;
            
        }
         
      }      
  }
    
}


//寻找拐点
void find_inflection_point(uint8 type, uint8 startline, uint8 endline)
{
    if(endline > MT9V03X_CSI_H-1)
    {
        endline = MT9V03X_CSI_H-1;
    }
    
    //array_init(inflection_lowerL, 3, 0);
    //array_init(inflection_lowerR, 3, 0);
   
    int error1 = 0;
    int error2 = 0;
    uint8 i;
    
    if(type == 1)
    {
            for(i=startline;i+10<endline;i++)
            {
            
                error1 = left_border[i+5]-left_border[i];
                error2 = left_border[i+10]-left_border[i+5];
                
                
                
                
                if(error1 * error2 < 0)       //判断为拐点
                {
                    
                        inflection_lowerL[0]=1;
                        inflection_lowerL[1]=i+5;
                        inflection_lowerL[2]=left_border[i+5];
                        return ;
                  
                }
              
            }
    }
    else if(type == 2)
    {
        
            for(i=startline;i+10<endline;i++)
            {
            
                error1 = right_border[i+5]-right_border[i];
                error2 = right_border[i+10]-right_border[i+5];
                
                
                
                if(error1 * error2 < 0)       //判断为拐点
                {
                    
                        inflection_lowerR[0]=1;
                        inflection_lowerR[1]=i+5;
                        inflection_lowerR[2]=right_border[i+5];
                    
                        return ;
                }
              
            }

    }
    
}



//赛道宽度突变
uint8 width_wave(uint8 startline, uint8 endline)
{
    if(startline <= 0)
    {
        startline = 0;
    }
    if(endline > MT9V03X_CSI_H-1)
    {
        endline = MT9V03X_CSI_H-1;
    }
  
    
    uint8 wave_threshold = 10;
    uint8 i;
    uint32 error;
    uint8 num1;
    uint8 num2;
    
    num1 = endline - startline;
    num2 = mymiddle(endline, startline);
    
    if(num1==0)
    {
      return 0;
      
    }
    
    for(i=startline;i<endline;i++)
    {  
        error = track_width[i];
    }
    
    
    error = (uint8)error/(num1);
    
    if(error > 2*track_width_normal[num2]+wave_threshold)
    {
      return 1;
    }
      
    
    return 0;
    
    
}


////计算车相对于赛道的偏差
float calculate_differ(uint8 start_point, uint8 end_point)
{
  if(end_point > MT9V03X_CSI_H-1)
  {
      end_point = MT9V03X_CSI_H-1;
  }
  
  if(end_point > 0)
  {
       
      uint8 i;
      
      float sum=0;
      
      for(i=start_point;i<end_point;i++)
      {
        sum+=(float)(Middle_point_w - middle_line[i]);
      }
      
      
      current_value=(float)(sum/(end_point - start_point));
      
 
  }
  else
  {
      current_value = current_value_last;
  }
  
  current_value_last = current_value;
  
  
  return current_value;
}


//计算出边界中线
void count_middle_line(uint8 startline, uint8 endline)
{
    uint8 i=0;
    if(endline >= middle_line_peak)
    {
        endline = middle_line_peak;
    }
    
    for(i=startline;i<endline;i++)
    {
      
        middle_line[i]=(uint8)((left_border[i]+right_border[i])/2);
        
    }
}


/*
//赛道元素
//状态机写法
uint8 Flag_island_L=0;    环岛
uint8 Flag_island_R=0;  
uint8 Flag_crossroad=0;   十字
uint8 Flag_straightway=0; 直线
uint8 Flag_curve=0;       弯道
uint8 Flag_zebra=0;       斑马线
uint8 Flag_rampway=0;     坡道
uint8 Flag_forkroad=0;    三岔路口
uint8 Flag_t_road=0;      出环岛的T形路口
uint8 Flag_depend_L=0;    靠左走
uint8 Flag_depend_R=0;    靠右走
*/

void image_ctrl()
{
  
  
  //采集赛道元素 赛道边界，中线，顶点，赛道宽度，边界标志位
    scan_line_base(); 
    count_throw_line(0,60);//60
    
    continuity_L=judge_continuty(1, 0, 60);
    continuity_R=judge_continuty(2, 0, 60);
 
    find_break_point(1, 0, 60);
    find_break_point(2, 0, 60);
    
   
 
    if(Flag_island_L ==0 && Flag_island_R ==0)
    {
 
      judge_image();
      
    }
    
    
    

  if(Flag_crossroad)
  {     
    
    //入十字前，拉线
    if(flag_right_border[20]==1 && flag_right_border[21]==1 && flag_left_border[20]==1 && flag_left_border[21]==1)
    {
      if(break_point_L1[0]==1 && break_point_L2[0]==1)
      {
           stayguy(1, break_point_L1, break_point_L2);
      }
     
      
      if(break_point_R1[0]==1 && break_point_R2[0]==1)
      {
          stayguy(2, break_point_R1, break_point_R2);
      }
      
      scan_line_base();  
    }  
  
    //入十字后拟合
    if(break_point_L2[0]==1  && break_point_R2[0]==1  && flag_right_border[20]==0 && flag_right_border[21]==0 
       && flag_left_border[20]==0 && flag_left_border[21]==0)
    {
         regression_normal(1, break_point_L1[1]+5, break_point_L1[1]+25);
         regression_normal(2, break_point_R1[1]+5, break_point_R1[1]+25); 
         
         uint8 i;
         
         for(i=0;i<80;i++)
         {
           left_border[i]=fitting_border_L[i];
           right_border[i]=fitting_border_R[i];
           
           
         }
           
         count_middle_line(0,80); 
    }
  
    
      
  }
 
  
 
 
 
  

  if(Flag_island_L)
  {
    
     
      if(flag_element_status ==0)
      {
        Flag_depend_R=1;
  
      }
      
      if(flag_element_status ==0 && throw_line_num_L > 40)
      {
          flag_element_status =1;
        
      }
      
      if(flag_element_status ==1 && throw_line_num_L < 10)
      {
          flag_element_status =2;
      }
        
      
      //入口拉线
      if(flag_element_status==2 && throw_line_num_L > 20)
      {
          Flag_depend_R =0;
         
          flag_element_status =3;
         
      }
      
      if(flag_element_status ==3 &&  continuity_R ==0 && throw_line_num_R > 20)
      {
       // Flag_depend_L =1;
        flag_element_status = 4;
      }
     
      //取消拉线
      if(flag_element_status ==4 && continuity_R ==1)
      {
         flag_element_status =5;
      }

      //出环岛拉线
      if(flag_element_status ==5 && middle_line_high < 80 && throw_line_num_L > 30 
       && throw_line_num_R > 10)
      {
       // Flag_depend_L =1;
        flag_element_status =6;
      
        
      }
        
      if(flag_element_status ==6 && middle_line_peak > 90 && middle_line_high > 50
         && throw_line_num_R < 3)
      {
        Flag_depend_L=0;
        Flag_depend_R =1;
        flag_element_status =7;
        
      }
      
      if(flag_element_status ==7 && throw_line_num_L < 10 && throw_line_num_R < 15)
      {
        Flag_island_L=0;
        Flag_depend_R=0;
        flag_element_status=0;
      }
        
   
    

      
  }
  
  //右环岛
  if(Flag_island_R)
  {
    
     
      if(flag_element_status ==0)
      {
        Flag_depend_L=1;
  
      }
      
      if(flag_element_status ==0 && throw_line_num_R > 40)
      {
          flag_element_status =1;
        
      }
      
      if(flag_element_status ==1 && throw_line_num_R < 10)
      {
          flag_element_status =2;
      }
        
      
      //入口拉线
      if(flag_element_status==2 && throw_line_num_R > 20)
      {
          Flag_depend_L =0;
         
          flag_element_status =3;
         
      }
      
      if(flag_element_status ==3 &&  continuity_L ==0 && throw_line_num_L > 20)
      {
      
        flag_element_status = 4;
      }
     
      //取消拉线
      if(flag_element_status ==4 && continuity_L ==1)
      {
         flag_element_status =5;
      }

      //出环岛拉线
      if(flag_element_status ==5 && middle_line_high < 80 && throw_line_num_R > 30 
       && throw_line_num_L > 15)
      {
       
        flag_element_status =6;
      
        
      }
        
      if(flag_element_status ==6 && middle_line_peak > 90 && middle_line_high > 50
         && throw_line_num_L < 3)
      {
        
        Flag_depend_L =1;
        flag_element_status =7;
        
      }
      
      if(flag_element_status ==7 && throw_line_num_R < 10 && throw_line_num_L < 15)
      {
        Flag_island_R=0;
        Flag_depend_L =0;
        flag_element_status=0;
      }
        
   
    

      
  }
  
 
  if(Flag_forkroad)
  {
    //入三岔
    if(forkroad_time ==0 && flag_element_status ==1)
    {
      /*
      stop_car(200);
      systick_delay_ms(1000); 
      
      time=0;
      while(time <=3 && forkroad_direction == 0)
      {
           ARTGetNumber();
          
          if(time==3 && forkroad_direction ==0)
          {
              forkroad_direction =1;//强制置1
          }
      }
    */ 
      
      
      forkroad_direction =1;
      
      
      //flag_element_status=1;
     
      //pwm_duty(MotorPWM_Go_L_CH, 12000);
     // pwm_duty(MotorPWM_Go_R_CH, 12000);  
     // systick_delay_ms(500);         
    //  pwm_duty(SteerPWM_CH, SteerMIN);
   //   systick_delay_ms(300); 
       
  
   /*   
      Motor_GO_L_PWM=8000;
      Motor_GO_R_PWM=8000;
      Flag_go=1;
    */  
      if(forkroad_direction ==2)
      {
          inflection_lowerL[0]=1;
          inflection_lowerL[1]=MT9V03X_CSI_H-1;
          inflection_lowerL[2]=MT9V03X_CSI_W-1;
          
          inflection_lowerR[0]=1;
          inflection_lowerR[1]=0;
          inflection_lowerR[2]=0;
          
          stayguy(2,inflection_lowerR, inflection_lowerL);
          
          scan_line_simple();            
       
      }   
      
      if(forkroad_direction ==1)
      {
      
          inflection_lowerL[0]=1;
          inflection_lowerL[1]=0;
          inflection_lowerL[2]=MT9V03X_CSI_W-1;
          
          inflection_lowerR[0]=1;
          inflection_lowerR[1]=MT9V03X_CSI_H-1;
          inflection_lowerR[2]=0;
          
          stayguy(1,inflection_lowerL, inflection_lowerR);
          
          scan_line_simple();          
      }
      
    }
    
    
     
    if(forkroad_time ==1 && flag_element_status ==1)
     {
       if(forkroad_direction ==2)
       {
          inflection_lowerL[0]=1;
          inflection_lowerL[1]=MT9V03X_CSI_H-1;
          inflection_lowerL[2]=MT9V03X_CSI_W-1;
          
          inflection_lowerR[0]=1;
          inflection_lowerR[1]=0;
          inflection_lowerR[2]=0;
          
          stayguy(2,inflection_lowerR, inflection_lowerL);
          
          scan_line_simple();            
           
       }
      
        if(forkroad_direction ==1)
        {
          inflection_lowerL[0]=1;
          inflection_lowerL[1]=0;
          inflection_lowerL[2]=MT9V03X_CSI_W-1;
          
          inflection_lowerR[0]=1;
          inflection_lowerR[1]=MT9V03X_CSI_H-1;
          inflection_lowerR[2]=0;
          
          stayguy(1,inflection_lowerL, inflection_lowerR);
          
          scan_line_simple();   
        }
      
       
       // pwm_duty(SteerPWM_CH, SteerMIN);
       // systick_delay_ms(300); 

     }
        
    if(forkroad_time ==2 && flag_element_status ==1)
    {
      if(forkroad_direction ==2)
      {
      
          inflection_lowerL[0]=1;
          inflection_lowerL[1]=0;
          inflection_lowerL[2]=MT9V03X_CSI_W-1;
          
          inflection_lowerR[0]=1;
          inflection_lowerR[1]=MT9V03X_CSI_H-1;
          inflection_lowerR[2]=0;
          
          stayguy(1,inflection_lowerL, inflection_lowerR);
          
          scan_line_simple();           
          
      }
      
      
        if(forkroad_direction ==1)
        {
          inflection_lowerL[0]=1;
          inflection_lowerL[1]=MT9V03X_CSI_H-1;
          inflection_lowerL[2]=MT9V03X_CSI_W-1;
          
          inflection_lowerR[0]=1;
          inflection_lowerR[1]=0;
          inflection_lowerR[2]=0;
          
          stayguy(2,inflection_lowerR, inflection_lowerL);
          
          scan_line_simple();            
        }
       // pwm_duty(SteerPWM_CH, SteerMAX);
       // systick_delay_ms(300); 
        
    }
    
    if(forkroad_time ==3 && flag_element_status ==1)
    {
      if(forkroad_direction ==2)
      {
      
          inflection_lowerL[0]=1;
          inflection_lowerL[1]=0;
          inflection_lowerL[2]=MT9V03X_CSI_W-1;
          
          inflection_lowerR[0]=1;
          inflection_lowerR[1]=MT9V03X_CSI_H-1;
          inflection_lowerR[2]=0;
          
          stayguy(1,inflection_lowerL, inflection_lowerR);
          
          scan_line_simple();           
          
      
      }
      
     
      
        if(forkroad_direction ==1)
        {
          inflection_lowerL[0]=1;
          inflection_lowerL[1]=MT9V03X_CSI_H-1;
          inflection_lowerL[2]=MT9V03X_CSI_W-1;
          
          inflection_lowerR[0]=1;
          inflection_lowerR[1]=0;
          inflection_lowerR[2]=0;
          
          stayguy(2,inflection_lowerR, inflection_lowerL);
          
          scan_line_simple();   
        }          
     
       // pwm_duty(SteerPWM_CH, SteerMAX);
      // systick_delay_ms(300); 
          
    }
      //  pwm_duty(SteerPWM_CH, SteerMIN);
        //pwm_duty(MotorPWM_Go_L_CH, 11000);
        //pwm_duty(MotorPWM_Go_R_CH, 11000);  
        //systick_delay_ms(250);         
     // Flag_forkroad=0;
        
    
      
  }
 

  
  
  
 
  if(Flag_island_L)
  {
    
      if(flag_element_status ==3 || flag_element_status ==4)
      {
        
              inflection_lowerR[0]=1;
              inflection_lowerR[1]=0;
              inflection_lowerR[2]=0;
            
             inflection_lowerL[0]=1;
             inflection_lowerL[1]=throw_line_end_L;
             inflection_lowerL[2]=MT9V03X_CSI_W-1;

              stayguy(2,inflection_lowerR ,inflection_lowerL);
              
              scan_line_simple();
      }
    
    
      if(flag_element_status ==6)
      {
         
          inflection_lowerR[0]=1;
          inflection_lowerR[1]=0;
          inflection_lowerR[2]=0;
          
          inflection_lowerL[0]=1;
          inflection_lowerL[1]=MT9V03X_CSI_H-1;
          inflection_lowerL[2]=MT9V03X_CSI_W-1;
          
          stayguy(2,inflection_lowerR, inflection_lowerL);
          
          scan_line_simple();
         
      }
      
 
  }
  
  //右环岛
  
  if(Flag_island_R)
  {
    
      if(flag_element_status ==3 || flag_element_status ==4)
      {
        
              inflection_lowerL[0]=1;
              inflection_lowerL[1]=0;
              inflection_lowerL[2]=MT9V03X_CSI_W-1;
            
             inflection_lowerR[0]=1;
             inflection_lowerR[1]=throw_line_end_R;
             inflection_lowerR[2]=0;
 
              stayguy(2,inflection_lowerL, inflection_lowerR);
              
              scan_line_simple();
      }
    
    
      if(flag_element_status ==6)
      {
         
          inflection_lowerL[0]=1;
          inflection_lowerL[1]=0;
          inflection_lowerL[2]=MT9V03X_CSI_W-1;
          
          inflection_lowerR[0]=1;
          inflection_lowerR[1]=MT9V03X_CSI_H-1;
          inflection_lowerR[2]=0;
          
          stayguy(2,inflection_lowerL, inflection_lowerR);
          
          scan_line_simple();
         
      }
      
 
  }
  
  
  
  
  
  //
  if(Flag_depend_L)
  {
      uint8 i=0;
      for(i=0;i<80;i++)
      {
        middle_line[i]=left_border[i]-track_width_normal[i];
        
      }
        
  }
  
  if(Flag_depend_R)
  {
      uint8 i=0;
      for(i=0;i<80;i++)
      {
        middle_line[i]=right_border[i]+track_width_normal[i];
      }
      
  }
  

  if(Flag_block)
  {
      pwm_duty(SteerPWM_CH, MiddleSteer_PWM);
      stop_car(250);
      systick_delay_ms(1000);
     /*
      time=0;
      qtimer_quad_clear(QTIMER_1, Qtimer1_LSB);
      qtimer_quad_clear(QTIMER_3, Qtimer2_LSB);
            UniformSpeed(-7000);
            systick_delay_ms(300);
            UniformSpeed(0);
   
      while(time <= 4 && block_direction==0)
      {
         block_direction = ARTGetAprilTagNumber();
            EncoderCalDistance(300, 8000);
            systick_delay_ms(400);
            while (encoder_value_R != 0 && encoder_value_L != 0);
  
         
      }
      UniformSpeed(0);
      
      while(block_direction == InLeft && time <=8)
      {
          pwm_duty(Yuntai1PWM_CH, MaxYunTai2SteerPWM);
          while(camera==0 && time <= 8)
          {
              camera = ARTGetImageMessage();
            EncoderCalDistance(300, 8000);
            systick_delay_ms(400);
            while (encoder_value_R != 0 && encoder_value_L != 0);              
          
          }
          
          if(camera == IsAnimal && time >= 4)
          {
             // gpio_set(BEEF, 1);
              break;
          }
          else if(camera == IsFruit)
          {
                while(time <= 8)
                {
                    theta = ARTGetFruitImageCenter();
               
                    if(theta < 80)
                    {
                        EncoderCalDistance(300, -8000);
                        systick_delay_ms(400);
                        while (encoder_value_R != 0 && encoder_value_L != 0);              
                    }
                    
                    if(theta >= 80)
                    {
                        theta = (uint32)(theta * 48.5 - 3900);
                        pwm_duty(Yuntai1PWM_CH, MaxYunTai2SteerPWM - theta);
                        systick_delay_ms(150);
                        pwm_duty(JiguangPWM_CH, 25000);
                            
                        systick_delay_ms(1000);
                    }
                }
            
          }
          else
          {
              break;
          }
          
      }
      
      
      while(block_direction == InRight && time <=8)
      {
          pwm_duty(Yuntai1PWM_CH, MinYunTai2SteerPWM);
          while(camera==0 && time <= 8)
          {
              camera = ARTGetImageMessage();
            EncoderCalDistance(300, 8000);
            systick_delay_ms(400);
            while (encoder_value_R != 0 && encoder_value_L != 0);              
          
          }
          
          if(camera == IsAnimal && time >= 4)
          {
             // gpio_set(BEEF, 1);
              break;
          }
          else if(camera == IsFruit)
          {
                while(time <= 8)
                {
                    theta = ARTGetFruitImageCenter();
               
                    if(theta < 80)
                    {
                        EncoderCalDistance(300, -8000);
                        systick_delay_ms(400);
                        while (encoder_value_R != 0 && encoder_value_L != 0);              
                    }
                    
                    if(theta >= 80)
                    {
  
                        theta = (uint32)((160 - theta) * 48.5 - 3900);
                        pwm_duty(Yuntai1PWM_CH, MinYunTai2SteerPWM + theta);
                        systick_delay_ms(150);
                        pwm_duty(JiguangPWM_CH, 25000);
                            
                        systick_delay_ms(1000);
                    }
                }
            
          }
          else
          {
              break;
          }
          
      }
      
      pwm_duty(JiguangPWM_CH, 0);
      pwm_duty(Yuntai1PWM_CH, MidYunTai2SteerPWM);
      theta =0;
      camera =0;
      block_direction =0;
      
      systick_delay_ms(300);
      */
     
      
      Motor_GO_L_PWM=8000;
      Motor_GO_R_PWM=8000;
      Flag_go =1;
  }
  
  

      if(Flag_zebra_L && island_time ==2)
      {
        Flag_go=0;
          garagein_L();
          Flag_zebra_L =1;
         
      }
 
      
      if(Flag_zebra_R && island_time ==2)
      {
        Flag_go=0;
          garagein_R();
          Flag_zebra_R =1;
          
      }
      
 /*       
        if(Sobel > 3000 && island_time ==2)
        {
            Flag_go =0;
            if(flag_garagein_L)
            {
                Flag_go=0;
                garagein_L();
                Flag_zebra_L =1;
                
                      
            }
            
            if(flag_garagein_R)
            {
               Flag_go=0;
                garagein_R();
                Flag_zebra_R =1;
                         
            }
        }
 */ 
    
    
        //得出中线值后，计算出偏差
        if(middle_line_peak < 70)
        {
              
            current_value=(float)calculate_differ(10, middle_line_peak);
        }
        else
        {
          
           current_value=(float)calculate_differ(10, 70);
        }
   
   /* 
      if(Flag_zebra_L || Flag_zebra_R || Sobel > 3000)
      {
          current_value= current_value/10;
      }
    */
    
    if(Flag_zebra_L && island_time < 2)
    {
      current_value= current_value/10;
    }
    
    if(Flag_zebra_R && island_time < 2)
    {
      current_value= current_value/10;
    }
  

}





void judge_image()
{

   
    //十字，正入
    if(continuity_L==0 && continuity_R==0 && JugeCrossload(10, 90)
       && throw_line_num_L > 20 && throw_line_num_R > 20)   
    {
        Flag_crossroad=1;
        
        Flag_straightway = 0;
        Flag_island_R=0;
        Flag_island_L=0;
        Flag_forkroad=0;
        Flag_depend_L =0;
        Flag_depend_R =0;
        Flag_zebra_L =0;
        Flag_block =0;
    }
    else 
    {
         Flag_crossroad=0;
    }
    
    //左环岛
    if(continuity_L==0 && continuity_R==1 && middle_line_high >= 90 
       && throw_line_num_R <10 && check_island(1))
    {
        Flag_island_L =1;
        
        Flag_straightway = 0;
        Flag_crossroad = 0;
        Flag_island_R=0;
        Flag_forkroad=0;
        flag_element_status =0;
        
        island_time+=1;

    }
       //右环岛
    if(continuity_L==1 && continuity_R==0 && middle_line_high >= 90 
       && throw_line_num_L <10 && check_island(2))
    {
        Flag_island_R =1;
        
        Flag_straightway = 0;
        Flag_crossroad = 0;
        Flag_island_L=0;
        Flag_forkroad=0;
        
        flag_element_status =0;
        
        island_time+=1;

    }
    

    if(flag_garagein_L ==1)
    {
        //斑马线L
        if(JudgeZebra_L())
        {
          Flag_zebra_L =1;
          Flag_zebra_R =0;
          Flag_straightway = 0;
            Flag_crossroad = 0;
            Flag_island_R=0;
            Flag_island_L=0;
            Flag_forkroad=0;
            Flag_depend_L =0;
            Flag_depend_R =0;
            
            

        }
        else
        {
          if(island_time !=2)
          {
            Flag_zebra_L=0;
          }
        }
    }
    //
    if(flag_garagein_R ==1)
    {
          //斑马线R
      if(JudgeZebra_R())
      {
        Flag_zebra_R =1;
        Flag_zebra_L =0;
        Flag_straightway = 0;
          Flag_crossroad = 0;
          Flag_island_R=0;
          Flag_island_L=0;
          Flag_forkroad=0;
          Flag_depend_L =0;
          Flag_depend_R =0;
          
         

      }
      else
      {
        if(island_time != 2)
        {
          Flag_zebra_R =0;
        }
      }
    
    }
    
    
    //三岔&& width_wave(middle_line_high-30, middle_line_high-5)
    if(middle_line_high < 80 && check_forkroad() 
       && track_width[middle_line_high-5] > MT9V03X_CSI_W-50)
    {
        Flag_forkroad =1;
        Flag_crossroad = 0;
        Flag_island_R=0;
        Flag_island_L=0;
        Flag_depend_L =0;
        Flag_depend_R =0;
        Flag_zebra_L =0;
        Flag_zebra_R =0;
        Flag_straightway =0;
        
        flag_element_status =0;
       
      
    }
    else
    {
      if(throw_line_num_L < 20 && throw_line_num_R < 20)
      {
        Flag_forkroad =0;
      }
    }
    
    
    
   
    if(Flag_forkroad==1 && flag_element_status ==0)
    {
        flag_element_status =1;
        
    }
    if(flag_element_status ==1 && Flag_forkroad ==0)
    {
        flag_element_status =0;
        forkroad_time +=1;
    }
     
  
 
    if(Flag_zebra_L ==0 && Flag_zebra_R ==0 && middle_line_high < 80 
       && check_block(0,80) && throw_line_num_L < 10 && throw_line_num_R < 10)
    {
        Flag_block =1;
        Flag_crossroad = 0;
        Flag_island_R=0;
        Flag_island_L=0;
       // Flag_forkroad=0;
       // Flag_depend_L =0;
       // Flag_depend_R =0;
       // Flag_zebra_L =0;
      
       
       
    }
    else
    {
      Flag_block =0;
      
    }



/*
   if(Flag_zebra_L ==0 && Flag_zebra_R ==0 && middle_line_high < 60
      && throw_line_num_L <= 10 && throw_line_num_R <= 10 
        && check_block_sobel())
   {
        Flag_block =1;
        Flag_crossroad = 0;
        Flag_island_R=0;
        Flag_island_L=0;
        Flag_forkroad=0;
        Flag_depend_L =0;
        Flag_depend_R =0;
        Flag_zebra_L =0;
       
   }
   else
   {
      Flag_block =0;       
   }
    
*/      
    
    
    //直线
    if(continuity_L==1 && continuity_R==1 && throw_line_num_L < 20 && throw_line_num_R < 20
       && middle_line_high >= 80 && middle_line_peak >= 80)
    {
        Flag_straightway = 1;
        Flag_crossroad = 0;
        Flag_island_R=0;
        Flag_island_L=0;
        Flag_forkroad=0;
        Flag_depend_L =0;
        Flag_depend_R =0;
        Flag_zebra_L =0;
        Flag_zebra_R=0;
        Flag_block =0;
    }

    
    
}
  



//检测环岛入口，type=1,左环岛; type=2,右环岛
uint8 check_island(uint8 type)
{
    
        uint8 time_w =0;
       uint8 time_w1 = 0;
       uint8 time_b =0;
       uint8 i=0;
       uint8 flag=0;
       
       int error=0;
       int break_point_threshold=25;
 
  
  if(type == 1)
  {

 
      array_init(break_point_L1, 3, 0);
      
      for(i=0; i+1<60; i++)
      {
        
         error=(left_border[i+1]-left_border[i]);

        if(error > break_point_threshold)
        {
           break_point_L1[0]=1;
           break_point_L1[1]=i;       
           break_point_L1[2]=left_border[i];
         
           
           break;
            
        }
      }
       
       if(break_point_L1[0]==1)
       {
         for(i=break_point_L1[1];i<middle_line_peak;i++)
         {
            if(image(i, break_point_L1[2])==0 &&  flag ==0)
            {
              time_w++;
              if(time_w >= 15)
              {
                flag=1;
              }
            }
            
            if(flag==1 && image(i,break_point_L1[2])==1)
            {
                time_b++;
                if(time_b >= 15)
                {
                    flag = 2;
                }
            }
            
            if(flag==2 && image(i, break_point_L1[2])==0)
            {
                time_w1++;
                if(time_w1 >= 5)
                {
                  flag=3;
                  return 1;
                 }
            }
            
         }
         
       }
    
  }
  else if(type ==2)
  {
 
       array_init(break_point_R1, 3, 0); 
       
      for(i=0; i+1<60; i++)
      {
        error=(right_border[i]-right_border[i+1]);
         
        if(error > break_point_threshold)
          {
             break_point_R1[0]=1;
             break_point_R1[1]=i;
             break_point_R1[2]=right_border[i];
         
             break;

          }
      }
       if(break_point_R1[0]==1)
       {
         for(i=break_point_R1[1];i<middle_line_peak;i++)
         {
            if(image(i, break_point_R1[2])==0 &&  flag ==0)
            {
              time_w++;
              if(time_w >= 15)
              {
                flag=1;
              }
            }
            
            if(flag==1 && image(i,break_point_R1[2])==1)
            {
                time_b++;
                if(time_b >= 15)
                {
                    flag = 2;
                }
            }
            
            if(flag==2 && image(i, break_point_R1[2])==0)
            {
                time_w1++;
                if(time_w1 >= 5)
                {
                  flag=3;
                  return 1;
                 }
            }
            
         }
         
       }
      
    
  }
  
  
  
  
  return 0;
  

}


uint8 check_forkroad()
{       
  
    find_inflection_point(1,0,middle_line_high);
    find_inflection_point(2,0,middle_line_high);
    
    if(inflection_lowerL[0]==1 && inflection_lowerR[0]==1)
    {
       uint8 i=0,j=0;
       uint8 w_time1;
       uint8 b_time;
       uint8 w_time2;
       uint8 flag=0;
       
       for(i=middle_line_high;i< middle_line_high+20;i++)
       {
          for(j=0;j<MT9V03X_CSI_W-1;j++)
          {
              if(flag==0 && image(i,j) == 0)
              {
                  w_time1++;
                  if(w_time1 > 30)
                  {
                      flag=1;
                  }
              }
              
              
              if(flag==1 && image(i,j)==1)
              {
                  b_time++;
                  if(b_time > 30)
                  {
                      flag=2;
                  }
              }
              
              if(flag==2 && image(i,j)==0)
              {
                  w_time2 ++;
                  if(w_time2 > 30)
                  {
                      flag=3;
                      return 1;
                  }
              }
              
          }
       }
    }

    
   return 0;
}




uint8 JugeCrossload(uint8 startline, uint8 endline)
{
  if(endline > middle_line_peak)
  {
     endline = middle_line_peak;
  }
  
  uint8 i;
  int8 error=0;
  uint8 break_point_threshold=30;

 
 
 
      for(i=startline; i+1<endline; i++)
      {
        
         error=(left_border[i+1]-left_border[i]);
         
        if(error > break_point_threshold)
        {
           break_point_L1[0]=1;
           break_point_L1[1]=i-5;       
           break_point_L1[2]=left_border[i-5];
           
                     
        }

        if(error < -break_point_threshold)
        {
           break_point_L2[0]=1;
           break_point_L2[1]=i+6;       
           break_point_L2[2]=left_border[i+6];
         
           
           break;
            
        }
      }
      
 
      for(i=startline; i+1<endline; i++)
      {
        
        error=(right_border[i+1]-right_border[i]);
        
        
        if(error < -break_point_threshold)
        {
             break_point_R1[0]=1;
             break_point_R1[1]=i-5;
             break_point_R1[2]=right_border[i-5];
            
                 
         }
        
        
        if(error > break_point_threshold)
          {
             break_point_R2[0]=1;
             break_point_R2[1]=i+6;
             break_point_R2[2]=right_border[i+6];
            
             break;
            
          }
        
        
            
        
      }
      
      if(break_point_L2[0]==1 && break_point_R2[0]==1 && middle_line_peak >= 90
         && my_abs_int(break_point_L2[1]-break_point_R2[1]) <= 20)
      {
        
          return 1;
      }
      /*
      else if(break_point_R1[0]==1 && break_point_R2[0]==1 
              && throw_line_num_L >= 40 && middle_line_high >= 90)
      {
          return 1;
      }
      else if(break_point_L1[0]==1 && break_point_L2[0]==1 
              && throw_line_num_R >= 40 && middle_line_high >= 90)
      {
          
          return 1;
      }
      */
  
    return 0;
  
}

uint8 JudgeZebra_L()
{
    uint8 i,j;
    uint8 status=0;
    uint8 time=0;
    
  
    for(i=throw_line_start_L;i<throw_line_start_L+40;i++)//
    {
          time =0;
         for(j=0;j+3< MT9V03X_CSI_W-1;j++)
        {
          if(image(i,j)==0 && image(i, j+1)==0 
             && image(i, j+2)==1 && image(i, j+3)==1)
          {
            time ++;
            
          }
            
        }
        
        if(time >= 8)
        {
          status ++;
        }
    }
    
    if(status >=5)
    {
        return 1;
    }
    
    return 0;
  
}

uint8 JudgeZebra_R()
{
    uint8 i,j;
    uint8 status=0;
    uint8 time=0;
    
  
    for(i=throw_line_start_R;i<throw_line_start_R+40;i++)
    {
          time =0;
         for(j=0;j+3< MT9V03X_CSI_W-1;j++)
        {
          if(image(i,j)==0 && image(i, j+1)==0 
             && image(i, j+2)==1 && image(i, j+3)==1)
          {
            time ++;
            
          }
            
        }
        
        if(time >= 8)
        {
          status ++;
        }
    }
    
    if(status >=5)
    {
        return 1;
    }
    
    return 0;   
}
  

void clear_image(uint8 h, uint8 w)
{
    uint8 i,j;
    uint8 h_r =30;
    uint8 w_r =40;
    
    
      
    for(i=h-h_r;i<h+h_r;i++)
    {
        for(j=w-w_r;j<w+w_r;j++)
        {
            otsu_image[MT9V03X_CSI_H-i-1][MT9V03X_CSI_W-j-1]=0;
        }
          
    }
      
  
}


void cover_image(uint8 h, uint8 w)
{
    uint8 i,j;
    uint8 h_r =30;
    uint8 w_r =40;
    
    
      
    for(i=h-h_r;i<h+h_r;i++)
    {
        for(j=w-w_r;j<w+w_r;j++)
        {
            otsu_image[MT9V03X_CSI_H-i-1][MT9V03X_CSI_W-j-1]=1;
        }
          
    }
      
  
}

void garagein_L()
{
    uint8 flag=0;
    uint8 i=0;
    uint8 top;
    uint8 time=0;
   
    
    for(i=0;i<100;i++)
    {
      if(flag ==0 && image(i,MT9V03X_CSI_W-1)==0 && image(i+1,MT9V03X_CSI_W-1)==0)
      {
        time++;
        if(time > 20)
        {
          flag =1;
        }
          
      }
      
      if(flag ==1 && image(i,MT9V03X_CSI_W-1)==1 && image(i+1,MT9V03X_CSI_W-1)==1)
      {
        flag=2;
        top = i;
      }
    }
    
 
    if(flag ==2 && top < 60)//不带二维码 top=70
    {
            Flag_go=0;
                    pwm_duty(SteerPWM_CH, 3600);
                   // pwm_duty(MotorPWM_Go_L_CH, 11000);//不带二维码注释掉
                 //  pwm_duty(MotorPWM_Go_R_CH, 11000);  
                    systick_delay_ms(300); 
                    stop_car(200);
                    while(1);
                      
         
    }
  
    
    
}

void garagein_R()
{
    uint8 flag=0;
    uint8 i=0;
    uint8 top;
    uint8 time=0;
   
    
    for(i=0;i<100;i++)
    {
      if(flag ==0 && image(i,0)==0 && image(i+1,0)==0)
      {
        time++;
        if(time > 20)
        {
          flag =1;
        }
          
      }
      
      if(flag ==1 && image(i,0)==1 && image(i+1,0)==1)
      {
        flag=2;
        top = i;
      }
    }
    

  
    if(flag ==2 && top < 60)//不带二维码 top = 70
    {
   
         Flag_go=0;
                   pwm_duty(SteerPWM_CH, 2850);
                 //  pwm_duty(MotorPWM_Go_L_CH, 11000);//不带二维码，注释掉
                 //  pwm_duty(MotorPWM_Go_R_CH, 11000);  
                    systick_delay_ms(300); 
                    stop_car(200);
                    while(1);
                           
      
    }
 
   
    
    
}

uint8 check_block(uint8 startline, uint8 endline)
{
      
    uint8 flag1=0;
    uint8 flag2=0;
    uint8 flag3=0;
    uint8 i=0;
    uint8 time_w1 =0;
    uint8 time_w2 =0;
    uint8 error1 =0;
    uint8 error2 =0;
    uint8 error3 =0;
    uint8 error =0;
    uint8 high1=0;
    uint8 high2=0;
    uint8 high3=0;
    uint8 conL=0;
    uint8 conR=0;
    
    //
    for(i=0;i<endline;i++)
    {
        if(flag1==0 && image(i,Middle_point_w)==0 && image(i+1,Middle_point_w)==0)
        {
            time_w1++;
            
        }
        
        if(flag1 ==0 && time_w1 > 10)
        {
            flag1=1;
        }
        
        if(flag1 ==1 && image(i,Middle_point_w)==1 && image(i+1,Middle_point_w)==1)
        {
            flag1=2;
            high1=i;
            break;
            
        }   
        
    }
    
    
    for(i=endline;i>high1;i--)
    {
        if(flag1==2 && image(i,Middle_point_w)==0 && image(i-1,Middle_point_w)==0)
        {
            time_w2++;
            
        }
        
        if(flag1==2 && time_w2 > 20)
        {
            flag1=3;
        }
        
        if(flag1==3 && image(i,Middle_point_w)==1)
        {
            flag1=4;
            error1 = i-high1;
            break;
        }
    }
    
    
    //
    time_w1 =0;
    time_w2 =0;
    for(i=0;i<endline;i++)
    {
        if(flag2==0 && image(i,Middle_point_w+20)==0 && image(i+1,Middle_point_w+20)==0)
        {
            time_w1++;
            
        }
        
        if(flag2 ==0 && time_w1 > 10)
        {
            flag2=1;
        }
        
        if(flag2 ==1 && image(i,Middle_point_w+20)==1 && image(i+1,Middle_point_w+20)==1)
        {
            flag2=2;
            high2=i;
            break;
            
        }   
        
    }
    
    
    for(i=endline;i>high1;i--)
    {
        if(flag2==2 && image(i,Middle_point_w+20)==0 && image(i-1,Middle_point_w+20)==0)
        {
            time_w2++;
            
        }
        
        if(flag2==2 && time_w2 > 20)
        {
            flag2=3;
        }
        
        if(flag2==3 && image(i,Middle_point_w+20)==1)
        {
            flag2=4;
            error2 = i-high2;
            break;
        }
    }
    //
    time_w1 =0;
    time_w2 =0;
    for(i=0;i<endline;i++)
    {
        if(flag3==0 && image(i,Middle_point_w-20)==0 && image(i+1,Middle_point_w-20)==0)
        {
            time_w1++;
            
        }
        
        if(flag3 ==0 && time_w1 > 10)
        {
            flag3=1;
        }
        
        if(flag3 ==1 && image(i,Middle_point_w-20)==1 && image(i+1,Middle_point_w-20)==1)
        {
            flag3=2;
            high3=i;
            break;
            
        }   
        
    }
    
    
    for(i=endline;i>high1;i--)
    {
        if(flag3==2 && image(i,Middle_point_w-20)==0 && image(i-1,Middle_point_w-20)==0)
        {
            time_w2++;
            
        }
        
        if(flag3==2 && time_w2 > 20)
        {
            flag3=3;
        }
        
        if(flag3==3 && image(i,Middle_point_w-20)==1)
        {
            flag3=4;
            error3 = i-high3;
            break;
        }
    }
  
     error = (uint8)((error1 + error2 + error3)/3);
    
    conL = judge_continuty(1,0,middle_line_high-10);
    conR = judge_continuty(2,0,middle_line_high-10);
    
    
    if((error1 > 15 || error2 > 15 || error3 > 15 || error > 15) && conL ==1 && conR ==1)
    {
        return 1;
    }
    
     
    if(track_width[20] < track_width_normal[20]+10 && track_width[21] < track_width_normal[21]+10
       && track_width[22] < track_width_normal[22]+10 && track_width[23] < track_width_normal[23]+10
         && track_width[24] < track_width_normal[24]+10 && track_width[25] < track_width_normal[25]+10
           && throw_line_num_L < 10 && throw_line_num_R < 10)
    {
        return 1;
    }
    
  
    
    
    
  
    return 0;
  
}


void garageout1()
{
    while(garageout)
    {    
//      //左边出库
 /*
     if (gpio_get(KEY1) == 0) {
        pwm_duty(SteerPWM_CH, 3450);
        systick_delay_ms(5000); 
        pwm_duty(SteerPWM_CH, 3450);
        pwm_duty(MotorPWM_Go_L_CH, 11000);
        pwm_duty(MotorPWM_Go_R_CH, 11000);
        systick_delay_ms(1000);     
        flag_garagein_L =1;
        flag_garagein_R =0;
        garageout =0; 
     }  
     //右边出库
        pwm_duty(SteerPWM_CH, 3000);
      if (gpio_get(KEY2) == 0) {
        systick_delay_ms(5000);                
        pwm_duty(MotorPWM_Go_L_CH, 11000);
        pwm_duty(MotorPWM_Go_R_CH, 11000);       
        systick_delay_ms(1000);       
        flag_garagein_L =0;
        flag_garagein_R =1;
        garageout =0; 
     } 
  //////   
     
 */
        if(mt9v03x_csi_finish_flag)
        {
             
            ImageBinary();
            
           //  scan_line_base();
            // Interactive();
             
             
            if (gpio_get(KEY1) == 0) {
              garageout_dir =1;
              systick_delay_ms(3000);  
            }
            if (gpio_get(KEY2) == 0) {
              garageout_dir =2;
              systick_delay_ms(3000);  
            }
              
              
            
            
           //左边出库
            if (garageout_dir ==1) 
            { 
              
              pwm_duty(SteerPWM_CH, MiddleSteer_PWM);
              pwm_duty(MotorPWM_Go_L_CH, 11000);
              pwm_duty(MotorPWM_Go_R_CH, 11000);      
              
                uint8 flag=0;
                uint8 i=0;
                uint8 top;       
                uint8 time=0;
                for(i=0;i<100;i++)
                {
                  if(flag ==0 && image(i,MT9V03X_CSI_W-10)==0 && image(i+1,MT9V03X_CSI_W-10)==0)
                  {
                    time++;
                    if(time > 20)
                    {
                      flag =1;
                    }
                      
                  }        
                  if(flag ==1 && image(i,MT9V03X_CSI_W-10)==1 && image(i+1,MT9V03X_CSI_W-10)==1)
                  {
                    flag=2;
                    top = i;
                  }
                }
                
                if(flag ==2 && top < 60)
                {
                    pwm_duty(SteerPWM_CH, SteerMAX);
                    pwm_duty(MotorPWM_Go_L_CH, 11000);
                    pwm_duty(MotorPWM_Go_R_CH, 11000);  
                    systick_delay_ms(500); 
                    flag_garagein_L =1;
                    flag_garagein_R =0;
                    garageout =0;                     
                                
                }
            }
            
                    
           //右边出库
            if (garageout_dir ==2) 
            { 
              
              pwm_duty(SteerPWM_CH, MiddleSteer_PWM);
              pwm_duty(MotorPWM_Go_L_CH, 11000);
              pwm_duty(MotorPWM_Go_R_CH, 11000);      
              
                uint8 flag=0;
                uint8 i=0;
                uint8 top;             
                for(i=0;i<100;i++)
                {
                  if(flag ==0 && image(i,0)==0 && image(i+1,0)==0)
                  {
                      flag =1;
                      
                  }        
                  if(flag ==1 && image(i,0)==1 && image(i+1,0)==1)
                  {
                    flag=2;
                    top = i;
                  }
                }
                
                if(flag ==2 && top < 60)
                {
                    pwm_duty(SteerPWM_CH, SteerMIN);
                    pwm_duty(MotorPWM_Go_L_CH, 11000);
                    pwm_duty(MotorPWM_Go_R_CH, 11000);  
                    systick_delay_ms(500); 
                    flag_garagein_L =0;
                    flag_garagein_R =1;
                    garageout =0;                     
                                
                }
            } 
          
            mt9v03x_csi_finish_flag = 0;       
        }

     
    }
}


  

uint8 count_lost_time(uint8 type, uint8 startline, uint8 endline)
{
  if(endline > middle_line_peak)
  {
    endline = middle_line_peak;
  }
  
  uint8 lost_time_L=0;
  uint8 lost_time_R=0;
  
  uint8 i=0;

 //记录丟线数据
  if(type ==1)
  {
      for(i=startline;i<endline;i++)
      {
        if(flag_left_border[i]==0)
        {
          lost_time_L++;
          
          
        }
      }
      
      return lost_time_L;
  }
  
  if(type==2)
  {
      for(i=startline; i< endline; i++)
      {
        if(flag_right_border[i]==0)
        {
            lost_time_R++;
            
        } 
      }
      
      return lost_time_R;
  }
   
    
   return 0;
    
  
}





void SobelTest()
{
   Sobel = 0;
    uint8 i,j;
    for (i = 5; i < 50; ++i) {
        for (j = 30; j < 150; ++j) {
            int64 Gx = 0, Gy = 0;
            Gx = (-1 * image(i - 1, j - 1) + image(i - 1, j + 1) - 2 * image(i, j - 1)
                  + 2 * image(i, j + 1) - image(i + 1, j - 1) + image(i + 1, j + 1));
            Gy = (-1 * image(i - 1, j - 1) - 2 * image(i - 1, j) - image(i - 1, j + 1)
                  + image(i + 1, j + 1) + 2 * image(i + 1, j) + image(i + 1, j + 1));
            Sobel += FastABS(Gx) + FastABS(Gy);
           
        }
    }
   
}

uint8 check_block_sobel()
{
    SobelTest();
    if(Sobel > 2500)
    {
        return 1;
    }
  
    return 0;
  
}


void ARTGetNumber() 
{
    if(forkroad_direction != Trident_Left && forkroad_direction != Trident_Right) 
    {
        ART_SendByte(0x01);
        GetCameraMessage();           
      //forkroad_direction = 1;           
        forkroad_direction = Camera;
    }
    if(forkroad_direction == Trident_Left && forkroad_direction == Trident_Right)
    {
        Camera =0;//清零
    }
    
}

uint8 ARTGetAprilTagNumber() {
    ART_SendByte(0X02);
    if(Camera != InLeft && Camera != InRight && Camera != TAGNOTFOUND)
        GetCameraMessage();
    uint8 temp = Camera;
    Camera = 0;
    if (temp == TAGNOTFOUND) {
        return 0;
    }
    return temp;
}

uint8 ARTGetImageMessage() {
    ART_SendByte(0X03);
    if(Camera != IsAnimal && Camera != IsFruit)
        GetCameraMessage();
    uint8 temp = Camera;
    Camera = 0;
    return temp;
}

uint8 ARTGetFruitImageCenter() {
    ART_SendByte(0X04);
    // while (Camera != Go && Camera != Return && Camera != Shoot) {
    //     if (!GetCameraMessage())
    //         ART_SendByte(0X04);
    // }
    if(!GetCameraMessage())
    {
        GetCameraMessage();
    }
    uint8 temp = Camera;
    Camera = 0;
    return temp;
}


uint8 GetCameraMessage()
{
    return uart_query(USART_4, &Camera);
}


void ART_SendByte(uint8 byte)
{
    uart_putchar(USART_4, byte);
}

