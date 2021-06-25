#include "image.h"
#include "headfile.h"
#include "Binarization.h"


#define Middle_point_w (MT9V03X_CSI_W/2 - 1)
#define Middle_point_h (MT9V03X_CSI_H/2 - 1)


/*

//配置摄像头参数
//#define MT9V03X_CSI_W               188             //图像宽度  范围1-752      RT105X RT106X 采集时列宽度必须为4的倍数
//#define MT9V03X_CSI_H               60             //图像高度	范围1-480

*/




//状态机写法
uint8 Flag_island_L=0;
uint8 Flag_island_R=0;
uint8 Flag_crossroad=0;
uint8 Flag_straightway=0;
uint8 Flag_curve=0;
uint8 Flag_zebra=0;
uint8 Flag_rampway=0;
uint8 Flag_forkroad=0;
uint8 Flag_t_road=0;
uint8 Flag_depend_R=0;
uint8 Flag_depend_L=0;


uint8 left_border[MT9V03X_CSI_H]={0};  //赛道左边界
uint8 right_border[MT9V03X_CSI_H]={0};  //赛道右边界
uint8 middle_line[MT9V03X_CSI_H]={0};  //赛道中点坐标
uint8 track_width[MT9V03X_CSI_H]={0};   //赛道宽度

//正常赛道宽度， 待测
uint8 track_width_normal[60]={60,59,58,57,56,55,54,53,52,51,50,49,48,48,47,46,45,44,44,43,42,41,40,40,39,38,37,36,36,31,35,34,33,33,32,31,30,30,29,28,27,27,26,25,24,23,22,21,21,20,19,18,17,16,15,14,14,13,12,11}; //待测

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

//元素状态标志位
uint8 flag_element_status=0;

//赛道边界连续性
uint8 continuity_L=0;
uint8 continuity_R=0;



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
 
  //特征量初始化
  throw_line_start_L =0;
  throw_line_end_L =0;
  throw_line_start_R =0;
  throw_line_end_R =0;
  throw_line_num_L =0;
  throw_line_num_R =0;
  middle_line_peak =MT9V03X_CSI_H-1;  

  //数组初始化
  all_array_init();
  uint8 old  = Middle_point_w;
  
  for(i=0;(i+2)<MT9V03X_CSI_H-1;i++)
  {

/*    
      
      //环岛改变扫线起始点
      if(Flag_island_L)
      {
          j=j + track_width_normal[i]/4;
      }
      else if(Flag_island_R)
      {
          j=j - track_width_normal[i]/4;
      }
      
 */ 
    /*
      //十字路口行扫线起点偏移
      if(Flag_crossroad)
      {
          if(current_value < 0)
          {
              old += 15;
          }
          else
          {
              old -= 15;
          }
            
      }
*/
        
      for(j=old; (j+2)<MT9V03X_CSI_W-1; j+=2)   //寻找左边界
      {
        if(image(i,j)==1 && image(i,j+1)==1 && image(i,j+2)==1)
        {
          left_border[i]=j;
          flag_left_border[i]=1; 
          break;
        }
       
      }
      
      
      for(j=old; (j-2)>0; j-=2)               //寻找右边界
      {
        if(image(i,j)==1 && image(i,j-1)==1 && image(i,j-2)==1)
        {
          right_border[i]=j;
          flag_right_border[i]=1; 
          break;
        }
       
      }
    
      //赛道宽度
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
     
    if(image(i,middle_line[i])==1 && image(i+1,middle_line[i])==1 && image(i+2,middle_line[i])==1)  //寻找上顶点
    {
        middle_line_peak=i;
        break;
    }
    
    old = middle_line[i];
    
  }
  
  //图像中点高度
  for(i=0;i+2<MT9V03X_CSI_H-1;i++)
  {
      if(image(i, Middle_point_w)==1 && image(i+1, Middle_point_w)==1
         && image(i+2, Middle_point_w)==1)
      {
          middle_line_high = i;
          break;
      }
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
      throw_line_end_L = middle_line_peak;
  }
  
  if(time_R ==1)
  {
      throw_line_end_R = middle_line_peak;
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

//拟合,生成fitting_line[]
void regression(uint8 type, uint8 startline, uint8 endline)
{
    if(endline > MT9V03X_CSI_H-1)
    {
      endline = MT9V03X_CSI_H-1;
    }
   
    int sum_x=0,sum_y=0;
    int average_x, average_y;
    int sum1=0,sum2=0;
    int i=0;
    int sumlines = endline - startline;
    

    if (type == 1)//拟合左线
    {
      
	for (i = startline; i < endline; i++)
	{
		sum_x += i;
		sum_y += left_border[i];
	}
	if (sumlines == 0) sumlines = 1;
	average_x = sum_x / sumlines;     //x的平均值
	average_y = sum_y / sumlines;     //y的平均值
	for (i = startline; i < endline; i++)
	{
		
		sum1 += (left_border[i] - average_y) * (i - average_x);
		sum2 += (i - average_x) * (i - average_x);
	}
	if (sum2 == 0) slope = 0;
	else slope = sum1 / sum2;
	intercept = average_y - slope * average_x;
        
       for(i=0; i<MT9V03X_CSI_H-1; i++)
        {
            fitting_border_L[i]=(uint8)(slope * i + intercept);
        }
    }
    else if (type == 2)//拟合右线
    {

	for (i = startline; i < endline; i++)
	{
		sum_x += i;
		sum_y += right_border[i];
	}
	if (sumlines == 0) sumlines = 1;
	average_x = sum_x / sumlines;     //x的平均值
	average_y = sum_y / sumlines;     //y的平均值
	for (i = startline; i < endline; i++)
	{
		sum1 += (right_border[i] - average_y) * (i - average_x);
		sum2 += (i - average_x) * (i - average_x);
	}
	if (sum2 == 0) slope = 0;
	else slope = sum1 / sum2;
	intercept = average_y - slope * average_x;
        
        for(i=0; i<MT9V03X_CSI_H-1; i++)
        {
            fitting_border_R[i]=(uint8)(slope * i + intercept);
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
  int8 slope=0;
  int8 intercept=0;
  
  if(type == 1) //对左边界经行拉线
  {
      slope = (inflection2[2] - inflection1[2])/(inflection2[1] - inflection1[1]);
      intercept = (inflection1[2] + inflection2[2])/2 - slope*(inflection1[1]+inflection2[1])/2;
      
      int i=0;
      for(i=inflection1[1];i<inflection2[1];i++)
      {
        left_border[i]=(uint8)(slope * i +intercept);
        set_pixel(i, left_border[i]);
        set_pixel(i, left_border[i]+1);
        set_pixel(i, left_border[i]+2);
        set_pixel(i, left_border[i]+3);
        set_pixel(i, left_border[i]+4);
      
      }
  }
  else if(type == 2)//对右边界经行拉线
  {
      slope = (inflection2[2] - inflection1[2])/(inflection2[1] - inflection1[1]);
      intercept = (inflection1[2] + inflection2[2])/2 - slope*(inflection1[1]+inflection2[1])/2;
      
      int i=0;
      for(i=inflection1[1];i<inflection2[1];i++)
      {
        right_border[i]=(uint8)(slope * i +intercept);
        set_pixel(i, right_border[i]);
        set_pixel(i, right_border[i]-1);
        set_pixel(i, right_border[i]-2);
        set_pixel(i, right_border[i]-3);
        set_pixel(i, right_border[i]-4);
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
  uint8 error=0;
  uint8 break_point_threshold=10;
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
          
          
        
        if(error < break_point_threshold && time==2)
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
          
        
        if(error < break_point_threshold && time==2)
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
    
    uint8 i=0;
    uint8 time=0;
    int error1 = 0;
    int error2 = 0;
    
    if(type == 1)
    {
            
        for(i=startline;i+2<endline;i+=2)
        {
            error1 = left_border[i+1]-left_border[i];
            error2 = left_border[i+2]-left_border[i+1];
            
            if((error1 * error2) < 0)       //判断为拐点
            {
                if(time == 0)
                {
                    inflection_lowerL[0]=1;
                    inflection_lowerL[1]=i+1;
                    inflection_lowerL[2]=left_border[i+1];
                    time=1;
                  
                }
                
                if(time == 1)
                {
                    inflection_upL[0]=1;
                    inflection_upL[1]=i+1;
                    inflection_upL[2]=left_border[i+1];
                    break;
                }
                  
              
            }
          
        }
    }
    else if(type == 2)
    {
        
        for(i=startline;i+2<endline;i+=2)
        {
            error1 = right_border[i+1]-right_border[i];
            error2 = right_border[i+2]-right_border[i+1];
            
            if((error1 * error2) < 0)       //判断为拐点
            {
                if(time == 0)
                {
                    inflection_lowerR[0]=1;
                    inflection_lowerR[1]=i+1;
                    inflection_lowerR[2]=right_border[i+1];
                    time=1;
                  
                }
                
                if(time == 1)
                {
                    inflection_upR[0]=1;
                    inflection_upR[1]=i+1;
                    inflection_upR[2]=right_border[i+1];
                    break;
                }
                  
              
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
//start_point = 10;
//end_point = 50;
float calculate_differ(uint8 start_point, uint8 end_point)
{
  if(end_point > MT9V03X_CSI_H-1)
  {
      end_point = MT9V03X_CSI_H-1;
  }
  
  uint8 i;
  
  float sum=0;
  
  for(i=start_point;i<end_point;i++)
  {
    sum+=(float)(Middle_point_w - middle_line[i]);
  }
  
  
  current_value=(float)(sum/(end_point - start_point));
  
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
    count_throw_line(0,60);
    
    continuity_L=judge_continuty(1, 0, 60);
    continuity_R=judge_continuty(2, 0, 60);
 
    find_break_point(1, 0, 60);
    find_break_point(2, 0, 60); 
    
   
 
    if(Flag_island_L !=1)
    {
 
      judge_image();
      
    }
    

  if(Flag_crossroad)
  {     
     
    //入十字前，拉线
    if(flag_right_border[20]==0 && flag_right_border[19]==0 && flag_right_border[18]==0
       && flag_left_border[20]==0 && flag_left_border[19]==0 && flag_left_border[18]==0)
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
    if(break_point_L1[0]==1 && break_point_L2[0]==0 && break_point_R1[0]==1 && break_point_R2[0]==0
       && flag_right_border[5]==0 && flag_right_border[6]==0 && flag_left_border[5]==0 && flag_left_border[6]==0)
    {
         regression(1, throw_line_end_L+10, throw_line_end_L+15);
         regression(2, throw_line_end_R+10, throw_line_end_R+15); 
         
         uint8 i;
         
         for(i=0;i<60;i++)
         {
           left_border[i]=fitting_border_L[i];
           right_border[i]=fitting_border_R[i];
           
           
         }
           
         count_middle_line(0,60); 
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
      
      if(flag_element_status ==1 && throw_line_num_L < 30)
      {
          flag_element_status =2;
      }
        
      
      //入口拉线
      if(flag_element_status==2 && throw_line_num_L > 50)
      {
          Flag_depend_R =0;
         
          flag_element_status =3;
         
      }
      
      if(flag_element_status ==3 && middle_line_peak < 40)
      {
       // Flag_depend_L =1;
        flag_element_status = 4;
      }
      
      //出环岛拉线
      if(flag_element_status ==4 && check_t_road())
      {
       // Flag_depend_L =1;
        flag_element_status =5;
      
        
      }
        
      if(flag_element_status ==5 && middle_line_high > 60)
      {
        Flag_depend_L=0;
        Flag_depend_R =1;
        flag_element_status =6;
        
      }
      
      if(flag_element_status ==6 && throw_line_num_L < 30)
      {
        Flag_island_L=0;
        Flag_depend_R=0;
        flag_element_status=0;
      }
        
   
      
  }
  
  
 
  if(Flag_island_L)
  {
      if(flag_element_status ==3)
      {
        
              inflection_lowerR[0]=1;
              inflection_lowerR[1]=0;
              inflection_lowerR[2]=0;
            
       //       inflection_lowerL[0]=1;
       //       inflection_lowerL[1]=throw_line_end_L;
       //       inflection_lowerL[2]=left_border[throw_line_end_L];
      
        
              uint8 break_point_island[3]={0};
    
              uint8 i;
              int16 error1=0;
              uint8 break_point_threshold=20;
            
             

              for(i=0; i+1<60; i++)
              {
                    error1=(left_border[i] - left_border[i+1]);
                    
                    if(error1 > break_point_threshold)//判断为断点
                    {
                        break_point_island[0]=1;
                        break_point_island[1]=i;
                        break_point_island[2]=left_border[i];
                      
                    }
                     
              }
              
              stayguy(2,inflection_lowerR ,break_point_island);
            scan_line_base(); 
          count_middle_line(0, middle_line_peak);
       
      }
    
      if(flag_element_status ==5)
      {
         
          inflection_lowerR[0]=1;
          inflection_lowerR[1]=0;
          inflection_lowerR[2]=0;
          
          inflection_lowerL[0]=1;
          inflection_lowerL[1]=throw_line_end_L+1;
          inflection_lowerL[2]=left_border[throw_line_end_L-1];
          
          stayguy(2,inflection_lowerR, inflection_lowerL);
          
    scan_line_base(); 

    count_middle_line(0, middle_line_peak);
         
      }
      
 
  }


  
  if(Flag_depend_L)
  {
      uint8 i=0;
      for(i=0;i<60;i++)
      {
        middle_line[i]=left_border[i]-track_width_normal[i];
        
      }
        
  }
  
  if(Flag_depend_R)
  {
      uint8 i=0;
      for(i=0;i<60;i++)
      {
        middle_line[i]=right_border[i]+track_width_normal[i];
      }
      
  }
  

        //得出中线值后，计算出偏差
        if(middle_line_peak < 40)
        {
        
            current_value=(float)calculate_differ(0, middle_line_peak);
        }
        else
        {
          
           current_value=(float)calculate_differ(0, 40);
        }
   
  

}



/*
（行，列）
顶点：(middle_line_peak，middle_line[middle_line_peak])

右线丟线数目：throw_line_num_R==0：右线顶点：(middle_line_peak，right_border[middle_line_peak])
右线丟线数目：throw_line_num_R > 0 :右线丟线起始行：（throw_line_start_R，0),右线丟线结束行：（throw_line_end_R，0）

左线丟线数目：throw_line_num_L ==0:左线顶点：（middle_line_peak, left_border[middle_line_peak]）
左线丟线数目：throw_line_num_L >0:左线丢线起始行：（throw_line_start_L,MT9V03X_CSI_W-1）,左线丟线结束行：（throw_line_end_L,MT9V03X_CSI_W-1）

原点：（0，middle_line[0]）


*/

/*
void judge_image()
{
    count_throw_line(0,middle_line_peak);
    
    continuity_L=judge_continuty(1, 0, middle_line_peak);
    continuity_R=judge_continuty(2, 0, middle_line_peak);
 
  //  find_break_point(1, 0, middle_line_peak);
   // find_break_point(2, 0, middle_line_peak); 
    
    if(throw_line_num_R >30 && throw_line_num_L >30)
    {
        find_break_point(1, 0, middle_line_peak);
        find_break_point(2, 0, middle_line_peak);
     
        if(middle_line_peak > (uint8)(4/5*MT9V03X_CSI_H) && continuity_L ==0 && continuity_R ==0
           && break_point_L1[0]==1 && break_point_R1[0]==1)
        {
            Flag_crossroad=1;
        }
        
        if(check_forkroad())
        {
             Flag_forkroad = 1;
        }
        
        
       
    }
    else if(throw_line_num_R > 30 && throw_line_num_L <30)
    {
     
        if(middle_line_peak >= MT9V03X_CSI_H*2/3 && continuity_L ==1 && continuity_R ==0)
        {
          
            Flag_island_R = 1;
        }
    
    }
    else if(throw_line_num_R <30 && throw_line_num_L >30)
    {
       
      if(middle_line_peak >= MT9V03X_CSI_H*2/3 && continuity_L ==0 && continuity_R ==1)
        {
          if(check_island(1))
          {
              Flag_island_L = 1;
              flag_element_status=1;
          }
        }
      
    }
    else if(throw_line_num_R <= 30 && throw_line_num_L <= 30)
    {
   
            Flag_straightway = 1;
            Flag_crossroad = 0;
            Flag_island_R=0;
            Flag_island_L=0;
            Flag_forkroad=0;

    }
   
}
*/


void judge_image()
{
    count_throw_line(0,60);
    
    continuity_L=judge_continuty(1, 0, 60);
    continuity_R=judge_continuty(2, 0, 60);
 
    find_break_point(1, 0, 60);
    find_break_point(2, 0, 60); 
    
    
    //十字，正入
    if(continuity_L ==0 && continuity_R ==0 && middle_line_high >= 60 && throw_line_num_R >30
       && my_abs_int(break_point_L1[1]-break_point_R1[1]) <= 20 && throw_line_num_L >30)   
    {
        Flag_crossroad=1;
        
        Flag_straightway = 0;
        Flag_island_R=0;
        Flag_island_L=0;
        Flag_forkroad=0;

    }
    
    //左环岛
    if(continuity_L ==0 && continuity_R ==1 && middle_line_peak >= 40 && throw_line_num_R <20
        && check_island(1))
    {
        Flag_island_L =1;
        
        Flag_straightway = 0;
        Flag_crossroad = 0;
        Flag_island_R=0;
        Flag_forkroad=0;

    }
    
    
    
    //直线
    if(continuity_L ==1 && continuity_R ==1 && throw_line_num_L < 20 && throw_line_num_R < 20)
    {
        Flag_straightway = 1;
        Flag_crossroad = 0;
        Flag_island_R=0;
        Flag_island_L=0;
        Flag_forkroad=0;
        Flag_depend_L =0;
        Flag_depend_R =0;
        
    }

    
    
}
  



//检测环岛入口，type=1,左环岛; type=2,右环岛
uint8 check_island(uint8 type)
{
    
  if(type == 1)
  {
       uint8 time_w =0;
       uint8 time_w1 = 0;
       uint8 time_b =0;
       uint8 i=0;
       uint8 flag=0;
       
       if(break_point_L1[0]==1)
       {
         for(i=break_point_L1[1];i<middle_line_peak;i++)
         {
            if(image(i, break_point_L1[2])==0 &&  flag ==0)
            {
              time_w++;
              if(time_w >= 10)
              {
                flag=1;
              }
            }
            
            if(flag==1 && image(i,break_point_L1[2])==1)
            {
                time_b++;
                if(time_b >= 8)
                {
                    flag = 2;
                }
            }
            
            if(flag==2 && image(i, break_point_L1[2])==0)
            {
                time_w1++;
                if(time_w1 >= 3)
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


uint8 check_island_status1()
{
  uint8 break_point_island[3]={0};
  
  uint8 i;
  int8 error1=0;
  int8 error2=0;
  uint8 break_point_threshold=40;
  uint8 time =0;
 

      for(i=0; i+1<60; i++)
      {
            error1=(left_border[i] - left_border[i+1]);
            
            if(error1 > break_point_threshold)//判断为断点
            {
                break_point_island[0]=1;
                break_point_island[1]=i+1;
                break_point_island[2]=left_border[i+1];
              
            }
             
      }
      
      
  if(break_point_island[0]==1)
  {
     for(i=0;i<break_point_island[1]-5;i++)
        {
            if(image(i,MT9V03X_CSI_W-2)==0)
            {
                time++;
            }
          
        }
  }      
  error1 = my_abs_int(left_border[break_point_island[1]+2]-left_border[break_point_island[1]+3]);
  error2 = my_abs_int(left_border[break_point_island[1]+4]-left_border[break_point_island[1]+5]);
    
   
       
   
  if(time > break_point_island[1]-10 && error1 < 5 && error2 < 5)
  {
      return 1;
  }
  
  
  
  
  return 0;
 
}

uint8 check_forkroad()
{       
   find_inflection_point(1, 0, 60);
   find_inflection_point(2, 0, 60);
   
   if(inflection_lowerL[0]==1 && inflection_lowerR[0]==1 && middle_line_high < 40
      && middle_line_peak < 40)
   {
     
      return 1;
      
   }
    
   return 0;
}


uint8 check_t_road()
{
  
    if(middle_line_high < 60 && throw_line_num_L > 25 
       && throw_line_num_R > 10)
    {
        return 1;
    }
    
    return 0;
}
  


//图像发送，串口发送
void image_send()
{
 
        
     csi_seekfree_sendimg_03x(USART_8,mt9v03x_csi_image[0],MT9V03X_CSI_W,MT9V03X_CSI_H);//串口初始化 由于默认开启了printf功能，因此在board_init已经初始化了串口1 因此本例程不在初始化   
 
}


