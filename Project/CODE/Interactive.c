//
// Created by xcs on 2021-03-08.
//
#include "headfile.h"

#include "Interactive.h"
#include "port.h"
#include "fastmath.h"
#include "image.h"
#include "tft.h"


extern PID_Struct Steer_PID;
extern PID_Struct Motor_GOL_PID;
extern PID_Struct Motor_GOR_PID;

extern uint8 Flag_island_L;
extern uint8 Flag_island_R;
extern uint8 Flag_crossroad;
extern uint8 Flag_zebra_L;
extern uint8 Flag_zebra_R;
extern uint8 Flag_rampway;
extern uint8 Flag_forkroad;
extern uint8 Flag_t_road;
extern uint8 Flag_depend_R;
extern uint8 Flag_depend_L;
extern uint8 Flag_block;

extern float current_value;
extern uint8 (*mt9v03x_csi_image)[MT9V03X_CSI_W];

extern uint8 Flag_straightway;
extern uint8 Flag_curve;

extern uint8 continuity_L;
extern uint8 continuity_R;
extern uint8 throw_line_num_R;
extern uint8 throw_line_num_L;
extern uint8 middle_line_peak;
extern uint8 middle_line_high;


extern uint8 break_point_L1[3];
extern uint8 break_point_R1[3];
extern uint8 inflection_lowerL[3];
extern uint8 inflection_lowerR[3];

extern uint8 flag_element_status;

extern uint32 duojipwm;

extern float encoder_value_L;
extern float encoder_value_R;
extern float encoder_value;
extern float speed;
extern float speed_error;
extern float Kp;
extern float Kd;
extern float Angle;

extern uint8 forkroad_time;
extern int64 Sobel;
extern int time;


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
    uint8 status = 0x00;
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
    uint8 Status;
    Status = bm_scan();
    uint8 Key_State = key_scan();
    switch (Status) {
        case 0X00: {
          
            //显示屏横向显示
            lcd_clear(BLACK);
          // lcd_displayimage032_zoom(mt9v03x_csi_image[0], MT9V03X_CSI_W, MT9V03X_CSI_H, 160, 128);
           tft_show_border(MT9V03X_CSI_W, MT9V03X_CSI_H, 160, 128);
           /*
            oled_fill(0X00);
            oled_p6x8str(0, 0, "straight:");
            oled_printf_float(50, 0, Flag_straightway, 5, 2);
            oled_p6x8str(0, 1, "Island_L:");
            oled_printf_float(50, 1, Flag_island_L, 5, 2);
            oled_p6x8str(0, 2, "Island_R:");
            oled_printf_float(50, 2, Flag_island_R, 5, 2);
            oled_p6x8str(0, 3, "crossroad:");
            oled_printf_float(50, 3, Flag_crossroad, 5, 2);
            oled_p6x8str(0, 4, "fork:");
            oled_printf_float(50, 4, Flag_forkroad, 5, 2);
            oled_p6x8str(0, 5, "dep_R:");
            oled_printf_float(50, 5, Flag_depend_R, 5, 2);
            oled_p6x8str(0, 6, "dep_L:");
            oled_printf_float(50, 6, Flag_depend_L, 5, 2);
            oled_p6x8str(0, 7, "current:");
            oled_printf_float(50, 7, current_value, 5, 2);
 
             */ 
            
            switch (Key_State) {
                case key1press: {
                   
                }
                    break;
                case key2press: {
                 
                    
                   } break;
                case key3press: {
                  
                    }break;
                case key4press: {
                    
                }
                    break;
      
            }
        }
        break;
        
        case 0x01: {
          
         // lcd_clear(WHITE);
          tft_show_otsu_image(MT9V03X_CSI_W, MT9V03X_CSI_H, 160, 128);
          
         // tft_show_border(MT9V03X_CSI_W, MT9V03X_CSI_H, 160, 128);
          
          
          /*
            lcd_clear(WHITE);
            lcd_showstr(0,0,num);
            lcd_showfloat(50,1,num1,5,2);
            lcd_showfloat(50,2,num2,5,2);
          */

            switch (Key_State) {
                case key1press: {
                  lcd_clear(BLACK);
                  
                 // num1++;
             
                }
                    break;
                case key2press: {
                  
                 // num2++;
                  
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
        break;
       case 0x10: {     
     
             // lcd_clear(WHITE);
              lcd_showstr(0,0,"crossroad:");
              lcd_showfloat(80,0,Flag_crossroad,5,2);
              
              lcd_showstr(0,1,"island_L:");
              lcd_showfloat(80,1,Flag_island_L,5,2);
              lcd_showstr(0,2,"island_R:");
              lcd_showfloat(80,2,Flag_island_R,5,2);
              
              lcd_showstr(0,3,"straight:");
              lcd_showfloat(80,3,Flag_straightway,5,2);
              
              lcd_showstr(0,4,"zebra_R:");
              lcd_showfloat(80,4,Flag_zebra_R,5,2);
              lcd_showstr(0,5,"zebra_L:");
              lcd_showfloat(80,5,Flag_zebra_L,5,2);
              
              lcd_showstr(0,6,"forkroad:");
              lcd_showfloat(80,6,Flag_forkroad,5,2);

              lcd_showstr(0,7,"block:");
              lcd_showfloat(80,7,Flag_block,5,2);
/*           
              lcd_showstr(0,0,"encoder_L:");
              lcd_showfloat(80,0,encoder_value_L,5,2);
              
              lcd_showstr(0,1,"encoder_R:");
              lcd_showfloat(80,1,encoder_value_R,5,2);
              
              lcd_showstr(0,2,"speed:");
              lcd_showfloat(80,2,speed,5,2);
              
              lcd_showstr(0,3,"spe_error:");
              lcd_showfloat(80,3,speed_error,5,2);
              
              lcd_showstr(0,4,"encoder:");
              lcd_showfloat(80,4,encoder_value,5,2);
  
              lcd_showstr(0,5,"Kp_L:");
              lcd_showfloat(80,5,Motor_GOL_PID->KP,5,2);
              
              lcd_showstr(0,6,"Kd:");
              lcd_showfloat(80,6,Motor_GOR_PID->KD,5,2);

              lcd_showstr(0,7,"Angle:");
              lcd_showfloat(80,7,Angle,5,2);
          
                  
            oled_fill(0X00);
            oled_p6x8str(0, 0, "KP:");
            oled_printf_float(50, 0, Steer_PID->KP, 5, 2);
            oled_p6x8str(0, 1, "con_L:");
            oled_printf_float(50, 1, continuity_L, 5, 2);
            oled_p6x8str(0, 2, "con_R:");
            oled_printf_float(50, 2, continuity_R, 5, 2);
            oled_p6x8str(0, 3, "var_R:");
            oled_printf_float(50, 3, var_R, 5, 2);
            oled_p6x8str(0, 4, "var_L:");
            oled_printf_float(50, 4, var_L, 5, 2);
            oled_p6x8str(0, 5, "TN_L:");
            oled_printf_float(50, 5, throw_line_num_L, 5, 2);
            oled_p6x8str(0, 6, "TN_R:");
            oled_printf_float(50, 6, throw_line_num_R, 5, 2);
            oled_p6x8str(0, 7, "tosu:");
            oled_printf_float(50, 7, T, 5, 2);
  */               

            switch (Key_State) {
                case key1press: {
                  
               // lcd_clear(BLACK);
                // Steer_PID->KP +=1.0;
                 // yuntaiPWM +=200;
                  Kp +=0.01;
                }
                    break;
                case key2press: {

              //  Steer_PID->KP -=1.0;
                //  yuntaiPWM -=200;
                  Kp -=0.01;
                  
                
                }
                    break;
                case key3press: {
                 // Steer_PID->KP +=1.0;
                  Kd +=0.01;
                    
                }
                    break;
                case key4press: {
                  // Steer_PID->KP -=1.0;
                  Kd -=0.01;
                }
                    break;
            }
      }
      break;
        case 0x11: {
          
             // lcd_clear(WHITE);
              lcd_showstr(0,0,"fork_T:");
              lcd_showfloat(80,0,forkroad_time,5,2);
              
              lcd_showstr(0,1,"inflec:");
              lcd_showfloat(80,1,inflection_lowerL[0],5,2);
              lcd_showstr(0,2,"inflec:");
              lcd_showfloat(80,2,inflection_lowerR[0],5,2);
              
              lcd_showstr(0,3,"time:");
              lcd_showfloat(80,3,time,5,2);
              lcd_showstr(0,4,"throw_nR:");
              lcd_showfloat(80,4,throw_line_num_R,5,2);
              
              lcd_showstr(0,5,"high:");
              lcd_showfloat(80,5,middle_line_high,5,2);
              lcd_showstr(0,6,"peak:");
              lcd_showfloat(80,6,middle_line_peak,5,2);
              
              lcd_showstr(0,7,"sobel:");
              lcd_showfloat(80,7,Sobel,5,2);

 
            switch (Key_State) {
                case key1press: {
                  
                 lcd_clear(BLACK);
               //   duojipwm +=10;
                 
                }
                    break;
                case key2press: {
                  
                //  duojipwm-=10;
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
    break;
    }
}






//////

void Interactive2() {
    uint8 Status;
    Status = bm_scan();
    uint8 Key_State = key_scan();
    switch (Status) {
        case 0X00: {
          
         
            oled_fill(0X00);
            oled_p6x8str(0, 0, "straight:");
            oled_printf_float(50, 0, Flag_straightway, 5, 2);
            oled_p6x8str(0, 1, "Island_L:");
            oled_printf_float(50, 1, Flag_island_L, 5, 2);
            oled_p6x8str(0, 2, "Island_R:");
            oled_printf_float(50, 2, Flag_island_R, 5, 2);
            oled_p6x8str(0, 3, "crossroad:");
            oled_printf_float(50, 3, Flag_crossroad, 5, 2);
            oled_p6x8str(0, 4, "fork:");
            oled_printf_float(50, 4, Flag_forkroad, 5, 2);
            oled_p6x8str(0, 5, "block:");
            oled_printf_float(50, 5, Flag_block, 5, 2);
            oled_p6x8str(0, 6, "zebra_R:");
            oled_printf_float(50, 6, Flag_zebra_R, 5, 2);
            oled_p6x8str(0, 7, "zebra_L:");
            oled_printf_float(50, 7, Flag_zebra_L, 5, 2);
 
            
            
            switch (Key_State) {
                case key1press: {
                   
                }
                    break;
                case key2press: {
                 
                    
                   } break;
                case key3press: {
                  
                    }break;
                case key4press: {
                    
                }
                    break;
      
            }
        }
        break;
        
        case 0x01: {
          
          
          
          
          
     
  
            switch (Key_State) {
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
        break;
       case 0x10: {     
    
          
                  
            oled_fill(0X00);
            oled_p6x8str(0, 0, "encoder_L:");
            oled_printf_float(50, 0, encoder_value_L, 5, 2);
            oled_p6x8str(0, 1, "encoder_R:");
            oled_printf_float(50, 1, encoder_value_R, 5, 2);
            oled_p6x8str(0, 2, "speed:");
            oled_printf_float(50, 2, speed, 5, 2);
            oled_p6x8str(0, 3, "spe_error:");
            oled_printf_float(50, 3, speed_error, 5, 2);
            oled_p6x8str(0, 4, "Angle:");
            oled_printf_float(50, 4, Angle, 5, 2);
            oled_p6x8str(0, 5, "encoder:");
            oled_printf_float(50, 5, encoder_value, 5, 2);
            oled_p6x8str(0, 6, "KP:");
            oled_printf_float(50, 6, Motor_GOL_PID->KP, 5, 2);
            oled_p6x8str(0, 7, "Kd:");
            oled_printf_float(50, 7, Motor_GOR_PID->KD, 5, 2);
              

            switch (Key_State) {
                case key1press: {
                  
               // lcd_clear(BLACK);
                // Steer_PID->KP +=1.0;
                 // yuntaiPWM +=200;
                  Kp +=0.01;
                }
                    break;
                case key2press: {

              //  Steer_PID->KP -=1.0;
                //  yuntaiPWM -=200;
                  Kp -=0.01;
                  
                
                }
                    break;
                case key3press: {
                 // Steer_PID->KP +=1.0;
                  Kd +=0.01;
                    
                }
                    break;
                case key4press: {
                  // Steer_PID->KP -=1.0;
                  Kd -=0.01;
                }
                    break;
            }
      }
      break;
        case 0x11: {
          
     
              
            oled_fill(0X00);
            oled_p6x8str(0, 0, "fork_T:");
            oled_printf_float(50, 0, forkroad_time, 5, 2);
            
            oled_p6x8str(0, 1, "inflec:");
            oled_printf_float(50, 1, inflection_lowerL[0], 5, 2);
            oled_p6x8str(0, 2, "inflec:");
            oled_printf_float(50, 2, inflection_lowerR[0], 5, 2);
            
            oled_p6x8str(0, 3, "time:");
            oled_printf_float(50, 3, time, 5, 2);
            
            oled_p6x8str(0, 4, "high:");
            oled_printf_float(50, 4, middle_line_high, 5, 2);
            oled_p6x8str(0, 5, "peak:");
            oled_printf_float(50, 5, middle_line_peak, 5, 2);
            
            oled_p6x8str(0, 6, "throw_R:");
            oled_printf_float(50, 6, throw_line_num_R, 5, 2);
            
            oled_p6x8str(0, 7, "throw_L:");
            oled_printf_float(50, 7, throw_line_num_L, 5, 2);
              

 
            switch (Key_State) {
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
    break;
    }
}
