
#include "zf_common_headfile.h"

int flag=0;
int a=0,b=0,c=0,d=0;
extern int Key_num0,Key_num1,Key_num2,Key_num3;
uint8_t key_count = 0;
uint8_t start_flag = 0;
uint8_t menu_count = 0;
volatile uint8_t Flag = 0;
void Key_Use(void)
{

//    key_state_enum
        key_scanner();
         Key_num3 = key_get_state(KEY_4);
         Key_num2 = key_get_state(KEY_3);
         Key_num1 = key_get_state(KEY_2);
         Key_num0 = key_get_state(KEY_1);
        if(Key_num0 == KEY_SHORT_PRESS)//中
        {
            key_count +=1 ;
            if(key_count==0)
            {
                base_speed = 0;
                start_flag=0;
                pwm_set_duty(FUYA,0);
            }
            else if(key_count==1)
            {
                pwm_set_duty(FUYA,1750);
            }
            else if(key_count==2)
            {
                start_flag=1;
                key_count+=1;
            }
            else if(key_count==4)
            {
             key_count=0;
             base_speed = 0;
             start_flag=0;
             pwm_set_duty(FUYA,0);
            }


        }
        else if(Key_num1 == KEY_SHORT_PRESS)//右 硬件没设计好 不要用
        {
//            Pid_speedR.P+= 0.5;
        }
        else if(Key_num2 == KEY_SHORT_PRESS)//上
        {

            menu_count  +=  1;
            if (menu_count==5)
            {
                menu_count =0;
            }
            else if(menu_count==1)
            {
                ips200_clear();
            ips200_show_uint(170,192,1,1);
            }
            else if(menu_count==2)
            {
                ips200_clear();
            ips200_show_uint(170,208,1,1);
            }
            else if(menu_count==3)
            {
                ips200_clear();
            ips200_show_uint(170,224,1,1);
            }
            else if(menu_count==4)
            {
                ips200_clear();
            ips200_show_uint(170,240,1,1);

            }
        }
        else if(Key_num3 == KEY_SHORT_PRESS)//左
        {
            if(menu_count==1)
            {
                Two_PD.KP2  += 0.1;
//                Pid_speedL.P  += 2;
            }
            else if(menu_count==2)
            {
                Two_PD.KD+= 0.03;
//                Pid_speedL.I+= 0.1;
            }
            else if(menu_count==3)
            {
                Two_PD.KP  += 1.5;
//                Pid_speedL.P+= 2;
            }
            else if(menu_count==4)
            {
                Two_PD.GKD += 0.02;
//                Pid_speedL.I+= 0.03;
            }
        }


}
/*
 * Key.c
 *
 *  Created on: 2023年11月9日
 *      Author: levi
 */

//#include "zf_common_headfile.h"

//#define KEY4                    (P20_6)
//#define KEY3                    (P20_8)
//#define KEY2                    (P20_7)
//#define KEY1                    (P20_9)
//
//
///*********************************************************************************************************
//*    KEY_Read()
//*    识别第几个按键按下
//*    mode：1 支持连按，0不支持连按
//**********************************************************************************************************/
////short KEY_Read(short mode)
////{
////    static short key_up=1;     //按键松开标志位
////    if(mode==1)    key_up=1;
////
////    if(key_up && (gpio_get_level(KEY1)==0 || gpio_get_level(KEY2)==0 || gpio_get_level(KEY3)==0
////                  || gpio_get_level(KEY4)==0 ))
////    {
////        system_delay_ms(10);
////        key_up=0;
////
////        if(gpio_get_level(KEY1)==0 )         return 1;
////        else if(gpio_get_level(KEY2)==0)     return 2;
////        else if( gpio_get_level(KEY3)==0)    return 3;
////        else if(gpio_get_level(KEY4)==0 )    return 4;
////
////    }
////
////    if(gpio_get_level(KEY1)==1 && gpio_get_level(KEY2)==1 && gpio_get_level(KEY3)==1
////       && gpio_get_level(KEY4)==1)    key_up=1;
////
////    return 0;
////}
//
//
//uint8  delay_time=20;
//void key_Init(void)
//{
//    gpio_init(KEY1, GPI, GPIO_HIGH, GPI_PULL_UP);           // 初始化 KEY1 输入 默认高电平 上拉输入
//    gpio_init(KEY2, GPI, GPIO_HIGH, GPI_PULL_UP);           // 初始化 KEY2 输入 默认高电平 上拉输入
//    gpio_init(KEY3, GPI, GPIO_HIGH, GPI_PULL_UP);           // 初始化 KEY3 输入 默认高电平 上拉输入
//    gpio_init(KEY4, GPI, GPIO_HIGH, GPI_PULL_UP);           // 初始化 KEY4 输入 默认高电平 上拉输入
//}
//
//key_num key_count;
//
//void Key_Get(void)
//{
//    key_count.key1_count=0;
//    key_count.key2_count=0;
//    key_count.key3_count=0;
//    key_count.key4_count=0;
//
//
//
//    if(gpio_get_level(KEY1)==0)
//    {
//        system_delay_ms(delay_time);
//        while(gpio_get_level(KEY1)==0);
//        system_delay_ms(delay_time);
//        key_count.key1_count++;
//    }
//    if(gpio_get_level(KEY2)==0)
//    {
//        system_delay_ms(delay_time);
//        while(gpio_get_level(KEY2)==0);
//        system_delay_ms(delay_time);
//        key_count.key2_count++;
//    }
//    if(gpio_get_level(KEY3)==0)
//    {
//        system_delay_ms(delay_time);
//        while(gpio_get_level(KEY3)==0);
//        system_delay_ms(delay_time);
//        key_count.key3_count++;
//    }
//    if(gpio_get_level(KEY4)==0)
//    {
//        system_delay_ms(delay_time);
//        while(gpio_get_level(KEY4)==0);
//        system_delay_ms(delay_time);
//        key_count.key4_count++;
//    }
//
//}

/*********************************************************************************************************
*    Key_Scan_canshu()
*    开始前菜单更改数据
*    key1作用是--，key2作用是++，key3的作用是切换数据，key4的作用是长按以后开始执行正常数据
**********************************************************************************************************/
//#define PARAMETER_N   300  //240
//#define PARAMETER_N_1 120
//uint16 dianya;
//uint16 parameter;           //指引标
//uint16 parameter_1;
//
//uint8 adjust_ok_0;
//uint8 adjust_ok_1;
//void Key_Scan_canshu(void)
// {
//     uint8 key1 = 1,       key2 = 1,       key3 = 1,       key4 = 1;
//     uint8 key1_last = 1,  key2_last = 1,  key3_last = 1,  key4_last = 1;
//     uint8 key_num = 0;
//
//     while(1)
//     {
//         static unsigned int key4_count = 0;
//         //保存按键状态
//         key1_last = key1;
//         key2_last = key2;
//         key3_last = key3;
//         key4_last = key4;
//
//               //读取当前按键状态
//       key1 = gpio_get_level(KEY1);
//       key2 = gpio_get_level(KEY2);
//       key3 = gpio_get_level(KEY3);
//       key4 = gpio_get_level(KEY4);
//
//         if((key1 && !key1_last))
//             key_num = 1;
//         if((key2 && !key2_last))
//             key_num = 2;
//         if((key3 && !key3_last))
//             key_num = 3;
//         if(key4 && !key4_last)
//             key_num = 4;
//         if(key3==0)    key4_count++;//长按
//         //根据键值选择对应功能
//         switch(key_num)
//         {
//             case 0:
//                 break;
//             case 4:
//                 parameter -= 8;
//                 parameter = (parameter >= 0) ? parameter : 0;
//                 key4_count = 0;
//                 break;
//             case 3:
//                 parameter += 8;
//                 parameter = (parameter <= PARAMETER_N) ? parameter : 0;
//                 break;
//             case 2:
//                 switch(parameter/8)
//                 {
////                     case 1:servo.Camera_duty  += 1; break;
////                     case 2:chu    +=1; break;
////                     case 3:jinru_yuansu    += 1; break;
////                     case 4:move_speed += 10; break;
////                     case 5:back_che_duojiduty += 1; break;
////                     case 6:hui_jiaodu += 5; break;
////                     case 7:arr  += 1; break;
////                     case 8:jinru_yuansu += 1; break;
////                     case 9:jinru_yuansu += 0.1; break;
////                     case 10:A_10  += 1; break;
////                     case 11:SpeedPID.Ki  += 0.1; break;
////                     case 12:SpeedPID.Kd  += 0.1; break;
////
////                     case 13:RingJSDTurnPID.Kp += 1; break;
////                     case 14:show += 1; break;
////
////
////                     case 15:bizhang1jieduan1 += 5; break;
////                     case 16:bizhang2jieduan1 += 5; break;
////                     case 17:bizhang3jieduan1 += 5; break;
////                     case 18:jieduan1wucha1 += 1; break;
////                     case 19:jieduan2wucha1 += 1; break;
////                     case 20:jieduan3wucha1 += 1; break;
////
////                     case 21:zuodiuxian1 += 1; break;
////                     case 22:zuobaidian1 += 1; break;
////                     case 23:zuokuandu1 += 1; break;
////                     case 24:youdiuxian1 += 1; break;
////                     case 25:youbaidian1 += 1; break;
////                     case 26:youkuandu1 += 1; break;
//
//                 }
//                 break;
//             case 1:
//                 switch(parameter/8)
//                 {
////                     case 1:Car   -= 1; break;
////                     case 2:chu    -= 1; break;
////                     case 3:jinru_yuansu   -= 1; break;
////                     case 4:move_speed  -= 10; break;
////                     case 5:back_che_duojiduty-= 1; break;
//////                     case 6:JSDTurnPID.Kd -= 0.1; break;
////                     case 6:hui_jiaodu -= 5; break;
////                     case 7:arr -= 1; break;
////                     case 8:jinru_yuansu-=1; break;
////                     case 9:jinru_yuansu -= 0.1; break;
////                     case 10:A_10  -= 1; break;
////                     case 11:SpeedPID.Ki  -= 0.1; break;
////                     case 12:SpeedPID.Kd  -= 0.1; break;
////
////                     case 13:RingJSDTurnPID.Kp  -= 1; break;
////                     case 14:show  -= 1; break;
////
////
////
////                     case 15:bizhang1jieduan1 -= 5; break;
////                     case 16:bizhang2jieduan1 -= 5; break;
////                     case 17:bizhang3jieduan1 -= 5; break;
////                     case 18:jieduan1wucha1 -= 1; break;
////                     case 19:jieduan2wucha1 -= 1; break;
////                     case 20:jieduan3wucha1 -= 1; break;
////
////                     case 21:zuodiuxian1 -= 1; break;
////                     case 22:zuobaidian1 -= 1; break;
////                     case 23:zuokuandu1 -= 1; break;
////                     case 24:youdiuxian1 -= 1; break;
////                     case 25:youbaidian1 -= 1; break;
////                     case 26:youkuandu1 -= 1; break;
//
//                 }
//                 break;
//         }
//         //显示当前参数
//         if(key_num)
//         {
//             if(parameter<=112)
//             {
//                 //显示游标
////                 tft180_clear();
////                 tft180_show_string(0, parameter, ">");
////                 tft180_show_string(92, parameter, ">");
////
//////显示调节后的参数
////                 //显示当前参数
////                 tft180_show_string(10,   0, "para");
////                 tft180_show_int(100, 0, (parameter/8), 2);
////
////                 //方向环pid参数
////                 tft180_show_string(10,    8, "servo.Camera_duty");
////                 tft180_show_int(100, 8, servo.Camera_duty,4);
////
////                 tft180_show_string(10,    16, "chu");
////                 tft180_show_int(100, 16, key4_count,8);
////
////                 tft180_show_string(10,    24, "jinru_yuansu");
////                 tft180_show_int(100, 24, jinru_yuansu,4);
////
////                 //速度环pid参数
////                 tft180_show_string(10,    32, "move_speed");
////                 tft180_show_int(100, 32, move_speed,4);
////
////                 tft180_show_string(10,    40, "back_che_duojiduty");
////                 tft180_show_int(100, 40, back_che_duojiduty,4);
////
////                 tft180_show_string(10,    48, "hui_jiaodu");
////                 tft180_show_int(100, 48, hui_jiaodu,4);
////
////                 tft180_show_string(10,    56, "arr");
////                 tft180_show_int(100, 56, arr,4);
//////                 tft180_show_string(10,    56, "ADC_Kp");
//////                 tft180_show_int(100, 56, ADCJSDTurnPID.Kp,4);
////
////                 tft180_show_string(10,    64, "ad[0]");
////                 tft180_show_float(100, 64, 144000/(arr*psc),3, 1);
//
////                 tft180_show_string(10,    72,"jinru_yuansu");
////                 tft180_show_int(100, 72, jinru_yuansu,4);
////
////                 tft180_show_string(10,    80, "jinru_yuansu");
////                 tft180_show_int(100, 80, jinru_yuansu,4);
//
////                 tft180_show_string(10,    88, "S_Ki");
////                 tft180_show_float(100, 88, SpeedPID.Ki, 3, 2);
////
////                 tft180_show_string(10,    96,"S_Kd");
////                 tft180_show_float(100, 96, SpeedPID.Kd, 3, 2);
////
////                 tft180_show_string(10,    104,"yuan_kp");
////                 tft180_show_int(100, 104, RingJSDTurnPID.Kp, 3);
////
////                 tft180_show_string(10,    112,"show");
////                 tft180_show_int(100, 112, show, 3);
//
//
//             }
////             else if(parameter<=224)
////             {
////                 tft180_clear();
////                 tft180_show_string(0, parameter-112, ">");
////                 tft180_show_string(92, parameter-112, ">");
////
////                     //显示调节后的参数
////                 //显示当前参数
////                 tft180_show_string(10,   0, "para");
////                 tft180_show_int(100, 0, (parameter/8), 2);
////
////                 //方向环pid参数
////                 tft180_show_string(10,    8, "BZ_time1");
////                 tft180_show_int(100, 8, bizhang1jieduan1, 5);
////
////                 tft180_show_string(10,    16, "BZ_time2");
////                 tft180_show_int(100, 16, bizhang2jieduan1, 5);
////
////                 tft180_show_string(10,    24, "BZ_time3");
////                 tft180_show_int(100, 24, bizhang3jieduan1, 5);
////
////                 //速度环pid参数
////                 tft180_show_string(10,    32, "BZ_err1");
////                 tft180_show_int(100, 32, jieduan1wucha1, 5);
////
////                 tft180_show_string(10,    40, "BZ_err2");
////                 tft180_show_int(100, 40, jieduan2wucha1, 5);
////
////                 tft180_show_string(10,    48, "BZ_err3");
////                 tft180_show_int(100, 48, jieduan3wucha1, 5);
////
////                 tft180_show_string(10,    56, "ZHD_diuxian");
////                 tft180_show_int(100, 56, zuodiuxian1,5);
////
////                 tft180_show_string(10,    64, "ZHD_baidian");
////                 tft180_show_int(100, 64, zuobaidian1, 5);
////
////                 tft180_show_string(10,    72,"ZHD_kuandu");
////                 tft180_show_int(100, 72, zuokuandu1, 5);
////
////                 tft180_show_string(10,    80, "YHD_diuxian");
////                 tft180_show_int(100, 80, youdiuxian1,5);
////
////                 tft180_show_string(10,    88, "YHD_baidian");
////                 tft180_show_int(100, 88, youbaidian1, 5);
////
////                 tft180_show_string(10,    96,"YHD_kuandu");
////                 tft180_show_int(100, 96, youkuandu1, 5);
////             }
//             //键值清零
//             key_num = 0;
//         }
//         if(( key4_count>600000))
//         {
//             adjust_ok_0=1;
//         }
//         //调参完成跳出循环
//         if(adjust_ok_0 == 1)
//         {
//             parameter = 0;
//             break;
//         }
//     }
//     //显示速度策略
//     tft180_clear();//清屏
// }
//
//


