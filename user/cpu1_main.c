
#include "zf_common_headfile.h"
#pragma section all "cpu1_dsram"
uint16 n = 0;
uint16 last_n = 0;
uint16 max_n = 0;
uint16 min_n = 99;
uint8_t OTSU_Flag = 0;
uint8_t OTSU_Num = 1;
uint8_t chujie = 0;
uint8_t chujie_count=0;
uint32_t time = 0;
void core1_main(void)
{
    uint8_t start_point_valid = 0U;

    disable_Watchdog();                     // 关闭看门狗
    interrupt_global_enable(0);             // 打开全局中断
    cpu_wait_event_ready();                 // 等待所有核心初始化完毕
    while (TRUE)
    {
//        ips200_show_uint(0,120,target_speed,4);
//        ips200_show();
//        Key_Use();
        system_start();
        if(mt9v03x_finish_flag)
            {
            if(OTSU_Flag%OTSU_Num==0)
            {

//                ips200_displayimage03x((const uint8 *)Pixle, image_w, image_h);                       // 显示原始图像
                nowThreshold = Otsu_TrackThreshold(Image_Use_linear, image_w, image_h);
                OTSU_Flag=0;
            }
             Get_BinaryImage2();
             Dashedline_Makeup();
             start_point_valid = get_start_point();
             Get_start_center();
             search_l_r(start_point_l,start_center_y,start_point_r,start_center_y);
             Get_lost_tip(2U);
             get_whitetip();
             Flag_Init();
             Deal_crossroads(T_Turn3);
             Deal_T_junction1(T_Turn1);
             Deal_T_junction2(T_Turn2);
//             ips200_show_uint(180,0,IF,5);
             Deal_Right_Angle();
             Deal_Left_Angle();
             Get_top_straightline();
             Lost_find();
             Line_Completion();
             if(Limit_protection(start_point_valid)==1)
             {
                 Stop_car();
                 protection_mode = 2;

             }
//             ips200_show_uint(180,0,base_speed,5);
//             ips200_show_uint(180,16,system_getval_us(),5);
             Select_Flag();
             Flag_Monitor();
             Turn_Control();
             deviationestimate();
             TIPS();

             time = system_getval_us();
//             target_speed = fangcha_speed(400.0f,440.0f);
            OTSU_Flag++;
            n++;
            mt9v03x_finish_flag=0;
            }
    }
}



#pragma section all restore
// **************************** 代码区域 ****************************
