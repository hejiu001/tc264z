#include "zf_common_headfile.h"
extern uint16 n;
extern uint16 max_n ;
extern uint16 min_n ;
uint8_t count = 0;
uint8_t Show_count = 20;

void ips200_show_FPS(void)
{
//    ips200_show_string(190,0,"FPS:");
    ips200_show_uint(190,116,max_n,3);
    ips200_show_uint(190,132,min_n,3);
    ips200_show_uint(190,148,n,3);
}
void ips200_show_encoder(void)
{
    ips200_show_int(96,240,DIST,5);
    ips200_show_int(96,256,L_motor.encoder_speed,4);
    ips200_show_int(96,272,R_motor.encoder_speed,4);
    ips200_show_int(96,288,Pid_speedL.Out,5);
    ips200_show_int(96,302,Pid_speedR.Out,5);
    ips200_show_int(96,208,Pid_speedL.Pout,5);
    ips200_show_int(96,224,Pid_speedR.Pout,5);
}

void ips200_show_flag(void)
{
    ips200_show_string(190,32,"Flag:");
//    ips200_show_uint(190,48,T_Turn1,2);
//    ips200_show_uint(190,64,T_Turn2,2);
    ips200_show_uint(190,80,Flag_Count,2);
    ips200_show_uint(190,96,Flag_Count2,2);
    ips200_show_uint(190,112,Flag_Count3,2);
    ips200_show_int(190,128,DIST,5);
    ips200_show_float(190,144,pose.data.yaw,3,1);
}

void ips200_show_PID1()
{
    ips200_show_string(190,176,"T_PD:");
    ips200_show_float(190,192,Two_PD.KP2,2,2);
    ips200_show_float(190,208,Two_PD.KD,2,2);
    ips200_show_float(190,224,Two_PD.KP,2,2);
    ips200_show_float(190,240,Two_PD.GKD,2,2);
//    ips200_show_float(190,224,Pid_speedL.P,2,2);
//    ips200_show_float(190,240,Pid_speedL.I,2,2);
}

void ips200_show_Temp(void)
{
    ips200_show_string(190,272,"Temp:");
    ips200_show_float(190,288,Temp,2,2);
//    ips200_show_float(190,302,nowThreshold,2,2);
}
void ips200_show_lost(void)
{

//     ips200_show_uint(0,240,l_lost_tip,2);
//     ips200_show_uint(0,256,r_lost_tip,2);
//     ips200_show_uint(0,272,t_lost_tip,2);
//     ips200_show_uint(0,296,b_lost_tip,2);
     ips200_show_uint(64,240,l_lost_num,2);
     ips200_show_uint(64,256,r_lost_num,2);
     ips200_show_uint(64,272,t_lost_num,2);
     ips200_show_uint(64,296,b_lost_num,2);
//                        ips200_show_uint(0,128,l_center[1],3);
//                        ips200_show_uint(0,144,r_center[1],3);
//                        ips200_show_uint(0,160,t_center[1],3);
//                        ips200_show_uint(0,176,b_center[1],3);
                        ips200_show_uint(0,208,l_center[0],3);
                        ips200_show_uint(0,224,r_center[0],3);
                        ips200_show_uint(0,240,t_center[0],3);
                        ips200_show_uint(0,256,b_center[0],3);
}
void ips200_show(void)
{
    count++;
    if(count%Show_count==0)
    {
//        ips200_show_encoder();
//        ips200_show_FPS();
        ips200_show_flag();
//        ips200_show_PID1();
//        ips200_show_Temp();
//        ips200_show_lost();
        count = 0;
    }
}

