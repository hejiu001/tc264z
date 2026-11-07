/*********************************************************************************************************************
* TC264 Opensourec Library 即（TC264 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 TC264 开源库的一部分
*
* TC264 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          cpu0_main
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          ADS v1.10.2
* 适用平台          TC264D
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2022-09-15       pudding            first version
********************************************************************************************************************/
#include "zf_common_headfile.h"
#pragma section all "cpu0_dsram"

// 将本语句与#pragma section all restore语句之间的全局变量都放在CPU0的RAM中

// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设

// **************************** 代码区域 ***************************
                                      // 双排排针 并口两寸屏 这里宏定义填写 IPS200_TYPE_PARALLEL8
int32 m;
int Key_num0,Key_num1,Key_num2,Key_num3;
uint8 image_copy[image_h][image_w];
uint8_t visited[image_h][image_w];
uint16_t count_time = 0;
extern uint8_t chujie ;
extern uint32_t time ;
uint16_t max_speed = 0;
uint8_t shibo = 1;
int core0_main(void)
{
    clock_init();                   // 获取时钟频率<务必保留>
    debug_init();                   // 初始化默认调试串口
    Init();
    cpu_wait_event_ready();         // 等待所有核心初始化完毕
    base_speed = 0;
    while (TRUE)
    {
        if(WIFI==1)
        {
            wifi_Receive();
        }
        if(WIFI==1&&mt9v03x_finish_flag==0)
        {
        memcpy(image_copy[0], Pixle[0], MT9V03X_IMAGE_SIZE/4);
//        memcpy(image_copy[0], Image_Use[0], MT9V03X_IMAGE_SIZE/4);
        seekfree_assistant_camera_send();
        }
        if(WIFI==1&&shibo == 1&&mt9v03x_finish_flag==0)
        {
                       seekfree_assistant_oscilloscope_data.data[0] = L_motor.encoder_speed;
                       seekfree_assistant_oscilloscope_data.data[1] = R_motor.encoder_speed;
                       seekfree_assistant_oscilloscope_data.data[2] = Pid_speedR.Out;
                       seekfree_assistant_oscilloscope_data.data[3] = Pid_speedL.Out;
                       seekfree_assistant_oscilloscope_data.data[4] = protection_mode;
                       seekfree_assistant_oscilloscope_data.data[5] = Pid_speedR.Out;
                       seekfree_assistant_oscilloscope_data.data[6] = Pid_speedL.Out;
                       seekfree_assistant_oscilloscope_data.data[7] = Turn1_Flag;
                       seekfree_assistant_oscilloscope_data.channel_num = 8;
                        seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data);


//                        Motor_pwm_Angle();
        }
//        Motor_pwm_Angle()
//       if(STOP_CAR)
//       {
//           Stop_car();
//       }

    }
}
void  pit_handler0_10ms(void)
{
    if(start_flag==1)
    {
        count_time++;
        if(count_time>=800)
        {
        base_speed=250;
        max_speed = 420;
        start_flag=0;
        Flag = 1;
        count_time=0;
        }
    }
    Get_dist();
    Get_encoder();
    if(Flag == 1)
    {
        if(base_speed<max_speed)
        {
            base_speed+=1;
        }
        else if(base_speed>max_speed)
        {
            base_speed-=1;
        }
        PID_speed(Two_PD.Angle_Out);
        Motor_pwm_Angle();
    }
    else
    {
        pwm_set_duty(PWM_L_1,0);
        pwm_set_duty(PWM_R_1,0);
    }

}

void  pit_handler0_5ms(void)
{
    calculatePose_Module(&pose, 0.005f);
         Key_Use();
          Two_PD.KP = 7.0;
          Two_PD.KP2 = 0.25;//1
          Two_PD.KD = 0.0;
          Two_PD.GKD = 0.18;///0.8
          Two_PD.AngleOut_Max = 180;
         if(chujie==0)
         {
          Angle_pid();
         }
}






#pragma section all restore
// **************************** 代码区域 ****************************
