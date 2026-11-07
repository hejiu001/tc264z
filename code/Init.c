
#include "zf_common_headfile.h"
uint8_t WIFI = 1;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            ;
void Init (void)
{
    key_init(2);
if (WIFI == 1)
{
    wifi_spi_init(WIFI_SSID_TEST, WIFI_PASSWORD_TEST);
    wifi_spi_socket_connect(                                          // 向指定目标 IP 的端口建立 TCP 连接
       "TCP",                                                              // 指定使用TCP方式通讯
       TCP_TARGET_IP,                                                      // 指定远端的IP地址，填写上位机的IP地址
       TCP_TARGET_PORT,                                                    // 指定远端的端口号，填写上位机的端口号，通常上位机默认是8080
       WIFI_LOCAL_PORT);                                                   // 指定本机的端口号
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIFI_SPI);
    seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, image_copy[0], MT9V03X_W, MT9V03X_H);
}
    mt9v03x_init();
    Sum_Init();
//    ips200_set_dir(IPS200_PORTAIT_180);
    ips200_set_dir(IPS200_PORTAIT);
    ips200_init(IPS200_TYPE);
    encoder_dir_init(ENCODER_L_TIM, ENCODER_L_PLUS, ENCODER_L_DIR);          // 初始化编码器模块与引脚 带方向增量编码器模式
    encoder_dir_init(ENCODER_R_TIM, ENCODER_R_PLUS, ENCODER_R_DIR);          // 初始化编码器模块与引脚 带方向增量编码器模式
    imu660rc_init(IMU660RC_QUARTERNION_120HZ);
    initPose_Module(&pose);
       pwm_init(FUYA, 50, 0);
//       gpio_init(FUYA, GPO, GPIO_LOW, GPO_PUSH_PULL);          // 初始化 LED1 输出 默认高电平 推挽输出模式
       pwm_init(PWM_L_1, 17000, 0);
       pwm_init(PWM_R_1, 17000, 0);
       gpio_init(PWM_L_2, GPO, GPIO_LOW, GPO_PUSH_PULL);          // 初始化 LED1 输出 默认高电平 推挽输出模式
       gpio_init(PWM_R_2, GPO, GPIO_LOW, GPO_PUSH_PULL);         // 初始化 LED2 输出 默认高电平 推挽输出模式
       gpio_init(P33_10, GPO, GPIO_LOW, GPO_PUSH_PULL);         // 初始化 LED2 输出 默认高电平 推挽输出模式

       gpio_init(LED1, GPO, 1, GPO_PUSH_PULL);          // 初始化 LED1 输出 默认高电平 推挽输出模式
       gpio_init(LED2, GPO, 1, GPO_PUSH_PULL);         // 初始化 LED2 输出 默认高电平 推挽输出模式
       gpio_init(LED3, GPO, 1, GPO_PUSH_PULL);         // 初始化 LED2 输出 默认高电平 推挽输出模式
       gpio_init(LED4, GPO, 1, GPO_PUSH_PULL);         // 初始化 LED2 输出 默认高电平 推挽输出模式
    pit_ms_init(CCU60_CH0,2);
    pit_ms_init(CCU60_CH1, 5);
    pit_ms_init(CCU61_CH1, 3);
//     menu_init();
}


