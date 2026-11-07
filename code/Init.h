
#ifndef CODE_INIT_H_
#define CODE_INIT_H_
#define IPS200_TYPE     (IPS200_TYPE_SPI)
#define WIFI_SSID_TEST          "88888888"
#define WIFI_PASSWORD_TEST      "12345678"                  // 如果需要连接的WIFI 没有密码则需要将 这里 替换为 NULL
#define TCP_TARGET_IP           "192.168.137.1"             // 连接目标的 IP
#define TCP_TARGET_PORT         "8086"                      // 连接目标的端口
#define WIFI_LOCAL_PORT         "6666"                      // 本机的端口 0：随机  可设置范围2048-65535  默认 6666
extern uint8 image_copy[image_h][image_w];
extern uint8 WIFI;
void Init (void);
#define LED1   (P33_9)
#define LED2   (P33_8)
#define LED3   (P33_7)
#define LED4   (P33_6)


#endif /* CODE_INIT_H_ */
