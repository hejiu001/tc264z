/*
 * flash.c
 *
 *  Created on: 2026年1月9日
 *      Author: 21184
 */

#include "flash.h"

#define FLASH_SECTION_INDEX  (0)
#define PID_Page_INDEX  (8)
uint8_t STOP_CAR = 0;

uint8 wifi_spi_get_data_buffer[256];
uint32 data_length;

void flash_action(void)
{
    flash_buffer_clear();

    flash_union_buffer[0].float_type = Two_PD.KP;
    flash_union_buffer[1].float_type = Two_PD.KP2;
    flash_union_buffer[2].float_type = Two_PD.KD;
    flash_union_buffer[3].float_type = Two_PD.GKD;
    flash_union_buffer[4].float_type = Pid_speedL.P;
    flash_union_buffer[5].float_type = Pid_speedL.I;

    if(flash_check(FLASH_SECTION_INDEX, PID_Page_INDEX))
    flash_erase_page(FLASH_SECTION_INDEX, PID_Page_INDEX);

    flash_write_page_from_buffer(FLASH_SECTION_INDEX, PID_Page_INDEX);
}

void flash_get_value(void)
{
    if(flash_check(FLASH_SECTION_INDEX, PID_Page_INDEX))
    {
        flash_read_page_to_buffer(FLASH_SECTION_INDEX, PID_Page_INDEX);
        Two_PD.KP = flash_union_buffer[0].float_type;
        Two_PD.KP2 = flash_union_buffer[1].float_type;
        Two_PD.KD = flash_union_buffer[2].float_type;
        Two_PD.GKD = flash_union_buffer[3].float_type;
        Pid_speedL.P = flash_union_buffer[4].float_type;
        Pid_speedL.I = flash_union_buffer[5].float_type;
        Pid_speedR.P = flash_union_buffer[4].float_type;
        Pid_speedR.I = flash_union_buffer[5].float_type;
    }
}

//SD:mp=60 60 mi=1.30 1.30 AP=15 AD=0.18 ap=0.17
void wifi_Receive(void)
{
    data_length = wifi_spi_read_buffer(wifi_spi_get_data_buffer, sizeof(wifi_spi_get_data_buffer));
            if(data_length)                                                     // 如果接收到数据 则进行数据类型判断
            {
                if(wifi_spi_get_data_buffer[0] == 'S'&& wifi_spi_get_data_buffer[1] == 'D')
                {
                    sscanf((char*)wifi_spi_get_data_buffer,"SD:mp=%f %f mi=%f %f AP=%f AD=%f ap=%f",&Pid_speedL.P,&Pid_speedR.P,&Pid_speedL.I,&Pid_speedR.I,&Two_PD.KP,&Two_PD.GKD,&Two_PD.KP2);
                }
                if(wifi_spi_get_data_buffer[0] == 'S' && wifi_spi_get_data_buffer[1] == 'T'
                && wifi_spi_get_data_buffer[2] == 'O' && wifi_spi_get_data_buffer[3] == 'P')
                {
                    STOP_CAR = 1;
                }
                if(!wifi_spi_send_buffer(wifi_spi_get_data_buffer, data_length))
                {
                    memset(wifi_spi_get_data_buffer, 0, data_length);           // 数据发送完成 清空数据
                }
            }
}
