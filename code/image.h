#ifndef __IMAGE_H__
#define __IMAGE_H__
#include "zf_common_headfile.h"
#define LCDH MT9V03X_H//ips200                              //用于图像处理的图像高度（行）
#define LCDW MT9V03X_W
#define image_h MT9V03X_H/2                             //用于图像处理的图像高度（行）
#define image_w MT9V03X_W/2
#define Judge_line 240
#define Black 0  //黑色
#define White 255  //白色
#define Lost_line 140
#define Make_up 150
#define Left_line 80
#define Right_line 80
#define Control_line 170
#define USE_num (image_w-1)*4
//Get_BT_bezier_line
extern uint8_t l2_white_tip;
extern uint8_t r2_white_tip;
extern uint8 *Image_Use_linear;
extern uint8_t TL_FSM ;
extern uint8_t TR_FSM ;
extern float Temp;
extern uint8 *Image_Use[image_h];
extern uint8 Pixle[image_h][image_w];
extern uint8 l_lost_tip ;
extern uint8 r_lost_tip ;
extern uint8 t_lost_tip ;
extern uint8 b_lost_tip ;
extern uint8 b_lost_num ;
extern uint8 t_lost_num ;
extern uint8 l_lost_num ;
extern uint8 r_lost_num ;
extern uint16 b_center[3] ;
extern uint16 l_center[3] ;
extern uint16 r_center[3] ;
extern uint16 t_center[3] ;
extern uint8 hightest ;
extern uint16 white_row_max[2],white_row_min[2];
extern uint8 white_col_max[2],white_col_min[2];
extern uint16 white_num_row[image_h+5],white_num_col[image_w+5];
extern int leftmost[2] ;//最左点记录
extern int rightmost[2] ;//最右点记录
extern uint8_t Flag_Count;
extern uint8_t Flag_Count2;
extern uint8_t nowThreshold;
extern uint8_t T1_flag ;
extern float target_speed;
extern uint8_t t_white_tip ;
extern uint8_t l_white_tip ;
extern uint8_t r_white_tip ;
extern volatile uint8_t protect ;

enum ImgFlag
{
    straightline,   //直道     0
    curve,          //折角     1
    Capacitor,      //电容     2
    T_junction2,    //t字路口第二种   3
    T_junction1_L,  //4
    T_junction1_R,  //5
    T_junction2_L,  //6
    T_junction2_R,  //7
    T_junction1_T,  //8
    crossroads_L,   //9
    crossroads_R,   //10
    crossroads_T    //11
};//图像标志位
extern enum ImgFlag IF ;

void Get_BinaryImag1(void);
uint8 search_l_r(uint8 start_l_x,uint8 start_l_y,uint8 start_r_x,uint8 start_r_y );
uint8 get_start_point(void);
uint8_t Deal_Capacitor(void);
uint8_t otsuThreshold_med(uint16_t col, uint16_t row, uint8_t Yasuo_[image_h][image_w], uint16_t width_offset);
uint8 Get_top_straightline(void);
void Get_lost_tip(uint8 length);
uint8_t Get_Lost_Inward_NonBlack_Width(void);
void Get_imgOSTU(void);
void deviationestimate(void);
uint8 Deal_crossroads(uint8 Cross_Turn);//0为直 1为左拐 2为右拐
uint8_t Deal_Left_Angle(void);
uint8_t Deal_Right_Angle(void);
uint8 Dashedline_Makeup(void);
void Get_start_center(void);
uint8_t Deal_T_junction1(uint8 Turn);
uint8_t Deal_T_junction2(uint8 Turn);
//Get_Lost_Inward_NonBlack_Width
uint8_t Otsu_TrackThreshold(uint8 *image, uint16_t width, uint16_t height);
uint8_t Complete_Lines(void);
void Get_oblique_Y_line_L(void);
void Get_oblique_Y_line_R(void);
void Get_oblique_B_line(void);
uint8_t Count_LT_line_white(void);
void Get_BT_bezier_line(void);
float fangcha_speed(float min_speed, float max_speed);
void Get_Top_bezier_line(uint8 x_top);
void Get_BinaryImage2(void);
void deviationestimate(void);
void Get_BinaryImage2(void);
uint8_t LT_black_ratio(void);
void Lost_find(void);
void Get_B_straight_line(uint8 x_end, uint8 y_end);
uint8_t RT_black_ratio(void);
float BT_K(void);
uint8_t BT_black_ratio(void);
uint8_t BT_black_ratio2(uint8_t destination);
uint8_t Limit_protection(uint8_t start_point_valid);
void Line_Completion(void);
void get_whitetip();
int my_abs(int value);
void Flag_Init(void);
void TIPS(void);
extern enum ImgFlag Best_Flag;
extern uint8 start_point_l ;
extern uint8 start_point_r ;
extern uint8 start_center_y;
#endif
