
#include "zf_common_headfile.h"

int16_t DIST = 0;
int16_t dist_L = 0;
int16_t dist_R = 0;
Pose_Module pose;
float arctan1(float tan)
{
    float angle = (func_abs(tan) > 1.0f) ?  90.0f - func_abs(1.0f / tan) * (45.0f - (func_abs(1.0f / tan) - 1.0f) * (14.0f + 3.83f * func_abs(1.0f / tan))):
                                func_abs(tan) * (45.0f - (func_abs(tan) - 1.0f) * (14.0f + 3.83f * func_abs(tan)));
    return (tan > 0) ? angle : -angle;
}

float arctan2(float x, float y)
{
    float tan, angle;

    if (x == 0 && y == 0) return 0;

    if (x == 0)
    {
        if (y > 0) return 90;
        else return -90;
    }

    if (y == 0)
    {
        if (x > 0) return 0;
        else return -180.0f;
    }
    tan = y / x;
    angle = arctan1(tan);
    if (x < 0 && angle > 0) angle -= 180.0f;
    else if (x < 0 && angle < 0) angle += 180.0f;
    return angle;
}

float arcsin(float i)
{
    return arctan1(i / sqrt(1 - i * i));
}



void calculatePose_Module(Pose_Module *pose, float cycle)
{
    float length;
    float x, y, z;
    imu660rc_get_quarternion();
    //修正陀螺仪值
    x = (float)(imu660rc_gyro_x / 10 * 10 / 16) * 0.01745329f;
    y = (float)(imu660rc_gyro_y / 10 * 10 / 16) * 0.01745329f;
    z = (float)(imu660rc_gyro_z / 10 * 10 / 16) * 0.01745329f;

    // 一阶龙格库塔更新四元数值
    pose->pro.qua[0] += ( -pose->pro.qua[1] * x - pose->pro.qua[2] * y - pose->pro.qua[3] * z) * cycle / 2.0f;
    pose->pro.qua[1] += ( pose->pro.qua[0] * x + pose->pro.qua[2] * z - pose->pro.qua[3] * y) * cycle / 2.0f;
    pose->pro.qua[2] += ( pose->pro.qua[0] * y - pose->pro.qua[1] * z + pose->pro.qua[3] * x) * cycle / 2.0f;
    pose->pro.qua[3] += ( pose->pro.qua[0] * z + pose->pro.qua[1] * y - pose->pro.qua[2] * x) * cycle / 2.0f;

    //四元数归一化
    length = sqrt(pose->pro.qua[0] * pose->pro.qua[0] + pose->pro.qua[1] * pose->pro.qua[1] + pose->pro.qua[2] * pose->pro.qua[2] + pose->pro.qua[3] * pose->pro.qua[3]);
    if (length != 0)
    {
        pose->pro.qua[0] /= length;
        pose->pro.qua[1] /= length;
        pose->pro.qua[2] /= length;
        pose->pro.qua[3] /= length;
    }
    //计算旋转矩阵
    pose->data.rot_mat[0][0] =      pose->pro.qua[0] * pose->pro.qua[0] + pose->pro.qua[1] * pose->pro.qua[1] - pose->pro.qua[2] * pose->pro.qua[2] - pose->pro.qua[3] * pose->pro.qua[3];
    pose->data.rot_mat[0][1] = 2 * (pose->pro.qua[1] * pose->pro.qua[2] + pose->pro.qua[0] * pose->pro.qua[3]);
    pose->data.rot_mat[0][2] = 2 * (pose->pro.qua[1] * pose->pro.qua[3] - pose->pro.qua[0] * pose->pro.qua[2]);
    pose->data.rot_mat[1][0] = 2 * (pose->pro.qua[1] * pose->pro.qua[2] - pose->pro.qua[0] * pose->pro.qua[3]);
    pose->data.rot_mat[1][1] =      pose->pro.qua[0] * pose->pro.qua[0] - pose->pro.qua[1] * pose->pro.qua[1] + pose->pro.qua[2] * pose->pro.qua[2] - pose->pro.qua[3] * pose->pro.qua[3];
    pose->data.rot_mat[1][2] = 2 * (pose->pro.qua[2] * pose->pro.qua[3] + pose->pro.qua[0] * pose->pro.qua[1]);
    pose->data.rot_mat[2][0] = 2 * (pose->pro.qua[1] * pose->pro.qua[3] + pose->pro.qua[0] * pose->pro.qua[2]);
    pose->data.rot_mat[2][1] = 2 * (pose->pro.qua[2] * pose->pro.qua[3] - pose->pro.qua[0] * pose->pro.qua[1]);
    pose->data.rot_mat[2][2] =      pose->pro.qua[0] * pose->pro.qua[0] - pose->pro.qua[1] * pose->pro.qua[1] - pose->pro.qua[2] * pose->pro.qua[2] + pose->pro.qua[3] * pose->pro.qua[3];
    //求解欧拉角
    pose->data.rol = arctan2(pose->data.rot_mat[2][2], pose->data.rot_mat[1][2]);
    pose->data.pit = -arcsin(pose->data.rot_mat[0][2]);
    pose->data.yaw = arctan2(pose->data.rot_mat[0][0], pose->data.rot_mat[0][1]);
}

void initPose_Module(Pose_Module *pose)
{
    pose->pro.qua[0] = 1;
    pose->pro.qua[1] = 0;
    pose->pro.qua[2] = 0;
    pose->pro.qua[3] = 0;
    pose->data.yaw = 0;
    pose->data.rol = 0;
    pose->data.pit = 0;
}


void Get_encoder(void)
{
  L_motor.encoder_speed = -encoder_get_count(ENCODER_L_TIM ); //左电机编码器读取
  R_motor.encoder_speed = +encoder_get_count(ENCODER_R_TIM); //右电机编码器读取

  encoder_clear_count(ENCODER_L_TIM);
  encoder_clear_count(ENCODER_R_TIM);
}

void Get_dist(void)//注意 此代码必须放在Get_encoder(void)之前！！！
{
    dist_L += -encoder_get_count(ENCODER_L_TIM );
    dist_R += encoder_get_count(ENCODER_R_TIM);
    DIST = (dist_L+dist_R)/2;
}







