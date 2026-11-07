
#ifndef CODE_POSE_H_
#define CODE_POSE_H_
typedef struct Pose_Data
{
    float yaw;
    float rol;
    float pit;
    float rot_mat[3][3];            //旋转矩阵
}Pose_Data;

typedef struct Pose_Process
{
    float qua[4];                   //四元数
}Pose_Process;

typedef struct Pose_Parameter
{
    float correct_kp;
}Pose_Parameter;

typedef struct Pose_Module
{
    Pose_Process pro;
    Pose_Data data;
    Pose_Parameter parameter;
}Pose_Module;


extern Pose_Module pose;
extern int16_t DIST;
extern int16_t dist_L ;
extern int16_t dist_R ;


void calculatePose_Module(Pose_Module *pose, float cycle);
float arcsin(float i);
float arctan2(float x, float y);
float arctan1(float tan);
void initPose_Module(Pose_Module *pose);
void Get_encoder(void);
void Get_dist(void);

#define ENCODER_R_TIM                 (TIM2_ENCODER)
#define ENCODER_R_PLUS                (TIM2_ENCODER_CH1_P33_7)
#define ENCODER_R_DIR                 (TIM2_ENCODER_CH2_P33_6)

#define ENCODER_L_TIM                (TIM3_ENCODER)
#define ENCODER_L_PLUS               (TIM3_ENCODER_CH1_P02_6)
#define ENCODER_L_DIR                (TIM3_ENCODER_CH2_P02_7)



#endif /* CODE_POSE_H_ */
