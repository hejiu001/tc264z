

#include "zf_common_headfile.h"
uint8_t protection_mode = 0;
float L_speed=0,R_speed=0;
Two_PD_Type Two_PD;
PID_Type Pid_speedL;
PID_Type Pid_speedR;
PID_Type Pid_Angel_velocity;
motor L_motor;           //定义左电机结构体
motor R_motor;           //定义右电机结构体324q5w
float base_speed;
volatile uint8_t motor_stall_fault = 0;
static uint16_t motor_stall_count_l = 0;
static uint16_t motor_stall_count_r = 0;
static uint8_t motor_stall_armed_l = 0;
static uint8_t motor_stall_armed_r = 0;
static int8_t motor_stall_direction_l = 0;
static int8_t motor_stall_direction_r = 0;



//参数初始化
void Sum_Init(void)
{

    Pid_speedL.err[0]=0;        //左轮
    Pid_speedL.err[1]=0;
    Pid_speedL.err[2]=0;
    Pid_speedL.P=90.0;     //400//120
    Pid_speedL.I=70.0;     //25//20
    Pid_speedL.D=-0;     //-50
    //
    Pid_speedL.POut_Max = 20000;
    Pid_speedL.Out_max=19500;
    //速度环参数初始化
    Pid_speedR.err[0]=0;        //右轮
    Pid_speedR.err[1]=0;
    Pid_speedR.err[2]=0;
    Pid_speedR.P=90.0;           //350//100
    Pid_speedR.I=70.0;            //14//15
    Pid_speedR.D=-0;           //-80
    //
    Pid_speedR.POut_Max = 20000;
    Pid_speedR.Out_max=19500;


    //位置环参数初始化
    Two_PD.err[0] = 0;
    Two_PD.err[1] = 0;

    Motor_stall_reset();


}






static float Motor_abs_float(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static uint8_t Motor_stall_detect(uint16_t *counter, uint8_t *armed,
                                  int8_t *armed_direction, float target,
                                  int measure, float output)
{
    int8_t target_direction;
    int directed_speed;

    if((target * output) <= 0.0f)
    {
        *counter = 0;
        *armed = 0;
        *armed_direction = 0;
        return 0u;
    }

    target_direction = (target >= 0.0f) ? 1 : -1;
    directed_speed = (target_direction > 0) ? measure : -measure;

    if(*armed_direction != target_direction)
    {
        *counter = 0;
        *armed = 0;
        *armed_direction = target_direction;
    }

    if(*armed == 0u)
    {
        if(directed_speed >= MOTOR_STALL_ARM_SPEED_THRESHOLD)
        {
            *armed = 1u;
        }
        return 0u;
    }

    if((Motor_abs_float(output) >= MOTOR_STALL_PWM_THRESHOLD) &&
       (directed_speed <= MOTOR_STALL_SPEED_THRESHOLD))
    {
        if(*counter < MOTOR_STALL_CONFIRM_CYCLES)
        {
            (*counter)++;
        }
    }
    else
    {
        *counter = 0;
    }

    return (*counter >= MOTOR_STALL_CONFIRM_CYCLES) ? 1u : 0u;
}

static void Motor_output_disable(void)
{
    Pid_speedL.Out = 0.0f;
    Pid_speedR.Out = 0.0f;
    L_motor.duty = 0;
    R_motor.duty = 0;
    pwm_set_duty(PWM_L_1, 0);
    pwm_set_duty(PWM_R_1, 0);
}

void Motor_stall_reset(void)
{
    motor_stall_count_l = 0;
    motor_stall_count_r = 0;
    motor_stall_armed_l = 0;
    motor_stall_armed_r = 0;
    motor_stall_direction_l = 0;
    motor_stall_direction_r = 0;
    motor_stall_fault = 0;

    Pid_speedL.err[0] = 0.0f;
    Pid_speedL.err[1] = 0.0f;
    Pid_speedL.err[2] = 0.0f;
    Pid_speedR.err[0] = 0.0f;
    Pid_speedR.err[1] = 0.0f;
    Pid_speedR.err[2] = 0.0f;
    Pid_speedL.Out = 0.0f;
    Pid_speedR.Out = 0.0f;
    L_motor.duty = 0;
    R_motor.duty = 0;
}


void Angle_pid(void)
{
//    imu660rc_get_quarternion();
    Two_PD.err[0] = Temp;
    Two_PD.POut = Two_PD.KP * Temp +  Two_PD.KP2  * abs((int)(Temp)) * Temp;

    Two_PD.DOut = Two_PD.KD * (Two_PD.err[0]-Two_PD.err[1]) + Two_PD.GKD *((float)(imu660rc_gyro_z) / 16.384);

    Two_PD.Angle_Out = (Two_PD.POut + Two_PD.DOut);
    //限幅
    Two_PD.Angle_Out = Limit02(Two_PD.Angle_Out,Two_PD.AngleOut_Max,-Two_PD.AngleOut_Max);
    Two_PD.err[1] = Two_PD.err[0];
}

float Limit02(float value, float pos, float low)
{
  value = value > pos?pos : value;
  value = value < low?low : value;
  return value;
}





void PID_speed(float Target_speed)
{
    if(Target_speed>=0)
    {
        L_speed = base_speed - Target_speed;
        R_speed = base_speed + Target_speed*0.55;
    }
    else if(Target_speed<0)
    {
        L_speed = base_speed - Target_speed*0.55;
        R_speed = base_speed + Target_speed;
    }
//    L_speed = base_speed - Target_speed;
//    R_speed = base_speed + Target_speed;

    Speed_PidR(R_motor.encoder_speed,R_speed);
    Speed_PidL(L_motor.encoder_speed,L_speed);

//    Speed_PidR(R_motor.encoder_speed,Target_speed);
//    Speed_PidL(L_motor.encoder_speed,-Target_speed);

}

//增量式PID 控制左速度
void Speed_PidL(int Measure, int Target)
{
    Pid_speedL.err[0] = Target - Measure;

    // === 增量式 PID 三项 ===
    Pid_speedL.Pout = Pid_speedL.P * (Pid_speedL.err[0] - Pid_speedL.err[1]);
    Pid_speedL.Iout = Pid_speedL.I * Pid_speedL.err[0];
    Pid_speedL.Dout = Pid_speedL.D * (Pid_speedL.err[0] - 2.0f*Pid_speedL.err[1] + Pid_speedL.err[2]);

    // 根据目标速度直接预估所需 PWM，减少对积分的依赖
    float feedforward = Target * 21.5;  // 前馈系数 增益需根据实际标定
    Pid_speedL.Out = feedforward;       // 用前馈替换纯积分起点
    Pid_speedL.Out += (Pid_speedL.Pout + Pid_speedL.Iout + Pid_speedL.Dout);
    // 注意：前馈项使得 Out 不必从0开始积累，大幅提升初始响应

    Pid_speedL.Out = Limit02(Pid_speedL.Out, Pid_speedL.Out_max, -Pid_speedL.Out_max);
    Pid_speedL.err[2] = Pid_speedL.err[1];
    Pid_speedL.err[1] = Pid_speedL.err[0];
}


void Speed_PidR(int Measure, int Target)
{
    Pid_speedR.err[0] = Target - Measure;

    // === 增量式 PID 三项 ===
    Pid_speedR.Pout = Pid_speedR.P * (Pid_speedR.err[0] - Pid_speedR.err[1]);
    Pid_speedR.Iout = Pid_speedR.I * Pid_speedR.err[0];
    Pid_speedR.Dout = Pid_speedR.D * (Pid_speedR.err[0] - 2.0f*Pid_speedR.err[1] + Pid_speedR.err[2]);

    // 根据目标速度直接预估所需 PWM，减少对积分的依赖
    float feedforward = Target * 21.5;  // 前馈系数 增益需根据实际标定
    Pid_speedR.Out = feedforward;       // 用前馈替换纯积分起点
    Pid_speedR.Out += (Pid_speedR.Pout + Pid_speedR.Iout + Pid_speedR.Dout);
    // 注意：前馈项使得 Out 不必从0开始积累，大幅提升初始响应

    Pid_speedR.Out = Limit02(Pid_speedR.Out, Pid_speedR.Out_max, -Pid_speedR.Out_max);
    Pid_speedR.err[2] = Pid_speedR.err[1];
    Pid_speedR.err[1] = Pid_speedR.err[0];
}
void Motor_pwm_Angle(void)
{
   uint8_t detected_fault = 0;

   if(motor_stall_fault != 0u)
   {
       Motor_output_disable();
       return;
   }

   if(Motor_stall_detect(&motor_stall_count_l, &motor_stall_armed_l,
                         &motor_stall_direction_l, L_speed,
                         L_motor.encoder_speed, Pid_speedL.Out))
   {
       detected_fault |= MOTOR_STALL_LEFT;
   }
   if(Motor_stall_detect(&motor_stall_count_r, &motor_stall_armed_r,
                         &motor_stall_direction_r, R_speed,
                         R_motor.encoder_speed, Pid_speedR.Out))
   {
       detected_fault |= MOTOR_STALL_RIGHT;
   }

   if(detected_fault != 0u)
   {
       motor_stall_fault |= detected_fault;
       Stop_car();
       protection_mode = 1;
       return;
   }

   L_motor.duty = (int)(Pid_speedL.Out);
   R_motor.duty = (int)(Pid_speedR.Out);
//   L_motor.duty = 10000;
//   R_motor.duty = 10000;

   if(L_motor.duty >= 0)
   {
       gpio_set_level(PWM_L_2,0);
       pwm_set_duty(PWM_L_1,L_motor.duty);
   }
   else
   {
    pwm_set_duty(PWM_L_1, -L_motor.duty);
    gpio_set_level(PWM_L_2,1);
   }
      if(R_motor.duty >= 0)
   {
          gpio_set_level(PWM_R_2,0);
    pwm_set_duty(PWM_R_1, R_motor.duty);
   }
   else
   {
       pwm_set_duty(PWM_R_1,-R_motor.duty);
       gpio_set_level(PWM_R_2,1);
   }
}

void Stop_car(void )
{
    base_speed=0;
    Flag=0;
    chujie=1;
    L_speed = 0;
    R_speed = 0;
    Two_PD.Angle_Out = 0;
    pwm_set_duty(FUYA,0);
    Motor_output_disable();

    }
