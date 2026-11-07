#ifndef CODE_PID_H_
#define CODE_PID_H_

#define PWM_L_1   (ATOM2_CH1_P11_2)
#define PWM_L_2   (P11_3)
#define PWM_R_1   (ATOM3_CH4_P11_9)
#define PWM_R_2    (P11_6)
#define FUYA   (ATOM3_CH0_P33_4)
extern uint8_t protection_mode ;
//#define FUYA   (ATOM0_CH5_P32_4)
//#define FUYA   (P32_4)

/*
 * 仅使用编码器的堵转保护参数。
 * 判定条件为：目标速度较大、PWM输出较大、输出方向与目标方向一致，
 * 但编码器在目标方向上的速度持续过低。以上条件必须同时满足才会触发。
 */

// 左电机故障位，motor_stall_fault & MOTOR_STALL_LEFT 非零表示左电机堵转。
// 这是状态位掩码，不是可调阈值，请勿修改数值。
#define MOTOR_STALL_LEFT                  (0x01u)

// 右电机故障位，motor_stall_fault & MOTOR_STALL_RIGHT 非零表示右电机堵转。
// 左右轮同时堵转时 motor_stall_fault 的值为 0x03，请勿修改此状态位数值。
#define MOTOR_STALL_RIGHT                 (0x02u)

// 电机控制及堵转检测的实际调用周期，单位 ms；当前 CCU60_CH0 每 2 ms 调用一次。
// 修改定时器周期后必须同步修改此值，否则实际确认时间会与设定值不一致。
#define MOTOR_STALL_CONTROL_PERIOD_MS     (2u)

// 堵转条件需要连续保持的时间，单位 ms。
// 调小：保护更快，但起步、换向或短暂卡顿时更容易误触发；调大：误触发更少，但电机堵转发热时间更长。
#define MOTOR_STALL_CONFIRM_TIME_MS       (16u)

// 根据确认时间和控制周期自动计算需要连续满足条件的次数，当前为 16 / 2 = 8 次。
// 不要直接修改此项，应通过上面两个时间参数调整。
#define MOTOR_STALL_CONFIRM_CYCLES        (MOTOR_STALL_CONFIRM_TIME_MS / MOTOR_STALL_CONTROL_PERIOD_MS)

// 启用堵转检测的最小PID输出，范围与 Pid_speed*.Out 相同，当前最大输出约为 19500。
// 调小：较小驱动力下也会保护、更加敏感；调大：误触发更少，但电机可能在更大电流下堵转更久。
#define MOTOR_STALL_PWM_THRESHOLD         (17000.0f)

// 起步保护解锁速度，单位为编码器计数/控制周期。
// 每次起步或换向后，实测速度达到此值之前不进行堵转判断；达到后保持解锁，直到目标撤销或再次换向。
#define MOTOR_STALL_ARM_SPEED_THRESHOLD   (200)

// 判为“几乎不转”的最大编码器速度，单位为编码器计数/控制周期，不是 RPM。
// 调大：能识别缓慢爬动式堵转，但低速行驶更容易误触发；调小：只识别接近完全不转的硬堵转。
#define MOTOR_STALL_SPEED_THRESHOLD       (100)//150

extern float L_speed,R_speed;
extern uint8_t chujie ;
// Latched bit mask: MOTOR_STALL_LEFT and/or MOTOR_STALL_RIGHT.
extern volatile uint8_t motor_stall_fault;
#define M_PI_2 1.57079632679489661923

//双PD
typedef struct{
    float KP;
    float KP2;
    float KD;
    float GKD;
    float err[3];            //0本次误差，1上次误差，2上上次误差
    float POut;
    float POut_Max;
    float DOut;
    float DOut_Max;
    float Angle_Out;           //转角值
    float AngleOut_Max;
} Two_PD_Type;

typedef struct pid_tag {
    float P;                 //比例系数
    float I;                 //积分系数
    float D;                 //微分系数
    float Pout;              //比例项输出
    float Iout;              //积分项输出
    float Dout;              //微分项输出
    float err[3];            //0本次误差，1上次误差，2上上次误差
    float integral_err;      //积分相累计误差
    float integral_err_max;  //积分相累计误差最大值
    float err_differ;        //微分相误差
    float   Out;               //总输出
    float last_Out;        //上次输出
    float POut_Max;          //
    float Out_max;           //总输出最大值
    float Out_mini;          //总输出最小值
} PID_Type;

typedef struct {
    int16_t encoder_speed;
    int16_t duty;
}motor;
extern Two_PD_Type Two_PD;
extern PID_Type Pid_speedL;
extern PID_Type Pid_speedR;
extern PID_Type Pid_Angel_velocity;
extern motor L_motor;
extern motor R_motor;
extern float base_speed;
// extern  Two_PD_Type Two_PD;
float Limit02(float value, float pos, float low);
void Angle_pid(void);
void Stop_car(void );
void Angle_velocity_pid(void);
void PID_speed(float Target_speed);
void Speed_PidR(int Measure,int Target);
void Speed_PidL(int Measure,int Target);
void Motor_pwm_Angle(void);
// Clear only after the obstruction is removed and the motion command is stopped.
void Motor_stall_reset(void);
void Sum_Init(void);


#endif /* CODE_PID_H_ */
