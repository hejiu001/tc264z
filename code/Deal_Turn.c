#include "zf_common_headfile.h"

/**
 * @file Deal_Turn.c
 * @brief 赛道元素处理与岔路决策模块
 *
 * 本模块负责智能车在赛道上的元素识别滤波、岔路通过检测以及岔路方向决策。
 * 主要包含三个核心函数：
 *   - Select_Flag():  对识别到的赛道元素进行滑动窗口投票滤波
 *   - Flag_Monitor(): 监测车辆是否通过岔路元素（T字路口、十字路口）
 *   - Turn_Control(): 根据已通过的岔路数量，按预设路线决定下一次岔路的方向
 *
 * 适用平台: TC264 (Infineon AURIX TriCore)
 * 编码格式: GBK
 */

////////// 调参区 //////////
int16_t T1_PASS = 19600;         // T字路口1距离预设值（当前未参与判定）
int16_t Pass_DIST = 10000;       // 直行通过岔路所需的最小距离阈值（编码器计数值）
int16_t Pass_DIST_MAX = 19600;   // 直行通过的兜底距离阈值（达到后不再要求双边线恢复）
float Pass_Angle = 47.0f;       // 转弯通过岔路时判定的偏航角阈值（单位：度）
uint8_t road = 2;               // 赛道选择: 0-预赛正发车, 1-决赛正发车, 2/3-备用路线
////////////////////////////

extern uint8_t chujie;          // 出界标志位（外部变量）

uint8_t Pass_Flag = 0;          // 岔路通过完成标志: 0-未通过, 1-已通过

/* 赛道元素识别滤波相关变量 */
uint8_t Element_Flag[5] = {0};      // 元素标志滑动窗口（保留最近5帧）
uint8_t Element_FlagCount[5] = {0}; // 各元素标志的投票计数值
uint8_t Current_Flag = straightline, Last_Flag = straightline;   // 当前/上一次元素标志
uint8_t Current_Flag2 = straightline, Last_Flag2 = straightline; // 备用标志（暂未使用）

/* 各类型岔路通过计数器 */
uint8_t Flag_Count = 0;     // T字路口1（T_junction1）通过次数
uint8_t Flag_Count2 = 0;    // T字路口2（T_junction2）通过次数
uint8_t Flag_Count3 = 0;    // 十字路口（crossroads）通过次数

/* 岔路方向决策变量 */
enum T_Flag  T_Turn1 = NO;      // T字路口1方向: YES-需转弯, NO-直行通过
enum T_Turn  T_Turn2 = T2_Left; // T字路口2方向: T2_Left-左转, T2_Right-右转
enum T_Flag2 T_Turn3 = C_Top;   // 十字路口方向: C_Left-左转, C_Right-右转, C_Top-直行

extern uint16_t max_speed;      // 最大速度（外部变量）

/* 岔路处理状态标志 */
uint8_t Turn1_Flag = 0;     // T字路口1状态: 0-空闲, 1-左转入弯, 2-右转入弯
uint8_t Turn1_Flag2 = 0;    // T字路口1直行状态: 0-空闲, 1-正在直行通过
uint8_t Turn2_Flag = 0;     // T字路口2状态: 0-空闲, 1-左转入弯, 2-右转入弯
uint8_t Turn3_Flag = 0;     // 十字路口状态: 0-空闲, 1-左转, 2-右转, 3-正在直行通过
//uint8_t DIST_read = 0;

/**
 * @brief 赛道元素滑动窗口投票滤波
 *
 * 将当前帧识别到的元素标志 IF 压入长度为5的滑动窗口，
 * 对最近3帧的元素标志进行投票统计，选出出现次数最多的作为 Best_Flag。
 * 这种多数投票机制可以有效滤除单帧误识别带来的干扰。
 */
void Select_Flag()
{
    // 清零投票计数器（仅使用前3个位置）
    for(uint8_t i = 0; i < 3; i++)
        Element_FlagCount[i] = 0;

    uint8_t FlagCount_Max = 0;  // 最高票数

    // 滑动窗口右移：将历史数据后移一位，为新数据腾出位置
    Element_Flag[4] = Element_Flag[3];
    Element_Flag[3] = Element_Flag[2];
    Element_Flag[2] = Element_Flag[1];
    Element_Flag[1] = Element_Flag[0];
    Element_Flag[0] = IF;       // 存入当前帧的元素标志

    // 对最近3帧进行投票统计（每个元素与窗口内前3个元素比较）
    for(uint8_t i = 0; i < 3; i++)
    {
        for(uint8_t j = 0; j < 3; j++)
        {
            if(Element_Flag[i] == Element_Flag[j])
            {
                Element_FlagCount[i]++; // 出现相同元素则计数加1
            }
        }
    }

    // 选出票数最多的元素作为最优标志
    for(uint8_t i = 0; i < 3; i++)
    {
        if(Element_FlagCount[i] > FlagCount_Max)
        {
            FlagCount_Max = Element_FlagCount[i];
            Best_Flag = Element_Flag[i];
        }
    }
}
//void Select_Flag()
//{
//        Best_Flag = IF;   // 备选方案：直接使用当前帧标志（不使用滤波）
//}


/**
 * @brief 监测T字路口1的进入与通过完成状态
 *
 * 执行次序保持为“识别入口 -> 判断通过完成 -> 判断直行距离达标”。
 * 该次序与原实现一致，不能交换，否则会改变同一帧内的状态转换结果。
 */
static void Flag_Monitor_T_Junction1(void)
{
    //////////////////////////////////////////////////////////////
    //                 T字路口1 (T_junction1) 处理
    //////////////////////////////////////////////////////////////
    // 情况1：检测到T字路口1-左转，且当前无其他岔路在处理，且配置为需要转弯
    if(Best_Flag == T_junction1_L && Turn1_Flag == 0 && Turn2_Flag == 0 && Turn3_Flag == 0 && Turn1_Flag2 == 0)
    {
        if(T_Turn1 == YES)
        {
            initPose_Module(&pose); // 重置姿态积分，开始记录转弯角度
            DIST = 0;               // 清零行驶距离
            dist_L = 0;             // 清零左轮距离
            dist_R = 0;             // 清零右轮距离
            Turn1_Flag = 1;         // 标记：T字路口1-左转入弯中
        }
    }
    // 情况2：检测到T字路口1-右转，且当前无其他岔路在处理，且配置为需要转弯
    else if(Best_Flag == T_junction1_R && Turn1_Flag == 0 && Turn2_Flag == 0 && Turn3_Flag == 0 && Turn1_Flag2 == 0)
    {
        if(T_Turn1 == YES)
        {
            initPose_Module(&pose); // 重置姿态积分
            DIST = 0;               // 清零行驶距离
            dist_L = 0;             // 清零左轮距离
            dist_R = 0;             // 清零右轮距离
            Turn1_Flag = 2;         // 标记：T字路口1-右转入弯中
        }
    }
    // 情况3：检测到T字路口1-直行标志，且当前无其他岔路在处理，且配置为不需要转弯
    else if(Best_Flag == T_junction1_T && Turn1_Flag == 0 && Turn2_Flag == 0 && Turn3_Flag == 0 && Turn1_Flag2 == 0)
    {
        if(T_Turn1 == NO)
        {
            Turn1_Flag2 = 1;        // 标记：T字路口1-直行通过中
            Pass_Flag = 0;          // 重置通过标志
            DIST = 0;               // 清零行驶距离
            dist_L = 0;             // 清零左轮距离
            dist_R = 0;             // 清零右轮距离
            initPose_Module(&pose); // 重置姿态积分
        }
    }

    // ----- T字路口1 转弯完成判定 -----
    // 偏航角绝对值超过阈值，且正在入弯中，且配置为需要转弯
    if(((pose.data.yaw > Pass_Angle || pose.data.yaw < -Pass_Angle)) && (Turn1_Flag == 1 || Turn1_Flag == 2) && T_Turn1 == YES)
    {
        Flag_Count++;               // T字路口1通过计数+1
        initPose_Module(&pose);     // 重置姿态积分，为下一个岔路做准备
        DIST = 0;               // 清零行驶距离
        dist_L = 0;             // 清零左轮距离
        dist_R = 0;             // 清零右轮距离
        Turn1_Flag = 0;             // 清除入弯标记
    }
    // ----- T字路口1 直行通过完成判定 -----
    // 已恢复到直道或弯道，且正在直行通过中，且已行驶足够距离
    else if((Best_Flag == straightline || Best_Flag == curve)
            && Turn1_Flag2 == 1
            && Pass_Flag == 1)
    {
        Flag_Count++;               // T字路口1通过计数+1
        DIST = 0;                   // 清零行驶距离
        dist_L = 0;
        dist_R = 0;
        Pass_Flag = 0;              // 重置通过标志
        initPose_Module(&pose);     // 重置姿态积分
        Turn1_Flag2 = 0;            // 清除直行标记
    }

    // ----- T字路口1 直行距离判定 -----
    // 两侧线均未丢失，且行驶距离超过阈值，且正在直行通过中
    if((((l_lost_num + r_lost_num) == 0 && (DIST >= Pass_DIST))||(DIST >= Pass_DIST_MAX)) && Turn1_Flag2 == 1)
    {
        Pass_Flag = 1;              // 标记：直行距离足够，允许判定通过
    }

}

/**
 * @brief 监测T字路口2的进入与转弯完成状态
 *
 * 本函数保留原实现的入口识别和完成判定顺序。即使同一帧内前一类
 * 路口刚清除了状态，本段仍会立即读取更新后的全局状态。
 */
static void Flag_Monitor_T_Junction2(void)
{

    //////////////////////////////////////////////////////////////
    //                 T字路口2 (T_junction2) 处理
    //////////////////////////////////////////////////////////////
    // 检测到T字路口2-左转，且当前无其他岔路在处理
    if(Best_Flag == T_junction2_L && Turn2_Flag == 0 && Turn1_Flag == 0 && Turn3_Flag == 0 && Turn1_Flag2 == 0)
    {
        initPose_Module(&pose);     // 重置姿态积分
        Turn2_Flag = 1;             // 标记：T字路口2-左转入弯中
    }
    // 检测到T字路口2-右转，且当前无其他岔路在处理
    else if(Best_Flag == T_junction2_R && Turn2_Flag == 0 && Turn1_Flag == 0 && Turn3_Flag == 0 && Turn1_Flag2 == 0)
    {
        initPose_Module(&pose);     // 重置姿态积分
        Turn2_Flag = 2;             // 标记：T字路口2-右转入弯中
    }

    // ----- T字路口2 转弯完成判定 -----
    // 偏航角绝对值超过阈值，且正在入弯中
    if((pose.data.yaw > Pass_Angle || pose.data.yaw < -Pass_Angle) && Turn2_Flag != 0)
    {
        Flag_Count2++;              // T字路口2通过计数+1
        initPose_Module(&pose);     // 重置姿态积分
        Turn2_Flag = 0;             // 清除入弯标记
    }

}

/**
 * @brief 监测十字路口的进入与通过完成状态
 *
 * 左转、右转和直行共用Turn3_Flag状态；判断条件、变量清零顺序及
 * 转弯/直行完成判定的先后关系均保持原样。
 */
static void Flag_Monitor_Crossroads(void)
{

    //////////////////////////////////////////////////////////////
    //                 十字路口 (crossroads) 处理
    //////////////////////////////////////////////////////////////
    // 情况1：检测到十字路口-左转标志，且当前无其他岔路在处理
    if(Best_Flag == crossroads_L && Turn2_Flag == 0 && Turn1_Flag == 0 && Turn3_Flag == 0 && Turn1_Flag2 == 0)
    {
        initPose_Module(&pose);     // 重置姿态积分
        Turn3_Flag = 1;             // 标记：十字路口-左转入弯中
    }
    // 情况2：检测到十字路口-右转标志，且当前无其他岔路在处理
    else if(Best_Flag == crossroads_R && Turn2_Flag == 0 && Turn1_Flag == 0 && Turn3_Flag == 0 && Turn1_Flag2 == 0)
    {
        initPose_Module(&pose);     // 重置姿态积分
        Turn3_Flag = 2;             // 标记：十字路口-右转入弯中
    }
    // 情况3：检测到十字路口-直行标志，且当前无其他岔路在处理
    else if(Best_Flag == crossroads_T && Turn2_Flag == 0 && Turn1_Flag == 0 && Turn3_Flag == 0 && Turn1_Flag2 == 0)
    {
        initPose_Module(&pose);     // 重置姿态积分
        DIST = 0;                   // 清零行驶距离
        dist_L = 0;
        dist_R = 0;
        Pass_Flag = 0;              // 重置通过标志
        Turn3_Flag = 3;             // 标记：十字路口-直行通过中
    }

    // ----- 十字路口 直行距离判定 -----
    if((l_lost_num + r_lost_num) == 0 && DIST >= Pass_DIST && Turn3_Flag == 3)
    {
        Pass_Flag = 1;              // 标记：直行距离足够
    }

    // ----- 十字路口 转弯完成判定 -----
    if((pose.data.yaw > Pass_Angle || pose.data.yaw < -Pass_Angle) && (Turn3_Flag == 1 || Turn3_Flag == 2))
    {
        Flag_Count3++;              // 十字路口通过计数+1
        initPose_Module(&pose);     // 重置姿态积分
        Turn3_Flag = 0;             // 清除入弯标记
    }
    // ----- 十字路口 直行通过完成判定 -----
    else if((Best_Flag == straightline || Best_Flag == curve)
            && Turn3_Flag == 3
            && Pass_Flag == 1)
    {
        Flag_Count3++;              // 十字路口通过计数+1
        DIST = 0;
        dist_L = 0;
        dist_R = 0;
        Pass_Flag = 0;
        initPose_Module(&pose);     // 重置姿态积分
        Turn3_Flag = 0;             // 清除直行标记
    }
}

/**
 * @brief 岔路通过监测统一入口
 *
 * 根据Best_Flag、偏航角和编码器距离依次更新三类岔路状态及计数器。
 * 三个子监测器的调用顺序就是原函数的代码顺序，属于现有行为的一部分：
 * 前一段对全局状态的修改必须能被后一段在同一帧内观察到。
 */
void Flag_Monitor(void)
{
    Flag_Monitor_T_Junction1();
    Flag_Monitor_T_Junction2();
    Flag_Monitor_Crossroads();
}

/**
 * @brief 岔路方向决策控制
 *
 * 根据当前选择的赛道(road)和已通过的各类型岔路计数(Flag_Count/Flag_Count2/Flag_Count3)，
 * 查表决定下一次遇到对应岔路时的行驶方向。
 *
 * 决策表说明：
 *   - T_Turn1: NO-直行通过岔口, YES-需要转弯
 *   - T_Turn2: T2_Left-左转, T2_Right-右转
 *   - T_Turn3: C_Left-左转, C_Right-右转, C_Top-直行
 *
 * 每个 road 对应一条完整的比赛路线，case 值按岔路出现顺序预先编排。
 * 当计数器超过预设上限时，调用 Stop_car() 停车。
 */
void Turn_Control(void)
{
    // ================================================================
    // road == 0: 预赛正发车路线（共14个T字路口1, 4个T字路口2, 0个十字路口）
    // ================================================================
    if(road == 0)
    {
        // ---- T字路口1 方向决策表（14个岔路点）----
        switch(Flag_Count)
        {
            case 0:  T_Turn1 = NO;  break;
            case 1:  T_Turn1 = YES; break;
            case 2:  T_Turn1 = YES; break;
            case 3:  T_Turn1 = NO;  break;
            case 4:  T_Turn1 = YES; break;
            case 5:  T_Turn1 = NO;  break;
            case 6:  T_Turn1 = YES; break;
            case 7:  T_Turn1 = YES; break;
            case 8:  T_Turn1 = NO;  break;
            case 9:  T_Turn1 = NO;  break;
            case 10: T_Turn1 = NO;  break;
            case 11: T_Turn1 = YES; break;
            case 12: T_Turn1 = YES; break;
            case 13: T_Turn1 = NO;  break;
            case 14: T_Turn1 = NO;  break;
//          case 15: T_Turn1 = NO;  break;  // 备用位（注释掉）
//          case 16: T_Turn1 = NO;  break;  // 备用位（注释掉）
        }

        // ---- T字路口2 方向决策表（4个岔路点）----
        switch(Flag_Count2)
        {
            case 0: T_Turn2 = T2_Right; break;
            case 1: T_Turn2 = T2_Right; break;
            case 2: T_Turn2 = T2_Left;  break;
            case 3: T_Turn2 = T2_Right; break;
            case 4: T_Turn2 = T2_Right; break;
        }

        // ---- 十字路口 方向决策表（0个岔路点，仅默认值）----
        switch(Flag_Count3)
        {
            case 0: T_Turn3 = C_Top; break;
        }

        // 停车条件：所有岔路均已通过
        if (Flag_Count >= 17)
            Stop_car();
        if (Flag_Count2 >= 6)
            Stop_car();
        if (Flag_Count3 >= 1)
            Stop_car();
    }
    // ================================================================
    // road == 1: 决赛正发车路线（共33个T字路口1, 12个T字路口2, 0个十字路口）
    // 路线分为左半场和右半场两段
    // ================================================================
    else if(road == 1)
    {
        // ---- T字路口1 方向决策表（33个岔路点）----
        switch(Flag_Count)
        {
            case 0:  T_Turn1 = NO;  break;
            case 1:  T_Turn1 = YES; break;
            case 2:  T_Turn1 = YES; break;
            case 3:  T_Turn1 = NO;  break;
            case 4:  T_Turn1 = YES; break;
            case 5:  T_Turn1 = NO;  break;
            case 6:  T_Turn1 = YES; break;
            case 7:  T_Turn1 = YES; break;
            case 8:  T_Turn1 = NO;  break;
            case 9:  T_Turn1 = NO;  break;
            case 10: T_Turn1 = YES; break;
            // 左半边结束，开始右半边
            case 11: T_Turn1 = NO;  break;
            case 12: T_Turn1 = YES; break;
            case 13: T_Turn1 = YES; break;
            case 14: T_Turn1 = NO;  break;
            case 15: T_Turn1 = YES; break;
            case 16: T_Turn1 = NO;  break;
            case 17: T_Turn1 = YES; break;
            case 18: T_Turn1 = YES; break;
            case 19: T_Turn1 = NO;  break;
            case 20: T_Turn1 = NO;  break;
            case 21: T_Turn1 = NO;  break;
            case 22: T_Turn1 = YES; break;
            case 23: T_Turn1 = YES; break;
            case 24: T_Turn1 = NO;  break;
            case 25: T_Turn1 = NO;  break;
            case 26: T_Turn1 = NO;  break;
            case 27: T_Turn1 = NO;  break;
            case 28: T_Turn1 = NO;  break;
            case 29: T_Turn1 = YES; break;
            case 30: T_Turn1 = YES; break;
            case 31: T_Turn1 = NO;  break;
            case 32: T_Turn1 = NO;  break;
            case 33: T_Turn1 = NO;  break;
        }

        // ---- T字路口2 方向决策表（12个岔路点）----
        switch(Flag_Count2)
        {
            case 0: T_Turn2 = T2_Right; break;
            // 第1个
            case 1: T_Turn2 = T2_Right; break;
            case 2: T_Turn2 = T2_Left;  break;
            case 3: T_Turn2 = T2_Left;  break;
            case 4: T_Turn2 = T2_Right; break;
            case 5: T_Turn2 = T2_Left;  break;
            // 左半边结束，开始右半边
            case 6:  T_Turn2 = T2_Right; break;

            case 7:  T_Turn2 = T2_Right; break;
            case 8:  T_Turn2 = T2_Right; break;
            case 9:  T_Turn2 = T2_Left;  break;
            case 10: T_Turn2 = T2_Right; break;
            case 11: T_Turn2 = T2_Right; break;
            case 12: T_Turn2 = T2_Right; break;
        }

        // ---- 十字路口 方向决策表（0个岔路点，仅默认值）----
        switch(Flag_Count3)
        {
            case 0: T_Turn3 = C_Right; break;
        }

        // 停车条件
        if (Flag_Count >= 36)
            Stop_car();
        if (Flag_Count2 >= 15)
            Stop_car();
        if (Flag_Count3 >= 1)
            Stop_car();
    }
    // ================================================================
    // road == 2: 备用路线2（共17个T字路口1, 14个T字路口2, 6个十字路口）
    // ================================================================
    else if(road == 2)
    {
        // ---- T字路口1 方向决策表（17个岔路点）----  注释备忘: 9 8 4
        switch(Flag_Count)
        {
            case 0:  T_Turn1 = YES; break;
            case 1:  T_Turn1 = YES; break;
            case 2:  T_Turn1 = NO;  break;
            case 3:  T_Turn1 = NO;  break;
            case 4:  T_Turn1 = YES; break;
            case 5:  T_Turn1 = NO;  break;
            case 6:  T_Turn1 = YES; break;
            case 7:  T_Turn1 = NO;  break;
            case 8:  T_Turn1 = NO;  break;
            case 9:  T_Turn1 = YES; break;
            case 10: T_Turn1 = NO;  break;
            case 11: T_Turn1 = YES; break;
            case 12: T_Turn1 = NO;  break;
            case 13: T_Turn1 = NO;  break;
            case 14: T_Turn1 = YES; break;
            case 15: T_Turn1 = YES; break;
            case 16: T_Turn1 = NO;  break;

            case 17: T_Turn1 = NO;  break;
        }

        // ---- T字路口2 方向决策表（14个岔路点）----
        switch(Flag_Count2)
        {
            case 0:  T_Turn2 = T2_Right; break;
            case 1:  T_Turn2 = T2_Left;  break;
            case 2:  T_Turn2 = T2_Left;  break;
            case 3:  T_Turn2 = T2_Right; break;
            case 4:  T_Turn2 = T2_Right; break;
            case 5:  T_Turn2 = T2_Left;  break;
            case 6:  T_Turn2 = T2_Right; break;
            case 7:  T_Turn2 = T2_Right; break;
            case 8:  T_Turn2 = T2_Right; break;
            case 9:  T_Turn2 = T2_Left;  break;
            case 10: T_Turn2 = T2_Right; break;
            case 11: T_Turn2 = T2_Right; break;

            case 12: T_Turn2 = T2_Left;  break;
            case 13: T_Turn2 = T2_Left;  break;
            case 14: T_Turn2 = T2_Left;  break;
        }

        // ---- 十字路口 方向决策表（6个岔路点）----
        switch(Flag_Count3)
        {
            case 0: T_Turn3 = C_Left;  break;
            case 1: T_Turn3 = C_Top;   break;
            case 2: T_Turn3 = C_Top;   break;
            case 3: T_Turn3 = C_Right; break;
            case 4: T_Turn3 = C_Top;   break;
            case 5: T_Turn3 = C_Top;   break;

            case 6: T_Turn3 = C_Top;   break;
        }

        // 停车条件
        if (Flag_Count >= 20)
            Stop_car();
        if (Flag_Count2 >= 15)
            Stop_car();
        if (Flag_Count3 >= 6)
            Stop_car();
    }
    // ================================================================
    // road == 3: 备用路线3（共21个T字路口1, 11个T字路口2, 3个十字路口）
    // ================================================================
    else if(road == 3)
    {
        // ---- T字路口1 方向决策表（21个岔路点）----  注释备忘: 15 6 1
        switch(Flag_Count)
        {
            case 0:  T_Turn1 = NO;  break;
            case 1:  T_Turn1 = NO;  break;
            case 2:  T_Turn1 = NO;  break;
            case 3:  T_Turn1 = NO;  break;
            case 4:  T_Turn1 = NO;  break;
            case 5:  T_Turn1 = NO;  break;
            case 6:  T_Turn1 = NO;  break;
            case 7:  T_Turn1 = YES; break;
            case 8:  T_Turn1 = NO;  break;
            case 9:  T_Turn1 = NO;  break;
            case 10: T_Turn1 = YES; break;
            case 11: T_Turn1 = YES; break;
            case 12: T_Turn1 = NO;  break;
            case 13: T_Turn1 = NO;  break;
            case 14: T_Turn1 = YES; break;
            case 15: T_Turn1 = NO;  break;
            case 16: T_Turn1 = YES; break;
            case 17: T_Turn1 = YES; break;
            case 18: T_Turn1 = YES; break;
            case 19: T_Turn1 = YES; break;
            case 20: T_Turn1 = NO;  break;
            case 21: T_Turn1 = NO;  break;
        }

        // ---- T字路口2 方向决策表（11个岔路点）----
        switch(Flag_Count2)
        {
            case 0:  T_Turn2 = T2_Right; break;
            case 1:  T_Turn2 = T2_Left;  break;
            case 2:  T_Turn2 = T2_Right; break;
            case 3:  T_Turn2 = T2_Right; break;
            case 4:  T_Turn2 = T2_Left;  break;
            case 5:  T_Turn2 = T2_Right; break;
            case 6:  T_Turn2 = T2_Right; break;
            case 7:  T_Turn2 = T2_Left;  break;
            case 8:  T_Turn2 = T2_Left;  break;
            case 9:  T_Turn2 = T2_Right; break;
            case 10: T_Turn2 = T2_Left;  break;
            case 11: T_Turn2 = T2_Left;  break;
        }

        // ---- 十字路口 方向决策表（3个岔路点）----
        switch(Flag_Count3)
        {
            case 0: T_Turn3 = C_Right; break;
            case 1: T_Turn3 = C_Right; break;
            case 2: T_Turn3 = C_Top;   break;
            case 3: T_Turn3 = C_Top;   break;
        }

        // 停车条件
        if (Flag_Count >= 23)
        {
            Stop_car();
        }
        if (Flag_Count2 >= 13)
        {
            Stop_car();
        }
        if (Flag_Count3 >= 5)
        {
            Stop_car();
        }
    }
}
