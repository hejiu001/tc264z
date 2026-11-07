#include "zf_common_headfile.h"
uint8 Pixle[image_h][image_w];
///////////////参数调整///////////////
int8_t close_Threshold = +3;//近端区域的二值化阈值补偿，范围-128~127
int8_t far_Threshold = 0;//远端区域的二值化阈值补偿，范围-128~127
int8_t mid_Threshold = 0;
uint8_t Angele_Turn = 11;///直角弯转向时机，范围0-56，数值越大转弯越晚
uint32_t antle_DIST = 1500;
uint8_t LR_Change =  25;
int8_t more_dist1 = -3;
int8_t more_dist2 = -1;
int8_t more_dist3 = +2;
uint8_t lost_find_dist = image_h/3;
float LM_Temp = 28;
float RM_Temp = 28;
float Lim_Temp = 35;
float gro_mid_offset = 18;
float gro_mid_offset2 = 18;
float TLim_k = 0.34;
#define TH_MIN   70    //140
#define TH_MAX   130  //180
#define GRAY_MIN  10  //Otsu统计使用的灰度下限，用于抑制暗部噪声
#define GRAY_MAX  160  //Otsu统计使用的灰度上限，用于限制过亮像素

#define OTSU_GRID_COLS              6       // Otsu局部统计的横向分块数；增大可适应左右光照差异，但每块样本减少、噪声和运算量增加
#define OTSU_GRID_ROWS              4       // Otsu局部统计的纵向分块数；增大可细分远近光照，但局部阈值更容易抖动
#define OTSU_HIST_BINS              32      // 灰度直方图箱数；箱数越多阈值分辨率越高，但每箱样本变少、抗噪性下降
#define OTSU_HIST_SHIFT             3       // 灰度压缩位数：灰度右移3位，相当于每8级灰度合并为1箱；需与HIST_BINS配套修改
#define OTSU_SAMPLE_STEP_X          4       // 横向采样步长；增大可降低计算量，减小可提高统计精度但更耗时
#define OTSU_SAMPLE_STEP_Y          2       // 纵向采样步长；增大可降低计算量，减小可提高统计精度但更耗时
#define OTSU_PERCENTILE_DEN         4U      // 暗部参考的分位数分母；4表示取约25%分位，增大参考值更偏暗，减小则更接近中间亮度
#define OTSU_MAX_LOCAL_RAISE        30      // 单个分块允许相对全局阈值抬高的最大差值；增大更能抑制局部亮斑，但可能使白线变细或丢失
#define OTSU_PEAK_SUPPORT_DEN       8U      // 局部暗峰的最小样本支持比例分母；增大更容易接受弱峰但抗噪性下降，减小则要求峰更明显
#define OTSU_INVALID_VALUE          255U    // 局部统计无效时使用的标记值；仅作内部哨兵值，不要用来调节图像效果
#define OTSU_GLOBAL_COL_FIRST       1       // 参与全局Otsu统计的起始横向分块，跳过最左边缘以减少边缘干扰
#define OTSU_GLOBAL_COL_LAST        (OTSU_GRID_COLS - 2) // 参与全局Otsu统计的结束横向分块，跳过最右边缘以减少边缘干扰
#define OTSU_MIN_CLASS_GAP          30.0f  // 暗、亮两类平均灰度的最小间隔；增大更稳定但低对比度时会保持旧阈值，减小则更灵敏但可能误更新
#define OTSU_INTERP_SCALE           256U   // 局部阈值纵向插值的定点比例，对应代码中的>>8；属于实现常量，不建议单独修改

///////////////图像缓冲区///////////////
static uint8 _Image_Use_buf[image_h][image_w];
uint8 *Image_Use[image_h];
uint8 *Image_Use_linear = _Image_Use_buf[0];  // 向外部提供图像缓冲区的线性首地址
static uint16 otsu_global_histogram[OTSU_HIST_BINS];
static uint16 otsu_local_histogram[OTSU_HIST_BINS];
static uint8 otsu_local_raise[OTSU_GRID_ROWS][OTSU_GRID_COLS];
static uint8 otsu_local_dark_peak[OTSU_GRID_ROWS][OTSU_GRID_COLS];
static uint8 otsu_local_valid[OTSU_GRID_ROWS][OTSU_GRID_COLS];
static uint8 otsu_previous_local_raise[OTSU_GRID_ROWS][OTSU_GRID_COLS];
static uint8 otsu_x_tile_boundary[OTSU_GRID_COLS + 1];
static uint8 otsu_y_top_node[image_h];
static uint8 otsu_y_weight[image_h];
static uint8 otsu_interpolation_ready = 0U;
static uint8 otsu_local_history_ready = 0U;
static uint8 otsu_previous_threshold = TH_MIN;
uint8_t top_white_max = image_w/5;
uint8_t bottom_white_max = image_w/4;
uint16 white_num_row[image_h+5],white_num_col[image_w+5];
uint16 white_row_max[2],white_row_min[2];

uint8 XX = image_w -1;
uint8 YY = image_h -1;

uint8 image_bottom = image_h-3;
uint8 image_top = 2;
uint8 image_left=2;
uint8 image_right=image_w-3;

uint8 Lost_Bottom = image_h-3;
uint8 Lost_Top = 2;
uint8 Lost_Left=2;
uint8 Lost_Right=image_w-3;
uint8 imgTran[image_h][image_w];

uint8_t nowThreshold = TH_MIN;

uint8 l_lost_tip = 0;
uint8 r_lost_tip = 0;
uint8 t_lost_tip = 0;
uint8 b_lost_tip = 0;

uint8 start_center_last=image_w / 2;
uint8 start_center=image_w/2;
uint8 start_point_l = 0;
uint8 start_point_r = image_w-1;
uint8 start_center_y=0;
uint8 start_center_x=image_w/2;
uint8 end_center=image_w/2;
uint8 end_center_x=image_w/2;
uint8 end_center_y=image_h;
uint8 angle_end_center_x = image_w/2;
uint8 angle_end_center_y = image_h;
uint8 mid_line[image_h] = {image_w/2};
uint16 b_center[3] = {0};
uint16 l_center[3] = {0};
uint16 r_center[3] = {0};
uint16 t_center[3] = {0};
uint8 b_lost_num = 0;
uint8 t_lost_num = 0;
uint8 l_lost_num = 0;
uint8 r_lost_num = 0;

int  Goto_U = 1;
int  Goto_D = 2;
int  Goto_L = 4;
int  Goto_R = 8;
int  UD = 3;
int  UL = 5;
int  UR = 9;
int  DL = 6;
int  DR = 10;
int  LR = 12;
int  UDL = 7;
int  UDR = 11;
int  ULR = 13;
int  DLR = 14;
int  UDLR = 15;
int seekcol_num = 20;
enum ImgFlag IF = straightline;
enum ImgFlag Best_Flag = straightline;
int disappear_total = 3;
uint8 dis_Solidline = 0;
uint8_t Top_Black;
float Temp=0;
uint8_t T1_flag = 0;
uint8_t T2_flag = 0;
uint8_t Oblique_Flag = 0;
uint8_t TL_FSM = 0;
uint8_t TR_FSM = 0;
uint8_t angle_flag_l = 0;
uint8_t angle_flag_r = 0;
float target_speed = 0.0f;

uint8_t l2_white_tip = 0;
uint8_t r2_white_tip = 0;

uint8_t t_white_tip = 0;
uint8_t l_white_tip = 0;
uint8_t r_white_tip = 0;
uint8_t boundary_line = 0;



//const int quan_mid[60]={
//    1,1,1,1,1,//1-5     1
//    3,3,3,3,3,//6-10    3
//    5,5,5,5,5,//11-15   5
//    7,7,7,7,7,//16-20   7
//    9,9,9,9,9,//21-25   9
//   11,11,11,11,11,//26-30 11
//   13,13,13,13,13,//31-35 13
//   11,11,11,11,11,//36-40 11
//    9,9,9,9,9,//41-45   9
//    7,7,7,7,7,//46-50   7
//    5,5,5,5,5,//51-55   5
//    3,3,3,3,3,//56-60   3
//};
const int quan_mid[60]={
    1,1,1,1,1,//1-5     1
    3,3,3,3,3,//6-10    3
    5,5,5,5,5,//11-15   5
    7,7,7,7,7,//16-20   7
    9,9,9,9,9,//21-25   9
   11,11,11,11,11,11,11,11,11,11,//26-30 11
    9,9,9,9,9,//31-35   9
    7,7,7,7,7,//36-40   7
    5,5,5,5,5,//41-45   5
    3,3,3,3,3,//46-50   3
    1,1,1,1,1,//1-5     1
};

//const int quan_mid[58]={
//    1,1,1,1,1,//1-5     1
//    3,3,3,3,3,//6-10    3
//    5,5,5,5,5,//11-15   5
//    7,7,7,7,7,//16-20   7
//    9,9,9,9,9,//21-25   9
//   11,11,11,11,11,11,11,11,//26-30 11
//    9,9,9,9,9,//31-35   9
//    7,7,7,7,7,//36-40   7
//    5,5,5,5,5,//41-45   5
//    3,3,3,3,3,//46-50   3
//    1,1,1,1,1,//1-5     1
//};

void Image_CompressInit(void)
{
    const int h = image_h;
    const int w = image_w;
    const int SRC_ROW_STEP = MT9V03X_W * 2;   /* 隔行采样：每次跳过两行 */
    const uint8_t *sp_row = mt9v03x_image[0];  /* 源图像首行基地址 */
    int i, j;

    for (i = 0; i < h; i++)
    {
        Image_Use[i] = _Image_Use_buf[i];

        const uint8_t *sp = sp_row;
        uint8_t       *dp = Image_Use[i];

        /* 四像素循环展开：将 sp[0,2,4,6] 写入 dp[0,1,2,3] */
        for (j = 0; j <= w - 4; j += 4)
        {
            dp[0] = sp[0];
            dp[1] = sp[2];
            dp[2] = sp[4];
            dp[3] = sp[6];
            dp += 4;
            sp += 8;
        }

        for (; j < w; j++)
        {
            *dp++ = *sp;
            sp += 2;
        }

        sp_row += SRC_ROW_STEP;
    }
}




#pragma optimize 2
static uint8 Otsu_HistogramPercentile(const uint16 *histogram, uint32 count)
{
    uint32 target;
    uint32 cumulative = 0U;
    int bin;

    if (count == 0U)
    {
        return OTSU_INVALID_VALUE;
    }

    target = (count + OTSU_PERCENTILE_DEN - 1U) / OTSU_PERCENTILE_DEN;
    for (bin = 0; bin < OTSU_HIST_BINS; bin++)
    {
        cumulative += histogram[bin];
        if (cumulative >= target)
        {
            return (uint8)((bin << OTSU_HIST_SHIFT)
                         + (1U << (OTSU_HIST_SHIFT - 1)));
        }
    }

    return OTSU_INVALID_VALUE;
}

static uint32 Otsu_HistogramSupport(const uint16 *histogram, int bin)
{
    uint32 support = histogram[bin];

    if (bin > 0)
    {
        support += histogram[bin - 1];
    }
    if (bin + 1 < OTSU_HIST_BINS)
    {
        support += histogram[bin + 1];
    }
    return support;
}

static uint32 Otsu_HistogramSmooth(const uint16 *histogram, int bin)
{
    uint32 value;

    if (bin < 0 || bin >= OTSU_HIST_BINS)
    {
        return 0U;
    }

    value = (uint32)histogram[bin] * 2U;
    if (bin > 0)
    {
        value += histogram[bin - 1];
    }
    if (bin + 1 < OTSU_HIST_BINS)
    {
        value += histogram[bin + 1];
    }
    return value;
}

static uint8 Otsu_FindFirstSupportedPeak(const uint16 *histogram, uint32 count)
{
    uint32 minimum_support;
    int bin;

    if (count == 0U)
    {
        return OTSU_INVALID_VALUE;
    }

    minimum_support = (count + OTSU_PEAK_SUPPORT_DEN - 1U)
                    / OTSU_PEAK_SUPPORT_DEN;
    for (bin = 0; bin < OTSU_HIST_BINS; bin++)
    {
        uint32 current = Otsu_HistogramSmooth(histogram, bin);

        if (current >= Otsu_HistogramSmooth(histogram, bin - 1)
            && current >= Otsu_HistogramSmooth(histogram, bin + 1)
            && Otsu_HistogramSupport(histogram, bin) >= minimum_support)
        {
            return (uint8)((bin << OTSU_HIST_SHIFT)
                         + (1U << (OTSU_HIST_SHIFT - 1)));
        }
    }

    return OTSU_INVALID_VALUE;
}

static uint8 Otsu_FindGlobalPeak(uint8 maximum_gray, uint8 *peak)
{
    uint32 best_smooth = 0U;
    uint8 found = 0U;
    int maximum_bin = maximum_gray >> OTSU_HIST_SHIFT;
    int bin;

    for (bin = 0; bin <= maximum_bin && bin < OTSU_HIST_BINS; bin++)
    {
        uint32 smooth;

        if (Otsu_HistogramSupport(otsu_global_histogram, bin) == 0U)
        {
            continue;
        }

        smooth = Otsu_HistogramSmooth(otsu_global_histogram, bin);
        if (found == 0U || smooth > best_smooth)
        {
            found = 1U;
            best_smooth = smooth;
            *peak = (uint8)((bin << OTSU_HIST_SHIFT)
                          + (1U << (OTSU_HIST_SHIFT - 1)));
        }
    }

    return found;
}

static uint8 Otsu_MedianU8(uint8 *values, int count)
{
    int i;

    for (i = 1; i < count; i++)
    {
        uint8 value = values[i];
        int j = i;

        while (j > 0 && values[j - 1] > value)
        {
            values[j] = values[j - 1];
            j--;
        }
        values[j] = value;
    }

    if ((count & 1) != 0)
    {
        return values[count / 2];
    }
    return (uint8)(((uint16)values[count / 2 - 1]
                  + values[count / 2]) / 2U);
}

static void Otsu_RestoreLocalRaiseHistory(void)
{
    int tile_y;
    int tile_x;

    for (tile_y = 0; tile_y < OTSU_GRID_ROWS; tile_y++)
    {
        for (tile_x = 0; tile_x < OTSU_GRID_COLS; tile_x++)
        {
            otsu_local_raise[tile_y][tile_x] =
                (otsu_local_history_ready != 0U)
                ? otsu_previous_local_raise[tile_y][tile_x] : 0U;
        }
    }
}

static void Otsu_RepairInvalidLocalRaise(void)
{
    uint8 valid_count = 0U;
    int tile_y;
    int tile_x;

    for (tile_y = 0; tile_y < OTSU_GRID_ROWS; tile_y++)
    {
        for (tile_x = 0; tile_x < OTSU_GRID_COLS; tile_x++)
        {
            if (otsu_local_valid[tile_y][tile_x] != 0U)
            {
                valid_count++;
            }
        }
    }

    if (valid_count == 0U)
    {
        Otsu_RestoreLocalRaiseHistory();
        return;
    }

    for (tile_y = 0; tile_y < OTSU_GRID_ROWS; tile_y++)
    {
        for (tile_x = 0; tile_x < OTSU_GRID_COLS; tile_x++)
        {
            uint8 nearest_values[OTSU_GRID_ROWS * OTSU_GRID_COLS];
            int nearest_count = 0;
            int minimum_distance = OTSU_GRID_ROWS + OTSU_GRID_COLS;
            int source_y;
            int source_x;

            if (otsu_local_valid[tile_y][tile_x] != 0U)
            {
                continue;
            }

            for (source_y = 0; source_y < OTSU_GRID_ROWS; source_y++)
            {
                for (source_x = 0; source_x < OTSU_GRID_COLS; source_x++)
                {
                    int distance;
                    int dy;
                    int dx;

                    if (otsu_local_valid[source_y][source_x] == 0U)
                    {
                        continue;
                    }

                    dy = source_y - tile_y;
                    dx = source_x - tile_x;
                    if (dy < 0) dy = -dy;
                    if (dx < 0) dx = -dx;
                    distance = dy + dx;

                    if (distance < minimum_distance)
                    {
                        minimum_distance = distance;
                        nearest_count = 0;
                    }
                    if (distance == minimum_distance)
                    {
                        nearest_values[nearest_count++] =
                            otsu_local_raise[source_y][source_x];
                    }
                }
            }

            otsu_local_raise[tile_y][tile_x] =
                (nearest_count > 0) ? Otsu_MedianU8(nearest_values, nearest_count) : 0U;
        }
    }

    for (tile_y = 0; tile_y < OTSU_GRID_ROWS; tile_y++)
    {
        for (tile_x = 0; tile_x < OTSU_GRID_COLS; tile_x++)
        {
            otsu_previous_local_raise[tile_y][tile_x] =
                otsu_local_raise[tile_y][tile_x];
        }
    }
    otsu_local_history_ready = 1U;
}

static int Otsu_GridCenter(int node, int length, int node_count)
{
    int start = node * length / node_count;
    int end = (node + 1) * length / node_count;
    return (start + end - 1) / 2;
}

static void Otsu_InitializeInterpolationMaps(void)
{
    int first_center;
    int last_center;
    int coordinate;
    int node;

    if (otsu_interpolation_ready != 0U)
    {
        return;
    }

    for (node = 0; node <= OTSU_GRID_COLS; node++)
    {
        int source_boundary = node * MT9V03X_W / OTSU_GRID_COLS;

        /* 压缩图第 x 列来自原图第 2*x 列，因此奇数边界需要向上取整。 */
        otsu_x_tile_boundary[node] = (uint8)((source_boundary + 1) / 2);
    }

    first_center = Otsu_GridCenter(0, image_h, OTSU_GRID_ROWS);
    last_center = Otsu_GridCenter(
        OTSU_GRID_ROWS - 1, image_h, OTSU_GRID_ROWS);

    for (coordinate = 0; coordinate < image_h; coordinate++)
    {
        if (coordinate <= first_center)
        {
            otsu_y_top_node[coordinate] = 0U;
            otsu_y_weight[coordinate] = 0U;
        }
        else if (coordinate >= last_center)
        {
            otsu_y_top_node[coordinate] = OTSU_GRID_ROWS - 1;
            otsu_y_weight[coordinate] = 0U;
        }
        else
        {
            for (node = 0; node < OTSU_GRID_ROWS - 1; node++)
            {
                int top_center = Otsu_GridCenter(node, image_h, OTSU_GRID_ROWS);
                int bottom_center = Otsu_GridCenter(node + 1, image_h, OTSU_GRID_ROWS);

                if (coordinate < bottom_center)
                {
                    uint32 weight = (uint32)(coordinate - top_center)
                                  * OTSU_INTERP_SCALE
                                  / (uint32)(bottom_center - top_center);
                    if (weight > 255U) weight = 255U;
                    otsu_y_top_node[coordinate] = (uint8)node;
                    otsu_y_weight[coordinate] = (uint8)weight;
                    break;
                }
            }
        }
    }

    otsu_interpolation_ready = 1U;
}

uint8_t Otsu_TrackThreshold(uint8 *image, uint16_t width, uint16_t height)
{
    uint32 total_count = 0U;
    uint32 total_gray = 0U;
    uint32 background_count = 0U;
    uint32 background_sum = 0U;
    uint32 best_background_count = 0U;
    uint32 best_background_sum = 0U;
    uint8 first_global_bin = OTSU_INVALID_VALUE;
    uint8 last_global_bin = 0U;
    uint8 global_background;
    uint8 global_peak = 0U;
    uint8 global_peak_valid = 0U;
    uint8 otsu_reliable = 0U;
    uint8 best_threshold = otsu_previous_threshold;
    uint8 final_threshold;
    float max_variance = 0.0f;
    int tile_y;
    int tile_x;
    int bin;

    if (image == NULL || width < OTSU_GRID_COLS || height < OTSU_GRID_ROWS)
    {
        Otsu_RestoreLocalRaiseHistory();
        return otsu_previous_threshold;
    }

    for (bin = 0; bin < OTSU_HIST_BINS; bin++)
    {
        otsu_global_histogram[bin] = 0U;
    }

    for (tile_y = 0; tile_y < OTSU_GRID_ROWS; tile_y++)
    {
        int y_start = tile_y * height / OTSU_GRID_ROWS;
        int y_end = (tile_y + 1) * height / OTSU_GRID_ROWS;

        for (tile_x = 0; tile_x < OTSU_GRID_COLS; tile_x++)
        {
            int x_start = tile_x * width / OTSU_GRID_COLS;
            int x_end = (tile_x + 1) * width / OTSU_GRID_COLS;
            uint32 local_count = 0U;
            int y;

            for (bin = 0; bin < OTSU_HIST_BINS; bin++)
            {
                otsu_local_histogram[bin] = 0U;
            }

            for (y = y_start; y < y_end; y += OTSU_SAMPLE_STEP_Y)
            {
                const uint8 *row = image + (uint32)y * width;
                int x;

                for (x = x_start; x < x_end; x += OTSU_SAMPLE_STEP_X)
                {
                    uint8 value = row[x];
                    otsu_local_histogram[value >> OTSU_HIST_SHIFT]++;
                    local_count++;
                }
            }

            otsu_local_raise[tile_y][tile_x] =
                Otsu_HistogramPercentile(otsu_local_histogram, local_count);
            otsu_local_dark_peak[tile_y][tile_x] =
                Otsu_FindFirstSupportedPeak(otsu_local_histogram, local_count);

            if (tile_x >= OTSU_GLOBAL_COL_FIRST
                && tile_x <= OTSU_GLOBAL_COL_LAST)
            {
                for (bin = 0; bin < OTSU_HIST_BINS; bin++)
                {
                    otsu_global_histogram[bin] = (uint16)(
                        otsu_global_histogram[bin] + otsu_local_histogram[bin]);
                }
            }
        }
    }

    for (bin = 0; bin < OTSU_HIST_BINS; bin++)
    {
        uint16 count = otsu_global_histogram[bin];

        if (count != 0U)
        {
            uint8 center = (uint8)((bin << OTSU_HIST_SHIFT)
                                 + (1U << (OTSU_HIST_SHIFT - 1)));
            if (first_global_bin == OTSU_INVALID_VALUE)
            {
                first_global_bin = (uint8)bin;
            }
            last_global_bin = (uint8)bin;
            total_count += count;
            total_gray += (uint32)center * count;
        }
    }

    if (total_count == 0U || first_global_bin == last_global_bin)
    {
        Otsu_RestoreLocalRaiseHistory();
        return otsu_previous_threshold;
    }

    global_background = Otsu_HistogramPercentile(
        otsu_global_histogram, total_count);

    for (bin = 0; bin < OTSU_HIST_BINS; bin++)
    {
        uint8 center = (uint8)((bin << OTSU_HIST_SHIFT)
                             + (1U << (OTSU_HIST_SHIFT - 1)));
        uint32 foreground_count;
        uint32 foreground_sum;
        float background_mean;
        float foreground_mean;
        float difference;
        float variance;

        background_count += otsu_global_histogram[bin];
        background_sum += (uint32)center * otsu_global_histogram[bin];
        if (background_count == 0U || background_count >= total_count)
        {
            continue;
        }

        foreground_count = total_count - background_count;
        foreground_sum = total_gray - background_sum;
        background_mean = (float)background_sum / background_count;
        foreground_mean = (float)foreground_sum / foreground_count;
        difference = foreground_mean - background_mean;
        variance = (float)background_count
                 * (float)foreground_count
                 * difference * difference;

        if (variance > max_variance)
        {
            max_variance = variance;
            best_threshold = (uint8)((bin << OTSU_HIST_SHIFT) + 7);
            best_background_count = background_count;
            best_background_sum = background_sum;
        }
    }

    if (best_background_count > 0U && best_background_count < total_count)
    {
        uint32 foreground_count = total_count - best_background_count;
        uint32 foreground_sum = total_gray - best_background_sum;
        float background_mean = (float)best_background_sum / best_background_count;
        float foreground_mean = (float)foreground_sum / foreground_count;

        if (foreground_mean - background_mean >= OTSU_MIN_CLASS_GAP)
        {
            otsu_reliable = 1U;
            global_peak_valid = Otsu_FindGlobalPeak(best_threshold, &global_peak);
        }
    }

    for (tile_y = 0; tile_y < OTSU_GRID_ROWS; tile_y++)
    {
        for (tile_x = 0; tile_x < OTSU_GRID_COLS; tile_x++)
        {
            uint8 local_reference;
            uint8 global_reference;
            int difference;

            if (otsu_local_raise[tile_y][tile_x] == OTSU_INVALID_VALUE)
            {
                otsu_local_valid[tile_y][tile_x] = 0U;
                otsu_local_raise[tile_y][tile_x] = 0U;
                continue;
            }

            if (global_peak_valid != 0U
                && otsu_local_dark_peak[tile_y][tile_x] != OTSU_INVALID_VALUE)
            {
                local_reference = otsu_local_dark_peak[tile_y][tile_x];
                global_reference = global_peak;
            }
            else
            {
                local_reference = otsu_local_raise[tile_y][tile_x];
                global_reference = global_background;
            }

            difference = (int)local_reference - (int)global_reference;
            if (difference > OTSU_MAX_LOCAL_RAISE)
            {
                otsu_local_valid[tile_y][tile_x] = 0U;
                otsu_local_raise[tile_y][tile_x] = 0U;
            }
            else
            {
                otsu_local_valid[tile_y][tile_x] = 1U;
                otsu_local_raise[tile_y][tile_x] =
                    (difference > 0) ? (uint8)difference : 0U;
            }
        }
    }

    Otsu_RepairInvalidLocalRaise();
    if (otsu_reliable == 0U)
    {
        return otsu_previous_threshold;
    }

    if (best_threshold < TH_MIN) best_threshold = TH_MIN;
    if (best_threshold > TH_MAX) best_threshold = TH_MAX;
    final_threshold = (uint8)(((uint16)otsu_previous_threshold * 3U
                             + best_threshold) / 4U);
    otsu_previous_threshold = final_threshold;
    return final_threshold;
}
#pragma endoptimize
int16_t img_threshold_group[3] = {0,0,0};
void Get_imgOSTU(void)
{
    Image_CompressInit();
    memset(Pixle,         Black, sizeof(Pixle));
    memset(white_num_row, 0,     sizeof(white_num_row));
    memset(white_num_col, 0,     sizeof(white_num_col));

    static uint8_t row_thresh[image_h];
    uint8_t h3  = image_h / 3;
    uint8_t h23 = image_h * 2 / 3;
    uint8_t th0 = (uint8_t)(nowThreshold + far_Threshold);
    uint8_t th1 = (uint8_t)(nowThreshold + mid_Threshold);
    uint8_t th2 = (uint8_t)(nowThreshold + close_Threshold);
    for(uint8 i = 0; i < image_h; i++)
        row_thresh[i] = (i < h3) ? th0 : (i < h23 ? th1 : th2);

    static uint32_t bin_row[image_h][3];
    for(uint8 i = 0; i < YY; i++)
    {
        uint8_t th = row_thresh[i];
        uint32_t w0 = 0, w1 = 0, w2 = 0;
        const uint8_t *src = Image_Use[i];   // 行指针直接取，无变化
        for(uint8 j = 0;  j < 32; j++) w0 |= ((uint32_t)(src[j]    >= th)) << j;
        for(uint8 j = 0;  j < 32; j++) w1 |= ((uint32_t)(src[j+32] >= th)) << j;
        for(uint8 j = 0;  j < 26; j++) w2 |= ((uint32_t)(src[j+64] >= th)) << j;
        bin_row[i][0] = w0;
        bin_row[i][1] = w1;
        bin_row[i][2] = w2;
    }

    #define BIN(r, c)  ((bin_row[(r)][(c) >> 5] >> ((c) & 31)) & 1u)

    for(uint8 i = 1; i < YY-1; i++)
    {
        for(uint8 j = 1; j < XX-1; j++)
        {
            if(!BIN(i, j)) continue;
            if(Pixle[i][j] == White) { white_num_row[i]++; white_num_col[j]++; continue; }

            uint8_t bright =
                BIN(i-1, j-1) + BIN(i-1, j) + BIN(i-1, j+1) +
                BIN(i,   j-1) +               BIN(i,   j+1) +
                BIN(i+1, j-1) + BIN(i+1, j) + BIN(i+1, j+1);

            if(bright < 7) continue;


            Pixle[i][j] = White;
            white_num_row[i]++;
            white_num_col[j]++;

            if(Pixle[i-1][j-1] != White) Pixle[i-1][j-1] = White;
            if(Pixle[i-1][j  ] != White) Pixle[i-1][j  ] = White;
            if(Pixle[i-1][j+1] != White) Pixle[i-1][j+1] = White;
            if(Pixle[i  ][j-1] != White) Pixle[i  ][j-1] = White;
            if(Pixle[i  ][j+1] != White) Pixle[i  ][j+1] = White;
            if(Pixle[i+1][j-1] != White) Pixle[i+1][j-1] = White;
            if(Pixle[i+1][j  ] != White) Pixle[i+1][j  ] = White;
            if(Pixle[i+1][j+1] != White) Pixle[i+1][j+1] = White;
        }
    }

    for(uint8 j = 0; j < XX; j++)
    {
        if(BIN(0, j)    && Pixle[0][j]    != White) { Pixle[0][j]    = White; white_num_row[0]++;    white_num_col[j]++; }
        if(BIN(YY-1, j) && Pixle[YY-1][j] != White) { Pixle[YY-1][j] = White; white_num_row[YY-1]++; white_num_col[j]++; }
    }

    #undef BIN
}


uint8 white_col_max[2],white_col_min[2];
uint8 white_col_max[2],white_col_min[2];

#pragma optimize 2
void Get_BinaryImage2(void)
{
    const int h = image_h;
    int i;

    Image_CompressInit();
    Otsu_InitializeInterpolationMaps();

    memset(white_num_row, 0, sizeof(white_num_row));
    memset(white_num_col, 0, sizeof(white_num_col));

    for (i = 0; i < h; i++)
    {
        const uint8_t *src = Image_Use[i];
        uint8_t       *dst = Pixle[i];
        uint8 top = otsu_y_top_node[i];
        uint8 wy = otsu_y_weight[i];
        int count = 0;
        int node;

        /* 纵向插值局部补偿，横向每个分块共用一个阈值。 */
        for (node = 0; node < OTSU_GRID_COLS; node++)
        {
            uint16 raise;
            uint16 threshold;
            int x = otsu_x_tile_boundary[node];
            int x_end = otsu_x_tile_boundary[node + 1];

            if (top >= OTSU_GRID_ROWS - 1)
            {
                raise = otsu_local_raise[OTSU_GRID_ROWS - 1][node];
            }
            else
            {
                uint16 upper = otsu_local_raise[top][node];
                uint16 lower = otsu_local_raise[top + 1][node];
                raise = (uint16)(
                    (upper * (OTSU_INTERP_SCALE - wy) + lower * wy) >> 8);
            }

            threshold = (uint16)nowThreshold + raise;
            if (threshold > 255U) threshold = 255U;

            for (; x + 3 < x_end; x += 4)
            {
                int w0 = ((uint16)src[x]     > threshold);
                int w1 = ((uint16)src[x + 1] > threshold);
                int w2 = ((uint16)src[x + 2] > threshold);
                int w3 = ((uint16)src[x + 3] > threshold);

                dst[x]     = (uint8)(0 - w0);
                dst[x + 1] = (uint8)(0 - w1);
                dst[x + 2] = (uint8)(0 - w2);
                dst[x + 3] = (uint8)(0 - w3);
                white_num_col[x]     += (uint16)w0;
                white_num_col[x + 1] += (uint16)w1;
                white_num_col[x + 2] += (uint16)w2;
                white_num_col[x + 3] += (uint16)w3;
                count += w0 + w1 + w2 + w3;
            }

            for (; x < x_end; x++)
            {
                int white = ((uint16)src[x] > threshold);
                dst[x] = (uint8)(0 - white);
                white_num_col[x] += (uint16)white;
                count += white;
            }
        }

        white_num_row[i] = (uint16)count;
    }
}
#pragma endoptimize
uint8 Dashedline_Makeup(void)
{
    const int lim_row = YY;
    const int lim_col = XX;
    const int seek    = seekcol_num;
    int i, j;

    /* 统计行、列白点数量的极值及其位置 */
    white_row_max[0] = 0;
    white_row_min[0] = XX;
    for (i = image_bottom; i >= image_top; i--)
    {
        uint16_t v = white_num_row[i];
        if (v > white_row_max[0])
        { white_row_max[0] = v; white_row_max[1] = i; }
        else if (v < white_row_min[0])
        { white_row_min[0] = v; white_row_min[1] = i; }
    }

    white_col_max[0] = 0;
    white_col_min[0] = YY;
    for (i = image_left; i <= image_right; i++)
    {
        uint16_t v = white_num_col[i];
        if (v > white_col_max[0])
        { white_col_max[0] = v; white_col_max[1] = i; }
        if (v < white_col_min[0])
        { white_col_min[0] = v; white_col_min[1] = i; }
    }

    memset(imgTran, Black, sizeof(imgTran));

    /* 从每个白点向四个方向扫描连续黑色间隔 */
    for (i = 0; i <= lim_row; i++)
    {
        /* 当前行没有白点时无需扫描 */
        if (white_num_row[i] == 0) continue;

        const uint8_t *row  = Pixle[i];
        uint8_t       *trow = imgTran[i];
        int            wlim = white_num_row[i];

        /* 缓存相邻行指针，减少垂直扫描中的重复索引计算 */
        const uint8_t *row_above = (i > 0)       ? Pixle[i - 1] : NULL;
        const uint8_t *row_below = (i < lim_row) ? Pixle[i + 1] : NULL;

        for (j = 0; j <= lim_col; j++)
        {
            if (row[j] != White) continue;

            /* 向右扫描，并用位 3 标记可连接的黑点 */
            {
                int w_end = j + wlim;
                if (w_end > lim_col) w_end = lim_col;
                const uint8_t *rp = row + j + 1;
                uint8_t       *tp = trow + j + 1;
                while (rp <= row + w_end)
                {
                    if (*rp != Black) break;
                    *tp += 8;
                    rp++; tp++;
                }
            }

            /* 向左扫描，并用位 2 标记可连接的黑点 */
            {
                int w_start = j - wlim;
                if (w_start < 0) w_start = 0;
                const uint8_t *rp = row + j - 1;
                uint8_t       *tp = trow + j - 1;
                while (rp >= row + w_start)
                {
                    if (*rp != Black) break;
                    *tp += 4;
                    rp--; tp--;
                }
            }

            /* 向上扫描，并用位 0 标记可连接的黑点 */
            if (row_above)
            {
                const int offset = image_w;
                const uint8_t *vp = &row_above[j];
                uint8_t       *tp = &imgTran[i - 1][j];
                int h_end = i - seek;
                if (h_end < 0) h_end = 0;
                int steps = i - h_end;
                while (steps--)
                {
                    if (*vp != Black) break;
                    *tp += 1;
                    vp -= offset;
                    tp -= offset;
                }
            }

            /* 向下扫描，并用位 1 标记可连接的黑点 */
            if (row_below)
            {
                const int offset = image_w;
                const uint8_t *vp = &row_below[j];
                uint8_t       *tp = &imgTran[i + 1][j];
                int h_start = i + seek;
                if (h_start > lim_row) h_start = lim_row;
                int steps = h_start - i;
                while (steps--)
                {
                    if (*vp != Black) break;
                    *tp += 2;
                    vp += offset;
                    tp += offset;
                }
            }
        }
    }

    /* 同时被左右或上下方向覆盖的黑点视为虚线间隙，并补成白点 */
    for (i = 0; i <= lim_row; i++)
    {
        const uint8_t *trow = imgTran[i];
        uint8_t       *prow = Pixle[i];
        for (j = 0; j <= lim_col; j++)
        {
            if (prow[j] == Black)
            {
                uint8_t val = trow[j];
                if ((val & 12) == 12 || (val & 3) == 3)
                    prow[j] = White;
            }
        }
    }

    /* 合并上下保护行中的白点，然后清空保护行 */
    {
        const uint8_t *g1_bot = Pixle[image_bottom + 1];
        const uint8_t *g2_bot = Pixle[image_bottom + 2];
        uint8_t       *bot    = Pixle[image_bottom];
        const uint8_t *g1_top = Pixle[image_top - 1];
        const uint8_t *g2_top = Pixle[image_top - 2];
        uint8_t       *top    = Pixle[image_top];

        for (j = 0; j < XX; j++)
        {
            if (g1_bot[j] == White || g2_bot[j] == White) bot[j] = White;
            if (g1_top[j] == White || g2_top[j] == White) top[j] = White;
        }
        memset(Pixle[image_bottom + 1], Black, image_w);
        memset(Pixle[image_bottom + 2], Black, image_w);
        memset(Pixle[image_top - 1],     Black, image_w);
        memset(Pixle[image_top - 2],     Black, image_w);
    }

    /* 合并左右保护列中的白点，然后清空保护列 */
    {
        const int row_end = image_bottom;
        const int il_1 = image_left - 1;
        const int il_2 = image_left - 2;
        const int ir_1 = image_right + 1;
        const int ir_2 = image_right + 2;

        for (i = 0; i <= row_end; i++)
        {
            uint8_t *row = Pixle[i];
            if (row[il_1] == White || row[il_2] == White) row[image_left]  = White;
            if (row[ir_1] == White || row[ir_2] == White) row[image_right] = White;
            row[il_1] = Black; row[il_2] = Black;
            row[ir_1] = Black; row[ir_2] = Black;
        }
    }

    return 0;
}

uint8 get_start_point(void)
{
    b_lost_tip = 0;
    start_center_y = 0;
    uint8 l_found = 0, r_found = 0;
    uint8 l_point = 0, r_point = 0;
    uint8_t Start_Black = 0;
    uint16_t area_left = 0;
    uint16_t area_right = 0;

    for(int i = image_bottom - 1; i >= (int)(image_bottom - image_h/4); i--)
    {
            /* 左右起点必须来自同一行，不能沿用上一行的查找结果。 */
            l_found = 0;
            r_found = 0;
            l_point = 0;
            r_point = 0;
            start_center_y = i;

            for (int j = image_left-1; j < image_right+1; j++)
            {
                if ((Pixle[start_center_y][j+1] == White) && Pixle[start_center_y][j] == Black)
                {
                    l_point = j;
                    l_found = 1;
                    break;
                }
            }

            for (int j = image_right+1; j > image_left-1; j--)
            {
                if ((Pixle[start_center_y][j-1] == White) && Pixle[start_center_y][j] == Black)
                {
                    r_point = j;
                    r_found = 1;
                    break;
                }
            }


        if(l_found && r_found)
        {
            for (uint8 i = l_point; i <= r_point; i++)
            {
                if (Pixle[start_center_y][i] == White)
                {
                    b_lost_tip++;
                    Pixle[Lost_Bottom][i] = Lost_line;
                }
                else if (start_center_y + 1 < image_h && Pixle[start_center_y + 1][i] == White)
                {
                    b_lost_tip++;
                    Pixle[Lost_Bottom][i] = Lost_line;
                }
                else if (Pixle[image_bottom][i] == White)
                {
                    b_lost_tip++;
                    Pixle[Lost_Bottom][i] = Lost_line;
                }
            }

            start_center_x = (l_point + r_point) / 2;
            start_point_l = l_point;
            start_point_r = r_point;
            return 1;
        }
    }
    return 0;
}

int my_abs(int value)
{
if(value>=0) return value;
else return -value;
}

/*********************************八邻域搜索变量*********************************/
uint8 points_l[(uint16)USE_num][2] = { {  0 } };//左线
uint8 points_r[(uint16)USE_num][2] = { {  0 } };//右线
uint8_t dir_r[(uint16)USE_num] = {0};
uint8_t dir_l[(uint16)USE_num] = {0};
uint32 l_data_statics = 0;//左边界已记录的点数
uint32 r_data_statics = 0;//右边界已记录的点数
uint8 hightest = 0;//最高点
int leftmost[2] ={0};//最左点记录
int rightmost[2] ={0};//最右点记录
int topmost[2] ={0};//最顶点记录

//左边变量
uint8 center_point_l[2] = {  0 };//左侧八邻域搜索的中心坐标
uint8 search_filds_l[8][2] = { {  0 } }; //左侧八邻域坐标
uint8 index_l = 0;//索引下标
uint8 temp_l[8][2] = { {  0 } };
// 左边界搜索方向：按逆时针顺序遍历八邻域
static const int8_t seeds_r[8][2] = { {0,-1},{-1,-1},{-1,0},{-1,1},{0,1},{1,1},{1,0},{1,-1} };
//{-1,+1},{0,+1},{+1,+1},   3  4  5
//{-1, 0},       {+1, 0},   2     6
//{-1,-1},{0,-1},{+1,-1},   1  0  7
// 先进入左侧黑色区域，再寻找黑白交界点

//右边变量
uint8 center_point_r[2] = { 0 };//右侧八邻域搜索的中心坐标
uint8 search_filds_r[8][2] = { {  0 } };//右侧八邻域坐标
uint8 index_r = 0;//索引下标
uint8 temp_r[8][2] = { {  0 } };
// 右边界搜索方向：按顺时针顺序遍历八邻域
static const int8_t seeds_l[8][2] = { {0,-1},{1,-1},{1,0},{1,1},{0,1},{-1,1},{-1,0},{-1,-1} };
static const int8_t trace_turn_offsets[8] = {0, 1, -1, 2, -2, 3, -3, 4};
static uint8_t trace_pending_lost_left[image_h] = {0};
static uint8_t trace_pending_lost_right[image_h] = {0};
static uint8_t trace_pending_lost_top[image_w] = {0};
//{-1,+1},{0,+1},{+1,+1},   5  4  3
//{-1, 0},       {+1, 0},   6     2
//{-1,-1},{0,-1},{+1,-1},   7  0  1
// 先进入右侧黑色区域，再寻找黑白交界点
/*********************************八邻域搜索变量*********************************/

static uint8_t trace_lost_marker_pending(int x, int y)
{
    if (x < 0 || x >= image_w || y < 0 || y >= image_h)
        return 0;

    if (x == Lost_Left && trace_pending_lost_left[y])
        return 1;
    if (x == Lost_Right && trace_pending_lost_right[y])
        return 1;
    if (y == Lost_Top && trace_pending_lost_top[x])
        return 1;

    return 0;
}

static void draw_boundary_segment(int x0, int y0, int x1, int y1, uint8 color)
{
    int dx = my_abs(x1 - x0);
    int sx = x0 < x1 ? 1 : -1;
    int dy = -my_abs(y1 - y0);
    int sy = y0 < y1 ? 1 : -1;
    int error = dx + dy;

    while (1)
    {
        if (x0 >= 0 && x0 < image_w && y0 >= 0 && y0 < image_h)
            Pixle[y0][x0] = color;

        if (x0 == x1 && y0 == y1)
            break;

        int error2 = error * 2;
        if (error2 >= dy) { error += dy; x0 += sx; }
        if (error2 <= dx) { error += dx; y0 += sy; }
    }
}

/******************************************************************************
* 函数名称     :uint8 search_l_r(uint8 start_l_x,uint8 start_l_y,uint8 start_r_x,uint8 start_r_y )
* 描述         :使用八邻域搜索左右边界，并记录边界点与丢线特征
* 进入参数     : start_l_x,start_l_y，左边界搜索起点
*               start_r_x,start_r_y，右边界搜索起点
*使用示例      :earch_l_r(start_point_l,start_center_y,start_point_r,start_center_y);
* 备注        :边界记录为黑色点部分
******************************************************************************/
/******************************************************************************/
uint8 search_l_r(uint8 start_l_x, uint8 start_l_y, uint8 start_r_x, uint8 start_r_y)
{
    const int IMG_LEFT  = image_left;
    const int IMG_RIGHT = image_right;
    const int IMG_TOP   = image_top;
    const int YY2       = YY / 2;

    hightest = image_h;
    leftmost[0]  = IMG_LEFT;
    rightmost[0] = IMG_RIGHT;
    topmost[1]   = 0;
    l_lost_tip = 0;
    r_lost_tip = 0;
    t_lost_tip = 0;
    l_data_statics = 0;
    r_data_statics = 0;
    memset(trace_pending_lost_left, 0, sizeof(trace_pending_lost_left));
    memset(trace_pending_lost_right, 0, sizeof(trace_pending_lost_right));
    memset(trace_pending_lost_top, 0, sizeof(trace_pending_lost_top));

    /* 八邻域访问会读取中心点四周一圈，起点必须位于图像内部。 */
    if (start_l_x == 0 || start_l_x >= image_w - 1 ||
        start_r_x == 0 || start_r_x >= image_w - 1 ||
        start_l_y == 0 || start_l_y >= image_h - 1 ||
        start_r_y == 0 || start_r_y >= image_h - 1)
    {
        return 1;
    }

    center_point_l[0] = start_l_x;  center_point_l[1] = start_l_y;
    center_point_r[0] = start_r_x;  center_point_r[1] = start_r_y;
    points_l[0][0] = start_l_x;     points_l[0][1] = start_l_y;
    points_r[0][0] = start_r_x;     points_r[0][1] = start_r_y;

    int clx = start_l_x, cly = start_l_y;
    int crx = start_r_x, cry = start_r_y;

    /* 随 y 坐标滑动缓存行指针，避免每次重新读取 Pixle[y] */
    const uint8_t *l_up  = Pixle[cly - 1];
    const uint8_t *l_cur = Pixle[cly];
    const uint8_t *l_dn  = Pixle[cly + 1];
    int l_prev_y = cly;

    const uint8_t *r_up  = Pixle[cry - 1];
    const uint8_t *r_cur = Pixle[cry];
    const uint8_t *r_dn  = Pixle[cry + 1];
    int r_prev_y = cry;

    /* points_*[0] 已保存起点，最多只能再记录 USE_num - 1 个点。 */
    uint32 break_flag = USE_num - 1;
    while (break_flag--)
    {
        /* 搜索左边界 */
        l_data_statics++;
        int lds = (int)l_data_statics;
        {
            /* 根据 y 坐标变化更新相邻行指针 */
            if (cly != l_prev_y) {
                if (cly == l_prev_y - 1) {
                    l_dn = l_cur; l_cur = l_up; l_up = Pixle[cly - 1];
                } else if (cly == l_prev_y + 1) {
                    l_up = l_cur; l_cur = l_dn; l_dn = Pixle[cly + 1];
                } else {
                    l_up  = Pixle[cly - 1];
                    l_cur = Pixle[cly];
                    l_dn  = Pixle[cly + 1];
                }
                l_prev_y = cly;
            }

            /* 预取八邻域像素，顺序为 N、NE、E、SE、S、SW、W、NW */
            uint8_t n0 = l_up[clx];       /* N  */
            uint8_t n1 = l_up[clx + 1];   /* NE */
            uint8_t n2 = l_cur[clx + 1];  /* E  */
            uint8_t n3 = l_dn[clx + 1];   /* SE */
            uint8_t n4 = l_dn[clx];       /* S  */
            uint8_t n5 = l_dn[clx - 1];   /* SW */
            uint8_t n6 = l_cur[clx - 1];  /* W  */
            uint8_t n7 = l_up[clx - 1];   /* NW */

            uint8_t neighbor[8] = {n0, n1, n2, n3, n4, n5, n6, n7};
            uint8_t last_dir = (lds >= 2) ? dir_l[lds - 2] : 0;
            int found = 0, best_x = 0, best_y = 0, best_dir = 0;

            for (int order = 0; order < 8; order++)
            {
                int candidate_dir = (last_dir + trace_turn_offsets[order] + 8) & 7;
                int next_dir = (candidate_dir + 1) & 7;
                int candidate_x = clx + seeds_l[candidate_dir][0];
                int candidate_y = cly + seeds_l[candidate_dir][1];

                if (candidate_x > 0 && candidate_x < image_w - 1 &&
                    candidate_y > 0 && candidate_y < image_h - 1 &&
                    (neighbor[candidate_dir] == Black || neighbor[candidate_dir] == Lost_line) &&
                    neighbor[next_dir] == White &&
                    !(lds >= 2 && candidate_x == points_l[lds - 2][0] &&
                                  candidate_y == points_l[lds - 2][1]))
                {
                    best_x = candidate_x;
                    best_y = candidate_y;
                    best_dir = candidate_dir;
                    found = 1;
                    break;
                }
            }

            if (found)
            {
                clx = best_x; cly = best_y;
                center_point_l[0] = (uint8)clx; center_point_l[1] = (uint8)cly;
                dir_l[lds - 1] = (uint8)best_dir;
            }
            else
            {
                int found2 = 0;
                for (int ny = cly - 1; ny >= IMG_TOP && ny >= cly - 5; ny--)
                {
                    const uint8_t *row = Pixle[ny];
                    int xs = clx - 3; if (xs < IMG_LEFT) xs = IMG_LEFT;
                    int xe = clx + 3; if (xe > IMG_RIGHT) xe = IMG_RIGHT;
                    for (int nx = xs; nx <= xe; nx++)
                    {
                        if ((row[nx] == Black || row[nx] == Lost_line) && row[nx + 1] == White)
                        {
                            clx = nx; cly = ny;
                            center_point_l[0] = (uint8)nx; center_point_l[1] = (uint8)ny;
                            dir_l[lds - 1] = (lds >= 2) ? dir_l[lds - 2] : 0;
                            found2 = 1; break;
                        }
                    }
                    if (found2) break;
                }
                if (!found2)
                    dir_l[lds - 1] = last_dir;
            }
        }

        /* 根据左侧搜索点的位置记录左右及顶部丢线特征 */
        {
            uint8_t *row = Pixle[cly];
            if (clx == IMG_LEFT - 1 && row[IMG_LEFT] == White &&
                row[Lost_Left] != Lost_line && !trace_lost_marker_pending(Lost_Left, cly))
            { l_lost_tip++; trace_pending_lost_left[cly] = 1; }
            else if (clx == IMG_RIGHT + 1 && row[IMG_RIGHT] == White &&
                     row[Lost_Right] != Lost_line && !trace_lost_marker_pending(Lost_Right, cly))
            { r_lost_tip++; trace_pending_lost_right[cly] = 1; }
            else if (cly == IMG_TOP - 1 && Pixle[IMG_TOP][clx] != Lost_line &&
                     !trace_lost_marker_pending(clx, Lost_Top))
            { t_lost_tip++; trace_pending_lost_top[clx] = 1; }

            if (cly < hightest) { hightest = cly; topmost[0] = clx; topmost[1] = cly; }
            if (clx < leftmost[0]) { leftmost[0] = clx; leftmost[1] = cly; }
        }

        /* 搜索右边界 */
        r_data_statics++;
        int rds = (int)r_data_statics;
        {
            /* 根据 y 坐标变化更新相邻行指针 */
            if (cry != r_prev_y) {
                if (cry == r_prev_y - 1) {
                    r_dn = r_cur; r_cur = r_up; r_up = Pixle[cry - 1];
                } else if (cry == r_prev_y + 1) {
                    r_up = r_cur; r_cur = r_dn; r_dn = Pixle[cry + 1];
                } else {
                    r_up  = Pixle[cry - 1];
                    r_cur = Pixle[cry];
                    r_dn  = Pixle[cry + 1];
                }
                r_prev_y = cry;
            }

            /* 预取八邻域像素，顺序为 N、NW、W、SW、S、SE、E、NE */
            uint8_t n0 = r_up[crx];       /* N  */
            uint8_t n1 = r_up[crx - 1];   /* NW */
            uint8_t n2 = r_cur[crx - 1];  /* W  */
            uint8_t n3 = r_dn[crx - 1];   /* SW */
            uint8_t n4 = r_dn[crx];       /* S  */
            uint8_t n5 = r_dn[crx + 1];   /* SE */
            uint8_t n6 = r_cur[crx + 1];  /* E  */
            uint8_t n7 = r_up[crx + 1];   /* NE */

            uint8_t neighbor[8] = {n0, n1, n2, n3, n4, n5, n6, n7};
            uint8_t last_dir = (rds >= 2) ? dir_r[rds - 2] : 0;
            int found = 0, best_x = 0, best_y = 0, best_dir = 0;

            for (int order = 0; order < 8; order++)
            {
                int candidate_dir = (last_dir + trace_turn_offsets[order] + 8) & 7;
                int next_dir = (candidate_dir + 1) & 7;
                int candidate_x = crx + seeds_r[candidate_dir][0];
                int candidate_y = cry + seeds_r[candidate_dir][1];

                if (candidate_x > 0 && candidate_x < image_w - 1 &&
                    candidate_y > 0 && candidate_y < image_h - 1 &&
                    (neighbor[candidate_dir] == Black || neighbor[candidate_dir] == Lost_line) &&
                    neighbor[next_dir] == White &&
                    !(rds >= 2 && candidate_x == points_r[rds - 2][0] &&
                                  candidate_y == points_r[rds - 2][1]))
                {
                    best_x = candidate_x;
                    best_y = candidate_y;
                    best_dir = candidate_dir;
                    found = 1;
                    break;
                }
            }

            if (found)
            {
                crx = best_x; cry = best_y;
                center_point_r[0] = (uint8)crx; center_point_r[1] = (uint8)cry;
                dir_r[rds - 1] = (uint8)best_dir;
            }
            else
            {
                int found2 = 0;
                for (int ny = cry - 1; ny >= IMG_TOP && ny >= cry - 5; ny--)
                {
                    const uint8_t *row = Pixle[ny];
                    int xs = crx - 3; if (xs < IMG_LEFT) xs = IMG_LEFT;
                    int xe = crx + 3; if (xe > IMG_RIGHT) xe = IMG_RIGHT;
                    for (int nx = xe; nx >= xs; nx--)
                    {
                        if ((row[nx] == Black || row[nx] == Lost_line) && row[nx - 1] == White)
                        {
                            crx = nx; cry = ny;
                            center_point_r[0] = (uint8)nx; center_point_r[1] = (uint8)ny;
                            dir_r[rds - 1] = (rds >= 2) ? dir_r[rds - 2] : 0;
                            found2 = 1; break;
                        }
                    }
                    if (found2) break;
                }
                if (!found2)
                    dir_r[rds - 1] = last_dir;
            }
        }

        /* 根据右侧搜索点的位置记录左右及顶部丢线特征 */
        {
            uint8_t *row = Pixle[cry];
            if (crx == IMG_RIGHT + 1 && row[IMG_RIGHT] == White &&
                row[Lost_Right] != Lost_line && !trace_lost_marker_pending(Lost_Right, cry))
            { r_lost_tip++; trace_pending_lost_right[cry] = 1; }
            else if (crx == IMG_LEFT - 1 && row[IMG_LEFT] == White &&
                     row[Lost_Left] != Lost_line && !trace_lost_marker_pending(Lost_Left, cry))
            { l_lost_tip++; trace_pending_lost_left[cry] = 1; }
            else if (cry == IMG_TOP - 1 && Pixle[Lost_Top][crx] != Lost_line &&
                     !trace_lost_marker_pending(crx, Lost_Top))
            { t_lost_tip++; trace_pending_lost_top[crx] = 1; }

            if (cry < hightest) { hightest = cry; topmost[0] = crx; topmost[1] = cry; }
            if (crx > rightmost[0]) { rightmost[0] = crx; rightmost[1] = cry; }
        }

        /* 记录本轮左右边界点 */
        points_l[lds][0] = (uint8)clx; points_l[lds][1] = (uint8)cly;
        points_r[rds][0] = (uint8)crx; points_r[rds][1] = (uint8)cry;

        /* 退出条件 1：连续三次得到同一个边界点 */
        uint8_t r_stalled = (rds >= 2 &&
             points_r[rds][0] == points_r[rds-1][0] && points_r[rds][0] == points_r[rds-2][0] &&
             points_r[rds][1] == points_r[rds-1][1] && points_r[rds][1] == points_r[rds-2][1]);
        uint8_t l_stalled = (lds >= 2 &&
             points_l[lds][0] == points_l[lds-1][0] && points_l[lds][0] == points_l[lds-2][0] &&
             points_l[lds][1] == points_l[lds-1][1] && points_l[lds][1] == points_l[lds-2][1]);
        if (l_stalled && r_stalled)
            break;

        /* 退出条件 2：左右搜索点已经相邻 */
        { int dx = crx - clx; int dy = cry - cly; if (dx >= -1 && dx <= 1 && dy >= -1 && dy <= 1) break; }

        /* 退出条件 3：搜索足够远后又回到起点附近 */
        if (!l_stalled && lds > YY2) {
            int dx = (int)start_l_x - clx; int dy = (int)start_l_y - cly;
            if (dx >= -1 && dx <= 1 && dy >= -1 && dy <= 1) break;
        }
        if (!r_stalled && rds > YY2) {
            int dx = (int)start_r_x - crx; int dy = (int)start_r_y - cry;
            if (dx >= -1 && dx <= 1 && dy >= -1 && dy <= 1) break;
        }
    }

    /* 搜索完成后再提交丢线标记，避免把白色赛道改色后破坏后续轮廓连通性。 */
    for (int y = 0; y < image_h; y++)
    {
        if (trace_pending_lost_left[y])
            Pixle[y][Lost_Left] = Lost_line;
        if (trace_pending_lost_right[y])
            Pixle[y][Lost_Right] = Lost_line;
    }
    for (int x = 0; x < image_w; x++)
    {
        if (trace_pending_lost_top[x])
            Pixle[Lost_Top][x] = Lost_line;
    }

    draw_boundary_segment(points_l[0][0], points_l[0][1],
                          points_l[0][0], points_l[0][1], Left_line);
    for (int i = 1; i <= (int)l_data_statics; i++)
        draw_boundary_segment(points_l[i-1][0], points_l[i-1][1],
                              points_l[i][0], points_l[i][1], Left_line);

    draw_boundary_segment(points_r[0][0], points_r[0][1],
                          points_r[0][0], points_r[0][1], Right_line);
    for (int i = 1; i <= (int)r_data_statics; i++)
        draw_boundary_segment(points_r[i-1][0], points_r[i-1][1],
                              points_r[i][0], points_r[i][1], Right_line);

    return 0;
}



void Get_start_center(void)
{
    if(b_lost_tip >= bottom_white_max && b_lost_num == 1)
    {
        uint32 max = 0, min = image_h, mid = image_h / 2;
        uint8 white_heigh[image_w] = {0};

        for(uint8 i = start_point_l; i <= start_point_r; i++)
        {
            if(Pixle[Lost_Bottom][i] == Lost_line)
            {
                white_heigh[i] = white_num_col[i];
                max = max > white_num_col[i] ? max : white_num_col[i];
                min = min < white_num_col[i] ? min : white_num_col[i];
            }
        }
        mid = ((max + min) * 2 + 1) / 3;  // 加 1 以减小整数除法的截断误差

        for(uint8 i = start_point_l; i <= start_point_r; i++)
        {
            if(Pixle[Lost_Bottom][i] == Lost_line && white_heigh[i] < mid)
            {
                b_lost_tip--;
                Pixle[Lost_Bottom][i] = Black;
            }
        }

        uint32 l_point = 0, r_point = 0;
        uint8 l_found = 0, r_found = 0;

        for (uint8 i = start_point_l; i < start_point_r; i++)
        {
            if (Pixle[Lost_Bottom][i] == Lost_line)
            {
                l_point = i;
                l_found = 1;
                break;
            }
        }
        for (int i = start_point_r; i > (int)start_point_l; i--)
        {
            if (Pixle[Lost_Bottom][i] == Lost_line)
            {
                r_point = i;
                r_found = 1;
                break;
            }
        }

        if(l_found && r_found)
            start_center_x = (l_point + r_point) / 2;
    }
    else if(b_lost_num == 2)
    {
        uint8 l_point = 0, r_point = image_right;
        uint8 l_found = 0, r_found = 0;

        for (uint8 i = start_point_l; i <= start_point_r; i++)
        {
            if (Pixle[Lost_Bottom][i] == Lost_line)
            {
                l_point = i;
                l_found = 1;
                break;
            }
        }
        for (int i = start_point_r; i >= (int)start_point_l; i--)
        {
            if (Pixle[Lost_Bottom][i] == Lost_line)
            {
                r_point = i;
                r_found = 1;
                break;
            }
        }

        if(l_found && r_found)
            start_center_x = (l_point + r_point) / 2;
    }
}//start_center_y






uint8 Get_top_straightline(void)
{
    if(Turn1_Flag2!=0||
            Turn1_Flag  !=0 ||
            b_lost_num  ==0 ||
            angle_flag_l!=0 ||
            angle_flag_r!=0 ||
            Turn2_Flag  !=0 ||
            Turn3_Flag  !=0)
    {
        return 0;
    }
    if( IF ==T_junction1_L ||  //5
        IF ==T_junction1_R ||  //6
        IF ==T_junction2_L || //7
        IF ==T_junction2_R ||//8
        IF ==T_junction1_T ||  //5
        IF ==crossroads_L  || //7
        IF ==crossroads_R  ||
        IF ==crossroads_T //7
         )  //9 )
    {
        return 0;
    }

    if(T1_flag == 1||T2_flag == 1)
    {
        return 0;
    }
    uint8 Black_num = 0;
    /**********************单个顶部特征区域处理**********************/
    if(t_lost_num == 1)
    {
//        顶部只有一个较宽的特征区域时，按列白点高度过滤较短部分
        if((t_lost_tip >= top_white_max  && t_lost_num == 1)||
            (t_lost_tip >= 2*top_white_max))
        {
            // 统计顶部丢线段各列的白点高度，并用高度中值进行筛选
            uint32 max = 0,min = image_h,mid = image_h/2;
            uint8 white_low[image_w-1] = {0};
            for(uint8 i = image_left; i <= image_right; i++)
            {
                if(Pixle[Lost_Top][i] == Lost_line)
                {
                 white_low[i] = white_num_col[i];
                 max = max > white_num_col[i] ? max : white_num_col[i];
                 min = min < white_num_col[i] ? min : white_num_col[i];
                }
            }
            mid = (max + min)/2;
            for(uint8 i = image_left; i <= image_right; i++)
            {
                if(Pixle[Lost_Top][i] == Lost_line && white_low[i] < mid)
                {
                    t_lost_tip--;
                    Pixle[Lost_Top][i] = White;
                }
            }

            uint32 l_point = 0,r_point = 0;
            end_center_y = image_top;
            for (uint8 i = image_left; i <= image_right; i++)
            {
                if (Pixle[Lost_Top][i] == Lost_line)
                {//找到左边起点
                    l_point = i;
                    break;
                }
            }
            //从右向左，先找黑白跳变点
            for (uint8 i = image_right; i >= image_left; i--)
            {
                if (Pixle[Lost_Top][i] == Lost_line)
                {//找到右边起点
                    r_point = i;//x
                    break;
                }
            }
            end_center_x = (l_point+r_point)/2;
         }
        else
        {
            end_center_y = image_top;
            end_center_x = t_center[0];
        }
        //通过顶点起点拟合直线
        if(end_center_y >= start_center_y)
        {
          return 0;
        }

        float x0 = (float)start_center_x, y0 = (float)start_center_y;
        float x2 = (float)end_center_x,   y2 = (float)end_center_y;
        float k  = (x0 - x2) / (y2 - y0);

        /* 第一步：统计候选直线路径上的非白像素，暂不绘制 */
        for(uint8 i = start_center_y; i >= end_center_y; i--)
        {
            uint8 x = (uint8)(x0 - k * (i - start_center_y));
            if(Pixle[i][x] != White)
                Black_num++;
        }

        if(Black_num == 0)
        {
            /* 直线 */
            for(uint8 i = start_center_y; i >= end_center_y; i--)
            {
                mid_line[i] = (float)(x0 - k * (i - start_center_y));
                Pixle[i][(uint8)mid_line[i]] = Control_line;
//                ips200_draw_point((uint8)mid_line[i], i + 120, RGB565_BLACK);
            }
        }
        else
        {
            /* 路径受阻时改用二阶贝塞尔曲线 */
            uint8 total_rows = start_center_y - end_center_y;
            float ratio = (float)Black_num / (float)total_rows;
            if(ratio > 1.0f) ratio = 1.0f;

            /* 非白像素比例越大，控制点越靠近 (x0, y2)，曲线弯曲越明显 */
            float cx = (x0 + x2) * 0.5f * (1.0f - ratio) + x0 * ratio;
            float cy = (y0 + y2) * 0.5f * (1.0f - ratio) + y2 * ratio;

            uint16_t steps = (uint16_t)total_rows * 3;
            for(uint16_t step = 0; step <= steps; step++)
            {
                float t  = (float)step / (float)steps;
                float mt = 1.0f - t;
                float bx = mt*mt*x0 + 2.0f*t*mt*cx + t*t*x2;
                float by = mt*mt*y0 + 2.0f*t*mt*cy + t*t*y2;
                uint8 row = (uint8)(by + 0.5f);
                if(row <= start_center_y && row >= end_center_y)
                {
                    mid_line[row] = bx;
                    Pixle[row][(uint8)bx] = Control_line;
//                    ips200_draw_point((uint8)bx, row + 120, RGB565_BLACK);
                }
            }
        }
        if(IF == T_junction1_L&&l_lost_num==1&&T_Turn1==NO)
        {
            IF = T_junction1_L;
        }
        else if(IF == T_junction1_R&&r_lost_num==1&&T_Turn1==NO)
        {
            IF = T_junction1_R;
        }
        else
        {
            IF = straightline;
        }
}
//    /**********************两个顶部特征区域的直线判定*********************
    else if(t_lost_num == 2)
    {
        end_center_y = image_top;
        uint32 t_center_l = t_center[0],t_center_r = t_center[1]; // 初始化左右顶部特征中心
        uint8 left_line[image_h] = {image_w/2};
        uint8 right_line[image_h] = {image_w/2};
        uint8 Black_num_l = 0,Black_num_r = 0;
        float kl = ((float)start_center_x - (float)t_center_l) / ((float)end_center_y - (float)start_center_y);
        for (uint8 i = start_center_y; i >= end_center_y; i--)
        {
            left_line[i] = (uint8)(start_center_x - kl * (i - start_center_y));
            if(Pixle[i][left_line[i]] != White)
            {
                Black_num_l++;
            }
            else
            {
              Pixle[i][left_line[i]] = Judge_line;
            }
        }
        float kr = (float)((float)start_center_x - (float)t_center_r) / ((float)end_center_y - (float)start_center_y);
        for (uint8 i = start_center_y; i >= end_center_y; i--)
        {
            right_line[i] = (uint8)(start_center_x - kr * (i - start_center_y));
            if(Pixle[i][right_line[i]] != White)
            {
                Black_num_r++;
            }
            else
            {
              Pixle[i][right_line[i]] = Judge_line;
            }
        }
        if(Black_num_l > Black_num_r)
        {
           end_center_x  = t_center_r;
           Black_num = Black_num_r;
           for (uint8 i = start_center_y; i >= end_center_y; i--)
           {
               mid_line[i] = right_line[i];
               Pixle[i][mid_line[i]] = Control_line;
           }
        }
        else
        {
            end_center_x  = t_center_l;
            Black_num = Black_num_l;
            for (uint8 i = start_center_y; i >= end_center_y; i--)
            {
                mid_line[i] = left_line[i];
                Pixle[i][mid_line[i]] = Control_line;
//                ips200_draw_point((uint8)mid_line[i],i+120,RGB565_BLACK);
            }
        }
        IF = straightline;
    }

    return end_center_y;
}



void Get_lost_tip(uint8 length)
{
    dis_Solidline = 0;
    b_lost_num = 0;
    t_lost_num = 0;
    l_lost_num = 0;
    r_lost_num = 0;
    // 清空各边缘特征段的中心位置
    memset(b_center, 0, sizeof(b_center));
    memset(l_center, 0, sizeof(l_center));
    memset(r_center, 0, sizeof(r_center));
    memset(t_center, 0, sizeof(t_center));
    /***********获取底部边缘特征***********/
    if(b_lost_tip)
    {
        for(uint8  i = image_left+1; i <= image_right-1; i++)   //从左到右检测紫色丢失线
        {
            if(Pixle[Lost_Bottom][i] == Lost_line)      // 检测到底部丢线标记
            {
                if(i <= image_left + length || i >= image_right - length  // 丢线段靠近左右边界
                   || (i + length <= XX && Pixle[Lost_Bottom][i+length] == Lost_line))  // 或丢线段长度达到阈值
                {
                    for(uint8 k = i; k <= XX; k++)
                    {
                        if(Pixle[Lost_Bottom][k] == Black)  //判断黑色为紫色丢失线尽头
                        {
                        b_center[b_lost_num] = ((uint32)k-1-(uint32)i)/2+(uint32)i;
                        b_lost_num ++;  //底部区域数量记录
                        i = k;
                        break;
                        }
                    }
                }
                else  // 删除长度不足的零散丢线标记
                {
                    for(uint8  k = i; k <= i + length; k++)
                    {
                        if(Pixle[Lost_Bottom][k] == Lost_line)
                        {
                        Pixle[Lost_Bottom][k] = Black;
                        b_lost_tip--;
                        }
                        else if(Pixle[Lost_Bottom][k] == Black)
                        {
                            break;
                        }
                    }
                }
            }
            if(b_lost_num == 2)
            {
                break;
            }
        }
    }
    if(t_lost_tip)
    {
        uint16 center[5] = {0};
        uint16 num = 0;
        uint8 i, j, k;

        /* 第一部分：处理顶部与左侧相连的丢线段
         * 检查 Lost_Left 列向下 5 个像素内是否存在 Lost_line；
         * 若存在，则清除 Lost_Top 行左端相连的丢线段。 */
        {
            uint8 left_conn = 0;
            for(j = Lost_Top; j <= Lost_Top + 5; j++)
            {
                if(Pixle[j][Lost_Left] == Lost_line)
                { left_conn = 1; break; }
            }
            if(left_conn)
            {
                for(k = Lost_Left; k <= Lost_Left + 5; k++)
                {
                    if(Pixle[Lost_Top][k] == Lost_line)
                    {
                        /* 向右清除连续段，直到遇到 Black */
                        while(k <= image_right && Pixle[Lost_Top][k] == Lost_line)
                        {
                            Pixle[Lost_Top][k] = Black;
                            t_lost_tip--;
                            k++;
                        }
                        break;
                    }
                }
            }
        }

        /* 第二部分：对称处理顶部与右侧相连的丢线段 */
        {
            uint8 right_conn = 0;
            for(j = Lost_Top; j <= Lost_Top + 5; j++)
            {
                if(Pixle[j][Lost_Right] == Lost_line)
                { right_conn = 1; break; }
            }
            if(right_conn)
            {
                for(k = Lost_Right; k >= Lost_Right - 5; k--)
                {
                    if(Pixle[Lost_Top][k] == Lost_line)
                    {
                        /* 向左清除连续段，直到遇到 Black */
                        while(k >= image_left && Pixle[Lost_Top][k] == Lost_line)
                        {
                            Pixle[Lost_Top][k] = Black;
                            t_lost_tip--;
                            k--;
                        }
                        break;
                    }
                }
            }
        }

        /* 第三部分：清除长度小于 length 的零散丢线段 */
        for(i = image_left; i <= image_right; i++)
        {
            if(Pixle[Lost_Top][i] == Lost_line)
            {
                uint8 seg_start = i;
                uint8 seg_len = 0;
                while(i <= image_right && Pixle[Lost_Top][i] == Lost_line)
                { seg_len++; i++; }
                if(seg_len < length)
                {
                    for(k = seg_start; k < seg_start + seg_len; k++)
                    {
                        Pixle[Lost_Top][k] = Black;
                        t_lost_tip--;
                    }
                }
            }
        }

        /* 第四部分：记录剩余有效丢线段的中心位置 */
        num = 0;
        for(i = image_left; i <= image_right; i++)
        {
            if(Pixle[Lost_Top][i] == Lost_line)
            {
                uint8 seg_start = i;
                while(i <= image_right && Pixle[Lost_Top][i] == Lost_line)
                    i++;
                uint8 seg_end = (uint8)(i - 1);
                center[num] = ((uint32)seg_end - (uint32)seg_start) / 2 + (uint32)seg_start;
                num++;
                if(num >= 5) break;
            }
        }

        /* ── 筛选：从image_w/2向两侧找最近的两个center ── */
        if(num > 2)
        {
            uint8 Get_center[2] = {0};
            t_lost_num = 0;
            for(k = 0; k <= image_w/2 - 1; k++)
            {
                for(j = 0; j < num; j++)
                {
                    if(center[j] == image_w/2 + k && t_lost_num < 2)
                    { Get_center[t_lost_num] = image_w/2 + k; t_lost_num++; }
                    if(t_lost_num >= 2) break;
                    if(image_w/2 + k != image_w/2 - k
                       && center[j] == image_w/2 - k && t_lost_num < 2)
                    { Get_center[t_lost_num] = image_w/2 - k; t_lost_num++; }
                }
                if(t_lost_num >= 2)
                {
                    t_center[0] = Get_center[0] < Get_center[1] ? Get_center[0] : Get_center[1];
                    t_center[1] = Get_center[0] > Get_center[1] ? Get_center[0] : Get_center[1];
                    break;
                }
            }
        }
        else
        {
            t_lost_num = num;
            t_center[0] = center[0];
            t_center[1] = (num > 1) ? center[1] : center[0];
        }
    }
    if(l_lost_tip)
    {
       for(uint8  i = image_bottom+1; i >= image_top-1; i--)  //从下到上检测紫色丢失线
       {
           if(Pixle[i][Lost_Left] == Lost_line)  // 检测到左侧丢线标记
           {
               if(i >= image_bottom - length)  //左侧边缘块过低，考虑底部连续，仅保留底部
               {
                   for(uint8 j = image_left; j <= image_left + length; j++)
                   {
                     if(Pixle[Lost_Bottom][j] == Lost_line)
                     {
                         Pixle[Lost_Bottom][Lost_Left] = Lost_line;
                         {
                            uint8 erase_i = i;
                            for(; erase_i >= image_bottom/2; erase_i--)
                            {
//                                if(Pixle[erase_i][Lost_Left] == Lost_line)
//                                {
//                                    Pixle[erase_i][Lost_Left] = Black;
//                                    l_lost_tip--;
//                                }
                                 if(Pixle[erase_i][Lost_Left] == Black)
                                {
                                    break;
                                }
                            }
                            i = erase_i;  // 更新扫描位置，避免外层 i-- 跳过下一段
                         }
                         break;
                     }
                   }
               }
               else  // 记录与底部不相连的正常丢线段
               {
                   uint8 k = i;                       /* k 为段起点，行号较大、位置靠下 */
                   for(; i >= Lost_Top; i--)
                   {
                     if(Pixle[i][Lost_Left] == Black)  /* i 为段终点外侧，行号较小、位置靠上 */
                     {
                         l_center[l_lost_num] = ((uint32)k - 1 - (uint32)i) / 2 + (uint32)i;
                         l_lost_num++;
                         break;
                     }
                   }
               }
           }
           if(l_lost_num == 2)
           {
               break;
           }
       }
    }
//    qout<<"l_lost_num"<<l_lost_num;
    /***********获取右部边缘特征***********/
    if(r_lost_tip)
    {
       for(uint8  i = image_bottom+1; i >= image_top-1; i--)  //从下到上检测紫色丢失线
       {
           if(Pixle[i][Lost_Right] == Lost_line)  // 检测到右侧丢线标记
           {
               if(i >= image_bottom - length)  //左侧边缘块过低，考虑底部连续，仅保留底部
               {
                   for(uint8 j = image_right; j <= image_right + length; j++)
                   {
                     if(Pixle[Lost_Bottom][j] == Lost_line)
                     {

                         Pixle[Lost_Bottom][Lost_Right] = Lost_line;
                         {
                            uint8 erase_i = i;
                            for(; erase_i >= image_bottom/2; erase_i--)
                            {
//                                if(Pixle[erase_i][Lost_Right] == Lost_line)
//                                {
//                                    Pixle[erase_i][Lost_Right] = Black;
//                                    r_lost_tip--;
//                                }
                               if(Pixle[erase_i][Lost_Right] == Black)
                                {
                                    break;
                                }
                            }
                            i = erase_i;  // 更新扫描位置，避免外层 i-- 跳过下一段
                         }
                         break;
                     }
                   }
               }
               else  // 记录与底部不相连的正常丢线段
               {
                   uint8 k = i;                        /* k 为段起点，行号较大、位置靠下 */
                   for(; i >= Lost_Top; i--)
                   {
                     if(Pixle[i][Lost_Right] == Black)  /* i 为段终点外侧，行号较小、位置靠上 */
                     {
                         r_center[r_lost_num] = ((uint32)k - 1 - (uint32)i) / 2 + (uint32)i;
                         r_lost_num++;
                         break;
                     }
                   }
               }
           }
           if(r_lost_num == 2)
           {
               break;
           }
       }
    }
    else
    {
      if(l_lost_tip + r_lost_tip == 0 && hightest <= image_top)
        {
           dis_Solidline = 1;
           int left_lost = 0,left_end = image_top;
           int right_lost = 0,right_end = image_top;
           /* 这里只检查 image_top 一行，直接使用 if 即可 */
           if(Pixle[image_top][image_left] == Left_line || Pixle[image_top][image_left] == Right_line)
           {
                   for(int k = image_top; k <= image_bottom; k++)
                   {
                       if(Pixle[k][image_left] != Black)
                       {
                           left_lost++;
                       }
                       else
                       {
                         if(k > image_h/2)
                         {
                           left_lost = 0;
                           break;
                         }
                         else
                         {
                           left_end = (image_top + k) / 2;
                           break;
                         }
                       }
                   }
           }
           for(int y = image_top;y <= image_bottom;y++)
           {

               if(Pixle[y][image_right] == Left_line || Pixle[y][image_right] == Right_line)
               {
                   for(int k = y;k <= image_bottom;k++)
                   {
                       if(Pixle[k][image_right] != Black)
                       {
                           right_lost ++;
                       }
                       else
                       {

                         if(k > image_h/2)
                         {
                           right_lost = 0;
                           break;
                         }
                         else
                         {
                           right_end = (y+k)/2;
                           break;
                         }
                       }
                   }
                   break;
               }
           }
           if(left_lost && !right_lost) // 补录左侧丢线特征
           {
               Pixle[left_end-1][Lost_Left] = Lost_line;
               Pixle[left_end][Lost_Left] = Lost_line;
               Pixle[left_end+1][Lost_Left] = Lost_line;
               l_center[0] = left_end;
               l_lost_num = 1;
           }
           else if(right_lost && !left_lost) // 补录右侧丢线特征
           {
               Pixle[right_end-1][Lost_Right] = Lost_line;
               Pixle[right_end][Lost_Right] = Lost_line;
               Pixle[right_end+1][Lost_Right] = Lost_line;
               r_center[0] = right_end;
               r_lost_num = 1;
           }
           else if(hightest <= image_top)// 补录顶部丢线特征
           {
           int x_mid = topmost[0];
           int x_l = topmost[0];
           int x_r = topmost[0];
           int y_mid = topmost[1];
           for(uint8 y = image_top;y <= image_bottom;y++)
           {
               if(Pixle[y][x_mid] == White)
               {
                   y_mid = y;
                   break;
               }
           }
           for(uint8 x = x_mid;x >= image_left;x--)
           {
               if(Pixle[y_mid][x] == White)
               {
                   Pixle[Lost_Top][x] = Lost_line;
               }
               else
               {
                   x_l = x;
                   break;
               }
           }
           for(uint8 x = x_mid;x <= image_right;x++)
           {
               if(Pixle[y_mid][x] == White)
               {
                   Pixle[Lost_Top][x] = Lost_line;
               }
               else
               {
                   x_r = x;
                   break;
               }
           }
           t_lost_num = 1;
           t_center[0] = (x_l+x_r)/2;
           }
        }
    }
   }


/**
 * @brief 获取单侧出界点向内5像素处的连续非黑线纵向宽度
 *
 * 左侧单独出界时，以l_center[0]为中心行，在Lost_Left右侧5像素处采样；
 * 右侧单独出界时，以r_center[0]为中心行，在Lost_Right左侧5像素处采样。
 * 从采样点分别向上、向下扫描，直到遇到Black或图像有效边界。
 * White、Lost_line、Left_line、Right_line及补线标记等非Black像素均计入宽度。
 *
 * @return 连续非黑线的纵向宽度；无单侧出界、坐标无效或采样点为Black时返回0
 */
uint8_t Get_Lost_Inward_NonBlack_Width(void)
{
    const uint8_t inward_offset = 5U;
    uint8_t sample_x;
    uint16_t center_y;

    /* T字路口1只允许一侧出界；双侧同时出界时无法确定要测量哪一侧。 */
    if(l_lost_num == 1U && r_lost_num == 0U)
    {
        if((uint16_t)Lost_Left + inward_offset > image_right)
        {
            return 0U;
        }
        sample_x = (uint8_t)(Lost_Left + inward_offset);
        center_y = l_center[0];
    }
    else if(r_lost_num == 1U && l_lost_num == 0U)
    {
        if(Lost_Right < (uint8_t)(image_left + inward_offset))
        {
            return 0U;
        }
        sample_x = (uint8_t)(Lost_Right - inward_offset);
        center_y = r_center[0];
    }
    else
    {
        return 0U;
    }

    if(center_y < image_top || center_y > image_bottom ||
       Pixle[center_y][sample_x] == Black)
    {
        return 0U;
    }

    uint8_t top_y = (uint8_t)center_y;
    uint8_t bottom_y = (uint8_t)center_y;

    while(top_y > image_top && Pixle[top_y - 1U][sample_x] != Black)
    {
        top_y--;
    }
    while(bottom_y < image_bottom && Pixle[bottom_y + 1U][sample_x] != Black)
    {
        bottom_y++;
    }

    return (uint8_t)(bottom_y - top_y + 1U);
}



///******************************************************************************
//* 函数名称     :uint8 Left_curve_line(void)
//* 描述        : 连接起始中心与左侧丢线中心；直线路径受阻时改用贝塞尔曲线
//* 进入参数     : void
//* 返回参数     : void
//* 使用实例     :Left_curve_line();
//*****************************************************************************
uint8 Left_curve_line(void)
{
    end_center_x = image_left;
    end_center_y = l_center[l_lost_num - 1];
    uint8 Black_num = 0;

    if(end_center_y >= start_center_y)
        return 0;

    float x0 = (float)start_center_x, y0 = (float)start_center_y;
    float x2 = (float)end_center_x,   y2 = (float)end_center_y;
    float k  = (x0 - x2) / (y2 - y0);

    /* 第一步：统计候选直线路径上的非白像素，暂不绘制 */
    for(uint8 i = start_center_y; i >= end_center_y; i--)
    {
        uint8 x = (uint8)(x0 - k * (i - start_center_y));
        if(Pixle[i][x] != White)
            Black_num++;
    }

    if(Black_num == 0)
    {
        /* ── Black_num=0：走直线（原始逻辑）── */
        for(uint8 i = start_center_y; i >= end_center_y; i--)
        {
            mid_line[i] = (float)(x0 - k * (i - start_center_y));
            Pixle[i][(uint8)mid_line[i]] = Control_line;
//            ips200_draw_point((uint8)mid_line[i], i + 120, RGB565_BLACK);
        }
    }
    else
    {
        /* 路径存在非白像素时使用二阶贝塞尔曲线 */
        uint8 total_rows = start_center_y - end_center_y;
        float ratio = (float)Black_num / (float)total_rows;
        if(ratio > 1.0f) ratio = 1.0f;

        // 非白像素比例越大，控制点越靠近 (x0, y2)，曲线弯曲越明显
        float cx = (x0 + x2) * 0.5f * (1.0f - ratio) + x0 * ratio;
        float cy = (y0 + y2) * 0.5f * (1.0f - ratio) + y2 * ratio;

        uint16_t steps = (uint16_t)total_rows * 3;
        for(uint16_t step = 0; step <= steps; step++)
        {
            float t  = (float)step / (float)steps;
            float mt = 1.0f - t;
            float bx = mt*mt*x0 + 2.0f*t*mt*cx + t*t*x2;
            float by = mt*mt*y0 + 2.0f*t*mt*cy + t*t*y2;
            uint8 row = (uint8)(by + 0.5f);
            if(row <= start_center_y && row >= end_center_y)
            {
                mid_line[row] = bx;
                Pixle[row][(uint8)bx] = Control_line;
//                ips200_draw_point((uint8)bx, row + 120, RGB565_BLACK);
            }
        }
    }

    IF = curve;
    return end_center_y;
}
///******************************************************************************
//* 函数名称     :void Right_curve_line(void)
//* 描述        : 连接起始中心与右侧丢线中心；直线路径受阻时改用贝塞尔曲线
//* 进入参数     : void
//* 返回参数     : void
//* 使用实例     :Right_curve_line();
//******************************************************************************/
uint8 Right_curve_line(void)
{
    end_center_x = image_right;
    end_center_y = r_center[r_lost_num - 1];
    uint8 Black_num = 0;

    if(end_center_y >= start_center_y)
        return 0;

    float x0 = (float)start_center_x, y0 = (float)start_center_y;
    float x2 = (float)end_center_x,   y2 = (float)end_center_y;
    float k  = (x0 - x2) / (y2 - y0);

    /* 第一步：统计候选直线路径上的非白像素，暂不绘制 */
    for(uint8 i = start_center_y; i >= end_center_y; i--)
    {
        uint8 x = (uint8)(x0 - k * (i - start_center_y));
        if(Pixle[i][x] != White)
            Black_num++;
    }

    if(Black_num == 0)
    {
        /* 路径全白时直接绘制直线 */
        for(uint8 i = start_center_y; i >= end_center_y; i--)
        {
            mid_line[i] = (float)(x0 - k * (i - start_center_y));
            Pixle[i][(uint8)mid_line[i]] = Control_line;
//            ips200_draw_point((uint8)mid_line[i], i + 120, RGB565_BLACK);
        }
    }
    else
    {
        /* 路径存在非白像素时使用二阶贝塞尔曲线 */
        uint8 total_rows = start_center_y - end_center_y;
        float ratio = (float)Black_num / (float)total_rows;
        if(ratio > 1.0f) ratio = 1.0f;

        float cx = (x0 + x2) * 0.5f * (1.0f - ratio) + x0 * ratio;
        float cy = (y0 + y2) * 0.5f * (1.0f - ratio) + y2 * ratio;

        uint16_t steps = (uint16_t)total_rows * 3;
        for(uint16_t step = 0; step <= steps; step++)
        {
            float t  = (float)step / (float)steps;
            float mt = 1.0f - t;
            float bx = mt*mt*x0 + 2.0f*t*mt*cx + t*t*x2;
            float by = mt*mt*y0 + 2.0f*t*mt*cy + t*t*y2;
            uint8 row = (uint8)(by + 0.5f);
            if(row <= start_center_y && row >= end_center_y)
            {
                mid_line[row] = bx;
                Pixle[row][(uint8)bx] = Control_line;
//                ips200_draw_point((uint8)bx, row + 120, RGB565_BLACK);
            }
        }
    }

    IF = curve;
    return end_center_y;
}

/*
uint8 Left_curve_line(void)
{
    end_center_x = image_left;
    end_center_y = l_center[l_lost_num -1];
    uint8 Black_num = 0;
    if(end_center_y >= start_center_y)
    {
      return 0;
    }

    float k = ((float)start_center_x - (float)end_center_x) / ((float)end_center_y - (float)start_center_y);
    for (uint8 i = start_center_y; i >= end_center_y; i--)
    {
        mid_line[i] = (float)(start_center_x - k * (i - start_center_y));
        if(Pixle[i][(uint8)mid_line[i]] != White)
        {
            Black_num++;
        }

        Pixle[i][(uint8)mid_line[i]] = Control_line;
        ips200_draw_point((uint8)mid_line[i],i+120,RGB565_BLACK);
    }

    IF = curve;
    return end_center_y;
}
uint8 Right_curve_line(void)
{
    end_center_x = image_right;
    end_center_y = r_center[r_lost_num - 1];
    uint8 Black_num = 0;
    if(end_center_y >= start_center_y)
    {
      return 0;
    }
    float k = ((float)start_center_x - (float)end_center_x) / ((float)end_center_y - (float)start_center_y);
    for (uint8 i = start_center_y; i >= end_center_y; i--)
    {
        mid_line[i] = (float)(start_center_x - k * (i - start_center_y));
        if(Pixle[i][(uint8)mid_line[i]] != White)
        {
            Black_num++;
        }

        Pixle[i][(uint8)mid_line[i]] = Control_line;
        ips200_draw_point((uint8)mid_line[i],i+120,RGB565_BLACK);
    }
    IF = curve;
    return end_center_y;
}
*/

void Top_Black_Count()
{
    Top_Black = 0;   // 每次统计前清零
    uint8 local_end_x = t_center[0];
    uint8 local_end_y = image_top;

    if(local_end_y >= start_center_y) return;  // 避免除零

    float k = ((float)start_center_x - (float)local_end_x)
            / ((float)local_end_y - (float)start_center_y);

    for (uint8 i = start_center_y; i >= local_end_y; i--)
    {
        uint8 x = (uint8)(start_center_x - k * (i - start_center_y));
        if(Pixle[i][x] != White)
        {
            Top_Black++;
        }
    }
}

void Flag_Init(void)
{
    IF = 0;
}

uint8 Deal_crossroads(uint8 Cross_Turn)//0 为直行，1 为左转，2 为右转
{
    T2_flag = 0;
    if( ((b_lost_num+t_lost_num+l_lost_num+r_lost_num) <= 3)&&Turn3_Flag==0)
    {
        return 0;
    }
    // 缺少底部或顶部特征时无法判断十字路口
    if(b_lost_num == 0 || t_lost_num == 0)
    {
        return 0;
    }
    if(Turn1_Flag!=0||Turn1_Flag2!=0||Turn2_Flag!=0)
    {
        return 0;
    }
    if(b_lost_tip > image_w/2) // 底部丢线范围过宽时放弃本次判断
    {
        return 0;
    }
    uint8_t LR_Change2 = my_abs(r_center[0]-l_center[0]);
    uint8_t Turn_Dist =(r_center[0]+l_center[0])/2;
    uint8_t prsent_l = LT_black_ratio();
    uint8_t prsent_r = RT_black_ratio();
    uint8_t prsent_t = BT_black_ratio();
    float judge_k = BT_K();
    uint8_t BT_black1 = 0;
    uint8_t BT_black2 = 0;
    //斜入十字
    if(Turn3_Flag==0)
    {
    if(
       b_lost_num==1&&
       t_lost_num==2&&
       ((l_lost_num==1&&r_lost_num==0)||
        (l_lost_num==0&&r_lost_num==1))
       )
    {
        BT_black1 = BT_black_ratio2(t_center[0]);
        BT_black2 = BT_black_ratio2(t_center[1]);
        if(BT_black1>BT_black2)
        {
            Get_Top_bezier_line(t_center[1]);
            T2_flag = 1;
            return 0;
        }
        else if(BT_black1<BT_black2)
        {
            Get_Top_bezier_line(t_center[0]);
            T2_flag = 1;
            return 0;
        }
    }

    if(b_lost_num==1
       &&t_lost_num==1
       &&l_lost_num==1
       &&r_lost_num==1
       &&(Turn_Dist<(Angele_Turn+more_dist3)
//       ||LR_Change2 > LR_Change
       ))
    {
        Get_BT_bezier_line();
        T2_flag = 1;
        return 0;
    }
//
    if((judge_k>=TLim_k||judge_k<=-TLim_k))
    {
        Get_BT_bezier_line();
        T2_flag = 1;
        return 0;
    }
    }

    //正入十字
    if((b_lost_num==1
        &&t_lost_num==1
        &&l_lost_num==1
        &&r_lost_num==1
        &&Turn_Dist>(Angele_Turn+more_dist3)
        &&prsent_l>70
        &&prsent_r>70
        &&prsent_t<25
        &&Turn3_Flag==0
       ))
    {
      DIST = 0;
      dist_L=0;
      dist_R=0;//ips
      if(Cross_Turn==C_Top)
      {
          Get_BT_bezier_line();
          IF=crossroads_T;
          return 0;
      }
      else if(Cross_Turn==C_Left)
      {
          Left_curve_line();
          IF=crossroads_L;
          return 0;
      }
      else if(Cross_Turn==C_Right)
      {
          Right_curve_line();
          IF=crossroads_R;
          return 0;
      }
    }
    if(Turn3_Flag==1)
    {
        if(l_lost_num!=0)
        {
            Left_curve_line();
            return 0;
        }
        if(l_lost_num==0&&t_lost_num!=0)
        {
            Get_BT_bezier_line();
            return 0;
        }
    }
    else if(Turn3_Flag==2)
    {
        if(r_lost_num!=0)
        {
            Right_curve_line();
            return 0;
        }
        if(r_lost_num==0&&t_lost_num!=0)
        {
            Get_BT_bezier_line();
            return 0;
        }
    }
    else if(Turn3_Flag==3)
    {
        Get_BT_bezier_line();
        return 0;
    }
    return 0;
}

uint8_t Deal_Left_Angle(void)
{
    if(Turn2_Flag!=0||
       Turn1_Flag2!=0||
       Turn1_Flag !=0||
       Turn3_Flag !=0||
       l_lost_num+r_lost_num>=2
       ||T1_flag == 1)
    {
        angle_flag_l = 0;
        angle_flag_r = 0;
        return 0;
    }
    if(angle_flag_l==1)
    {
        if((t_lost_num==1&&t_center[0]>image_w/4))
        {
            angle_flag_l = 0;
            angle_flag_r = 0;
        }
    }

    if(l_lost_num==1&&b_lost_num==1&&r_lost_num==0&&t_lost_num==0&&(IF==0||IF==1)&&angle_flag_l==0
            )
    {
                if(l_center[0]>(Angele_Turn+more_dist1))
                {
                angle_flag_l = 1;
                }
                else if(l_center[0] <= (Angele_Turn + more_dist1))
                {
                    uint8 scan_start = Angele_Turn + more_dist1 + 4;
                    uint8 scan_end   = scan_start + 11;
                    if(scan_end > image_bottom) scan_end = image_bottom;
                    uint8 found = 0;
                    uint8 left_b = 0, right_b = 0, cur_row = 0;

                    for(uint8 row = scan_start; row <= scan_end; row++)
                    {
                        /* 从左向右查找“黑-白-白”跳变，定位赛道左边界 */
                        left_b = 0;
                        for(uint8 col = image_left; col <= image_right - 2; col++)
                        {
                            if((Pixle[row][col]   == Black || Pixle[row][col]   == Left_line||Pixle[row][col]   == Right_line)
                               && Pixle[row][col+1] == White
                               && Pixle[row][col+2] == White)
                            {
                                left_b = col + 1;
                                break;
                            }
                        }

                        /* 从右向左查找“白-白-黑”跳变，定位赛道右边界 */
                        right_b = 0;
                        for(uint8 col = image_right; col >= image_left + 2; col--)
                        {
                            if(Pixle[row][col-2] == White
                               && Pixle[row][col-1] == White
                               && (Pixle[row][col]   == Black || Pixle[row][col]   == Left_line||Pixle[row][col]   == Right_line))
                            {
                                right_b = col - 1;
                                break;
                            }
                        }

                        /* 左右边界必须在同一行同时找到 */
                        if(left_b != 0 && right_b != 0)
                        {
                            cur_row = row;
                            found = 1;
                            break;
                        }
                    }

                    if(found)
                    {
                        uint8 end_x = (left_b + right_b) / 2;
                        Get_B_straight_line(end_x, cur_row);
                    }
                }
    }
//    ips200_show_uint(180,48,angle_flag_l,1);
    if(angle_flag_l==1)
    {
        if(l_lost_num!=0)
        {
            Left_curve_line();
            return 0;
        }
        else if(l_lost_num==0&&t_lost_num!=0)
        {
            Get_BT_bezier_line();
            return 0;
        }
        else if(l_lost_num==0&&t_lost_num==0&&r_lost_num!=0)
        {
//            Right_curve_line();
            return 0;
        }
    }
    return 1;
}

uint8_t Deal_Right_Angle(void)
{
    if(Turn2_Flag!=0||
       Turn1_Flag2!=0||
       Turn1_Flag !=0||
       Turn3_Flag !=0||
       l_lost_num+r_lost_num>=2
       ||T1_flag == 1)
    {
        angle_flag_l = 0;
        angle_flag_r = 0;
        return 0;
    }
//    ips200_show_uint(0,180,angle_flag_r,9);
    if(angle_flag_r==1)
    {
        if((t_lost_num==1&&t_center[0]<image_w*3/4))
        {
            angle_flag_l = 0;
            angle_flag_r = 0;
        }
    }
    if(r_lost_num==1&&b_lost_num==1&&l_lost_num==0&&t_lost_num==0&&(IF==0||IF==1)&&angle_flag_r ==0
            )
    {
                if(r_center[0]>(Angele_Turn+more_dist1))
                {
                angle_flag_r = 1;
                }
                else if(r_center[0] <= (Angele_Turn + more_dist1))
                {
//                    ips200_show_uint(180,0,11111,5);
                    uint8 scan_start = Angele_Turn + more_dist1+4;
                    uint8 scan_end   = scan_start + 11;
                    if(scan_end > image_bottom) scan_end = image_bottom;
                    uint8 found = 0;
                    uint8 left_b = 0, right_b = 0, cur_row = 0;

                    for(uint8 row = scan_start; row <= scan_end; row++)
                    {
                        /* 从左向右查找“黑-白-白”跳变，定位赛道左边界 */
                        left_b = 0;
                        for(uint8 col = image_left; col <= image_right - 2; col++)
                        {
//                            ips200_show_uint(96,32,3333,5);
                            if((Pixle[row][col]   == Black || Pixle[row][col]   == Left_line||Pixle[row][col]   == Right_line)
                               && Pixle[row][col+1] == White
                               && Pixle[row][col+2] == White)
                            {
                                left_b = col + 1;
                                break;
                            }
                        }

                        /* 从右向左查找“白-白-黑”跳变，定位赛道右边界 */
                        right_b = 0;
                        for(uint8 col = image_right; col >= image_left + 2; col--)
                        {
                            if(Pixle[row][col-2] == White
                               && Pixle[row][col-1] == White
                               && (Pixle[row][col]   == Black || Pixle[row][col]   == Left_line||Pixle[row][col]   == Right_line))
                            {
                                right_b = col - 1;
                                break;
                            }
                        }

                        /* 左右边界必须在同一行同时找到 */
                        if(left_b != 0 && right_b != 0)
                        {
                            cur_row = row;
                            found = 1;

                            break;
                        }
                    }

                    if(found)
                    {
//                        ips200_show_uint(180,0,11111,5);
                        uint8 end_x = (left_b + right_b) / 2;
                        Get_B_straight_line(end_x, cur_row);
                    }
                }
    }
    if(angle_flag_r==1)
    {
        if(r_lost_num!=0)
        {
            Right_curve_line();
            return 0;
        }
        else if(r_lost_num==0&&t_lost_num!=0)
        {
            Get_BT_bezier_line();
            return 0;
        }
        else if(r_lost_num==0&&t_lost_num==0&&l_lost_num!=0)
        {
//            Left_curve_line();
            return 0;
        }
    }
    return 1;
}

void Get_BT_bezier_line(void)
{
    if(b_lost_num == 0 || t_lost_num == 0)
        return;
    end_center_x = (float)t_center[0];
    end_center_y = image_top;    // 控制线终点位于图像顶部
    float x0 = (float)b_center[0],  y0 = (float)image_bottom;  /* P0 底部奇点 */
    float x2 = (float)t_center[0],  y2 = (float)image_top;     /* P2 顶部终点 */
    float k  = (x0 - x2) / (y2 - y0);                         /* 直线斜率    */

    uint8 Black_num = 0;
    uint8 total_rows = image_bottom - image_top;

    /* 第一步：统计底部与顶部中心连线上的非白像素，暂不绘制 */
    for(uint8 i = image_bottom; i >= image_top; i--)
    {
        uint8 x = (uint8)(x0 - k * (i - image_bottom));
        if(x < image_left)  x = image_left;
        if(x > image_right) x = image_right;
        if(Pixle[i][x] != White)
            Black_num++;
    }

    if(Black_num == 0)
    {
        /* 路径全白时直接绘制直线 */
        for(uint8 i = image_bottom; i >= image_top; i--)
        {
            float x = x0 - k * (i - image_bottom);
            if(x < image_left)  x = image_left;
            if(x > image_right) x = image_right;
            mid_line[i] = x;
            Pixle[i][(uint8)x] = Control_line;
//            ips200_draw_point((uint8)x, i + 120, RGB565_BLACK);
        }
    }
    else
    {
        /* 路径存在非白像素时使用贝塞尔曲线，非白点越多弯曲越明显 */
        float ratio = (float)Black_num / (float)total_rows;
        if(ratio > 1.0f) ratio = 1.0f;

        /* ratio=0 时控制点位于中点，曲线退化为直线；
         * ratio=1 时控制点位于 (x0, y2)，弯曲程度最大。 */
        float cx = (x0 + x2) * 0.5f * (1.0f - ratio) + x0 * ratio;
        float cy = (y0 + y2) * 0.5f * (1.0f - ratio) + y2 * ratio;

        uint16_t steps = (uint16_t)total_rows * 3;
        for(uint16_t s = 0; s <= steps; s++)
        {
            float t  = (float)s / (float)steps;
            float mt = 1.0f - t;
            float bx = mt*mt*x0 + 2.0f*t*mt*cx + t*t*x2;
            float by = mt*mt*y0 + 2.0f*t*mt*cy + t*t*y2;
            uint8 row = (uint8)(by + 0.5f);
            if(row > image_bottom || row < image_top) continue;
            uint8 col = (uint8)(bx + 0.5f);
            if(col < image_left)  col = image_left;
            if(col > image_right) col = image_right;
            mid_line[row] = (float)col;
            Pixle[row][col] = Control_line;
//            ips200_draw_point(col, row + 120, RGB565_BLACK);
        }
    }

    IF = straightline;
}

/* 计算顶部特征中心与左侧特征中心连线上的非白像素比例。
 * 返回 0~100 的整数百分比：0 表示全白，100 表示全为非白像素。
 */
uint8_t LT_black_ratio(void)
{
    if(l_lost_num == 0 || t_lost_num == 0)
        return 0;

    float x0 = (float)t_center[0],            y0 = (float)image_top;
    float x1 = (float)image_left,             y1 = (float)l_center[l_lost_num - 1];

    float dy = y1 - y0;   /* 正值：终点行号 > 起点行号 */
    float dx = x1 - x0;

    if(dy == 0.0f) return 0;

    float k = dx / dy;    /* y 每增加一行时 x 的增量 */

    uint8_t total = 0;
    uint8_t black = 0;

    /* 从顶部特征中心逐行扫描到左侧特征中心 */
    for(float y = y0; y <= y1; y += 1.0f)
    {
        float x  = x0 + k * (y - y0);
        uint8 col = (uint8)(x + 0.5f);
        uint8 row = (uint8)(y + 0.5f);
        if(col < image_left)  col = image_left;
        if(col > image_right) col = image_right;

        total++;
        if(Pixle[row][col] != White)
            black++;
    }

    if(total == 0) return 0;
    return (uint8_t)((uint16_t)black * 100u / total);
}
uint8_t RT_black_ratio(void)
{
    if(r_lost_num == 0 || t_lost_num == 0)
        return 0;

    float x0 = (float)t_center[0],            y0 = (float)image_top;
    float x1 = (float)image_right,            y1 = (float)r_center[r_lost_num - 1];

    float dy = y1 - y0;
    float dx = x1 - x0;

    if(dy == 0.0f) return 0;

    float k = dx / dy;

    uint8_t total = 0;
    uint8_t black = 0;

    for(float y = y0; y <= y1; y += 1.0f)
    {
        float x   = x0 + k * (y - y0);
        uint8 col = (uint8)(x + 0.5f);
        uint8 row = (uint8)(y + 0.5f);
        if(col < image_left)  col = image_left;
        if(col > image_right) col = image_right;

        total++;
        if(Pixle[row][col] != White)
            black++;
    }

    if(total == 0) return 0;
    return (uint8_t)((uint16_t)black * 100u / total);
}

uint8_t BT_black_ratio(void)
{
    if(b_lost_num == 0 || t_lost_num == 0)
        return 0;

    float x0 = (float)t_center[0],   y0 = (float)image_top;
    float x1 = (float)b_center[0],   y1 = (float)image_bottom;

    float dy = y1 - y0;
    float dx = x1 - x0;

    if(dy == 0.0f) return 0;

    float k = dx / dy;

    uint8_t total = 0;
    uint8_t black = 0;

    for(float y = y0; y <= y1; y += 1.0f)
    {
        float x   = x0 + k * (y - y0);
        uint8 col = (uint8)(x + 0.5f);
        uint8 row = (uint8)(y + 0.5f);
        if(col < image_left)  col = image_left;
        if(col > image_right) col = image_right;

        total++;
        if(Pixle[row][col] != White)
            black++;
    }

    if(total == 0) return 0;
    return (uint8_t)((uint16_t)black * 100u / total);
}

uint8_t BT_black_ratio2(uint8_t destination)
{
    if(b_lost_num == 0 || t_lost_num == 0)
        return 0;

    float x0 = (float)destination,   y0 = (float)image_top;
    float x1 = (float)b_center[0],   y1 = (float)image_bottom;

    float dy = y1 - y0;
    float dx = x1 - x0;

    if(dy == 0.0f) return 0;

    float k = dx / dy;

    uint8_t total = 0;
    uint8_t black = 0;

    for(float y = y0; y <= y1; y += 1.0f)
    {
        float x   = x0 + k * (y - y0);
        uint8 col = (uint8)(x + 0.5f);
        uint8 row = (uint8)(y + 0.5f);
        if(col < image_left)  col = image_left;
        if(col > image_right) col = image_right;

        total++;
        if(Pixle[row][col] != White)
            black++;
    }

    if(total == 0) return 0;
    return (uint8_t)((uint16_t)black * 100u / total);
}

/* 从底部特征中心绘制控制线到顶部目标列 x_top。
 * 用法：Get_Top_bezier_line(t_center[0]);  或指定任意目标列
 */
void Get_Top_bezier_line(uint8 x_top)
{
    end_center_x = x_top;
    end_center_y = image_top;    // 控制线终点位于图像顶部
    float x0 = (float)b_center[0],  y0 = (float)image_bottom; /* P0：底部特征中心 */
    float x2 = (float)x_top,       y2 = (float)image_top;  /* P2：顶部目标点 */

    if(y0 <= y2)   /* 起点已到顶或超出，无需补线 */
        return;

    float k = (x0 - x2) / (y2 - y0);   /* 斜率为 dx/dy，图像行号向下增大 */

    uint8 Black_num  = 0;
    uint8 total_rows = (uint8)(y0 - y2);

    /* 第一步：统计候选直线路径上的非白像素 */
    for(uint8 i = (uint8)y0; i >= (uint8)y2; i--)
    {
        uint8 x = (uint8)(x0 - k * (i - y0));
        if(x < image_left)  x = image_left;
        if(x > image_right) x = image_right;
        if(Pixle[i][x] != White)
            Black_num++;
    }

    if(Black_num == 0)
    {
        /* 全白：画直线 */
        for(uint8 i = (uint8)y0; i >= (uint8)y2; i--)
        {
            float x = x0 - k * (i - y0);
            if(x < image_left)  x = image_left;
            if(x > image_right) x = image_right;
            mid_line[i] = x;
            Pixle[i][(uint8)x] = Control_line;
//            ips200_draw_point((uint8)x, i + 120, RGB565_BLACK);
        }
    }
    else
    {
        /* 路径受阻时绘制二阶贝塞尔曲线，非白点越多弯曲越明显 */
        float ratio = (float)Black_num / (float)total_rows;
        if(ratio > 1.0f) ratio = 1.0f;

        /* ratio=0 时控制点位于中点，曲线退化为直线；
         * ratio=1 时控制点移至 (x0, y2)，以最大弯曲绕过障碍。 */
        float cx = (x0 + x2) * 0.5f * (1.0f - ratio) + x0 * ratio;
        float cy = (y0 + y2) * 0.5f * (1.0f - ratio) + y2 * ratio;

        uint16_t steps = (uint16_t)total_rows * 3;
        for(uint16_t s = 0; s <= steps; s++)
        {
            float t  = (float)s / (float)steps;
            float mt = 1.0f - t;
            float bx = mt*mt*x0 + 2.0f*t*mt*cx + t*t*x2;
            float by = mt*mt*y0 + 2.0f*t*mt*cy + t*t*y2;
            uint8 row = (uint8)(by + 0.5f);
            if(row > (uint8)y0 || row < (uint8)y2) continue;
            uint8 col = (uint8)(bx + 0.5f);
            if(col < image_left)  col = image_left;
            if(col > image_right) col = image_right;
            mid_line[row] = (float)col;
            Pixle[row][col] = Control_line;
//            ips200_draw_point(col, row + 120, RGB565_BLACK);
        }
    }
}

float BT_K(void)
{
    end_center_x = t_center[0];
    end_center_y = image_top;
    if(end_center_y >= start_center_y)
    {
      return 0;
    }
    float k = ((float)start_center_x - (float)end_center_x) / ((float)end_center_y - (float)start_center_y);
    return k;
}

uint8_t Deal_T_junction1(uint8 Turn)
{
        T1_flag = 0;
        uint8_t T3_Flag = 0;
        if(t_lost_tip>20)
        {
            return 0;
        }
        if(Turn2_Flag != 0||Turn3_Flag!=0)
        {
            return 0;
        }
        if((t_lost_num+r_lost_num+b_lost_num+l_lost_num)>=4&&t_lost_num==1)
        {
            return 0;
        }
        if((t_lost_num>=1)&&
           b_lost_num==1&&
           ((l_lost_num==1&&r_lost_num==0)||
           (l_lost_num==0&&r_lost_num==1))
           )
        {
//            uint8_t Turning_point_L = 0;
//            uint8_t Turning_point_R = 0;
            uint8_t prsent_l = LT_black_ratio();
            uint8_t prsent_r = RT_black_ratio();
            uint8_t prsent_t = BT_black_ratio();
            float judge_k = BT_K();


//            ips200_show_uint(0,80,prsent_l,3);
//            ips200_show_uint(64,80,prsent_r,3);
//            ips200_show_uint(128,80,prsent_t,3);
//            ips200_show_float(64,96,judge_k,3,3);
//            ips200_show_uint(128,96,Turning_point_R,3);

            Top_Black_Count();

//            if((l_lost_num==1&&(prsent_l>95||prsent_t<10)))
//            {
//                T3_Flag=1;
//            }
//            else if((r_lost_num==1&&(prsent_r>95||prsent_t<10)))
//            {
//                T3_Flag=1;
//            }

            if(Turn1_Flag==0&&Turn1_Flag2==0&&T3_Flag==0)
            {
               if(l_lost_num==1&&((prsent_t>38||prsent_l<60)||l_center[l_lost_num - 1]<Angele_Turn))//t0 l100
               {
                   if(l_center[l_lost_num - 1]<Angele_Turn)
                   Get_BT_bezier_line();
                   else
                   Get_oblique_Y_line_L();
                   T1_flag = 1;
                   return 0;
               }
               else if(r_lost_num==1&&((prsent_t>38||prsent_r<55)||r_center[r_lost_num - 1]<Angele_Turn))
               {
                   if(r_center[r_lost_num - 1]<Angele_Turn)
                   Get_BT_bezier_line();
                   else
                   Get_oblique_Y_line_R();
                   T1_flag = 1;
                   return 0;
               }

               if(t_lost_num==1&&(judge_k>=TLim_k||judge_k<=-TLim_k))
               {
                   Get_BT_bezier_line();
                   T1_flag = 1;
                   return 0;
               }
               if(t_white_tip>22||(l_white_tip>8&&l_center[l_lost_num - 1]<image_h/2)||(r_white_tip>8&&r_center[r_lost_num - 1]<image_h/2)
                       ||((white_row_max[0]>(image_w*1)
                       ||l_white_tip>=10
                       ||r_white_tip>=10
                       ||white_row_max[0]<(image_w*1/4)))
//                       ||boundary_line >= 4
                       )
               {
                   Get_BT_bezier_line();
                   T1_flag = 1;
                   return 0;
               }
               if((l_lost_num==1&&(r_lost_tip!=0||r2_white_tip!=0))||(r_lost_num==1&&(l_lost_tip!=0||l2_white_tip!=0)))
               {
                   Get_BT_bezier_line();
                   T1_flag = 1;
                   return 0;
               }
            }


            if(b_lost_num!=0&&(t_lost_num>=1)&&Turn1_Flag==0&&Turn1_Flag2==0)
            {
                if(l_lost_num==1&&l_center[0]>=Angele_Turn)
                {
                    if(Turn==0)
                    {
                        Get_top_straightline();
                        IF = T_junction1_T;
                    }
                    else if(Turn==1)
                    {
                        Left_curve_line();
                        IF = T_junction1_L;
                    }
                }
                else if(r_lost_num==1&&r_center[0]>=Angele_Turn)
                {
                    if(Turn==0)
                    {
                        Get_top_straightline();
                        IF = T_junction1_T;
                    }
                    else if(Turn==1)
                    {
                        Right_curve_line();
                        IF = T_junction1_R;
                    }
                }
            }
        }

        if(Turn1_Flag==1)
        {
            if(l_lost_num!=0)
            {
                Left_curve_line();
                return 0;
            }
            if(t_lost_num==1&&r_lost_num==1&&l_lost_num==0)
            {
//               Get_BT_bezier_line();
                return 0;
            }
            if(t_lost_num==2&&l_lost_num==0&&r_lost_num==0)
            {
            Get_Top_bezier_line(t_center[0]);
            }
            if(t_lost_num==1&&r_lost_num==0&&l_lost_num==0)
            {
                    Get_BT_bezier_line();
                    return 0;
            }

        }
        else if(Turn1_Flag==2)
        {
            if(r_lost_num!=0)
            {
                Right_curve_line();
                return 0;
            }
            if(t_lost_num==1&&l_lost_num==1&&r_lost_num==0)
            {
//                Get_BT_bezier_line();
                return 0;
            }
            if(t_lost_num==2&&l_lost_num==0&&r_lost_num==0)
            {
            Get_Top_bezier_line(t_center[1]);
            return 0;
            }
            if(t_lost_num==1&&r_lost_num==0&&l_lost_num==0)
            {
                    Get_BT_bezier_line();
                    return 0;
            }

        }
        else if(Turn1_Flag2==1)
        {
            if(t_lost_num!=0)
            {
                Get_BT_bezier_line();
                return 0;
            }
            if(t_lost_num==0&&l_lost_num==1&&r_lost_num==0)
            {
                Left_curve_line();
                return 0;
            }
            if(t_lost_num==0&&l_lost_num==1&&r_lost_num==1)
            {
                Left_curve_line();
                return 0;
            }
            if(t_lost_num==0&&l_lost_num==0&&r_lost_num==1)
            {
//                Right_curve_line();
            return 0;
            }
        }
        return 0;
}

void Get_B_straight_line(uint8 x_end, uint8 y_end)
{
    if(b_lost_num == 0)
        return;

    if(y_end >= image_bottom)
        return;

    end_center_x = x_end;
    end_center_y = y_end;

    float x0 = (float)b_center[0],  y0 = (float)image_bottom;
    float x2 = (float)x_end,        y2 = (float)y_end;
    float k  = (x0 - x2) / (y2 - y0);

    for(uint8 i = image_bottom; i >= y_end; i--)
    {
        float x = x0 - k * (i - image_bottom);
        if(x < image_left)  x = image_left;
        if(x > image_right) x = image_right;
        mid_line[i] = x;
        Pixle[i][(uint8)x] = Control_line;
    }

    IF = straightline;
}

void Get_oblique_B_line(void)
{
    if(l_lost_num == 0 || r_lost_num == 0 || b_lost_num == 0)
        return;

    /* 左右侧特征点 */
    float Lx = (float)image_left,              Ly = (float)l_center[l_lost_num - 1];
    float Rx = (float)image_right,             Ry = (float)r_center[r_lost_num - 1];
    float Bx = (float)b_center[b_lost_num - 1], By = (float)image_bottom;

    /* LR方向向量 */
    float dx = Rx - Lx;
    float dy = Ry - Ly;

    /* 垂直方向 n = (-dy, dx) */
    float px = -dy;
    float py =  dx;

    /* 令 py < 0，使法向量在图像坐标系中指向上方 */
    if(py > 0.0f) { px = -px; py = -py; }
    if(py == 0.0f) return;   /* 避免除零 */

    /* 行坐标每变化 1，列坐标按 k = px/py 偏移 */
    float k = px / py;

    end_center_y = image_top;
    end_center_x = (uint8)(Bx + k * (float)(image_top - (int)By));

    for(uint8 i = image_bottom; i >= end_center_y; i--)
    {
        float x = Bx + k * (float)(i - (int)By);
        if(x < (float)image_left)  x = (float)image_left;
        if(x > (float)image_right) x = (float)image_right;
        mid_line[i] = x;
        Pixle[i][(uint8)x] = Control_line;
    }
    IF = straightline;
}

void Get_oblique_Y_line_L(void)
{
    if(t_lost_num == 0 || l_lost_num == 0)
        return;

    /* 顶部与左侧特征点 */
    float Tx = (float)t_center[0],             Ty = (float)image_top;
    float Lx = (float)image_left,              Ly = (float)l_center[l_lost_num - 1];
    float Sx = (float)start_center_x,         Sy = (float)start_center_y;

    /* TL方向向量 */
    float dx = Lx - Tx;
    float dy = Ly - Ty;

    /* 垂直方向 n = (-dy, dx) */
    float px = -dy;
    float py =  dx;

    /* 令 py < 0，使法向量在图像坐标系中指向上方 */
    if(py > 0.0f) { px = -px; py = -py; }
    if(py == 0.0f) return;   /* 顶部与左侧特征点水平时避免除零 */

    /* 行坐标每变化 1，列坐标按 k = px/py 偏移 */
    float k = px / py;

    end_center_y = image_top;
    end_center_x = (uint8)(Sx + k * (float)(image_top - (int)Sy));

    for(uint8 i = start_center_y; i >= end_center_y; i--)
    {
        float x = Sx + k * (float)(i - (int)Sy);
        if(x < (float)image_left)  x = (float)image_left;
        if(x > (float)image_right) x = (float)image_right;
        mid_line[i] = x;
        Pixle[i][(uint8)x] = Control_line;
//        ips200_draw_point((uint8)x, i + 120, RGB565_BLACK);
    }
    IF = straightline;
}

void Get_oblique_Y_line_R(void)
{
    if(t_lost_num == 0 || r_lost_num == 0)
        return;

    /* 顶部与右侧特征点 */
    float Tx = (float)t_center[0],             Ty = (float)image_top;
    float Rx = (float)image_right,             Ry = (float)r_center[r_lost_num - 1];
    float Sx = (float)start_center_x,         Sy = (float)start_center_y;

    /* TR方向向量 */
    float dx = Rx - Tx;
    float dy = Ry - Ty;

    /* 垂直方向 n = (-dy, dx) */
    float px = -dy;
    float py =  dx;

    /* 令 py < 0，使法向量在图像坐标系中指向上方 */
    if(py > 0.0f) { px = -px; py = -py; }
    if(py == 0.0f) return;

    /* 行坐标每变化 1，列坐标按 k = px/py 偏移 */
    float k = px / py;

    end_center_y = image_top;
    end_center_x = (uint8)(Sx + k * (float)(image_top - (int)Sy));

    for(uint8 i = start_center_y; i >= end_center_y; i--)
    {
        float x = Sx + k * (float)(i - (int)Sy);
        if(x < (float)image_left)  x = (float)image_left;
        if(x > (float)image_right) x = (float)image_right;
        mid_line[i] = x;
        Pixle[i][(uint8)x] = Control_line;
//        ips200_draw_point((uint8)x, i + 120, RGB565_BLACK);
    }
    IF = straightline;
}
/* 计算底部特征中心与左侧特征中心连线上的非白像素比例。
 * 返回 0~100 的整数百分比：0 表示全白，100 表示全为非白像素。
 */
uint8_t BL_black_ratio(void)
{
    if(b_lost_num == 0 || l_lost_num == 0)
        return 0;

    float x0 = (float)b_center[0],            y0 = (float)image_bottom;
    float x1 = (float)image_left,             y1 = (float)l_center[l_lost_num - 1];

    float dy = y1 - y0;   /* 负值表示终点位于起点上方 */
    float dx = x1 - x0;

    if(dy == 0.0f) return 0;

    float k = dx / dy;

    uint8_t total = 0;
    uint8_t black = 0;

    /* 从底部特征中心向上扫描到左侧特征中心 */
    for(float y = y0; y >= y1; y -= 1.0f)
    {
        float x  = x0 + k * (y - y0);
        uint8 col = (uint8)(x + 0.5f);
        uint8 row = (uint8)(y + 0.5f);
        if(col < image_left)  col = image_left;
        if(col > image_right) col = image_right;

        total++;
        if(Pixle[row][col] != White)
            black++;
    }

    if(total == 0) return 0;
    return (uint8_t)((uint16_t)black * 100u / total);
}

uint8_t BR_black_ratio(void)
{
    if(b_lost_num == 0 || r_lost_num == 0)
        return 0;

    float x0 = (float)b_center[0],            y0 = (float)image_bottom;
    float x1 = (float)image_right,            y1 = (float)r_center[r_lost_num - 1];

    float dy = y1 - y0;   /* 负值表示终点位于起点上方 */
    float dx = x1 - x0;

    if(dy == 0.0f) return 0;

    float k = dx / dy;

    uint8_t total = 0;
    uint8_t black = 0;

    /* 从底部特征中心向上扫描到右侧特征中心 */
    for(float y = y0; y >= y1; y -= 1.0f)
    {
        float x  = x0 + k * (y - y0);
        uint8 col = (uint8)(x + 0.5f);
        uint8 row = (uint8)(y + 0.5f);
        if(col < image_left)  col = image_left;
        if(col > image_right) col = image_right;

        total++;
        if(Pixle[row][col] != White)
            black++;
    }

    if(total == 0) return 0;
    return (uint8_t)((uint16_t)black * 100u / total);
}

uint8_t Deal_T_junction2(uint8 Turn)
{
   if(Turn1_Flag2!=0||
      Turn1_Flag !=0||
      b_lost_num ==0||
      t_lost_num !=0||
      Turn3_Flag !=0
      )
   {
       return 0;
   }
   if((t_lost_num+r_lost_num+b_lost_num+l_lost_num)>=4)
   {
       return 0;
   }
   uint8_t R_blac_cnt = 0;
   uint8_t L_blac_cnt = 0;

   uint8_t LR_Change1 = my_abs(r_center[0]-l_center[0]);


   if(l_lost_num&&r_lost_num&&Turn2_Flag==0&&t_lost_num==0)
   {
       uint8_t Turn_Dist =(r_center[0]+l_center[0])/2;
       uint8_t R_blac_cnt = BR_black_ratio() ;
       uint8_t L_blac_cnt = BL_black_ratio() ;

       if(R_blac_cnt<35||L_blac_cnt<35)
       {
           return 0;
       }
           if(Turn_Dist>((Angele_Turn+more_dist2))&&LR_Change1<LR_Change)
           {
               if(Turn==0)
               {
                   Left_curve_line();
                   IF = T_junction2_L;
                   T1_flag = 0;
               }
               else if(Turn==1)
               {
                   Right_curve_line();
                   IF = T_junction2_R;
                   T1_flag = 0;
               }
           }
           else if((Turn_Dist<=(Angele_Turn+more_dist2))||LR_Change1>LR_Change)
           {
               Get_oblique_B_line();
               T1_flag = 1;
               return 0;
           }

   }

   if(Turn2_Flag==1)
   {
       if(l_lost_num!=0)
       {
           Left_curve_line();
           return 0;
       }
       else if(l_lost_num==0&&t_lost_num!=0)
       {
           Get_BT_bezier_line();
           return 0;
       }
       else if(l_lost_num==0&&t_lost_num==0&&r_lost_num!=0)
       {
           Right_curve_line();
           return 0;
       }
   }
   else if(Turn2_Flag==2)
   {
       if(r_lost_num!=0)
       {
           Right_curve_line();
           return 0;
       }
       else if(r_lost_num==0&&t_lost_num!=0)
       {
           Get_BT_bezier_line();
           return 0;
       }
       else if(r_lost_num==0&&t_lost_num==0&&l_lost_num!=0)
       {
           Left_curve_line();
           return 0;
       }
   }
   return 0;
}
void deviationestimate()
{
    float CenterSum=0;
    float CenterMeanValue=0;
    float WeightSum=0;
    float mid_offset = 0;
    if(start_center_y <= end_center_y)
    {
        Temp = 0;
        return;
    }

    for(int i = start_center_y; i >= (int)end_center_y; i--)
    {
          CenterSum+=mid_line[i]*quan_mid[i];
          WeightSum+=quan_mid[i];
    }
//Get_Top_bezier_line()
    if(Turn2_Flag==1||Turn1_Flag==1||Turn3_Flag==1)
    {
        mid_offset = gro_mid_offset2;
    }
    else if(Turn2_Flag==2||Turn1_Flag==2||Turn3_Flag==2)
    {
        mid_offset = -gro_mid_offset2;
    }
    else if(end_center_x==image_right||angle_flag_r==1)
    {
        mid_offset = -gro_mid_offset;
    }
    else if(end_center_x==image_left||angle_flag_l==1)
    {
        mid_offset = gro_mid_offset;
    }
    else if(IF == straightline&&angle_flag_r==0&&angle_flag_l==0)
    {
        mid_offset=0;
    }
     CenterMeanValue=(CenterSum/WeightSum);
     Temp=((image_w/2.0f)+mid_offset)-CenterMeanValue;

     if(Turn2_Flag==1||Turn1_Flag==1||Turn3_Flag==1)
     {
         if(Temp<LM_Temp)
         Temp=LM_Temp;
     }
     else if(Turn2_Flag==2||Turn1_Flag==2||Turn3_Flag==2)
     {
         if(Temp>-RM_Temp)
          {Temp=-RM_Temp;}
     }


     if(Temp>Lim_Temp)
      {Temp=Lim_Temp;}
     else if(Temp<-Lim_Temp)
      {Temp=-Lim_Temp;}
}

void Lost_find(void)
{
    if(b_lost_num!=0&&l_lost_num==0&&r_lost_num==0&&t_lost_num==0)
    {
        uint8 scan_start = lost_find_dist;
        uint8 scan_end   = lost_find_dist + image_h/3;
        if(scan_end > image_bottom) scan_end = image_bottom;
        uint8 found = 0;
        uint8 left_b = 0, right_b = 0, cur_row = 0;

        for(uint8 row = scan_start; row <= scan_end; row++)
        {
            /* 从左向右查找“黑-白-白”跳变，定位赛道左边界 */
            left_b = 0;
            for(uint8 col = image_left; col <= image_right - 2; col++)
            {
                if((Pixle[row][col]   == Black || Pixle[row][col]   == Left_line||Pixle[row][col]   == Right_line)
                   && Pixle[row][col+1] == White
                   && Pixle[row][col+2] == White)
                {
                    left_b = col + 1;
                    break;
                }
            }

            /* 从右向左查找“白-白-黑”跳变，定位赛道右边界 */
            right_b = 0;
            for(uint8 col = image_right; col >= image_left + 2; col--)
            {
                if(Pixle[row][col-2] == White
                   && Pixle[row][col-1] == White
                   && (Pixle[row][col]   == Black || Pixle[row][col]   == Left_line||Pixle[row][col]   == Right_line))
                {
                    right_b = col - 1;
                    break;
                }
            }

            /* 左右边界必须在同一行同时找到 */
            if(left_b != 0 && right_b != 0)
            {
                cur_row = row;
                found = 1;
                break;
            }
        }

        if(found)
        {
            uint8 end_x = (left_b + right_b) / 2;
            Get_B_straight_line(end_x, cur_row);
        }
    }
}

void get_whitetip()
{
    l2_white_tip = 0;
    r2_white_tip = 0;
    t_white_tip = 0;
    l_white_tip = 0;
    r_white_tip = 0;
    boundary_line = 0;
    for(uint8 col = image_left; col <= image_right ; col++)
    {
        if(Pixle[image_top][col] !=Black)
        {
            t_white_tip++;
        }
        if(Pixle[image_top][col] == Left_line || Pixle[image_top][col] == Right_line)
        {
            boundary_line ++;
        }
    }
    for(uint8 row = image_top; row <= image_bottom ; row++)
    {
        if(Pixle[row][image_left + 1] !=Black)
        {
            l_white_tip++;
        }
        if(Pixle[row][image_right - 1] !=Black)
        {
            r_white_tip++;
        }

        if(Pixle[row][image_left + 6] !=Black)
        {
            l2_white_tip++;
        }
        if(Pixle[row][image_right - 6] !=Black)
        {
            r2_white_tip++;
        }
    }
}

void TIPS(void)
{
    if(Turn1_Flag!=0||Turn1_Flag2!=0)
    {
    for(uint8 col = image_left; col <= image_left+5 ; col++)
    {
        Pixle[image_top][col]= 100;
    }
    }
    else if(Turn2_Flag!=0)
    {
        for(uint8 row = image_top; row <= image_top+5 ; row++)
        {
            Pixle[row][image_left ] = 100;
        }
    }
    else if(Turn3_Flag!=0)
    {
        for(uint8 row = image_top; row <= image_top+5 ; row++)
        {
            Pixle[row][image_left ]=100;
        }
        for(uint8 col = image_left; col <= image_left+5 ; col++)
        {
            Pixle[image_top][col]= 100;
        }
    }
}
void Line_Completion(void)
{
  if(road == 3)
  {
      if(Flag_Count ==15&&Flag_Count2 ==6 &&Flag_Count3 ==1)
      {
          if(t_lost_num!=0&&r_lost_num!=0&&l_lost_num!=0)
          {
              Right_curve_line();
          }
      }
  }
  if(road == 2)
  {
      if(Flag_Count ==10&&Flag_Count2 ==8 &&Flag_Count3 ==4)
      {
          if(t_lost_num!=0&&r_lost_num!=0&&l_lost_num!=0)
          {
              Right_curve_line();
          }
      }
  }
}

volatile uint8_t protect = 0;
uint8_t stop_Flag = 20;
uint8_t Limit_protection(uint8_t start_point_valid)
{
    /* 达到停车阈值后锁存，重新检测到白线也不再清零。 */
    if(protect >= stop_Flag)
    {
        return 1U;
    }

    if(start_point_valid != 0U)
    {
        protect = 0U;
        return 0U;
    }

    protect++;
    return (protect >= stop_Flag) ? 1U : 0U;
}

