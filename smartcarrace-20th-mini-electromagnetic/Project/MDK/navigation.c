#include "headfile.h"

PathPoint path_points[MAX_PATH_POINTS] = {0};

uint16 path_point_count = 0; // 第一个点为起点已经初始化为0,0
uint16 j = 1; // 用于移动路径点的索引
uint16 path_point_count_threshold = 100; // 路径点阈值
uint8 flag_isturn = 0; // 0表示上一段是直道,1表示上一段是弯道
uint8 flag_end = 0; // 0表示未到达终点,1表示到达终点
uint8 send_flag_nav = 0;
uint8 flag_fast_start = 0;

float speed_high = 320;
float speed_low = 230;
float speed_turn = 160;
float speed_adjust = 200;

uint8 a = 0;

void Path_record(void)
{
    // 判断是否需要记录路径点 (角速度超过阈值)
    if (flag_isturn == 0) {
        if ((path_point_count < MAX_PATH_POINTS - 1) && (abs(imu660ra_gyro_z) / 16.4f > gyro_threshold_high)
             && fabs(yaw) >= angle_turn && encoder_ave > distance_min) {
            Point_record();
        }
    }
    else {
        if ((path_point_count < MAX_PATH_POINTS - 1) && (abs(imu660ra_gyro_z) / 16.4f < gyro_threshold_low)
             && encoder_ave > distance_min) {
            Point_record();
        }
    }

    // 调试用 记录路径点达到阈值时停止记录
    // if (path_point_count >= path_point_count_threshold) {
    //     flag = 4;
    //     set_leftspeed = 0;
    //     set_rightspeed = 0;
    //     normal_speed = 0;
    // }
}

void Point_record(void)
{
    path_point_count++;
    path_points[path_point_count].distance = encoder_ave;
    path_points[path_point_count].yaw_relative = yaw;
    path_points[path_point_count].isturn = flag_isturn;

    send_flag_nav = 1;

    // refresh();
    flag_isturn = !flag_isturn;
}

// 快速循迹函数 (第二圈使用)
void fast_tracking(void)
{
    // if (flag_fast_start == 0) {
    //     flag_fast_start = 1;
    //     refresh();
    //     delay_ms(3000);
    // }
    // a = 1;
    if (j + 1 < path_point_count) {
        if (encoder_ave >= path_points[j].distance) {
            // 移动到下一个路径点
            j++;
            // refresh();
        }
        speed_select(path_points[j].isturn, (path_points[j-1].distance + path_points[j].distance) / 2,
                     path_points[j].distance - path_points[j-1].distance, path_points[j].yaw_relative, path_points[j + 1].yaw_relative);
    }
    else {
        if (flag_end == 0) { // 第一次到达终点
            if (encoder_ave >= path_points[j].distance) {
                // 移动到下一个路径点
                j = 1;
                flag_end = 1;
                refresh();
            }
        }
        else {
            flag = 5;
        }
    }
}

void speed_select(uint8 isturn, float middle, float dis, float angle, float angle_next)
{
    // 仅区分直角
    // if (isturn == 1) // 弯道
    // {
    //     if (fabs(angle) >= 70)  normal_speed = speed_turn;
    //     else  normal_speed = speed_low;
    // }
    // else  normal_speed = speed_high;

    if (isturn == 1) // 弯道
    {
        if (fabs(angle) >= angle_90)  normal_speed = speed_turn;
        else  normal_speed = speed_low;
    }
    else // 直道
    {
        if (dis > distance_threshold) {// 直线距离达到50cm以上为长直线
            if (encoder_ave < middle)  normal_speed = speed_high;
            else {
                if (fabs(angle_next) >= angle_90)  normal_speed = speed_turn;
                else  normal_speed = speed_low;
            }
        }
        else {
            if (fabs(angle_next) >= angle_90)  normal_speed = speed_turn;
            else  normal_speed = speed_low;
        }
        
        // if (fabs(angle_next) >= angle_90)  normal_speed = speed_turn;
        // else {
        //     if (middle > distance_threshold / 2) {// 直线距离达到50cm以上
        //         if (encoder_ave < middle)  normal_speed = speed_high;
        //         else  normal_speed = speed_low;
        //     }
        // }
    }
}

void refresh(void) {
    encoder_clear();
    yaw = 0;
}

void write_path(void) {
    // 将路径数据写入 EEPROM，起始地址选择 0x200，避免与 my_peripheral.c 中参数区冲突
    const uint16 PATH_BASE_ADDR = 0x200;   // 第二页起始地址 (0x000-0x1FF 为参数区)
    uint16 addr = PATH_BASE_ADDR;
    uint16 i;

    // 计算最多可能用到的页并提前擦除，最多 500 个点，大约 10 kB
    iap_erase_page(0x200);   // 0x200 – 0x3FF
    iap_erase_page(0x400);   // 0x400 – 0x5FF
    iap_erase_page(0x600);   // 0x600 – 0x7FF
    iap_erase_page(0x800);   // 0x800 – 0x9FF
    iap_erase_page(0xA00);   // 0xA00 – 0xBFF
    iap_erase_page(0xC00);   // 0xC00 – 0xDFF
    iap_erase_page(0xE00);   // 0xE00 – 0xFFF
    iap_erase_page(0x1000);   // 0x1000 – 0x11FF
    iap_erase_page(0x1200);   // 0x1200 – 0x13FF
    iap_erase_page(0x1400);   // 0x1400 – 0x15FF
    iap_erase_page(0x1600);   // 0x1600 – 0x17FF
    iap_erase_page(0x1800);   // 0x1800 – 0x19FF
    iap_erase_page(0x1A00);   // 0x1A00 – 0x1BFF
    iap_erase_page(0x1C00);   // 0x1C00 – 0x1DFF
    iap_erase_page(0x1E00);   // 0x1E00 – 0x1FFF
    iap_erase_page(0x2000);   // 0x2000 – 0x21FF
    iap_erase_page(0x2200);   // 0x2200 – 0x23FF
    iap_erase_page(0x2400);   // 0x2400 – 0x25FF
    iap_erase_page(0x2600);   // 0x2600 – 0x27FF

    /*
     * 先存储路径点数量，方便读取复原
     * 采用 extern_iap_write_float 写入：整数位 3 位，小数 0 位
     * 长度 = num + pointnum + 3 = 3 + 0 + 3 = 6 字节
     */
    extern_iap_write_float((float)path_point_count, 3, 0, addr);
    addr += 6;

    /*
     * 依次写入各个路径点：
     * distance : 整数 5 位，小数 1 位  -> 5 + 1 + 3 = 9 字节
     * yaw      : 整数 3 位，小数 1 位  -> 3 + 1 + 3 = 7 字节
     * isturn   : 整数 1 位，小数 0 位  -> 1 + 0 + 3 = 4 字节
     * 总计每点 20 字节
     */
    for (i = 0; i < path_point_count; i++) {
        // 距离
        extern_iap_write_float((float)path_points[i].distance, 5, 1, addr);
        addr += 9;

        // 角度
        extern_iap_write_float((float)path_points[i].yaw_relative, 3, 1, addr);
        addr += 7;

        // 是否转弯标志（0/1）
        extern_iap_write_float((float)path_points[i].isturn, 1, 0, addr);
        addr += 4;
    }
}

void read_path(void) {
    const uint16 PATH_BASE_ADDR = 0x200;
    uint16 addr = PATH_BASE_ADDR;
    uint16 i;

    /* 读取路径点数量 */
    // path_point_count = (uint16)iap_read_float(6, addr);
    if (path_point_count > MAX_PATH_POINTS) {
        path_point_count = MAX_PATH_POINTS;
        flag_key_control = 0;
    }
    addr += 6;

    /* 按顺序读取各路径点数据 */
    for (i = 0; i < path_point_count; i++) {
        /* distance 5 整 1 小 : 9 字节 */
        path_points[i].distance = iap_read_float(9, addr);
        addr += 9;

        /* yaw 3 整 1 小 : 7 字节 */
        path_points[i].yaw_relative = iap_read_float(7, addr);
        addr += 7;

        /* isturn 1 整 0 小 : 4 字节 */
        path_points[i].isturn = (uint8)iap_read_float(4, addr);
        addr += 4;
    }
}