#include "headfile.h"

PathPoint path_points[MAX_PATH_POINTS] = {0};

uint8 path_point_count = 0; // 第一个点为起点已经初始化为0,0
uint8 j = 1; // 用于移动路径点的索引
uint8 path_point_count_threshold = 100; // 路径点阈值
uint8 flag_isturn = 0; // 0表示上一段是直道,1表示上一段是弯道
uint8 flag_end = 0; // 0表示未到达终点,1表示到达终点
uint8 send_flag_nav = 0;
uint8 flag_fast_start = 0;

uint8 a = 0;

void Path_record(void)
{
    // 判断是否需要记录路径点 (角速度超过阈值)
    if (flag_isturn == 0) {
        if ((path_point_count < MAX_PATH_POINTS - 1) && (abs(imu660ra_gyro_z) / 16.4f > gyro_threshold_high)
             && fabs(yaw) >= angle_turn && encoder_ave > distance_min && fabs(aaddcc.err_dir) <= err_straight) {
            Point_record();
        }
    }
    else {
        if ((path_point_count < MAX_PATH_POINTS - 1) && (abs(imu660ra_gyro_z) / 16.4f < gyro_threshold_low)
             && encoder_ave > distance_min && fabs(aaddcc.err_dir) >= err_turn) {
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

void refresh(void)
{
    encoder_clear();
    yaw = 0;
}