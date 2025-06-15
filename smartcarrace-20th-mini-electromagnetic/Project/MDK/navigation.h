#ifndef __NAVIGATION_H__
#define __NAVIGATION_H__

#include "headfile.h"

// 路径点结构体
typedef struct {
    float distance;     // 路段距离 (编码器计数值)
    float yaw_relative; // 转弯处的绝对航向角 (°)
    uint8 isturn;       // 是否为转弯点
} PathPoint;

// 路径记忆数组
#define MAX_PATH_POINTS 500
extern PathPoint path_points[MAX_PATH_POINTS];
extern uint16 path_point_count;
extern uint16 j;
extern uint16 path_point_count_threshold; // 路径点计数阈值调试用
extern uint8 flag_isturn;
extern uint8 flag_end;
extern uint8 send_flag_nav;
extern uint8 flag_fast_start;

extern uint8 a;

#define gyro_threshold_high 100
#define gyro_threshold_low 10

#define angle_turn 10
#define angle_90 70

#define err_straight 15
#define err_turn 30

#define distance_threshold 50 * distance_ratio  // 路径点距离阈值，用于判断是否为长直道
#define distance_min 15 * distance_ratio  // 最小路径点距离，用于筛除很短的路径点
#define distance_ratio 134 / 30  // 编码器计数值与实际距离的比例系数(计数值/实际距离cm)

extern float speed_high;
extern float speed_low;
extern float speed_turn;
extern float speed_adjust;

// 函数声明
void Path_record(void);
void Point_record(void);
void print_point();
void fast_tracking(void);
void speed_select(uint8 f_isturn, float middle, float dis,float angle, float angle_next);

void refresh(void);

extern void read_path(void);
extern void write_path(void);

#endif /* __NAVIGATION_H__ */
