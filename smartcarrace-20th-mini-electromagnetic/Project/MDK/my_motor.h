#ifndef __MY_MOTOR_H
#define __MY_MOTOR_H
#include "headfile.h"

extern uint8 huandao_count;

#define MINN(a, b) (((a) < (b)) ? (a) : (b))
#define MAXX(a, b) (((a) > (b)) ? (a) : (b))
#define MINMAX(input, low, upper) MINN(MAXX(input, low), upper)

// 定义蜂鸣器控制引脚
#define BEEP P67

// 编码器引脚定义
#define SPEEDL_PULSE CTIM0_P34 // 定义脉冲引脚
#define SPEEDR_PULSE CTIM3_P04
#define SPEEDL_DIR P35 // 定义方向引脚
#define SPEEDR_DIR P53
// 电机驱动引脚定义
#define LEFT_PWM PWMA_CH4P_P66
#define LEFT_DIR P64
#define RIGHT_PWM PWMA_CH2P_P62
#define RIGHT_DIR P60
// 定义正常速度

extern uint8 flag2;
extern uint16 cnt_circle_in;

// 电机速度环结构体
typedef struct
{
    int16 setspeed;
    int16 actspeed;
    int16 err;          // 当前误差
    int16 err1;         // 前一次误差
    int16 err2;         // 前两次误差
    int16 encoder_data; // 编码器值
    int16 duty1;        // 输出pwm占空比
    int16 out_p;
    int16 out_i;
    int16 out_d;
    float Kp_motor;
    float Ki_motor;
    float Kd_motor;
    float out_motor_pid;
} motor_struct;

void car_stop_judge();
void encoder_init(void);
void encoder_get(void);
void encoder();
void encoder_clear();
void motor_struct_parameter_init(motor_struct *sptr, int16 sspeed);
void dir_pid (float, float);
void dir_pid_sep (float, float);
void gyro_pd_control(void); // 新增角速度PD控制函数
void motor_driver_init_dr(void);
void motor_driver_init_ir(void);
void motor_driver_open_out(void);
int16 motor_closed_loop_control(motor_struct *sptr);
void motor_control(int16 speed_l, int16 speed_r);
void speed_change();

extern uint8 cout_test;/***************/
/***********速度决策*********************/
extern float adjust_speed_after_huandao ;
extern float adjust_speed_after_block ;
extern float adjust_speed_after_ramp ;
extern float adjust_speed_after_shizi ;
/****************************************/
extern float kp_motor; //电机闭环
extern float ki_motor;
extern float kd_motor;

extern int16 test_speed;

extern float kp_direction; // 方向环的pid
extern float kd_direction;
extern float kp_direction_2; // 方向环的pid
extern float kd_direction_2;
extern float kp_direction_3; // 方向环的pid
extern float kd_direction_3;

extern float kpa; //三次函数拟合方向环
extern float kpb;
extern float kd; // 两次误差之差
extern float kd_imu; // 陀螺仪

extern float kp_gyro; // 角速度环的pd
extern float kd_gyro;
extern float target_gyro_z; // 期望角速度
extern float gyro_err;      // 角速度环当前误差
extern float gyro_last_err; // 角速度环前一次误差

extern motor_struct motor_left, motor_right;
extern int16 set_leftspeed;
extern int16 set_rightspeed;

extern uint8 flag;
extern uint8 flag_stop;
extern uint8 flag_key_control;
extern uint8 flag_key_fast;
extern uint8 flag_start;

extern uint8 flag_huandao; 
extern float target_angle_in;
extern float target_angle_out;

extern float encoder_ave;
extern float encoder_temp;
extern int16 normal_speed;
extern int16 speed_huandao;
extern float huandao_hight_speed[4];
extern float huandao_low_speed[4];
extern float hightv_huandao;
extern float lowv_huandao;
extern float max_speed;
extern float block_out_encode;
extern float block_back_encode;
extern float block_judge; // tof障碍阈值
extern float block_speed;
extern float block_out_angle;
extern float block_back_angle;

extern float distance_before_huandao;
extern float distance_after_huandao;
extern float angle_in_threshold;
extern float angle_out_threshold;

extern float s;

extern float gyro_z;
#endif
