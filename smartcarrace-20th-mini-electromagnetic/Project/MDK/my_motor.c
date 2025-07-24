#include "my_motor.h"
#include "pid.h"
//******************* 需要调的参数********************/
// 环岛
#define huandao_num 1 // 环岛个数
uint8 huandao_count = 0; // 环岛计数
uint8 huandao_directions[huandao_num] = {1};      // 0 表示左环岛, 1 表示右环岛
// float huandao_hight_speed[huandao_num] = {190, 170, 0, 0}; // 第n个环岛高速轮
// float huandao_low_speed[huandao_num] = {65, 75, 75, 0};   // 第n个环岛低速轮
uint16 huandao_r[huandao_num] = {250}; // 第n个环岛半径
// const uint16 r_out = 800; // 出环半径
float ratio = 0;
uint8 flag2 = 0;

const uint16 d = 165; // 车宽

float distance_before_huandao = 135;
float distance_after_huandao = 140;
float angle_in_threshold = 30; // 环岛入口角度阈值
float angle_out_threshold = 30; // 环岛出口角度阈值

// 速度
int16 normal_speed = 0;
int16 normal_speed_pre = 0;
int16 speed_huandao = 0;
int16 normal_speed_cal = 0;

// 旧方向环
float kp_direction = 5; // 方向环的pid
float kd_direction = 11;
float kp_direction_2 = 5;	
float kd_direction_2 = 11;
float kp_direction_3 = 5;
float kd_direction_3 = 11;

// 新方向环
float kpa = 22.0f; // 25 27
float kpb = 20.0f; // 25 30
float kd = 80.0f; // 100 110
float kd_imu = 0.0f;

// 角速度环
float kp_gyro = 1.3f;
float kd_gyro = 0.7f;
float target_gyro_z = 0.0f; // 期望角速度
float gyro_err = 0.0f;      // 角速度环当前误差
float gyro_last_err = 0.0f; // 角速度环前一次误差

// 速度环
float kp_motor = 18.0f;
float ki_motor = 9.0f;
float kd_motor = 0.0f;

uint8 flag = 0; // 0: 正常模式；1: 预环岛模式；2: 环岛模式；3: 出环调整；4: 障碍模式；5: 坡道模式
uint8 flag_stop = 0; // 0: 未停止；1: 停止
uint8 flag_key_control = 0; // 0：调参模式；1：跑车模式
uint8 flag_key_fast = 0; // 0：正常模式；1：快速模式
uint8 flag_start = 0; // 0: 未开始快速循迹；1: 开始快速循迹
uint8 cnt_start = 0;
uint8 cnt_stop = 0;

int16 test_speed = 0;       // setspeed
int16 changed_speed = 0;
int16 set_leftspeed = 0;
int16 set_rightspeed = 0;
// 编码器积分值
float encoder_left = 0.0;
float encoder_right = 0.0;
float encoder_ave = 0.0;
float encoder_temp = 0.0;

float lpf_encoder = 0.2; //编码器低通滤波系数
float lpf_motor = 0.1;   //电机低通滤波系数
float lpf_gyro = 0.2;   //陀螺仪低通滤波系数

uint8 flag_huandao = 0;     // 0表示左环岛，1表示右环岛
uint8 flag_set_angle = 0;
uint16 flag_circle_in = 0, cnt_circle_in = 0;
uint16 flag_circle_out = 0, cnt_circle_out = 0;

float target_angle_in = 0;
float target_angle_out = 0;
motor_struct motor_left, motor_right; // 定义电机速度闭环变量

float k;
float s = 0;

float gyro_z;

uint8 time = 0;

void speed_change()
{
    if (flag == 0)
        car_stop_judge();
    if (flag_stop == 0)
    {
        // if (flag_key_fast == 1)  fast_tracking();
        // else
        //     if (normal_speed != 0)  Path_record();
        
        if (imu660ra_gyro_z <= 4 && imu660ra_gyro_z >= -4)
            imu660ra_gyro_z = 0;
        gyro_z = gyro_z * lpf_gyro + (float)imu660ra_gyro_z / 16.4f * (1 - lpf_gyro); // 陀螺仪低通滤波

        switch (flag)
        {
        case 0: // 正常模式
//            if(fabs(aaddcc.err_dir) < 1 && fabs(aaddcc.last_err_dir) < 1 && !(aaddcc.err_dir ==0 && aaddcc.last_err_dir ==0 ))
           				// test_speed += 0.001;
            // if (time == 0)
                dir_pid(aaddcc.err_dir, aaddcc.last_err_dir, gyro_z); // 方向环输出期望角速度target_gyro_z
            // gyro_pd_control(); // 角速度环计算changed_speed
            
            // time = (time + 1) % 2;

            // 速度策略
            // if (flag_start && cnt_start < 100) {
            if (normal_speed_pre == 0 && normal_speed != 0)
                flag_start = 1;
            
            if (flag_start && cnt_start < 100) {
                cnt_start++;
                // normal_speed_cal = (int16)normal_speed / (100 * 100) * cnt_start * cnt_start;
                normal_speed_cal = (int16)((float)normal_speed * cnt_start / 100.0f);
            }
            else {
                flag_start = 0;
                cnt_start = 0;
                normal_speed_cal = (int16)-s * aaddcc.err_dir * aaddcc.err_dir + normal_speed;
            }

            normal_speed_pre = normal_speed;
            test_speed = (int16)normal_speed_cal;

            if (flag_key_fast == 1) {
                // 直道和弯道不同的差速限幅
                if (path_points[j].isturn == 1) {
                    changed_speed = MINMAX(changed_speed, -100, 100); // 角速度环输出的changed_speed也需要限幅
                
                    // 加少减多
                    k = fabs(aaddcc.err_dir / 50.0f);
                    if (changed_speed > 0) {
                        set_leftspeed = test_speed - changed_speed * (1 + k);
                        set_rightspeed = test_speed + changed_speed;
                    }
                    else {
                        set_leftspeed = test_speed - changed_speed;
                        set_rightspeed = test_speed + changed_speed * (1 + k);
                    }

                    set_leftspeed = MINMAX(set_leftspeed, -100, 400);
                    set_rightspeed = MINMAX(set_rightspeed, -100, 400);
                }
                else {
                    changed_speed = MINMAX(changed_speed, -300, 300); // 角速度环输出的changed_speed也需要限幅

                    set_leftspeed = test_speed - changed_speed;
    			    set_rightspeed = test_speed + changed_speed;

                    set_leftspeed = MINMAX(set_leftspeed, -100, 600);
                    set_rightspeed = MINMAX(set_rightspeed, -100, 600);
                }
            }
            else {
                changed_speed = MINMAX(changed_speed, -110, 110); // 角速度环输出的changed_speed也需要限幅
                
                //加少减多
                k = fabs(aaddcc.err_dir / 100.0f);
                if (changed_speed > 0) {
                    set_leftspeed = test_speed - changed_speed * (1 + k);
                    set_rightspeed = test_speed + changed_speed;
                }
                else {
                    set_leftspeed = test_speed - changed_speed;
                    set_rightspeed = test_speed + changed_speed * (1 + k);
                }

                // set_leftspeed = test_speed - changed_speed;
                // set_rightspeed = test_speed + changed_speed;

                set_leftspeed = MINMAX(set_leftspeed, -100, 500);
                set_rightspeed = MINMAX(set_rightspeed, -100, 500);
            }
			
            break;
        case 1: // 预环岛模式
            // if (AD_ONE[0] > AD_ONE[4])
            //     flag_huandao = 0; // 左环岛
            // else
            //     flag_huandao = 1; // 右环岛
            flag_huandao = huandao_directions[huandao_count];
            
            if (encoder_ave - encoder_temp < distance_before_huandao) { //没到环岛交点
				//直行
				// set_leftspeed = normal_speed;
				// set_rightspeed = normal_speed;
                set_leftspeed = 0;
                set_rightspeed = 0;
            }
            else  flag = 2;
            break;

        case 2: // 环岛模式
            if (flag_set_angle == 0) { // 第一次进入
                target_angle_in = yaw;
                flag_set_angle = 1;
            }

            // 环岛差速计算
            ratio = (float)(huandao_r[huandao_count] - (d/2)) / (float)(huandao_r[huandao_count] + (d/2));

            if (flag_huandao == 0) { // 左环岛
                target_angle_out = target_angle_in - 350; // 目标出环角度

                set_leftspeed = (int16)(normal_speed * ratio);
                set_rightspeed = normal_speed;

                if (yaw < target_angle_out) {
                    flag = 3; // 出环
                    encoder_temp = encoder_ave;
                }
            }
            else { // 右环岛
                target_angle_out = target_angle_in + 350; // 目标出环角度
                
                set_leftspeed = normal_speed;
                set_rightspeed = (int16)(normal_speed * ratio);

                if (yaw > target_angle_out) {
                    flag = 3; // 出环
                    encoder_temp = encoder_ave;
                }
            }
            break;

        case 3: // 出环
            if (encoder_ave - encoder_temp <= distance_after_huandao) {
                // if (flag_huandao == 0) { // 左环岛
                //     set_leftspeed = normal_speed * (r_out - (d/2)) / (r_out + (d/2));
                //     // set_leftspeed = normal_speed * 0.9;
				//     set_rightspeed = normal_speed;
                // }
				// else { // 右环岛
                //     set_leftspeed = normal_speed;
				//     set_rightspeed = normal_speed * (r_out - (d/2)) / (r_out + (d/2));
                //     // set_rightspeed = normal_speed * 0.9;
				// }
                set_leftspeed = normal_speed;
                set_rightspeed = normal_speed;
			}
			else {
				// 恢复到正常循迹
				flag = 0;
				flag1 = 0;
                flag_huandao = 0;
                // 各种标志位清零
				flag_set_angle = 0;
				
				// cnt_circle_in = 0;
				flag_circle_in = 0;
				// cnt_circle_out = 0;
				flag_circle_out = 0;

                huandao_count = (huandao_count + 1) % huandao_num; // 切换环岛
            }
            break;

        case 4: // 起步发车
            if (encoder_ave >= 133)  flag = 0;

            set_leftspeed = normal_speed;
            set_rightspeed = normal_speed;

            break;

        case 5: // 慢速停车
            if (cnt_stop < 200) {
                cnt_stop++;
                normal_speed_cal = (int16)normal_speed / 200 * (200 - cnt_stop);
                set_leftspeed = normal_speed_cal;
                set_rightspeed = normal_speed_cal;
            }
            else {
                cnt_stop = 0;
                normal_speed = 0;
                set_leftspeed = 0;
                set_rightspeed = 0;
                flag_key_control = 0;
            }

        default:
            break;
        }
    }
}

//脱线保护
void car_stop_judge() {
	if (AD_ONE[0] < 0.5 && AD_ONE[1] < 0.5 && AD_ONE[3] < 0.5 && AD_ONE[4] < 0.5) {
        set_leftspeed = 0;
        set_rightspeed = 0;
		normal_speed = 0;
        // flag_stop = 1;
    }
}

//****************************************
// 函数简介 编码器获得速度值
// 参数说明 void
// 参数说明 无
// 返回参数 无
// 使用示例 encoder_get();
// 前进时两个值都为正值,赋值给motor_left.encoder_data和motor_right.encoder_data
//****************************************
void encoder_get(void) {
    motor_left.encoder_data = (int16)ctimer_count_read(SPEEDL_PULSE);
    motor_right.encoder_data = (int16)ctimer_count_read(SPEEDR_PULSE);
//	motor_left.encoder_data = motor_left.encoder_data * lpf_encoder + (int16)ctimer_count_read(SPEEDL_PULSE) * (1.0f - lpf_encoder);
//	motor_right.encoder_data = motor_right.encoder_data * lpf_encoder + (int16)ctimer_count_read(SPEEDR_PULSE) * (1.0f - lpf_encoder);

    ctimer_count_clean(SPEEDL_PULSE);
    ctimer_count_clean(SPEEDR_PULSE);

    if (SPEEDL_DIR == 1) //观察屏幕输出调整
        motor_left.encoder_data = -motor_left.encoder_data;
    if (SPEEDR_DIR == 0)
        motor_right.encoder_data = -motor_right.encoder_data;
}

void encoder() {
    encoder_left += 0.017 * motor_left.encoder_data;
    encoder_right += 0.017 * motor_right.encoder_data; // 0.017怎么算出来的
    encoder_ave = (encoder_left + encoder_right) / 2;
}

void encoder_clear() {
    encoder_left = 0.0;
    encoder_right = 0.0;
    encoder_ave = 0.0;
}

void dir_pid (float error, float last_error, float gyro) {
    int16 p_out, d_out, output;
    
    p_out = (int16)((kpa / 10) * error + (kpb / 10000) * error * error * error);

	d_out = (int16)(kd * (error - last_error)) + (int16)(kd_imu / 100.0 * gyro);
    output = p_out + d_out;

    // target_gyro_z = -(float)output; // 方向环输出期望角速度
    changed_speed = output;
}

void dir_pid_sep (float error, float last_error) {
    int16 output;
	
    if (fabs(error) <= 20)
        output = kp_direction * error + kd_direction * (error - last_error);
    else if (fabs(error) > 20 && fabs(error) <= 40)
             output = kp_direction_2 * error + kd_direction_2 * (error - last_error);
    else
        output = kp_direction_3 * error + kd_direction_3 * (error - last_error);
	
    changed_speed = output;
}

//****************************************
// 函数简介 角速度PD闭环控制
// 参数说明 void
// 参数说明 无
// 返回参数 无
// 使用示例 gyro_pd_control();
//****************************************
void gyro_pd_control(void) {
    gyro_last_err = gyro_err;
    gyro_err = target_gyro_z - gyro_z; // 期望角速度 - 实际角速度

    changed_speed = (int16)(-kp_gyro * gyro_err - kd_gyro * (gyro_err - gyro_last_err));
}

//****************************************
// 函数简介 电机开环输出
// 参数说明 void
// 参数说明 无
// 返回参数 无
// 使用示例 motor_driver_open_out_ir();
//****************************************
void motor_driver_open_out_ir(void) {
    if (motor_left.duty1 >= 0) {
        pwm_duty(PWMA_CH1P_P60, (uint32)motor_left.duty1);
		pwm_duty(PWMA_CH3P_P64, 0);
    }
    else {
        pwm_duty(PWMA_CH1P_P60, 0);
        pwm_duty(PWMA_CH3P_P64, (uint32)(-motor_left.duty1));
    }

    // 右轮
    if (motor_right.duty1 >= 0) {
        pwm_duty(PWMA_CH4P_P66, 0);
        pwm_duty(PWMA_CH2P_P62, (uint32)motor_right.duty1);
    }
    else {
        pwm_duty(PWMA_CH4P_P66, (uint32)(-motor_right.duty1));
        pwm_duty(PWMA_CH2P_P62, 0);
    }
}

//****************************************
// 函数简介 电机开环输出
// 参数说明 void
// 参数说明 无
// 返回参数 无
// 使用示例 motor_driver_open_out_dr();
void motor_driver_open_out_dr(void) {
    if (motor_left.duty1 >= 0) {
        pwm_duty(PWMA_CH2P_P62, (uint32)motor_left.duty1);  // 正转 PWM
         P60 = 0 ; // 方向控制：0=正转
    }
    else {
        pwm_duty(PWMA_CH2P_P62, (uint32)(-motor_left.duty1));  // 反转 PWM（取绝对值）
         P60 = 1 ;// 方向控制：1=反转
    }

    if (motor_right.duty1 >= 0) {
        pwm_duty(PWMA_CH4P_P66, (uint32)motor_right.duty1);  // 正转 PWM
         P64 = 0 ; // 方向控制：0=正转
    }
    else {
        pwm_duty(PWMA_CH4P_P66, (uint32)(-motor_right.duty1));  // 反转 PWM（取绝对值）
        P64 = 1 ;  // 方向控制：1=反转
	}
}

//****************************************
// 函数简介 电机增量式PID闭环
// 参数说明 void
// 参数说明 无
// 返回参数 无
// 使用示例 motor_closed_loop_control(motor_left);
//****************************************
int16 motor_closed_loop_control(motor_struct *sptr) {
    sptr->err2 = sptr->err1;
    sptr->err1 = sptr->err;
//	sptr->err2 = sptr->err2 * lpf_motor + sptr->err1 * (1.0f - lpf_motor);
//	sptr->err1 = sptr->err1 * lpf_motor + sptr->err * (1.0f - lpf_motor);
    sptr->err = sptr->setspeed - sptr->encoder_data;

    sptr->out_p = sptr->err - sptr->err1;
    sptr->out_i = sptr->err;
    sptr->out_d = sptr->err - 2 * sptr->err1 + sptr->err2;

	sptr->out_i = abs(sptr->out_i) > 7000 ? 0 : sptr->out_i; // 死区

    // sptr->out_motor_pid += (int16)(sptr->Kp_motor * (float)sptr->out_p + sptr->Ki_motor * (float)sptr->out_i + sptr->Kd_motor * (float)sptr->out_d);
    sptr->out_motor_pid = MINMAX(sptr->out_motor_pid + (int16)(sptr->Kp_motor * (float)sptr->out_p + sptr->Ki_motor * (float)sptr->out_i + sptr->Kd_motor * (float)sptr->out_d), -8000, 8000);
	
//	if (sptr->err >= 150)  sptr->out_i = 0;
    // if (sptr->Ki_motor != 0)  sptr->out_i = MINMAX(sptr->out_i, -6000, 6000); // 限幅
    
    // sptr->out_motor_pid = MINMAX(sptr->out_motor_pid, -8000, 8000);

    return sptr->out_motor_pid;
}

//****************************************
// 函数简介 电机闭环实现函数
// 参数说明 speed_l：左电机设定速度 speed_r：右电机设定速度
// 参数说明 无
// 返回参数 无
// 使用示例 motor_control(motor_left);
//****************************************
void motor_control(int16 speed_l, int16 speed_r) {
	if (normal_speed == 0) {
		motor_left.setspeed = 0;
		motor_right.setspeed = 0;
	}
	else {
		motor_left.setspeed = speed_l;
		motor_right.setspeed = speed_r;
	}

    motor_closed_loop_control(&motor_left);
    motor_closed_loop_control(&motor_right);
	
    // 12mm
    motor_left.duty1 = motor_left.setspeed < 1000 ? 
                       motor_left.setspeed * 1000 / 65 + motor_left.out_motor_pid :
                       1000 + (motor_left.setspeed - 65) * 10 + motor_left.out_motor_pid;
    motor_right.duty1 = motor_right.setspeed < 1000 ? 
                        motor_right.setspeed * 1000 / 65 + motor_right.out_motor_pid :
                        1000 + (motor_right.setspeed - 65) * 10 + motor_right.out_motor_pid;

    // 19mm
    // motor_left.duty1 = motor_left.setspeed < 1000 ? 
    //                     motor_left.setspeed * 1000 / 65 + motor_left.out_motor_pid :
    //                     1000 + (motor_left.setspeed - 65) / 40 * 500 + motor_left.out_motor_pid;
    // motor_right.duty1 = motor_right.setspeed < 1000 ? 
    //                      motor_right.setspeed * 1000 / 45 + motor_right.out_motor_pid :
    //                      1000 + (motor_right.setspeed - 45) / 40 * 500 + motor_right.out_motor_pid;
	
	// motor_left.duty1 = 2000;
	// motor_right.duty1 = 1000;

    motor_left.duty1 *= (12600.0f / voltage);
    motor_right.duty1 *= (12600.0f / voltage);

    motor_left.duty1 = MINMAX(motor_left.duty1, -8000, 8000);
    motor_right.duty1 = MINMAX(motor_right.duty1, -8000, 8000);

    motor_driver_open_out_ir();
//	motor_driver_open_out_dr();
}

// 编码器初始化
void encoder_init(void) {                                    
    ctimer_count_init(SPEEDL_PULSE); // 初始化定时器0作为外部计数
    ctimer_count_init(SPEEDR_PULSE); // 初始化定时器3作为外部计数
}

// 电机初始化
void motor_driver_init_ir(void) {                                                
    pwm_init(PWMA_CH1P_P60, 17000, 0);           // 初始化PWM1  使用P60引脚  初始化频率为17Khz
    pwm_init(PWMA_CH2P_P62, 17000, 0);           // 初始化PWM2  使用P62引脚  初始化频率为17Khz
    pwm_init(PWMA_CH3P_P64, 17000, 0);           // 初始化PWM3  使用P64引脚  初始化频率为17Khz
    pwm_init(PWMA_CH4P_P66, 17000, 0);           // 初始化PWM4  使用P66引脚  初始化频率为17Khz
    motor_struct_parameter_init(&motor_left, 0); // 速度环结构体参数初始化
    motor_struct_parameter_init(&motor_right, 0);
}

void motor_driver_init_dr(void) {
    pwm_init(PWMA_CH2P_P62, 12500, 0);           // 初始化PWM2  使用P62引脚  初始化频率为17Khz
    pwm_init(PWMA_CH4P_P66, 12500, 0);           // 初始化PWM4  使用P66引脚  初始化频率为17Khz
	P6M1 &= ~(1<<4); P6M0 |= (1<<4);  			 // 设置P64为推挽输出
	P6M1 &= ~(1<<0); P6M0 |= (1<<0);  			 // 设置P60为推挽输出
	motor_struct_parameter_init(&motor_left, 0); // 速度环结构体参数初始化
    motor_struct_parameter_init(&motor_right, 0);
}

// 电机速度环初始化；*sptr：电机结构体的首地址 sspeed：设置的初速度
void motor_struct_parameter_init(motor_struct *sptr, int16 sspeed) {
    sptr->setspeed = sspeed; // int16
    sptr->actspeed = 0;      // int16
    sptr->encoder_data = 0;  // int16

    sptr->err = 0;  // int16
    sptr->err1 = 0; // int16
    sptr->err2 = 0;

    sptr->duty1 = 0; // int32
    sptr->out_p = 0;
    sptr->out_i = 0;
    sptr->out_d = 0;
    sptr->out_motor_pid = 0;

    sptr->Kp_motor = kp_motor; // 宏定义其比例系数
    sptr->Ki_motor = ki_motor;
    sptr->Kd_motor = kd_motor;
}
