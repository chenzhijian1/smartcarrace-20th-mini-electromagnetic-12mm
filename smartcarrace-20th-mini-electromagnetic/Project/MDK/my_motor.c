#include "my_motor.h"
#include "pid.h"
//******************* 需要调的参数********************/
// 环岛
uint8 huandao_directions[4] = {0, 0, 0, 1};      // 0 表示左环岛, 1 表示右环岛
float huandao_hight_speed[4] = {190, 170, 0, 0}; // 第n个环岛高速轮
float huandao_low_speed[4] = {65, 75, 75, 0};   // 第n个环岛低速轮
uint8 flag2 = 0;

#define straight_speed_huandao 100 // 预环岛直行段速度和出环调整段直线速度
#define HUANDAO_DISTANCE 180       // 出环岛直行距离
#define HUANDAO_DISTANCE_adjust 60 // 出环岛调姿态限制

// #define distance_before_huandao 150	// 预环岛距离
// #define distance_after_huandao 150  // 出环岛距离

float distance_before_huandao = 150;
float distance_after_huandao = 150;
float angle_in_threshold = 30; // 环岛入口角度阈值
float angle_out_threshold = 30; // 环岛出口角度阈值

#define high_speed_huandao 230
#define low_speed_huandao  100

// 障碍

float block_out_angle = 25; // 右过为-35，左过为+35
float block_back_angle = 10;

float block_out_encode = 120;  // 出障编码器值
float block_back_encode = 220; // 回来编码器值

float block_judge = 600; // tof障碍阈值
float block_speed = 100;

#define ramp_encoder 450
float ramp_speed = 180 ; //坡道速度
/*****************速度决策**********************/
// 过完标志位后的调整速度
float adjust_speed_after_left_huandao = 120;
float adjust_speed_after_right_huandao = 120;
float adjust_speed_after_block = 120;
float adjust_speed_after_ramp = 230;
// float adjust_speed_after_shizi = 0;
// 过完标志位加速的距离，直到遇到下一个直角或需要减速的元素的距离（要预留减速距离）
float encoder_speed_up_after_left_huandao = 0;  // flag_speed_adjust =1
float encoder_speed_up_after_right_huandao = 0; // flag_speed_adjust =2
float encoder_speed_up_after_block = 0;         // flag_speed_adjust =3
float encoder_speed_up_after_ramp = 0;          // flag_speed_adjust =4
// float encoder_speed_up_after_shizi = 0;      // flag_speed_adjust =5
uint8 flag_speed_adjust = 0; // 0: 回到正常速度；1: 左环岛后加速；2: 右环岛后加速；3: 障碍后加速；4: 坡道后加速；5: 最后加速

// 速度
float normal_speed = 0;
float speed_huandao = 0;
float normal_speed_cal;
float max_speed = 140;
float MAX_changed_speed = 0;
int speed_chujiemin = -50;
int speed_chujiemax = 300;

// 方向环
float kp_direction = 5; // 方向环的pid
float kd_direction = 11;
float kp_direction_2 = 5;	
float kd_direction_2 = 11;
float kp_direction_3 = 5;
float kd_direction_3 = 11;

float kpa = 4;
float kpb = 10;
float kd = 25;
float kd_imu = 0; 
//************不需要调的参数************//

/*#define kp_motor 0      // 13.35         47  
#define ki_motor 0 // 电机闭环的pid0.00322
#define kd_motor 0*/
float kp_motor = 20.0;
float ki_motor = 4.0;
float kd_motor = 0.0;

uint8 flag = 0; // 0: 正常模式；1: 预环岛模式；2: 环岛模式；3: 出环调整；4: 障碍模式；5: 坡道模式
uint8 flag_stop = 0; // 0: 未停止；1: 停止
uint8 flag_key_control = 0; // 0：调参模式；1：跑车模式
uint8 flag_key_fast = 0; // 0：正常模式；1：快速模式
uint8 flag_start = 0; // 0: 未开始快速循迹；1: 开始快速循迹
uint8 cnt_start = 0;
uint8 cnt_stop = 0;

float test_speed = 0;       // setspeed
int16 changed_speed = 0;
int set_leftspeed = 0;
int set_rightspeed = 0;
// 编码器积分值
float encoder_left = 0.0;
float encoder_right = 0.0;
float encoder_ave = 0.0;
float encoder_temp = 0.0;

float lpf_encoder = 0.2; //编码器低通滤波系数
float lpf_motor = 0.1;   //电机低通滤波系数

uint8 flag_huandao = 0;     // 0表示左环岛，1表示右环岛
uint8 flag_set_angle = 0;
uint16 flag_circle_in = 0, cnt_circle_in = 0;
uint16 flag_circle_out = 0, cnt_circle_out = 0;
uint8 huandao_count = 0;    // 环岛计数
float hightv_huandao = 170; // 环岛高速轮
float lowv_huandao = 75;    // 环岛低速轮
float target_angle_in = 0;
float target_angle_out = 0;
motor_struct motor_left, motor_right; // 定义电机速度闭环变量

float k;
float s;

float gyro_z;

void speed_change()
{
    if (flag != 4)
        car_stop_judge();
    if (flag_stop == 0)
    {
        // if (flag_key_fast == 1)  fast_tracking();
        // else
        //     if (normal_speed != 0)  Path_record();
        
        switch (flag)
        {
        case 0: // 正常模式
//            if(fabs(aaddcc.err_dir) < 1 && fabs(aaddcc.last_err_dir) < 1 && !(aaddcc.err_dir ==0 && aaddcc.last_err_dir ==0 ))
//            				test_speed += 0.001;
			dir_pid(aaddcc.err_dir, aaddcc.last_err_dir, (float)imu660ra_gyro_z);
//			dir_pid_sep(aaddcc.err_dir, aaddcc.last_err_dir, (float)imu660ra_gyro_z);
			
            // 速度策略
            if (flag_start && cnt_start < 100) {
                cnt_start++;
                // normal_speed_cal = 150 + (normal_speed - 150) * cnt_start / 100.0;
                normal_speed_cal = normal_speed / 10000.0f * cnt_start * cnt_start;
            }
            else {
                flag_start = 0;
                normal_speed_cal = -s * aaddcc.err_dir * aaddcc.err_dir + normal_speed;
            }

//             switch (flag_speed_adjust) //加速判断
//             {
//             case 0: // 回到正常速度
//                 test_speed = normal_speed_cal;
//                 break;
//             case 1: // 左环岛后加速
//                 if (encoder_ave < encoder_speed_up_after_left_huandao)
//                     test_speed = adjust_speed_after_left_huandao;
//                 else
//                 {
//                     flag_speed_adjust = 0;
//                     encoder_clear();
//                 }
//                 break;
//             case 2: // 右环岛后加速
//                 if (encoder_ave < encoder_speed_up_after_right_huandao)
//                     test_speed = adjust_speed_after_right_huandao;
//                 else
//                 {
//                     flag_speed_adjust = 0;
//                     encoder_clear();
//                 }
//                 break;
//             case 3: // 障碍后加速
//                 if (encoder_ave < encoder_speed_up_after_block)
//                     test_speed = adjust_speed_after_block;
//                 else
//                 {
//                     flag_speed_adjust = 0;
//                     encoder_clear();
//                 }
//                 break;
//             case 4: // 坡道后加速
//                 if (encoder_ave < encoder_speed_up_after_ramp)
//                     test_speed = adjust_speed_after_ramp;
//                 else
//                 {
//                     flag_speed_adjust = 0;
//                     encoder_clear();
//                 }
// //			case 5:
// //				if (encoder_ave < 3000)//最后的加速
// //                    test_speed = 170;
// //                else
// //                {
// //                    flag_speed_adjust = 0;
// //                    encoder_clear();
// //                }
// //                break;
// //			case 6:
// //				if (encoder_ave < 3000)//最后的加速
// //                    test_speed = 180;
// //                else
// //                {
// //                    flag_speed_adjust = 0;
// //                    encoder_clear();
// //                }
// //                break;
// //			case 7:
// //				if (encoder_ave < 3000)//最后的加速
// //                    test_speed = 180;
// //                else
// //                {
// //                    flag_speed_adjust = 0;
// //                    encoder_clear();
// //                }
// //                break;
// //			case 8:
// //				if (encoder_ave < 3000)//最后的加速
// //                    test_speed = 180;
// //                else
// //                {
// //                    flag_speed_adjust = 0;
// //                    encoder_clear();
// //                }
// //                break;
//             default:
//                 break;
//             }
            test_speed = normal_speed_cal;

            if (flag_key_fast == 1) {
                // 直道和弯道不同的差速限幅
                if (path_points[j].isturn == 1) {
                    changed_speed = MINMAX(changed_speed, -100, 100);
                
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
                    changed_speed = MINMAX(changed_speed, -300, 300);

                    set_leftspeed = test_speed - changed_speed;
    			    set_rightspeed = test_speed + changed_speed;

                    set_leftspeed = MINMAX(set_leftspeed, -100, 600);
                    set_rightspeed = MINMAX(set_rightspeed, -100, 600);
                }
            }
            else {
                changed_speed = MINMAX(changed_speed, -100, 100);
                
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
			
            break;
        case 1: // 预环岛模式
            if (AD_ONE[0] > AD_ONE[4])
                flag_huandao = 0; // 左环岛
            else
                flag_huandao = 1; // 右环岛
            
            if (encoder_ave - encoder_temp < distance_before_huandao) { //没到环岛交点
				//直行
				set_leftspeed = normal_speed;
				set_rightspeed = normal_speed;
			}
            else  flag = 2;
            break;

        case 2: // 圆环4
			
            // else { //到环岛交点
                if (flag_set_angle == 0) { // 第一次进入
                    target_angle_in = yaw;
                    flag_set_angle = 1;
                }

                if (flag_huandao == 0) { // 左环岛
					target_angle_out = target_angle_in + 350;
					
					if (yaw - target_angle_in > angle_in_threshold) // 进去一段路了
						flag_circle_in = 1;
					
					if (flag_circle_in == 0) { //左转进环
						set_leftspeed = low_speed_huandao;
						set_rightspeed = high_speed_huandao;
					}
					else { //正常循迹
						dir_pid(aaddcc.err_dir, aaddcc.last_err_dir, (float)imu660ra_gyro_z);
						changed_speed = MINMAX(changed_speed, -50, 50);

						test_speed = speed_huandao;
						set_leftspeed = test_speed - changed_speed;
						set_rightspeed = test_speed + changed_speed;
						set_leftspeed = MINMAX(set_leftspeed, -40, 300);
						set_rightspeed = MINMAX(set_rightspeed, -40, 300);
                    }
					if (target_angle_out - yaw < angle_out_threshold) // 离出来只有一点了
						flag_circle_out = 1;
						
					if (flag_circle_out == 0) {
                        // 左转出弯
                        set_leftspeed = low_speed_huandao;
						set_rightspeed = high_speed_huandao;  
                    }
                    else {
                        encoder_temp = encoder_ave;
                        flag = 3; // 出环模式
					}
                }

                else { // 右环岛
                    target_angle_out = target_angle_in - 350;
//                    if (cnt_circle_in++ >= delay_ms_in / 5) //延时达到
//                        flag_circle_in = 1;
					if (target_angle_in - yaw > angle_in_threshold)
						flag_circle_in = 1;
                    
                    if (flag_circle_in == 0) {
                        // flag2 = 1; // 调试
                        //右转进环
                        set_leftspeed = high_speed_huandao;
                        set_rightspeed = low_speed_huandao;
                    }
                    else { //正常循迹
                        dir_pid(aaddcc.err_dir, aaddcc.last_err_dir, (float)imu660ra_gyro_z);
                        changed_speed = MINMAX(changed_speed, -50, 50);

                        test_speed = speed_huandao;
                        set_leftspeed = test_speed - changed_speed;
                        set_rightspeed = test_speed + changed_speed;
                        set_leftspeed = MINMAX(set_leftspeed, -40, 300);
                        set_rightspeed = MINMAX(set_rightspeed, -40, 300);
                    }
					if (yaw - target_angle_out < angle_out_threshold)
						flag_circle_out = 1;

                    if (flag_circle_out == 0) {
                        // 右转出环
                        set_leftspeed = high_speed_huandao;
                        set_rightspeed = low_speed_huandao;
                    }
                    else {
						encoder_temp = encoder_ave;
						flag = 3; // 出环模式
                    }
                }
            // }

            break;

        case 3: // 出环
            if (encoder_ave - encoder_temp <= distance_after_huandao) {
				set_leftspeed = normal_speed;
				set_rightspeed = normal_speed;
			}
			else {
				// 恢复到正常循迹
				flag = 0;
				
				// 各种标志位清零
				flag_set_angle = 0;
				
				cnt_circle_in = 0;
				flag_circle_in = 0;
				cnt_circle_out = 0;
				flag_circle_out = 0;
			}
            break;
//        case 4: // 避障
//            BEEP = 0;
//            if (P74 == 1) // 从上往下第二个开关 往左拨 // 左过避障
//            {
//                if (yaw < block_out_angle && encoder_ave < block_out_encode) // 控制打角直至陀螺仪角度<-35
//                {
//                    set_leftspeed = 45;
//                    set_rightspeed = 238; // 打角避障
//                }
//                else if (encoder_ave < block_out_encode) // 打够角度后直行一段确保车子拐过路障
//                {
//                    set_leftspeed = 100;
//                    set_rightspeed = 100;
//                }
//                else if (yaw > block_back_angle) // 拐过路障后回调车身
//                {
//                    set_leftspeed = 180;
//                    set_rightspeed = 75;
//                }
//                else if (encoder_ave < block_back_encode) // 过障后打角回线
//                {
//                    set_leftspeed = 153;
//                    set_rightspeed = 82.5;
//                }
//            }
//            else // 往有拨右拐过
//            {
//                if (yaw > -block_out_angle && encoder_ave < block_out_encode) // 控制打角直至陀螺仪角度<-35
//                {
//                    set_leftspeed = 238;
//                    set_rightspeed = 45; // 打角避障
//                }
//                else if (encoder_ave < block_out_encode) // 打够角度后直行一段确保车子拐过路障
//                {
//                    set_leftspeed = block_speed;
//                    set_rightspeed = block_speed;
//                }
//                else if (yaw < -block_back_angle) // 拐过路障后回调车身
//                {
//                    set_leftspeed = 80;
//                    set_rightspeed = 150;
//					
//                }
//                else if (encoder_ave < block_back_encode) // 过障后打角回线
//                {
//                    set_leftspeed = 85;
//                    set_rightspeed = 95;
//					
//                }
//            }

//            if (encoder_ave > block_back_encode)
//            { // 行至一定距离后标志位清0，回到正常循迹调整小车
//                flag = 0;
//                encoder_clear();
//                yaw = 0;
//                test_speed = adjust_speed_after_block;
//                flag_speed_adjust = 3;
//                BEEP = 0;
//            }
//            break;
          case 4: // 起步发车
              if (encoder_ave >= 133)  flag = 0;
              set_leftspeed = 150;
              set_rightspeed = 150;
              break;


//        case 5:
//            BEEP = 0;
//            if (encoder_ave < ramp_encoder || ad_ave[2] > 2300)

//            { 
//				test_speed = ramp_speed;
//                if (abs(aaddcc.err_dir) <= 1)
//                    changed_speed = kp_direction * aaddcc.err_dir + kd_direction * (aaddcc.err_dir - aaddcc.last_err_dir);
//                else if (abs(aaddcc.err_dir) > 1 && abs(aaddcc.err_dir) <= 14)
//                    changed_speed = kp_direction_2 * aaddcc.err_dir + kd_direction_2 * (aaddcc.err_dir - aaddcc.last_err_dir);
//                else
//                    changed_speed = kp_direction_3 * aaddcc.err_dir + kd_direction_3 * (aaddcc.err_dir - aaddcc.last_err_dir);
//                // MAX_changed_speed = 2 * test_speed;
//                set_leftspeed = test_speed - changed_speed;
//                set_rightspeed = test_speed + changed_speed;
//                set_leftspeed = MINMAX(set_leftspeed, -40, 350);
//                set_rightspeed = MINMAX(set_rightspeed, -40, 350);
//            }
//            else if (encoder_ave >= 450 && ad_ave[2] < 3000)
//            {
//                flag = 0;
//                encoder_clear();
//                test_speed = adjust_speed_after_ramp;
//                flag_speed_adjust = 4;
//            }
//            break;
            case 5: // 慢速停车
                if (cnt_stop < 200) {
                    cnt_stop++;
                    normal_speed_cal = normal_speed / 200 * (200 - cnt_stop);
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

void car_stop_judge() //脱线保护
{
	if (ad_ave[0] < 50 && ad_ave[3] < 50 && ad_ave[1] < 50 && ad_ave[4] < 50)
    {
        set_leftspeed = 0;
        set_rightspeed = 0;
		normal_speed = 0;
        flag_stop = 1;
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
void encoder_get(void)
{
    motor_left.encoder_data = (int16)ctimer_count_read(SPEEDL_PULSE);
    motor_right.encoder_data = (int16)ctimer_count_read(SPEEDR_PULSE);
//	motor_left.encoder_data = motor_left.encoder_data * lpf_encoder + (int16)ctimer_count_read(SPEEDL_PULSE) * (1.0f - lpf_encoder);
//	motor_right.encoder_data = motor_right.encoder_data * lpf_encoder + (int16)ctimer_count_read(SPEEDR_PULSE) * (1.0f - lpf_encoder);

    ctimer_count_clean(SPEEDL_PULSE);
    ctimer_count_clean(SPEEDR_PULSE);

    if (SPEEDL_DIR == 0) //观察屏幕输出调整
    {
        motor_left.encoder_data = -motor_left.encoder_data;
    }
    if (SPEEDR_DIR == 1)
    {
        motor_right.encoder_data = -motor_right.encoder_data;
    }
}

void encoder()
{
    encoder_left += 0.017 * motor_left.encoder_data;
    encoder_right += 0.017 * motor_right.encoder_data; // 0.017怎么算出来的
    encoder_ave = (encoder_left + encoder_right) / 2;
}

void encoder_clear()
{
    encoder_left = 0.0;
    encoder_right = 0.0;
    encoder_ave = 0.0;
}

void dir_pid (float error, float last_error, float gyro) {
    int16 p_out, d_out, output;
    
    p_out = (int16)((kpa / 10) * error + (kpb / 10000) * error * error * error);

//	d_out = (int16)(kd * (error - last_error));
	d_out = (int16)(kd * (error - last_error)) - (int16)(kd_imu / 100.0 * (gyro - 2.42));
    output = p_out + d_out;

    changed_speed = output;
}

void dir_pid_sep (float error, float last_error, float gyro) {
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
// 函数简介 电机开环输出
// 参数说明 void
// 参数说明 无
// 返回参数 无
// 使用示例 motor_driver_open_out_ir();
//****************************************
void motor_driver_open_out_ir(void)
{
    if (motor_left.duty1 >= 0)
    {
        pwm_duty(PWMA_CH1P_P60, (uint32)motor_left.duty1);
		pwm_duty(PWMA_CH3P_P64, 0);
    }
    else
    {
        pwm_duty(PWMA_CH1P_P60, 0);
        pwm_duty(PWMA_CH3P_P64, (uint32)(-motor_left.duty1));
    }

    // 右轮
    if (motor_right.duty1 >= 0)
    {
        pwm_duty(PWMA_CH4P_P66, 0);
        pwm_duty(PWMA_CH2P_P62, (uint32)motor_right.duty1);
    }
    else
    {
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
//****************************************
void motor_driver_open_out_dr(void)
{
    if (motor_left.duty1 >= 0)
    {
        pwm_duty(PWMA_CH2P_P62, (uint32)motor_left.duty1);  // 正转 PWM
         P60 = 0 ; // 方向控制：0=正转
    }
    else
    {
        pwm_duty(PWMA_CH2P_P62, (uint32)(-motor_left.duty1));  // 反转 PWM（取绝对值）
         P60 = 1 ;// 方向控制：1=反转
    }

    // 右电机控制（使用 PWM_1 和 DIR_1）
    if (motor_right.duty1 >= 0)
    {
        pwm_duty(PWMA_CH4P_P66, (uint32)motor_right.duty1);  // 正转 PWM
         P64 = 0 ; // 方向控制：0=正转
    }
    else
    {
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
int16 motor_closed_loop_control(motor_struct *sptr)
{
    sptr->err2 = sptr->err1;
    sptr->err1 = sptr->err;
//	sptr->err2 = sptr->err2 * lpf_motor + sptr->err1 * (1.0f - lpf_motor);
//	sptr->err1 = sptr->err1 * lpf_motor + sptr->err * (1.0f - lpf_motor);
    sptr->err = sptr->setspeed - sptr->encoder_data;

    sptr->out_p = sptr->err - sptr->err1;
    sptr->out_i = sptr->err;
    sptr->out_d = sptr->err - 2 * sptr->err1 + sptr->err2;
	
    sptr->out_motor_pid += (int16)(sptr->Kp_motor * sptr->out_p + sptr->Ki_motor * sptr->out_i + sptr->Kd_motor * sptr->out_d);
	
//	if (sptr->err >= 150)  sptr->out_i = 0;
    if (sptr->Ki_motor != 0)  sptr->out_i = MINMAX(sptr->out_i, -5000, 5000);
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
void motor_control(int16 speed_l, int16 speed_r)
{
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
	
    motor_left.duty1 = motor_left.setspeed < 1000 ? 
                       motor_left.setspeed * 1000 / 65 + motor_left.out_motor_pid :
                       1000 + (motor_left.setspeed - 65) * 10 + motor_left.out_motor_pid;
    motor_right.duty1 = motor_right.setspeed < 1000 ? 
                        motor_right.setspeed * 1000 / 65 + motor_right.out_motor_pid :
                        1000 + (motor_right.setspeed - 65) * 10 + motor_right.out_motor_pid;
	
	// motor_left.duty1 = 2000;
	// motor_right.duty1 = 1000;

    motor_left.duty1 *= (12600.0f / voltage);
    motor_right.duty1 *= (12600.0f / voltage);

    motor_left.duty1 = MINMAX(motor_left.duty1, -10000, 10000);
    motor_right.duty1 = MINMAX(motor_right.duty1, -10000, 10000);

    motor_driver_open_out_ir();
//	motor_driver_open_out_dr();
}

void encoder_init(void)
{                                    // 编码器初始化
    ctimer_count_init(SPEEDL_PULSE); // 初始化定时器0作为外部计数
    ctimer_count_init(SPEEDR_PULSE); // 初始化定时器3作为外部计数
}

void motor_driver_init_ir(void)
{                                                // 电机初始化
    pwm_init(PWMA_CH1P_P60, 12500, 0);           // 初始化PWM1  使用P60引脚  初始化频率为17Khz
    pwm_init(PWMA_CH2P_P62, 12500, 0);           // 初始化PWM2  使用P62引脚  初始化频率为17Khz
    pwm_init(PWMA_CH3P_P64, 12500, 0);           // 初始化PWM3  使用P64引脚  初始化频率为17Khz
    pwm_init(PWMA_CH4P_P66, 12500, 0);           // 初始化PWM4  使用P66引脚  初始化频率为17Khz
    motor_struct_parameter_init(&motor_left, 0); // 速度环结构体参数初始化
    motor_struct_parameter_init(&motor_right, 0);
}

void motor_driver_init_dr(void){
    pwm_init(PWMA_CH2P_P62, 12500, 0);           // 初始化PWM2  使用P62引脚  初始化频率为17Khz
    pwm_init(PWMA_CH4P_P66, 12500, 0);           // 初始化PWM4  使用P66引脚  初始化频率为17Khz
	P6M1 &= ~(1<<4); P6M0 |= (1<<4);  			 // 设置P64为推挽输出
	P6M1 &= ~(1<<0); P6M0 |= (1<<0);  			 // 设置P60为推挽输出
	motor_struct_parameter_init(&motor_left, 0); // 速度环结构体参数初始化
    motor_struct_parameter_init(&motor_right, 0);
}

// 电机速度环初始化；*sptr：电机结构体的首地址 sspeed：设置的初速度
void motor_struct_parameter_init(motor_struct *sptr, int16 sspeed)
{
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
