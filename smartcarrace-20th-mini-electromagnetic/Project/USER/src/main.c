#include "headfile.h"

uint8 text[100];
uint32 length;
uint8 send_flag;

extern uint8 timing_started_start;
extern uint32 timer_cnt;

void main(void)
{
    board_init(); // 初始化寄存器，勿删除
    EA = 0;
	
    delay_ms(100);        // 延时100ms，等待主板其他外设上电
    ips114_init();        // 屏幕初始化
    beep_init();          // 蜂鸣器
    encoder_init();       // 编码器
    direction_adc_init(); // 电感     
    motor_driver_init_ir();  // 电机
	// motor_driver_init_dr();
    imu660ra_init();      // 陀螺仪
	// offset_init();        // 零漂
    wireless_uart_init(); // 无线串口
	voltage_init();       // 电压检测
	
	// while (voltage < 11500)  voltage = read_voltage();

    pit_timer_ms(TIM_1, 5);  // 电感、陀螺仪、编码器、串口
    pit_timer_ms(TIM_4, 5);  // 电机、电压检测、路径记忆
    // DataInit(); 

    // 旧方向环
    // kp_direction = 0.5f;
    // kd_direction = 10.5f;
    // kp_direction_2 = 1.8f;
    // kd_direction_2 = 22.0f;
    // kp_direction_3 = 2.1f;
    // kd_direction_3 = 90.0f;
	
    // 新方向环
//     kpa = 4.0f;   // 对应小error
//     kpb = 10.0f;  // 对应大error
//     kd = 30.0f;   //
// 	kd_imu = 0.0f;
	
//     kp_motor = 58.0f; //45
//     ki_motor = 5.5f; //3.35
//     kd_motor = 0.0f;
	
//     normal_speed = 0.0f;    // 运行速度
//     speed_huandao = 150.0f; // 环岛速度
// //	s = 0.27f; // 速度策略系数
// 	s = 0.0f;

    // normal_speed = 200.0f;    // 运行速度
	
    // 电感系数逐飞
	// A_ = 1.0f;
	// B_ = 5.0f;
	// C_ = 0.0f;

    // 电感系数差比和差
    A_ = 1.2f; // 25 25 100
	B_ = 1.7f;
	C_ = 0.5f;
    // A_ = 1.3f;
    // B_ = 1.4f;
    // C_ = 1.1f;

    

    // 电感系数差比和差
    // A_ = 1.0f;
    // B_ = 1.0f;
    // C_ = 1.0f;

    // 使能全局中断
    EA = 1;

    while(1) {
        if(P75 == 0) // 调参模式 开关在上
        {
            flag_key_control = 1;
            // key_scan();
            // ui_display();
        }
        else // 跑车模式 开关在下
        {
            flag_key_control = 1;
            ips114_show();
        }

        // 此处所有的按键都是调试用，实际需要通过检测磁钢停车来
        if (KEY1_PIN == 0) {
            write_path();
        }

        if (KEY2_PIN == 0) // 切换模式  从上往下第二个
        {
            refresh();
            j = 1;
            flag_end = 0;

            timing_started_start = 1;
        }

        if (timer_cnt >= 200) { // 1s切换状态
            flag_key_fast = !flag_key_fast;
            flag_start = 1; // 开始跑，初始逐渐加速
            timing_started_start = 0;
            timer_cnt = 0;
        }

        if (KEY3_PIN == 0) // 重置   从上往下第四个
        {
        	refresh();
            huandao_count = 0;
            j = 1;
            flag_end = 0;
        }

        if (KEY4_PIN == 0)  flag_stop = !flag_stop;  // 从上往下第三个
		
 		if (send_flag) {
 			send_flag = 0;
            printf("%.1f,%.1f,%.1f,%.1f,%.3f,%.3f,%.1f,%.1f,",
                   kpa, kpb, kd, kd_imu,
                   kp_gyro, kd_gyro,
                   motor_left.Kp_motor, motor_left.Ki_motor);

            printf("%.2f,%.1f,", aaddcc.err_dir, target_gyro_z);

 			// printf("%d,%d,%d,%d,%d,%d,%d,%.2f,%.2f,",
 			// 		motor_left.setspeed, motor_left.encoder_data,
 			// 		motor_right.setspeed, motor_right.encoder_data,
 			// 		abs(motor_left.setspeed - motor_right.setspeed),
 			// 		motor_left.duty1,
 			// 		motor_right.duty1,
 			// 		s,
 			// 		normal_speed);

            printf("%d,%d,%d,%d,%d,%d,%d,",
                    motor_left.setspeed, motor_left.encoder_data,
                    motor_right.setspeed, motor_right.encoder_data,
                    motor_left.duty1,
                    motor_right.duty1,
                    normal_speed);
			
 			printf("%d,", voltage);
					
 			// printf("%.1f,%.1f,%.1f,%.1f,%.1f,", AD_ONE[0],AD_ONE[1],AD_ONE[2],AD_ONE[3],AD_ONE[4]);
            printf("%d,%d,%d,%d,%d,", ad_ave[0], ad_ave[1], ad_ave[2], ad_ave[3], ad_ave[4]);
			
 			// printf("%.2f,", (motor_left.encoder_data + motor_right.encoder_data) / 2 / 122.5);

            printf("%.1f,", gyro_z);

            printf("%.1f,%.1f,", distance_before_huandao, distance_after_huandao);

            printf("%.1f,%.1f,%.1f\r\n", A_, B_, C_);
            // printf("%.2f,%.6f,%d\r\n", yaw, Gyro_offset_z, imu660ra_gyro_z / 16.4);
 		}
        // if (send_flag_nav && path_point_count < path_point_count_threshold) {
        //     send_flag_nav = 0;
        //     printf("%d,%.2f,%.2f,%d,%.2f\r\n", path_point_count, path_points[path_point_count].distance,
        //         path_points[path_point_count].yaw_relative, path_points[path_point_count].isturn, imu660ra_gyro_z / 16.4f);
        // }
    }
}