#include "headfile.h"
#define LED P52
#define TOF_DISTANCE 700

extern uint8 send_flag;
uint32 timer_cnt = 0;
uint8 timing_started_start = 0;
uint16 voltage;

void TM0_Isr() interrupt 1
{
	
}
void TM1_Isr() interrupt 3
{
	send_flag = 1;
    // 电感读取及数???处理
    direction_adc_get();
    encoder_get();
    encoder();

    pit_callback(); // 陀螺仪
    angle_gyro();
}
void TM2_Isr() interrupt 12
{
    
    // TIM2_CLEAR_FLAG; // 清除??????标志  
}
void TM3_Isr() interrupt 19
{
    // 右编码器
    TIM3_CLEAR_FLAG; // 清除??????标志
}

void TM4_Isr() interrupt 20
{
    TIM4_CLEAR_FLAG; // 清除??????标志
    if (timing_started_start)  timer_cnt++;
    // 编码器获得速度???
//	count_tof +=1; 
//	if(count_tof == 7 && (count_flag_4 ==0 || count_flag_4 == 1) )  //过完障???和坡道后不在判断tof
//	{		
//		count_tof =0;
//		//block_judgement();
//	}
	voltage = read_voltage();

	// if (normal_speed != 0) {
		speed_change();
	// }
	if (flag_key_control)  motor_control(set_leftspeed, set_rightspeed);
    else  motor_control(0, 0);
}

// UART1??????
void UART1_Isr() interrupt 4
{
    uint8 res;
    static uint8 dwon_count;
    if (UART1_GET_TX_FLAG)
    {
        UART1_CLEAR_TX_FLAG;
        busy[1] = 0;
    }
    if (UART1_GET_RX_FLAG)
    {
        UART1_CLEAR_RX_FLAG;
        res = SBUF;
        // 程序???动下???
        if (res == 0x7F)
        {
            if (dwon_count++ > 20)
                IAP_CONTR = 0x60;
        }
        else
        {
            dwon_count = 0;
        }
    }
}

// UART2??????
void UART2_Isr() interrupt 8
{
    if (UART2_GET_TX_FLAG)
    {
        UART2_CLEAR_TX_FLAG;
        busy[2] = 0;
    }
    if (UART2_GET_RX_FLAG)
    {
        UART2_CLEAR_RX_FLAG;
        // 接收数据寄存器为：S2BUF
    }
}

// UART3??????
void UART3_Isr() interrupt 17
{
    if (UART3_GET_TX_FLAG)
    {
        UART3_CLEAR_TX_FLAG;
        busy[3] = 0;
    }
    if (UART3_GET_RX_FLAG)
    {
        UART3_CLEAR_RX_FLAG;
        // 接收数据寄存器为：S3BUF
    }
}

#include "isr.h"
#include <string.h>

// 定义接收缓冲???
uint8 uart_buf[50];
uint8 uart_buf_index = 0;
uint8 receive_data;
uint8 i;
float receive_values[9];

float string_to_float(const char *str) {
    float result = 0.0f;
    float sign = 1.0f;
    int decimal_place = 0;
    int is_fraction = 0;

    // 处理符号
    if (*str == '-') {
        sign = -1.0f;
        str++;
    } else if (*str == '+') {
        str++;
    }

    // 逐字符处理
    while (*str) {
        if (*str == '.') {
            is_fraction = 1;
            str++;
            continue;
        }
        if (*str < '0' || *str > '9') {
            break; // 非法字符
        }
        if (is_fraction) {
            decimal_place++;
        }
        result = result * 10.0f + (*str - '0');
        str++;
    }
	
    // 处理小数部分
    for (i = 0; i < decimal_place; i++) {
        result /= 10.0f;
    }

    return result * sign;
}

// 串口4??????接收函数
void UART4_Isr() interrupt 18
{
    if (UART4_GET_TX_FLAG)
    {
        UART4_CLEAR_TX_FLAG;
        busy[4] = 0;
    }
    if (UART4_GET_RX_FLAG)
    {
        UART4_CLEAR_RX_FLAG;
		
//		if(wireless_type == WIRELESS_SI24R1)
//        {
//            wireless_uart_callback();           //无线模块串口回调函数
//        }
//        else if(wireless_type == WIRELESS_CH9141)
//        {
//            bluetooth_ch9141_uart_callback();   //蓝牙模块串口回调函数
//        }
//        else if(wireless_type == WIRELESS_CH573)
//        {
//            wireless_ch573_callback();          //CH573无线模块回调函数
//        }
        receive_data = S4BUF;  // 使用S4BUF接收数据
        
        // 存储接收到的数据
        if (receive_data == '\n' || receive_data == '\r')  // 接收到换行符或回车符，表示一帧数据接收完成
        {
            uart_buf[uart_buf_index] = '\0';  // 添加字符串结束符
            // 解析接收到的数据
            if (uart_buf_index >= 2)  // 至少需要一个字母和一个数字
            {
                char param_type = uart_buf[0];  // 获取参数类型
                float value = string_to_float(uart_buf + 1);  // 替换数值部分
                
                // 根据参数类型更新相应的??
                switch(param_type)
                {
//                    case 'P':  // PD组的kp
//                        kp_direction = value;
//                        break;
//                    case 'D':  // PD组的kd
//                        kd_direction = value;
//                        break;
//                    case 'Q':  // QE组的kp
//                        kp_direction_2 = value;
//                        break;
//                    case 'E':  // QE组的kd
//                        kd_direction_2 = value;
//                        break;
//                    case 'R':  // RF组的kp
//                        kp_direction_3 = value;
//                        break;
//                    case 'F':  // RF组的kd
//                        kd_direction_3 = value;
//                        break;
					case 'a':
						kpa = value;
						break;
					case 'b':
						kpb = value;
						break;
					case 'd':
						kd = value;
						break;
					case 'D':
						kd_imu = value;
						break;
                    // case 'P':
                    //     kp_gyro = value;
                    //     break;
                    // case 'D':
                    //     kd_gyro = value;
                    //     break;
                    case 't':
                        target_gyro_z = value;
                        break;
                    case 'p':  // pid组的kp
                        kp_motor = value;
						motor_left.Kp_motor = kp_motor;
						motor_right.Kp_motor = kp_motor;
                        break;
                    case 'i':  // pid组的ki
                        ki_motor = value;
						motor_left.Ki_motor = ki_motor;
						motor_right.Ki_motor = ki_motor;
                        break;
					case 'n':
						normal_speed = (int)value;
						break;
					case 's':
						s = value;
						break;
					case 'A':
						A_ = value;
						break;
					case 'B':
						B_ = value;
						break;
					case 'C':
						C_ = value;
						break;
                    case 'm':
                        path_point_count_threshold = value;
                        break;
                    case 'x':
                        distance_before_huandao = value;
                        break;
                    case 'y':
                        distance_after_huandao = value;
                        break;
                }
            }
            
            uart_buf_index = 0;  // 重置缓冲区索引
        }
        else if (uart_buf_index < 199)  // 确保不会溢出
        {
            uart_buf[uart_buf_index++] = receive_data;
        }
    }
}

void INT0_Isr() interrupt 0
{
    LED = 0; // 点亮LED
}
void INT1_Isr() interrupt 2
{
}
void INT2_Isr() interrupt 10
{
    INT2_CLEAR_FLAG; // 清除??????标志
}
void INT3_Isr() interrupt 11
{
    INT3_CLEAR_FLAG; // 清除??????标志
}

void INT4_Isr() interrupt 16
{
    INT4_CLEAR_FLAG; // 清除??????标志
}

// void  INT0_Isr()  interrupt 0;
// void  TM0_Isr()   interrupt 1;
// void  INT1_Isr()  interrupt 2;
// void  TM1_Isr()   interrupt 3;
// void  UART1_Isr() interrupt 4;
// void  ADC_Isr()   interrupt 5;
// void  LVD_Isr()   interrupt 6;
// void  PCA_Isr()   interrupt 7;
// void  UART2_Isr() interrupt 8;
// void  SPI_Isr()   interrupt 9;
// void  INT2_Isr()  interrupt 10;
// void  INT3_Isr()  interrupt 11;
// void  TM2_Isr()   interrupt 12;
// void  INT4_Isr()  interrupt 16;
// void  UART3_Isr() interrupt 17;
// void  UART4_Isr() interrupt 18;
// void  TM3_Isr()   interrupt 19;
// void  TM4_Isr()   interrupt 20;
// void  CMP_Isr()   interrupt 21;
// void  I2C_Isr()   interrupt 24;
// void  USB_Isr()   interrupt 25;
// void  PWM1_Isr()  interrupt 26;
// void  PWM2_Isr()  interrupt 27;