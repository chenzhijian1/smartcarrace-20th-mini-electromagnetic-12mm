#ifndef __MY_PERIPHERAL_H
#define __MY_PERIPHERAL_H
#include "headfile.h"

#define PI 3.1415926f
#define ratio1 0.347 // 角速度比例

// 定义按键引脚
#define KEY1_PIN P73
#define KEY2_PIN P72
#define KEY3_PIN P70
#define KEY4_PIN P71

// 定义拨码开关引脚
#define SW1_PIN P74
#define SW2_PIN P75
#define SW3_PIN P76
#define SW4_PIN P77

// 定义蜂鸣器控制引脚
#define BEEP P67
// 定义UI菜单结构体
typedef struct
{
    void (*Disp)(void); // 函数指针
    int16 cursor;
    int16 page;
    int16 last;
} ui_struct;

void block_judgement();
void DataInit();
void key_scan(void);
void ips114_show(void);
void angle_gyro(void);
void pit_callback(void);
void beep_init(void);
void beep_test(void);
float StrToDouble(const char *s);
float iap_read_float(uint8 len, uint16 addr);
void UI_DispUIStrings(uint8 strings[8][30]);
void ui_display(void);

extern float yaw;
extern uint8 count_tof;
extern uint8 count_flag_4;

extern float Gyro_offset_z;
#endif
