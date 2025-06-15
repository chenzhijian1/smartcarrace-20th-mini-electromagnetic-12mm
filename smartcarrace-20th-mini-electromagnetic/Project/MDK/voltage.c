#include "headfile.h"

uint16 ad_result = 0;           //引脚电压
uint16 battery_voltage = 0;     //电池电压

uint32 temp;

void voltage_init() {
	adc_init(ADC_P15, ADC_SYSclk_DIV_32);
}

uint16 read_voltage() {
	ad_result = adc_once(ADC_P15, ADC_12BIT);
	temp = (((uint32)ad_result * 5000) / 4096);  //计算出当前adc引脚的电压 计算公式为 ad_result*VCC/ADC分辨率    VCC单位为mv
	battery_voltage =  temp * 11;//根据引脚电压  和分压电阻的阻值计算电池电压 计算公司为   引脚电压*(R2+R3)/R3   R3为接地端电阻
	//这个1.5098的系数是直接用电压实际值/检测值算出来的，不用管分压公式，反正都是线性的
	
//	//在TFT上显示，需要初始化1.8寸TFT屏幕，才能使用。
//	lcd_showstr(0, 0, "voltage:");
//	lcd_showuint16(8*8, 0, battery_voltage);
//		
//	//在1.14IPS屏幕上显示，需要初始化1.14寸ips屏幕，才能使用。
//	ips114_showstr(0, 0, "voltage:");
//	ips114_showuint16(8*8, 0, battery_voltage);
//	ips114_showstr(0, 1, "ADC = ");
//	ips114_showuint16(8*8, 1, ad_result);
//	
//	//在OLED屏幕上显示，需要初始化OLED屏幕，才能使用。
//	oled_p6x8str_spi(0, 0, "voltage:");
//	oled_uint16_spi(8*8, 0, battery_voltage);
//		
//	//延时100ms
//	delay_ms(100);
	
	return battery_voltage;
}