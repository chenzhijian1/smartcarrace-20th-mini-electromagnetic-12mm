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
	
	return battery_voltage;
}