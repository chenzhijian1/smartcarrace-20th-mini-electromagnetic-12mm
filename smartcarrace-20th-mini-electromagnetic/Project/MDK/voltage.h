#ifndef __VOLTAGE_H
#define __VOLTAGE_H

void voltage_init();
uint16 read_voltage(); //读取P15上的电压值
extern uint16 voltage;

#endif
