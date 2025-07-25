/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：一群：179029047(已满)  二群：244861897(已满)  三群：824575535
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file       		eeprom
 * @company	   		成都逐飞科技有限公司
 * @author     		逐飞科技(QQ790875685)
 * @version    		查看doc内version文件 版本说明
 * @Software 		MDK FOR C251 V5.60
 * @Target core		STC32G12K128
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2020-4-14
 ********************************************************************************************************************/


#ifndef __ZF_EEPROM_H
#define __ZF_EEPROM_H

#include "common.h"



void iap_init(void);
void iap_idle(void);
void iap_set_tps(void);
uint8 iap_get_cmd_state(void);
void iap_read_bytes(uint32 addr, uint8 *buf, uint16 len);
void iap_write_bytes(uint32 addr, uint8 *buf, uint16 len);
void iap_erase_page(uint32 addr);
void extern_iap_write_bytes(uint16 addr, uint8 *buf, uint16 len);
void extern_iap_write_float(double dat,uint8 num,uint8 pointnum,uint16 addr);

#endif


