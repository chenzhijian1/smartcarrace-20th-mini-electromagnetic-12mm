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
#include "zf_eeprom.h"
#include "board.h"
#include "intrins.h"
#include "zf_delay.h"
#include "headfile.h"
//-------------------------------------------------------------------------------------------------------------------
//  @brief      EEPROM触发操作，
//  @param      
//  @return     void
//  Sample usage:       		内部使用用户无需关心
//-------------------------------------------------------------------------------------------------------------------
void eeprom_trig(void)
{
    F0 = EA;    //保存全局中断
    IAP_TRIG = 0x5A;
    IAP_TRIG = 0xA5;                    //先送5AH，再送A5H到IAP触发寄存器，每次都需要如此
                                        //送完A5H后，IAP命令立即被触发启动
                                        //CPU等待IAP完成后，才会继续执行程序。
    _nop_();   //由于STC32G是多级流水线的指令系统，触发命令后建议加4个NOP，保证IAP_DATA的数据完成准备
    _nop_();
    _nop_();
    _nop_();
	
    EA = F0;    //恢复全局中断
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      初始化EEPROM
//  @param      NULL
//  @return     void
//  Sample usage:           
//-------------------------------------------------------------------------------------------------------------------
void iap_init(void)
{
	IAP_CONTR = 0x80;	 	//使能EEPROM操作
	iap_set_tps();			//设置擦除等待时间

	
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      关闭EEPROM
//  @param      NULL
//  @return     void
//  Sample usage:           
//-------------------------------------------------------------------------------------------------------------------
void iap_idle(void)
{
	IAP_CONTR = 0;			//失能EEPROM操作
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      获取EEPROM操作失败状态位，需要软件清零
//  @param      NULL
//  @return     void
//  Sample usage:           
//								操作失败返回1;
//-------------------------------------------------------------------------------------------------------------------
uint8 iap_get_cmd_state(void)
{
	return ((IAP_CONTR&0x01) == 0x01);
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      设置IAP等待时间
//  @param      NULL
//  @return     void
//  Sample usage:           
//-------------------------------------------------------------------------------------------------------------------
void iap_set_tps(void)
{
	uint8 write_time;
	write_time = (sys_clk / 1000000) ;
	IAP_TPS = write_time;
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      EEPROM读取多个字节
//  @param      addr			需要读取的eeprom地址
//  @param      *buf			需要读取的数据地址
//  @param      len				需要读取的数据长度
//  @return     void
//  Sample usage:               uint8 str[10];
//								iap_read_bytes(0x00,str,10);
//								将0x00-0x0A地址中的数据，读取到str中。
//-------------------------------------------------------------------------------------------------------------------
void iap_read_bytes(uint32 addr, uint8 *buf, uint16 len)
{

	
	IAP_CMD = 1; 				//设置 IAP 读命令	

	while(len--)
	{
        IAP_ADDRE = 0;
		IAP_ADDRL = addr; 		//设置 IAP 低地址
		IAP_ADDRH = addr >> 8; 	//设置 IAP 高地址
		IAP_ADDRE = addr >> 16;	//设置 IAP 最高地址
        eeprom_trig();
		*buf++ = IAP_DATA; 		//读 IAP 数据
		addr++;
		
	}
	
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      EEPROM写多个字节
//  @param      addr			需要写的eeprom地址
//  @param      *buf			需要写的数据地址
//  @param      len				需要写的数据长度
//  @return     void
//  Sample usage:       		iap_write_bytes(0x00,(uint8 *)"0123456789",10);
//								将"0123456789"写入0x00-0x0A地址中;
//-------------------------------------------------------------------------------------------------------------------
void iap_write_bytes(uint32 addr, uint8 *buf, uint16 len)
{

	IAP_CMD = 2; 				//设置 IAP 读命令	
	
	while(len--)
	{
        IAP_ADDRE = 0;
		IAP_ADDRL = addr; 		//设置 IAP 低地址
		IAP_ADDRH = addr >> 8; 	//设置 IAP 高地址
		IAP_ADDRE = addr >> 16;	//设置 IAP 最高地址
		IAP_DATA = *buf++; 		//写 IAP 数据
		addr++;

		eeprom_trig();
	}
	
}



//-------------------------------------------------------------------------------------------------------------------
//  @brief      EEPROM擦除目标地址所在的一页（1扇区/512字节）
//  @param      addr			需要写的eeprom地址
//  @return     void
//  Sample usage:       		iap_erase_page(0x20);
//								擦除0x00-0x200的数据
//-------------------------------------------------------------------------------------------------------------------
void iap_erase_page(uint32 addr) 
{ 

	IAP_CMD = 3; 				//设置 IAP 擦除命令
	IAP_ADDRL = addr; 			//设置 IAP 低地址
	IAP_ADDRH = addr >> 8;  	//设置 IAP 高地址
	IAP_ADDRE = addr >> 16;		//设置 IAP 最高地址
    eeprom_trig();		
	delay_ms(10);				//擦除1扇区(512字节)：约4-6ms
}



////-------------------------------------------------------------------------------------------------------------------
////  @brief      扩展EEPROM写多个字节(无需擦除)
////  @param      addr			需要写的eeprom地址
////  @param      *buf			需要写的数据地址
////  @param      len				需要写的数据长度
////  @return     void
////  Sample usage:       		extern_iap_write_bytes(0x0000,(uint8 *)"0123456789";,10);
////								将"0123456789"写入0x00-0x0A地址中;
////	@note：						不要跨扇区使用。
////								addr地址：0-511为一个扇区,512-1023为一个扇区，1024-1535为一个扇区，依次类推。
////-------------------------------------------------------------------------------------------------------------------
void extern_iap_write_bytes(uint16 addr, uint8 *buf, uint16 len)
{ 
	uint8 temp[512];
	uint16 i;
	
	for(i=0; i<512 ;i++)	temp[i] = 0;			//清0
	iap_read_bytes(addr&0xFE00, temp, 512);			//读取
	for(i=0; i<len; i++)	temp[(addr&0x1FF) + i] = buf[i];	//改
	iap_erase_page(addr);							//擦除
	iap_write_bytes(addr&0xFE00, temp, 512);		//写入
}

//eeprom地址
//	extern_iap_write_float(0.34,2,2,0x07);
//	extern_iap_write_float(300,3,1,0x10);	//70.0,14.0
//	extern_iap_write_float(15,3,1,0x17);
//	extern_iap_write_float(300,3,1,0x20);
//	extern_iap_write_float(15,3,1,0x27);
//	extern_iap_write_float(50,3,0,0x30);
//	
//	extern_iap_write_float(0,2,0,0x50);
//  extern_iap_write_float(0,2,0,0x56);
//  extern_iap_write_float(0,1,0,0x60);
//  extern_iap_write_float(0,2,0,0x65);
//  extern_iap_write_float(0,2,0,0x70);
//  extern_iap_write_float(0,10,0,0x80);
//  extern_iap_write_float(0,1,0,0x90);
//  extern_iap_write_float(0,2,0,0xa0);
//  extern_iap_write_float(0,2,0,0xa6);
//  extern_iap_write_float(0,2,0,0xb0);
//  extern_iap_write_float(0,2,0,0xb6);	
//	
//	extern_iap_write_float(26,3,0,0xc0);
//  extern_iap_write_float(40,3,0,0xc7);
//  extern_iap_write_float(40,3,0,0xd0);
//  extern_iap_write_float(45,3,0,0xd7);
//  extern_iap_write_float(1766,5,0,0xe0);
//  extern_iap_write_float(14959,5,0,0xf0);
//  extern_iap_write_float(35,3,0,0x100);
//  extern_iap_write_float(270,3,0,0x107);
//  extern_iap_write_float(0.86,1,2,0x110);
//  extern_iap_write_float(1.14,1,2,0x117);
//	extern_iap_write_float(71,3,0,0x1c7);
//	
//  extern_iap_write_float(800,4,0,0x120);
//  extern_iap_write_float(40,3,0,0x130);
//  extern_iap_write_float(40,3,0,0x137);
//  extern_iap_write_float(8500,5,0,0x140);
//  extern_iap_write_float(7500,5,0,0x150);
//  extern_iap_write_float(0.86,1,2,0x160);
//  extern_iap_write_float(1.14,1,2,0x167);
//	extern_iap_write_float(71,3,0,0x1c0);
//	
//  extern_iap_write_float(20,3,0,0x170);
//  extern_iap_write_float(65,3,0,0x177);
//	
//  extern_iap_write_float(70,3,0,0x180);
//	extern_iap_write_float(0.36,1,2,0x190);
//  extern_iap_write_float(1.64,1,2,0x197);
//	extern_iap_write_float(16000,5,0,0x1a0);
//	extern_iap_write_float(43,3,0,0x1d0);
//	
//	extern_iap_write_float(1.60,1,2,0x1b0);
//	extern_iap_write_float(1.60,1,2,0x1b7);

//	extern_iap_write_float(50,3,0,0x1d7);
//	extern_iap_write_float(10,3,0,0x1e0);
//	extern_iap_write_float(20000,5,0,0x1f0);

//	extern_iap_write_float(-8,3,0,0x200);
//	extern_iap_write_float(8,3,0,0x207);
//	extern_iap_write_float(71,3,0,0x217);
//	extern_iap_write_float(400,3,0,0x220);
//  extern_iap_write_float(50,3,0,0x227);
//	extern_iap_write_float(0,2,0,0x230);
//	extern_iap_write_float(80,3,0,0x237);
//	extern_iap_write_float(10,3,0,0x240);
//	extern_iap_write_float(15435,5,0,0x250);
//  extern_iap_write_float(950,4,0,0x260);
//	extern_iap_write_float(60,3,0,0x270);
//	extern_iap_write_float(320,3,0,0x277);
//	extern_iap_write_float(1,1,2,0x280);
//	extern_iap_write_float(0,2,0,0x287);
//	extern_iap_write_float(0,2,0,0x290);
//	extern_iap_write_float(0,2,0,0x297);
//	extern_iap_write_float(0,2,0,0x2a0);
//	extern_iap_write_float(0,2,0,0x2a7);
//	extern_iap_write_float(0,2,0,0x2b0);
//	extern_iap_write_float(0,2,0,0x2b7);
//	extern_iap_write_float(0,2,0,0x2c0);
//	extern_iap_write_float(0,2,0,0x2c7);
//	extern_iap_write_float(0,2,0,0x2d0);
//	extern_iap_write_float(0,2,0,0x2d7);
//	extern_iap_write_float(0,2,0,0x2e0);
//	extern_iap_write_float(0,2,0,0x2e7);
//	extern_iap_write_float(0,2,0,0x2f0);

//extern_iap_write_float(0.15,2,2,0x00);//(30,3,1,0x00)中的30代表要写入的数，3代表整数位，1代表小数位。
//这句话的意思是把30转换为"+030.0"存入0x00-0x07地址中，
//因为"+030.0"占六个地址，加上字符串的结束字符'/0'，所以占了8个地址，
//所以是0x00到0x07,下个数据要存以到0x08为起始地址。
//extern_iap_write_float(0.34,2,2,0x07);
void extern_iap_write_float(double dat,uint8 num,uint8 pointnum,uint16 addr)
{
  uint32   length;
	int8    buff[34];
	int8    start,end,point;

	if(0>dat)   
		length = zf_sprintf( &buff[0],"%f",dat);//负数
	else
	{
		length = zf_sprintf( &buff[1],"%f",dat);
		length++;
	}
	point = length - 7;         //计算小数点位置
	start = point - num - 1;    //计算起始位
	end = point + pointnum + 1; //计算结束位
	while(0>start)//整数位不够  末尾应该填充空格
	{
		buff[end] = ' ';
		end++;
		start++;
	}
    
	if(0>dat)   buff[start] = '-';
  else        buff[start] = '+';
    
	buff[end-1] = '\0';
	buff[end] = '\n';
	extern_iap_write_bytes(addr,(uint8 *)buff,num+pointnum+3);
}
/*extern_iap_write_float(motor_left.encoder_data, 2, 2, 0x00);//地址是0x00开始，要求整数位数为2，小数位数为2			
extern_iap_write_float(motor_right.encoder_data, 2, 2, 0x08);
iap_read_bytes(0x00, read_buff, 16);// 从EEPROM中读取数据到read_buff数组中				
wireless_uart_send_buff(read_buff, 16);	*/		

