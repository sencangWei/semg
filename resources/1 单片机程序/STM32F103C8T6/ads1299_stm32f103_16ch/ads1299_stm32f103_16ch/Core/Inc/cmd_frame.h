#ifndef __CMD_FRAME_H
#define __CMD_FRAME_H

#include "stdint.h"
#include <stdio.h>
#include <string.h>

// 帧格式定义
#define FRAME_START_BYTE     0xA5    // 帧起始字节
#define FRAME_END_BYTE       0x5A    // 帧结束字节

#pragma pack(push, 1)   // 设置对齐基数为1，即无填充
typedef struct {
    
	  // 起始字节
		// 用于帧开始标识
		// 请求帧（主机->设备）：帧头为0xA5
		// 响应帧（设备->主机）：帧头为0xAA
	  uint8_t  start_byte;    
    
	  // 帧长度 
		// 2字节无符号整数，小端模式
		// 表示整帧的总字节数（包含帧头、长度、地址、命令、帧头校验、数据、数据校验、帧尾）
		// 最小帧长度：6字节（无数据的情况）
		// 最大数据长度：65535-6 = 65529字节  
	  uint16_t frame_length;   
	  
	  // 地址:
		// 请求帧（主机->设备）：地址字段 = 目标设备地址（即设备地址）
		// 响应帧（设备->主机）：地址字段 = 设备自己的地址（即源地址）
		// 0x00: 广播地址（所有设备）
		// 0x01-0xFE: 设备地址（1-254）
    // 0xFF: 主机地址（PC/上位机）
	  uint8_t addr;           // 设备地址
	
		// 命令码
	  // 位定义：
		// 位[7]: 读/写标志位
		//    0 = 读操作 (Read)
		//    1 = 写操作 (Write)
		// 位[6:0]: 命令码（0x00-0x7F）
		//    0x00-0x7F: 自定义命令
    uint8_t cmd;   

    // 帧头校验
    // 1个字节，异或校验
    // 校验范围：长度+地址+命令 三个字段
    uint8_t header_check; 
	  
		//  变长数据，0-65529字节
		// 数据格式由具体命令决定
    uint8_t data[2560];      
		
	
		// 数据校验
    //	1个字节，异或校验
    // 校验范围：数据字段 所有字节
    uint8_t data_check;         
		
		// 结束字节
    // 用于帧结束标识
    // 请求帧（主机->设备）：帧头为0x5A
    // 响应帧（设备->主机）：帧头为0x55
    uint8_t  end_byte;      
				
} frame_info_t;
#pragma pack(pop)   

enum
{   
	  CMD_SW_VERSION = 0x00,  //获取软件版本
	  CMD_HW_VERSION = 0x01,  //获取硬件版本  
	  CMD_CONN_STATE =  0x03, // 连接状态更新
	  CMD_SAMPLE_CONTROL = 0x04,//开始/停止采集数据
	  CMD_SMAPLE_PAR = 0x05,  //采样参数配置
	  CMD_RAW_DATA  = 0x06, // 返回采样数据包
	
};//命令码


typedef struct
{
	// 0x00: 停止采集
	// 0x01: 开始采集,返回采样数据
	uint8_t sample_state;
	
	// 0x00: 与上位机软件断开连接
	// 0x01: 与上位机软件连接成功
	uint8_t conn_state;
	  
} sys_var_t; // 系统变量


extern sys_var_t g_sys_var;
extern frame_info_t ack_frame_info;

void command_parser(uint8_t *buffer, uint16_t buffer_len);
void ack_frame_info_pack(frame_info_t *p,uint8_t addr, uint8_t cmd,uint8_t *data,uint16_t data_len);
void ack_frame_info_pack_2(frame_info_t *p,uint8_t addr, uint8_t cmd,uint16_t data_len);
#endif
