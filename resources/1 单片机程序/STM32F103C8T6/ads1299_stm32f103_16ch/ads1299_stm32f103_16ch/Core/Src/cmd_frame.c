#include "cmd_frame.h"
#include "usart.h"
#include "ads1299.h"
#include "ring_buffer.h"

// 全局系统变量
sys_var_t g_sys_var;

// 回应数据帧
frame_info_t ack_frame_info;


/**
 * @brief  回应数据帧打包
 * @param  p: 指令结构
 * @retval 元
 */
void ack_frame_info_pack(frame_info_t *p,uint8_t addr, uint8_t cmd,uint8_t *data,uint16_t data_len)
{
  p->start_byte = 0xAA;
	p->frame_length = 8+data_len;
	p->addr = addr;
	p->cmd = cmd;
	p->header_check = 0x00;
	
	for(uint8_t i=1;i<5;i++)
	{
	  p->header_check ^= ((uint8_t *)p)[i];
	}
	memcpy(p->data, data, data_len);	 
	
	p->data_check = 0x00;
	for(uint16_t i=0;i<data_len;i++)
	{
	  p->data_check ^= p->data[i];
	}
	p->end_byte = 0x55;
	
	((uint8_t *)p)[p->frame_length -2] = p->data_check;
	((uint8_t *)p)[p->frame_length -1] = p->end_byte;
}

/**
 * @brief  回应数据帧打包
 * @param  p: 指令结构
 * @retval 元
 */
void ack_frame_info_pack_2(frame_info_t *p,uint8_t addr, uint8_t cmd,uint16_t data_len)
{
  p->start_byte = 0xAA;
	p->frame_length = 8+data_len;
	p->addr = addr;
	p->cmd = cmd;
	p->header_check = 0x00;
	
	for(uint8_t i=1;i<5;i++)
	{
	  p->header_check ^= ((uint8_t *)p)[i];
	}
	//memcpy(p->data, data, data_len);	 
	
	p->data_check = 0x00;
	for(uint16_t i=0;i<data_len;i++)
	{
	  p->data_check ^= p->data[i];
	}
	p->end_byte = 0x55;
	
	((uint8_t *)p)[p->frame_length -2] = p->data_check;
	((uint8_t *)p)[p->frame_length -1] = p->end_byte;
}


/**
 * @brief  连接状态更新回调函数
 * @param  p: 指令结构
 * @retval 元
 */
void order_conn_status_update_callback(frame_info_t *p)
{
	if ((p->cmd&0x80) == 0x00)
	{
		g_sys_var.conn_state = p->data[0];
	}

	printf("g_sys_var.conn_state = %d \r\n", g_sys_var.conn_state);
	
	p->data[0] = g_sys_var.conn_state;
	ack_frame_info_pack(p,p->addr,p->cmd,p->data,1);
	
	// 串口发送数据包
	HAL_UART_Transmit(&huart1, (uint8_t *)p, p->frame_length, 0xffff);	 
}


/**
 * @brief  采样参数配置回调函数
 * @param  p: 指令结构
 * @retval 元
 */
void order_sample_parameter_callBack(frame_info_t *p)
{
	if ((p->cmd&0x80) == 0x00)
	{
		memcpy((uint8_t *)&(ads1299_info[0].rate), p->data, 2);	 
		ads1299_info[0].pga = p->data[2];
		ads1299_info[0].ch_sw = p->data[3];
		
		ads1299_info[1].rate = ads1299_info[0].rate;
		ads1299_info[1].pga = ads1299_info[0].pga;
		ads1299_info[1].ch_sw = ads1299_info[0].ch_sw;
		
		ads1299_info[0].ch_en = p->data[4];
		ads1299_info[1].ch_en = p->data[5];
		
		ads1299_update_sample_parameter();
	}

	memcpy(p->data,(uint8_t *)&(ads1299_info[0].rate), 2);		
	p->data[2]=ads1299_info[0].pga;
  p->data[3]=ads1299_info[0].ch_sw;
	p->data[4]=ads1299_info[0].ch_en;
	p->data[5]=ads1299_info[1].ch_en;
	ack_frame_info_pack(p,p->addr,p->cmd,p->data,6);
	
	// 串口发送数据包
	HAL_UART_Transmit(&huart1, (uint8_t *)p, p->frame_length, 0xffff);	  
}

/**
 * @brief  开始采集指令回调函数
 * @param  p: 指令结构
 * @retval 元
 */
void order_sample_control_callback(frame_info_t *p)
{
	printf("order_sample_control_callback\r\n");
	
	if((p->cmd&0x80) == 0x00)
	{
		uint8_t sample_status = p->data[0] == 0x00 ? 0 : 1;
		if (sample_status)
		{ 
			  extern ring_buffer_t g_ads129x_ring_buffer;
			  ring_buffer_clear(&g_ads129x_ring_buffer);
	      ads1299_start();
		}
		else
		{
			 ads1299_stop();
		}		
	
		g_sys_var.sample_state = sample_status;
    printf("g_sys_var.sample_state = %d\r\n", g_sys_var.sample_state);
	}
}

/**
 * @brief  指令帧解析函数
 * @param  cmd_info: 指令帧结构体
 * @retval 无
 */
void cmd_info_parse(frame_info_t *cmd_info)
{
	// 获取命令码
  uint8_t cmd = cmd_info->cmd & 0x7F;
	switch(cmd)
	{ 
		case CMD_CONN_STATE: order_conn_status_update_callback(cmd_info);break;
		case CMD_SMAPLE_PAR: order_sample_parameter_callBack(cmd_info);break;
		case CMD_SAMPLE_CONTROL: order_sample_control_callback(cmd_info);break;
	}
	
	//printf("cmd_info_parse cmd = 0x%x",cmd_info->cmd);
}


/**
 * @brief  指令解析函数
 * @param  buffer: 
 * @param  buffer_len:
 * @retval 无
 */
void command_parser(uint8_t *buffer, uint16_t buffer_len)
{
	static uint16_t addr_n = 0;
	static frame_info_t cmd_info;

	addr_n = 0;
	while (buffer_len - addr_n > 5)
	{
		// 查询帧头
		if (buffer[addr_n] == FRAME_START_BYTE)
		{
			// 整帧长度
			uint16_t frame_len =  (buffer[addr_n + 2]<<8 | buffer[addr_n + 1]);

			// 判断剩于数据长度是否大于帧长度
			if (frame_len <= (buffer_len - addr_n))
			{	
				// 校验帧头
				if(buffer[addr_n+5] == (buffer[addr_n+1] ^ buffer[addr_n+2]^buffer[addr_n+3]^buffer[addr_n+4]))
				{
						// 校验帧尾
						if (buffer[addr_n + frame_len - 1] == FRAME_END_BYTE)
						{
							  memcpy((uint8_t *)(&cmd_info), &buffer[addr_n],frame_len-2);
							  cmd_info.data_check = buffer[addr_n+frame_len-2];
							  cmd_info.end_byte = buffer[addr_n+frame_len-1];
							  cmd_info_parse(&cmd_info);		
							  addr_n += frame_len;	
							  break;
						}
						else // 帧尾出错
						{
							addr_n++;
							printf("frame end error! = 0x%x \n",buffer[addr_n + frame_len - 1]);
						}			
				}
				else // 帧头校验出错
				{
				   addr_n++;
					 printf("frame header check error!\n");
				}
			}
			else // 剩于数据长度不足
			{
				addr_n++;
				printf("data length error! \n");
			}
		}
		else // 帧头出错
		{
			addr_n++;
			printf("frame heafer error! \n");
		}
	}
}