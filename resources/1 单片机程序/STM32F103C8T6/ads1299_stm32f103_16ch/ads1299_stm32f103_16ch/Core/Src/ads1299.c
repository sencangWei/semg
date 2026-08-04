#include "ads1299.h"
#include "spi.h"
#include "usart.h"
#include "delay.h"
#include "string.h"
#include "stdio.h"

// SPI 发送缓存
static uint8_t spi_tx_buf[32];

// SPI 接收缓存
static uint8_t spi_rx_buf[64];

// ADS1299寄存器
uint8_t ads1299_regs[2][24]; 

// ADS1299 配置信息
ads1299_info_t ads1299_info[2];

/**
 * @brief  SPI读写
 * @param  tx: 发送数据缓冲区指针 
 * @param  rx: 接收数据缓冲区指针
 * @param  len: 发送和接收的数据长度
 * @retval 无
 */
void ads1299_spi_transfer(uint8_t *tx, uint8_t *rx, uint16_t len)
{
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, len, 100);
}

/**
 * @brief  设置SPI速率
 * @param  baud_rate_prescaler : 分频系数  
 * @retval 无
 */
void ads1299_set_spi_rate(uint32_t baud_rate_prescaler) 
{
    hspi1.Init.BaudRatePrescaler = baud_rate_prescaler;
    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        Error_Handler();
    }
}


void ads1299_cs_set_low(uint8_t cs_num)
{
	switch(cs_num)
	{
	  case 0:  ADS1299_CS1_L;break;
		case 1:  ADS1299_CS2_L;break;
	}
}

void ads1299_cs_set_high(uint8_t cs_num)
{
	switch(cs_num)
	{
	  case 0:  ADS1299_CS1_H;break;
		case 1:  ADS1299_CS2_H;break;
	}
}

/**
 * @brief  发送命令
 * @param  cmd: 命令码 
 * @retval 无
 */
void ads1299_write_command(uint8_t cs_num,uint8_t cmd)
{
	 ads1299_cs_set_low(cs_num);
   spi_tx_buf[0] = cmd;
   ads1299_spi_transfer(spi_tx_buf, spi_rx_buf, 1);
	ads1299_cs_set_high(cs_num);
}

/**
 * @brief  连续写入寄存器
 * @param  addr: 寄存器起始地址 
 * @param  regs: 寄存器数组指针
 * @param  len:  写入寄存器数量
 * @retval 无
 */
void ads1299_write_regs(uint8_t cs_num,uint8_t addr, uint8_t *regs, uint8_t len)
{
    spi_tx_buf[0] = 0x40 + addr;
    spi_tx_buf[1] = 0x00 + len - 1;
	  ads1299_cs_set_low(cs_num);
    ads1299_spi_transfer(spi_tx_buf, spi_rx_buf, 2);
    ads1299_spi_transfer(regs, spi_rx_buf, len);
	  ads1299_cs_set_high(cs_num);
}

/**
 * @brief  连续读取寄存器
 * @param  addr: 寄存器起始地址 
 * @param  regs: 寄存器数组指针
 * @param  len:  读取寄存器数量
 * @retval 无
 */
void ads1299_read_regs(uint8_t cs_num,uint8_t addr, uint8_t *regs, uint8_t len)
{
    spi_tx_buf[0] = 0x20 + addr;
    spi_tx_buf[1] = 0x00 + len - 1;
	  ads1299_cs_set_low(cs_num);
    ads1299_spi_transfer(spi_tx_buf, spi_rx_buf, 2);
    memset(spi_tx_buf, 0x00, sizeof(spi_tx_buf));
    ads1299_spi_transfer(spi_tx_buf, regs, len);
	  ads1299_cs_set_high(cs_num);
}

/**
 * @brief  ADS1299上电复位
 * @retval 无
 */
void ads1299_reset(void)
{
    ADS1299_CS1_H;
	  ADS1299_CS2_H;
    ADS1299_START_L;
    ADS1299_REST_H;
    ADS1299_PWDN_H; 
    delay_ms(200); // 将PWDN引脚拉高后，等待稳定   
    
	  for(uint8_t cs = 0;cs<2;cs++)
		{
			 //发送停止连续读取指令
	     ads1299_write_command(cs,ADS1299_SDATAC); 
		}

    // 复位设备
    ADS1299_REST_L;
    delay_ms(10);
    ADS1299_REST_H;
    delay_ms(50); 
    
		for(uint8_t cs = 0;cs<2;cs++)
		{
					// 发送停止连续读取指令
			ads1299_write_command(cs,ADS1299_SDATAC); 
			
			// 发送停止采集指令
			ads1299_write_command(cs,ADS1299_STOP);   
		}
}

/**
 * @brief  ADS1299 寄存器初始化
 * @retval 无
 */
void ads1299_regs_init(void)
{
	  for(uint8_t cs = 0;cs<2;cs++)
		{
				ads1299_regs[cs][0x00] = 0x1E;   // ID
				ads1299_regs[cs][0x01] = 0xD3;  //0x93;   // CONFIG1 0x96(0.25kSPS) 0x95(0.5kSPS)  0x94(1kSPS)  0x93(2kSPS) 
				ads1299_regs[cs][0x02] = 0xD4;   // CONFIG2 在内部生成测试信号
				ads1299_regs[cs][0x03] = 0xEC;   // CONFIG3 
				ads1299_regs[cs][0x04] = 0x00;   // LOFF

				// 通道配置 
				for(uint8_t ch = 0;ch<8;ch++)
				{
					// 采集输入信号
					ads1299_regs[cs][0x05 + ch] = 0x50;  
				
		//			// 采集短路噪声
		//      ads1299_regs[cs][0x05 + ch] = 0x51;  
		//			
		//			// 采集测试信号 
		//      ads1299_regs[cs][0x05 + ch] = 0x55;  
				}	
				
				ads1299_regs[cs][0x0D] = 0x00;   // RLD_SENSP
				ads1299_regs[cs][0x0E] = 0x00;   // RLD_SENSN
				ads1299_regs[cs][0x0F] = 0x00;   // LOFF_SENSP 关闭导联脱落检测
				ads1299_regs[cs][0x10] = 0x00;   // LOFF_SENSN 关闭导联脱落检测
				ads1299_regs[cs][0x11] = 0x00;   // LOFF_FLIP
				ads1299_regs[cs][0x12] = 0x00;   // LOFF_STATP
				ads1299_regs[cs][0x13] = 0x00;   // LOFF_STATN
				ads1299_regs[cs][0x14] = 0x10;   // GPIO 控制GPIO1输出高电平，点亮LED灯
				ads1299_regs[cs][0x15] = 0x20;   // MISC1  //单端模式
				ads1299_regs[cs][0x16] = 0x00;   // MISC2
				ads1299_regs[cs][0x17] = 0x00;   // CONFIG4 连续转化模式

			  ads1299_info[cs].ch_en = 0xFF;  // 默认开启全部通道
				ads1299_info[cs].ch_sw = 0x00;  // 测量电极输入信号
				ads1299_info[cs].pga = 12;      // 默认PGA为12倍，与初始化寄存器保持一致
				ads1299_info[cs].rate = 500;    // 默认采样率为500sps，与初始化寄存器保持一致  
		}
}

/**
 * @brief  ADS1299 初始化
 * @retval 无
 */
void ads1299_init(void)
{    
    // 上电复位
    ads1299_reset();

    // 寄存器初始化
	  ads1299_regs_init();
	
		for(uint8_t cs = 0;cs<2;cs++)
		{
				// 写入寄存器 
			ads1299_write_regs(cs,0x01, &(ads1299_regs[cs][0]) + 1, 23);
			
			// 延时1毫秒
			delay_ms(1);
			
			// 读取寄存器
			ads1299_read_regs(cs,0x00, &(ads1299_regs[cs][0]), 24);
			
			// ID寄存器，去掉版本号, ID = 0x1E
			ads1299_regs[cs][0] &= 0x1F;
			
			// 提高SPI速率传输速率
			ads1299_set_spi_rate(SPI_BAUDRATEPRESCALER_8);
		
			// 打印从器件读取的寄存器值 
			for (uint8_t n = 0; n < 24; n++)
			{
					printf("ads1299_regs[0x%x] = 0x%x \r\n", n, ads1299_regs[cs][n]);
			}
			
			 // 判断ADS1299芯片是否初始化成功
		   ads1299_info[cs].init_state = (ads1299_regs[cs][0] == 0x1E) ? 0x01:0x00; 
		}
}

// 设置采样率、PGA放大倍数、通道输入
// 采样率: 250、500、1000、2000、4000
// 量程: ±4.5V(PGA=1)、±2.2V(PGA=2)、±1.1V(PGA=4)、±750mV(PGA=6)、±560mV(PGA=8)、±375mV(PGA=12)、±186mV(PGA=24)
// 通道选择:
// 0x00: 电极输入
// 0x01: 正负极内部短路
// 0x02: 测试信号
/**
 * @brief  更新采样参数
 * @retval 无
 */
void ads1299_update_sample_parameter()
{ 
	  // 降低SPI速率，配置寄存器
    ads1299_set_spi_rate(SPI_BAUDRATEPRESCALER_64);
    delay_ms(1);
	
	
		for(uint8_t cs = 0;cs<2;cs++)
		{	
				// 采样率
				uint8_t rate_reg = 0x05;
				switch(ads1299_info[cs].rate)
				{
					case 250: rate_reg = 0x06;break;
					case 500: rate_reg = 0x05;break;
					case 1000: rate_reg = 0x04;break;
					case 2000: rate_reg = 0x03;break;	
					case 4000: rate_reg = 0x02;break;	
				}
				ads1299_regs[cs][0x01] &= 0xF8;
				ads1299_regs[cs][0x01] |= rate_reg;  
				
				// PGA
				uint8_t pga_reg = 0x00;
				switch(ads1299_info[cs].pga)
				{
					case 1: pga_reg = 0x00;break;
					case 2: pga_reg = 0x01;break;
					case 4: pga_reg = 0x02;break;
					case 6: pga_reg = 0x03;break;	
					case 8: pga_reg = 0x04;break;	
					case 12: pga_reg = 0x05;break;	
					case 24: pga_reg = 0x06;break;	
				}
				for (uint8_t ch = 0; ch < 8; ch++)
				{
						uint8_t reg = ads1299_regs[cs][0x05 + ch];
						reg &= 0x8F;
						reg |= pga_reg << 4;
						ads1299_regs[cs][0x05 + ch] = reg;
				} 
				
				// 通道开关
				if (ads1299_info[cs].ch_sw <= 0x02)
				{ 
						uint8_t ch_regs[3] = {0x00, 0x01, 0x05};     
						for (uint8_t ch = 0; ch < 8; ch++)
						{                    
								uint8_t reg = ads1299_regs[cs][0x05 + ch];
								reg &= 0xF8;
								reg |= ch_regs[ads1299_info[cs].ch_sw];     
								ads1299_regs[cs][0x05 + ch] = reg;
						}
				}
				
				 // 通道使能
				for(uint8_t ch=0;ch<8;ch++)
				{					
						uint8_t reg = ads1299_regs[cs][0x05+ch];
					
						if((ads1299_info[cs].ch_en >> ch) & 0x01)
						{
							reg &= 0x7F; // 开启通道
						}
						else
						{
							reg |= 0x80; // 关闭通道
							reg = (reg&0xF8)|0x01;  // 将输入端短路
						}
						ads1299_regs[cs][0x05+ch] = reg;
				}		
				
				
				ads1299_write_command(cs,ADS1299_SDATAC); // 停止连续读取
				ads1299_write_command(cs,ADS1299_STOP);   // 停止采集
				delay_ms(1);
				
				// 写入寄存器 
				ads1299_write_regs(cs,0x01, &(ads1299_regs[cs][0]) + 1, 25);
				
				// 加入延时，确保能读取寄存器成功
				delay_ms(1);
				
				// 读取寄存器
				ads1299_read_regs(cs,0x00, &(ads1299_regs[cs][0]), 26);
				
				// ID寄存器，去掉版本号表示位后 ADS1299 ID = 0x1E
				ads1299_regs[cs][0] &= 0x1F;
				
				// 打印从器件读取的寄存器值 
				for (uint8_t n = 0; n < 25; n++)
				{
						printf("ads1299_regs[0x%x] = 0x%x \r\n", n, ads1299_regs[cs][n]);
				}	
		}
    // 提高SPI速率，读取数据
    ads1299_set_spi_rate(SPI_BAUDRATEPRESCALER_8);
}

/**
 * @brief  停止采集
 * @retval 无
 */
void ads1299_stop(void)
{  
    ads1299_write_command(0,ADS1299_SDATAC); // 停止连续读取
	  ads1299_write_command(1,ADS1299_SDATAC); // 停止连续读取
    ADS1299_START_L;
	  ads1299_cs_set_high(0);
	  ads1299_cs_set_high(1);
}

/**
 * @brief  开始采集
 * @retval 无
 */
void ads1299_start(void)
{
    memset(spi_tx_buf, 0x00, sizeof(spi_tx_buf));
    ADS1299_START_H;	
   	ads1299_write_command(0,ADS1299_SDATAC); 
	  ads1299_write_command(1,ADS1299_SDATAC); 
}

/**
 * @brief  读取数据
 * @param  ch_val: 通道数据数组指针
 * @retval 无
 */
void ads1299_read_data(uint8_t *chx_buff)
{    
	  static float val_test = 0;
	  static uint8_t cs_num = 0;
	  static uint8_t cs_index	= 0;
	  static uint8_t cs_ch_index = 0;
	  static uint8_t ch = 0; 
	  static uint8_t index = 0;
	  static float val = 0;
	
	
	  spi_tx_buf[0] = ADS1299_RDATA;
	
		ads1299_cs_set_low(0);
	  //ads1299_spi_transfer(spi_tx_buf, chx_buff, 1); 
	  //ads1299_spi_transfer(spi_tx_buf+1, chx_buff + 1, 27);
	  ads1299_spi_transfer(spi_tx_buf, chx_buff, 28);
	  ads1299_cs_set_high(0);	
	
		ads1299_cs_set_low(1);	
	  //ads1299_spi_transfer(spi_tx_buf, chx_buff+28, 1); 
	  //ads1299_spi_transfer(spi_tx_buf+1, chx_buff + 29, 27);
	  ads1299_spi_transfer(spi_tx_buf, chx_buff + 28, 28);
	  ads1299_cs_set_high(1);	
}