#ifndef __ADS1299_H
#define __ADS1299_H

#include "main.h"

#define ADS1299_CS1_PORT               ADS129X_CS1_GPIO_Port
#define ADS1299_CS1_PIN                ADS129X_CS1_Pin
#define ADS1299_CS2_PORT               ADS129X_CS2_GPIO_Port
#define ADS1299_CS2_PIN                ADS129X_CS2_Pin
#define ADS1299_START_PORT             ADS129X_START_GPIO_Port
#define ADS1299_START_PIN              ADS129X_START_Pin
#define ADS1299_PWDN_PORT              ADS129X_PWDN_GPIO_Port
#define ADS1299_PWDN_PIN               ADS129X_PWDN_Pin
#define ADS1299_DRDY_PORT              ADS129X_DRDY_GPIO_Port
#define ADS1299_DRDY_PIN               ADS129X_DRDY_Pin
#define ADS1299_REST_PORT              ADS129X_REST_GPIO_Port
#define ADS1299_REST_PIN               ADS129X_REST_Pin

//#define ADS1299_CS1_H			HAL_GPIO_WritePin(ADS1299_CS1_PORT, ADS1299_CS1_PIN,GPIO_PIN_SET)
//#define ADS1299_CS1_L			HAL_GPIO_WritePin(ADS1299_CS1_PORT, ADS1299_CS1_PIN,GPIO_PIN_RESET)
//#define ADS1299_CS2_H			HAL_GPIO_WritePin(ADS1299_CS2_PORT, ADS1299_CS2_PIN,GPIO_PIN_SET)
//#define ADS1299_CS2_L			HAL_GPIO_WritePin(ADS1299_CS2_PORT, ADS1299_CS2_PIN,GPIO_PIN_RESET)

// 寄存器操作版本，不影响其他IO
#define ADS1299_CS1_H            (GPIOA->BSRR = ADS1299_CS1_PIN)
#define ADS1299_CS1_L            (GPIOA->BRR  = ADS1299_CS1_PIN)
#define ADS1299_CS2_H            (GPIOB->BSRR = ADS1299_CS2_PIN)
#define ADS1299_CS2_L            (GPIOB->BRR  = ADS1299_CS2_PIN)

#define ADS1299_PWDN_H		HAL_GPIO_WritePin(ADS1299_PWDN_PORT, ADS1299_PWDN_PIN,GPIO_PIN_SET)
#define ADS1299_PWDN_L		HAL_GPIO_WritePin(ADS1299_PWDN_PORT, ADS1299_PWDN_PIN,GPIO_PIN_RESET)
#define ADS1299_START_H		HAL_GPIO_WritePin(ADS1299_START_PORT, ADS1299_START_PIN,GPIO_PIN_SET)
#define ADS1299_START_L		HAL_GPIO_WritePin(ADS1299_START_PORT, ADS1299_START_PIN,GPIO_PIN_RESET)
#define ADS1299_REST_H		HAL_GPIO_WritePin(ADS1299_REST_PORT, ADS1299_REST_PIN,GPIO_PIN_SET)
#define ADS1299_REST_L		HAL_GPIO_WritePin(ADS1299_REST_PORT, ADS1299_REST_PIN,GPIO_PIN_RESET)


#define LED_H		HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin,GPIO_PIN_SET)
#define LED_L		HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin,GPIO_PIN_RESET)

/*ADS1299命令定义*/
/*系统命令*/
#define ADS1299_WAKEUP	        	0X02	//从待机模式唤醒
#define ADS1299_STANDBY	        0X04	//进入待机模式
#define ADS1299_ADSRESET        	0X06	//复位
#define ADS1299_START	        	0X08	//启动或转换
#define ADS1299_STOP	        		0X0A	//停止转换
#define ADS1299_OFFSETCAL				0X1A	//通道偏移校准
/*数据读取命令*/
#define ADS1299_RDATAC	        	0X10	//启用连续的数据读取模式,默认使用此模式
#define ADS1299_SDATAC	        	0X11	//停止连续的数据读取模式
#define ADS1299_RDATA	        	0X12	//通过命令读取数据;支持多种读回。
/*寄存器读取命令*/
#define	ADS1299_RREG	        		0X20	//读取001r rrrr 000n nnnn  这里定义的只有高八位，低8位在发送命令时设置  r rrrr=要读、写的寄存器地址
#define ADS1299_WREG	        		0X40	//写入010r rrrr 000n nnnn   n nnnn=要读、写的数据

/* ADS1299内部寄存器地址定义	*/
#define ADS1299_REG_ID            0x00        // ID Control Register : The ID Control Register is programmed during device manufacture to indicate device characteristics
#define ADS1299_REG_CONFIG1       0x01        // Configuration Register 1
#define ADS1299_REG_CONFIG2       0x02        // Configuration Register 2
#define ADS1299_REG_CONFIG3       0x03        // Configuration Register 3
#define ADS1299_REG_LOFF          0x04        // Lead-Off Control Register
#define ADS1299_REG_CH1SET        0x05        // The CH[1]SET Control Register configures the power mode, PGA gain, and multiplexer settings channels
#define ADS1299_REG_CH2SET        0x06        // The CH[2]SET Control Register configures the power mode, PGA gain, and multiplexer settings channels
#define ADS1299_REG_CH3SET        0x07        // The CH[3]SET Control Register configures the power mode, PGA gain, and multiplexer settings channels
#define ADS1299_REG_CH4SET        0x08        // The CH[4]SET Control Register configures the power mode, PGA gain, and multiplexer settings channels
#define ADS1299_REG_CH5SET        0x09        // The CH[5]SET Control Register configures the power mode, PGA gain, and multiplexer settings channels
#define ADS1299_REG_CH6SET        0x0A        // The CH[6]SET Control Register configures the power mode, PGA gain, and multiplexer settings channels
#define ADS1299_REG_CH7SET        0x0B        // The CH[7]SET Control Register configures the power mode, PGA gain, and multiplexer settings channels
#define ADS1299_REG_CH8SET        0x0C        // The CH[8]SET Control Register configures the power mode, PGA gain, and multiplexer settings channels
#define ADS1299_REG_RLD_SENSP     0x0D        // Controls the selection of the positive signals from each channel for right leg drive derivation
#define ADS1299_REG_RLD_SENSN     0x0E        // Controls the selection of the negative signals from each channel for right leg drive derivation
#define ADS1299_REG_LOFF_SENSP    0x0F        // Selects the positive side from each channel for lead-off detection
#define ADS1299_REG_LOFF_SENSN    0x10        // Selects the negative side from each channel for lead-off detection
#define ADS1299_REG_LOFF_FLIP     0x11        // Controls the direction of the current used for lead-off derivation
#define ADS1299_REG_LOFF_STATP    0x12        // Stores the status of whether the positive electrode on each channel is on or off (Read-Only Register)
#define ADS1299_REG_LOFF_STATN    0x13        // Stores the status of whether the negative electrode on each channel is on or off (Read-Only Register)
#define ADS1299_REG_GPIO          0x14        // General-Purpose I/O Register: controls the action of the three GPIO pins
#define ADS1299_REG_PACE          0x15        // PACE Detect Register: configure the channel signal used to feed the external PACE detect circuitry
#define ADS1299_REG_RESP          0x16        // Respiration Control Register: provides the controls for the respiration circuitry
#define ADS1299_REG_CONFIG4       0x17        // Configuration Register 4: 
#define ADS1299_REG_WCT1          0x18        // Wilson Central Terminal and Augmented Lead Control Register
#define ADS1299_REG_WCT2          0x19        // Wilson Central Terminal Control Register


typedef struct {
	
	// 采样率
	uint16_t rate; 
	
	// 模拟前端放大倍数
	uint8_t pga;
	
	// 通道使能
	uint8_t ch_en;
	
	// 通道输入
	uint8_t ch_sw;
	
	// 芯片初始化状态
	uint8_t init_state;
	
}ads1299_info_t;

extern ads1299_info_t ads1299_info[2];

void ads1299_init(void); //初始化ADS1292R
void ads1299_start(); //开始采集
void ads1299_stop();  //停止采集
//void ads1299_read_data(float *chx_val);//读ADS1299数据
void ads1299_read_data(uint8_t *chx_buff);
void ads1299_update_sample_parameter();

//void ads1299_read_data1(uint8_t *chx_buff);
//void ads1299_read_data2(uint8_t *chx_buff);

#endif

