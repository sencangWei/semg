/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "ring_buffer.h"
#include "ads1299.h"
#include "cmd_frame.h"
#include "delay.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// 每个从缓存区，获取的采样数据长度，单位字节
#define READ_BUFFER_LEN  2240  //2000*0.02*56

// 采样数据缓存区
uint8_t read_buffer[READ_BUFFER_LEN];

// 环形缓存区，用于缓存采样数据
ring_buffer_t g_ads129x_ring_buffer;  

// 采集状态标置位
uint8_t g_sample_status;	

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  delay_init(72);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

	// ADS1299初始化
  ads1299_init();
	
	// ADS1299初始化成功LED亮，否则灭
	if(ads1299_info[0].init_state && ads1299_info[1].init_state)
	{
	   LED_L; 
	}
	else
	{
	   LED_H;
	}

	// 初始化环形缓存区
  if (!ring_buffer_init(&g_ads129x_ring_buffer, READ_BUFFER_LEN*2)) {
       printf("ads129x_ring_buffer init fail! \n");
       return -1;
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
		// 接收串口数据包
		if(g_usart1_buff.rx_done > 0)
		{
			 g_usart1_buff.rx_done = 0;		
			
			 // 解析指令-》执行指令
			 command_parser(g_usart1_buff.rx,g_usart1_buff.rx_len);
			
			 memset(g_usart1_buff.rx,0,g_usart1_buff.rx_len);
			 g_usart1_buff.rx_len = 0; 
			 
			 // 开始DMA接收串口数据
			 HAL_UART_Receive_DMA(&huart1,g_usart1_buff.rx,USART_RX_BUFF_SIZE);
		}
		
		 
		if(g_sys_var.sample_state == 1)
		{  
			// 判断采样缓存区，是否达到READ_BUFFER_LEN字节，如果达到，则需要从缓存区读取数据
			if(g_ads129x_ring_buffer.dataCount >= READ_BUFFER_LEN)
			{
					 // 从环形缓存区提取READ_BUFFER_LEN
					 ring_buffer_read(&g_ads129x_ring_buffer,read_buffer,READ_BUFFER_LEN);	

				   // 将缓存的数据，计算成电压值，单位uV
					 uint8_t n_num = READ_BUFFER_LEN/28;
					 for(uint8_t n = 0;n < n_num ; n++)			
					 {
							static float chx_val[8];									
							for (uint8_t ch = 0; ch < 8; ch++)
							{
								 uint16_t index = 28*n + 1 + 3 + 3 * ch;
								 float val = ((int32_t)(read_buffer[index] << 24 | read_buffer[index + 1] << 16 | read_buffer[index + 2] << 8)) / 256.0f;
									
								 // ((2*4.5)/2^24)*10^6  =  0.5364 
								 chx_val[ch] = (val * 0.5364f) / ads1299_info[0].pga;  // 单位uV 
							}
							memcpy(ack_frame_info.data+32*n , (uint8_t *)chx_val,32);
					 }	

					 // 数据帧打包
					 ack_frame_info_pack_2(&ack_frame_info,0x00,CMD_RAW_DATA,n_num*32);
										
					 // 串口发送数据包
					 HAL_UART_Transmit_DMA(&huart1, (uint8_t *)(&ack_frame_info), ack_frame_info.frame_length);	
			}
		}
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
