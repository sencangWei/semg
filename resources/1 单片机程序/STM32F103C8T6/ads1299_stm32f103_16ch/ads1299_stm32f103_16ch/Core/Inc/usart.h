/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */


// 串口接收缓存区大小
#define USART_RX_BUFF_SIZE  100

// 串口发送缓存区大小
#define USART_TX_BUFF_SIZE  255

typedef struct
{
	// 串口接收缓存区
	uint8_t rx[USART_RX_BUFF_SIZE];
	
	// 串口实际接收数据长度
	uint8_t rx_len;
	
	// 串口接收数据完成
	uint8_t rx_done;
	
	// 串口发送缓存区
	uint8_t tx[USART_TX_BUFF_SIZE];
	
	// 串口实际放送数据长度
	uint8_t tx_len;
//	
//	// 串口发送数据完成
//	uint8_t tx_done;
	
} usart_buff_t;


extern usart_buff_t g_usart1_buff;

/* USER CODE END Includes */

extern UART_HandleTypeDef huart1;

extern UART_HandleTypeDef huart3;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_USART1_UART_Init(void);
void MX_USART3_UART_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

