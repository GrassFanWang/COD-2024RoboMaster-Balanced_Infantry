/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_uart.c
  * @brief          : bsp uart functions 
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : none
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BSP_UART_H
#define BSP_UART_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include <stdio.h>
#include <stdarg.h>
#include "string.h"

/* Exported functions prototypes ---------------------------------------------*/
/**
  * @brief  Configures the USART.
  */
extern void BSP_USART_Init(void);
extern void usart_printf(const char *fmt,...);
#define printf(title, fmt, args...) usart_printf("{"#title"}"fmt"\n", ##args)
#endif //BSP_UART_H
