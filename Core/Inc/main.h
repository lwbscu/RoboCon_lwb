/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "gpio.h"
#include "tim.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_TIM1_Init(void);
void MX_TIM2_Init(void);
void MX_TIM3_Init(void);
void MX_TIM4_Init(void);     // 新增：主电机编码器初始化函数
void MX_USART2_UART_Init(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */
#define PWM_MAX_VALUE 1000

// GPIO定义 - 更新引脚分配注释
// PA8  - TIM1_CH1 - PWM输出
// PA9  - AIN1     - 从电机方向控制1
// PA10 - AIN2     - 从电机方向控制2
// PA0  - TIM2_CH1 - 从电机编码器A相
// PA1  - TIM2_CH2 - 从电机编码器B相
// PA2  - USART2_TX - 串口发送
// PA3  - USART2_RX - 串口接收
// PB0  - STBY     - TB6612使能控制（修正）
// PB6  - TIM4_CH1 - 主电机编码器A相
// PB7  - TIM4_CH2 - 主电机编码器B相
// PC13 - BUTTON   - 模式切换按键
// PC14 - BUTTON   - 正转按键
// PC15 - BUTTON   - 反转按键

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */