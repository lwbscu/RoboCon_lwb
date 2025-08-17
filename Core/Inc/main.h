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
// 包含自定义头文件（在HAL库包含之后）
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
void MX_TIM1_Init(void);  // PWM输出
void MX_TIM2_Init(void);  // 编码器输�?
void MX_TIM3_Init(void);  // 控制周期定时�?
void MX_USART2_UART_Init(void);  // 串口通信(改为USART2)
void Error_Handler(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */
// TB6612硬件引脚定义
#define MOTOR_PWM_PIN GPIO_PIN_8     // PA8 - TIM1_CH1
#define MOTOR_PWM_PORT GPIOA
#define MOTOR_AIN1_PIN GPIO_PIN_9    // PA9 - 方向控制1
#define MOTOR_AIN1_PORT GPIOA
#define MOTOR_AIN2_PIN GPIO_PIN_10   // PA10 - 方向控制2
#define MOTOR_AIN2_PORT GPIOA
#define MOTOR_STBY_PIN GPIO_PIN_6    // PB6 - 待机控制
#define MOTOR_STBY_PORT GPIOB

#define ENCODER_A_PIN GPIO_PIN_0     // PA0 - TIM2_CH1
#define ENCODER_A_PORT GPIOA
#define ENCODER_B_PIN GPIO_PIN_1     // PA1 - TIM2_CH2
#define ENCODER_B_PORT GPIOA

#define BUTTON_UP_PIN GPIO_PIN_13    // PC13 - 正转按键
#define BUTTON_UP_PORT GPIOC
#define BUTTON_DOWN_PIN GPIO_PIN_14  // PC14 - 反转按键
#define BUTTON_DOWN_PORT GPIOC
#define BUTTON_MODE_PIN GPIO_PIN_15  // PC15 - 模式切换按键
#define BUTTON_MODE_PORT GPIOC

// 系统参数定义
#define CONTROL_FREQUENCY 1000  // 1kHz控制频率
#define ENCODER_PPR 1000        // 编码器每转脉冲数
#define GEAR_RATIO 1.0f         // 齿轮�?
#define PWM_MAX_VALUE 1000      // PWM�?大�??

// 目标转�?�定义（�?/秒）
#define SPEED_LEVEL_1 1.0f
#define SPEED_LEVEL_2 5.0f
#define SPEED_LEVEL_3 10.0f
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
