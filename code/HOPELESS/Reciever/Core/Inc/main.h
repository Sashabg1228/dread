/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define HAND_PWM_Pin GPIO_PIN_1
#define HAND_PWM_GPIO_Port GPIOA
#define HAND_DIR_Pin GPIO_PIN_2
#define HAND_DIR_GPIO_Port GPIOA
#define DISK_DIR_Pin GPIO_PIN_4
#define DISK_DIR_GPIO_Port GPIOA
#define WHEEL_LEFT_DIR_Pin GPIO_PIN_5
#define WHEEL_LEFT_DIR_GPIO_Port GPIOA
#define WHEEL_RIGHT_DIR_Pin GPIO_PIN_6
#define WHEEL_RIGHT_DIR_GPIO_Port GPIOA
#define DISK_PWM_Pin GPIO_PIN_7
#define DISK_PWM_GPIO_Port GPIOA
#define WHEEL_LEFT_PWM_Pin GPIO_PIN_0
#define WHEEL_LEFT_PWM_GPIO_Port GPIOB
#define WHEEL_RIGHT_PWM_Pin GPIO_PIN_1
#define WHEEL_RIGHT_PWM_GPIO_Port GPIOB
#define HAND_END_RIGHT_Pin GPIO_PIN_14
#define HAND_END_RIGHT_GPIO_Port GPIOB
#define HAND_END_RIGHT_EXTI_IRQn EXTI15_10_IRQn
#define HAND_END_LEFT_Pin GPIO_PIN_15
#define HAND_END_LEFT_GPIO_Port GPIOB
#define HAND_END_LEFT_EXTI_IRQn EXTI15_10_IRQn
#define NRF24L01_CE_Pin GPIO_PIN_10
#define NRF24L01_CE_GPIO_Port GPIOA
#define NRF24L01_CSN_Pin GPIO_PIN_15
#define NRF24L01_CSN_GPIO_Port GPIOA
#define OPT_END_WHEEL_RIGHT_Pin GPIO_PIN_7
#define OPT_END_WHEEL_RIGHT_GPIO_Port GPIOB
#define OPT_END_WHEEL_RIGHT_EXTI_IRQn EXTI9_5_IRQn
#define OPT_END_WHEEL_LEFT_Pin GPIO_PIN_8
#define OPT_END_WHEEL_LEFT_GPIO_Port GPIOB
#define OPT_END_WHEEL_LEFT_EXTI_IRQn EXTI9_5_IRQn
#define OPT_END_DISK_Pin GPIO_PIN_9
#define OPT_END_DISK_GPIO_Port GPIOB
#define OPT_END_DISK_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
