/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32h5xx_hal.h"

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
#define LED_4_Pin GPIO_PIN_2
#define LED_4_GPIO_Port GPIOC
#define LED_3_Pin GPIO_PIN_3
#define LED_3_GPIO_Port GPIOC
#define LED_2_Pin GPIO_PIN_0
#define LED_2_GPIO_Port GPIOA
#define LED_1_Pin GPIO_PIN_1
#define LED_1_GPIO_Port GPIOA
#define Synchronization_flag_Pin GPIO_PIN_2
#define Synchronization_flag_GPIO_Port GPIOA
#define Synchronization_clock_Pin GPIO_PIN_3
#define Synchronization_clock_GPIO_Port GPIOA
#define FLASH_IO3_Pin GPIO_PIN_6
#define FLASH_IO3_GPIO_Port GPIOA
#define FLASH_IO2_Pin GPIO_PIN_7
#define FLASH_IO2_GPIO_Port GPIOA
#define FLASH_IO1_Pin GPIO_PIN_0
#define FLASH_IO1_GPIO_Port GPIOB
#define FLASH_IO0_Pin GPIO_PIN_1
#define FLASH_IO0_GPIO_Port GPIOB
#define FLASH_CLK_Pin GPIO_PIN_2
#define FLASH_CLK_GPIO_Port GPIOB
#define FLASH_SS_Pin GPIO_PIN_10
#define FLASH_SS_GPIO_Port GPIOB
#define LIM1_1_Pin GPIO_PIN_14
#define LIM1_1_GPIO_Port GPIOB
#define LIM1_1_EXTI_IRQn EXTI14_IRQn
#define LIM1_2_Pin GPIO_PIN_15
#define LIM1_2_GPIO_Port GPIOB
#define LIM1_2_EXTI_IRQn EXTI15_IRQn
#define LIM2_1_Pin GPIO_PIN_11
#define LIM2_1_GPIO_Port GPIOD
#define LIM2_1_EXTI_IRQn EXTI11_IRQn
#define LIM2_2_Pin GPIO_PIN_12
#define LIM2_2_GPIO_Port GPIOD
#define LIM2_2_EXTI_IRQn EXTI12_IRQn
#define ENC1_B_Pin GPIO_PIN_6
#define ENC1_B_GPIO_Port GPIOC
#define ENC1_A_Pin GPIO_PIN_7
#define ENC1_A_GPIO_Port GPIOC
#define LIM3_1_Pin GPIO_PIN_8
#define LIM3_1_GPIO_Port GPIOC
#define LIM3_1_EXTI_IRQn EXTI8_IRQn
#define LIM3_2_Pin GPIO_PIN_9
#define LIM3_2_GPIO_Port GPIOC
#define LIM3_2_EXTI_IRQn EXTI9_IRQn
#define ENC2_B_Pin GPIO_PIN_8
#define ENC2_B_GPIO_Port GPIOA
#define ENC2_A_Pin GPIO_PIN_9
#define ENC2_A_GPIO_Port GPIOA
#define LIM4_2_Pin GPIO_PIN_10
#define LIM4_2_GPIO_Port GPIOA
#define LIM4_2_EXTI_IRQn EXTI10_IRQn
#define ID_4_SW_Pin GPIO_PIN_15
#define ID_4_SW_GPIO_Port GPIOA
#define ID_2_SW_Pin GPIO_PIN_10
#define ID_2_SW_GPIO_Port GPIOC
#define POWER_MONITOR_Pin GPIO_PIN_3
#define POWER_MONITOR_GPIO_Port GPIOB
#define ENC3_B_Pin GPIO_PIN_4
#define ENC3_B_GPIO_Port GPIOB
#define ENC3_A_Pin GPIO_PIN_5
#define ENC3_A_GPIO_Port GPIOB
#define ENC4_B_Pin GPIO_PIN_6
#define ENC4_B_GPIO_Port GPIOB
#define ENC4_A_Pin GPIO_PIN_7
#define ENC4_A_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
