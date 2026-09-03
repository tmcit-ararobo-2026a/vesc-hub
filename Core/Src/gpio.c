/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins
     PA3   ------> S_TIM2_CH4
     PA6   ------> OCTOSPI1_IO3
     PA7   ------> OCTOSPI1_IO2
     PB0   ------> OCTOSPI1_IO1
     PB1   ------> OCTOSPI1_IO0
     PB2   ------> OCTOSPI1_CLK
     PA13(JTMS/SWDIO)   ------> DEBUG_JTMS-SWDIO
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_4_Pin|LED_3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_2_Pin|LED_1_Pin|Synchronization_flag_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_4_Pin LED_3_Pin */
  GPIO_InitStruct.Pin = LED_4_Pin|LED_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_2_Pin LED_1_Pin Synchronization_flag_Pin */
  GPIO_InitStruct.Pin = LED_2_Pin|LED_1_Pin|Synchronization_flag_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Synchronization_clock_Pin */
  GPIO_InitStruct.Pin = Synchronization_clock_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(Synchronization_clock_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : FLASH_IO3_Pin */
  GPIO_InitStruct.Pin = FLASH_IO3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF6_OCTOSPI1;
  HAL_GPIO_Init(FLASH_IO3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : FLASH_IO2_Pin */
  GPIO_InitStruct.Pin = FLASH_IO2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OCTOSPI1;
  HAL_GPIO_Init(FLASH_IO2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : FLASH_IO1_Pin FLASH_IO0_Pin */
  GPIO_InitStruct.Pin = FLASH_IO1_Pin|FLASH_IO0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF6_OCTOSPI1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : FLASH_CLK_Pin */
  GPIO_InitStruct.Pin = FLASH_CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_OCTOSPI1;
  HAL_GPIO_Init(FLASH_CLK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LIM1_1_Pin LIM1_2_Pin */
  GPIO_InitStruct.Pin = LIM1_1_Pin|LIM1_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LIM2_1_Pin LIM2_2_Pin */
  GPIO_InitStruct.Pin = LIM2_1_Pin|LIM2_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : LIM3_1_Pin LIM3_2_Pin */
  GPIO_InitStruct.Pin = LIM3_1_Pin|LIM3_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LIM4_2_Pin */
  GPIO_InitStruct.Pin = LIM4_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LIM4_2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_TRACE;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ID_4_SW_Pin */
  GPIO_InitStruct.Pin = ID_4_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ID_4_SW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ID_2_SW_Pin */
  GPIO_InitStruct.Pin = ID_2_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ID_2_SW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : POWER_MONITOR_Pin */
  GPIO_InitStruct.Pin = POWER_MONITOR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(POWER_MONITOR_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI8_IRQn, 8, 0);
  HAL_NVIC_EnableIRQ(EXTI8_IRQn);

  HAL_NVIC_SetPriority(EXTI9_IRQn, 8, 0);
  HAL_NVIC_EnableIRQ(EXTI9_IRQn);

  HAL_NVIC_SetPriority(EXTI10_IRQn, 8, 0);
  HAL_NVIC_EnableIRQ(EXTI10_IRQn);

  HAL_NVIC_SetPriority(EXTI11_IRQn, 8, 0);
  HAL_NVIC_EnableIRQ(EXTI11_IRQn);

  HAL_NVIC_SetPriority(EXTI12_IRQn, 8, 0);
  HAL_NVIC_EnableIRQ(EXTI12_IRQn);

  HAL_NVIC_SetPriority(EXTI14_IRQn, 8, 0);
  HAL_NVIC_EnableIRQ(EXTI14_IRQn);

  HAL_NVIC_SetPriority(EXTI15_IRQn, 8, 0);
  HAL_NVIC_EnableIRQ(EXTI15_IRQn);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
