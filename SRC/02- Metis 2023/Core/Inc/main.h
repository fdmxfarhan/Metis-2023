/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DIP2_Pin GPIO_PIN_14
#define DIP2_GPIO_Port GPIOC
#define DIP1_Pin GPIO_PIN_15
#define DIP1_GPIO_Port GPIOC
#define L2B_Pin GPIO_PIN_7
#define L2B_GPIO_Port GPIOE
#define L1B_Pin GPIO_PIN_8
#define L1B_GPIO_Port GPIOE
#define L1A_Pin GPIO_PIN_10
#define L1A_GPIO_Port GPIOE
#define L2A_Pin GPIO_PIN_12
#define L2A_GPIO_Port GPIOE
#define MOTORS_EN_Pin GPIO_PIN_15
#define MOTORS_EN_GPIO_Port GPIOE
#define SW3_Pin GPIO_PIN_12
#define SW3_GPIO_Port GPIOB
#define SW2_Pin GPIO_PIN_13
#define SW2_GPIO_Port GPIOB
#define SW1_Pin GPIO_PIN_14
#define SW1_GPIO_Port GPIOB
#define SHOOT_Pin GPIO_PIN_15
#define SHOOT_GPIO_Port GPIOB
#define SPIN_Pin GPIO_PIN_8
#define SPIN_GPIO_Port GPIOD
#define ADD3_Pin GPIO_PIN_9
#define ADD3_GPIO_Port GPIOD
#define ADD2_Pin GPIO_PIN_10
#define ADD2_GPIO_Port GPIOD
#define ADD1_Pin GPIO_PIN_11
#define ADD1_GPIO_Port GPIOD
#define ADD0_Pin GPIO_PIN_12
#define ADD0_GPIO_Port GPIOD
#define R2B_Pin GPIO_PIN_13
#define R2B_GPIO_Port GPIOD
#define R2A_Pin GPIO_PIN_14
#define R2A_GPIO_Port GPIOD
#define R1B_Pin GPIO_PIN_15
#define R1B_GPIO_Port GPIOD
#define R1A_Pin GPIO_PIN_6
#define R1A_GPIO_Port GPIOC
#define GREEN_Pin GPIO_PIN_7
#define GREEN_GPIO_Port GPIOD
#define BUZZER_Pin GPIO_PIN_3
#define BUZZER_GPIO_Port GPIOB
#define BLUE_Pin GPIO_PIN_4
#define BLUE_GPIO_Port GPIOB
#define RED_Pin GPIO_PIN_5
#define RED_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
