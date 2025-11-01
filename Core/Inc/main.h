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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define IN_2_1_Pin GPIO_PIN_4
#define IN_2_1_GPIO_Port GPIOE
#define IN_1_2_Pin GPIO_PIN_14
#define IN_1_2_GPIO_Port GPIOC
#define IN_1_1_Pin GPIO_PIN_2
#define IN_1_1_GPIO_Port GPIOF
#define L_EN_Pin GPIO_PIN_6
#define L_EN_GPIO_Port GPIOA
#define Break_On_Pin GPIO_PIN_10
#define Break_On_GPIO_Port GPIOE
#define Break_Off_Pin GPIO_PIN_12
#define Break_Off_GPIO_Port GPIOE
#define IN_2_1D8_Pin GPIO_PIN_8
#define IN_2_1D8_GPIO_Port GPIOD
#define R1_Pin GPIO_PIN_9
#define R1_GPIO_Port GPIOD
#define IN_2_2_Pin GPIO_PIN_10
#define IN_2_2_GPIO_Port GPIOD
#define R2_Pin GPIO_PIN_11
#define R2_GPIO_Port GPIOD
#define IN_3_1_Pin GPIO_PIN_12
#define IN_3_1_GPIO_Port GPIOD
#define Status_Led_Pin GPIO_PIN_13
#define Status_Led_GPIO_Port GPIOD
#define IN_3_2_Pin GPIO_PIN_14
#define IN_3_2_GPIO_Port GPIOD
#define R3_Pin GPIO_PIN_15
#define R3_GPIO_Port GPIOD
#define R4_Pin GPIO_PIN_3
#define R4_GPIO_Port GPIOG
#define R8_Pin GPIO_PIN_4
#define R8_GPIO_Port GPIOG
#define R7_Pin GPIO_PIN_6
#define R7_GPIO_Port GPIOG
#define IN_1_2G7_Pin GPIO_PIN_7
#define IN_1_2G7_GPIO_Port GPIOG
#define R6_Pin GPIO_PIN_8
#define R6_GPIO_Port GPIOG
#define IN_1_1C6_Pin GPIO_PIN_6
#define IN_1_1C6_GPIO_Port GPIOC
#define R5_Pin GPIO_PIN_7
#define R5_GPIO_Port GPIOC
#define R_EN_Pin GPIO_PIN_15
#define R_EN_GPIO_Port GPIOA
#define LED2_Pin GPIO_PIN_13
#define LED2_GPIO_Port GPIOG
#define LED1_Pin GPIO_PIN_14
#define LED1_GPIO_Port GPIOG
#define IN_3_2B4_Pin GPIO_PIN_4
#define IN_3_2B4_GPIO_Port GPIOB
#define IN_3_1B6_Pin GPIO_PIN_6
#define IN_3_1B6_GPIO_Port GPIOB
#define IN_2_2E0_Pin GPIO_PIN_0
#define IN_2_2E0_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
