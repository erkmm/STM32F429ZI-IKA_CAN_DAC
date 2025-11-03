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
extern uint32_t Actuator_Feedback[2];
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
#define Break_On_Pin GPIO_PIN_0
#define Break_On_GPIO_Port GPIOA
#define L_EN_Pin GPIO_PIN_6
#define L_EN_GPIO_Port GPIOA
#define left_direction_Pin GPIO_PIN_4
#define left_direction_GPIO_Port GPIOC
#define right_direction_Pin GPIO_PIN_5
#define right_direction_GPIO_Port GPIOC
#define Break_Off_Pin GPIO_PIN_9
#define Break_Off_GPIO_Port GPIOE
#define R1_Pin GPIO_PIN_12
#define R1_GPIO_Port GPIOE
#define R2_Pin GPIO_PIN_13
#define R2_GPIO_Port GPIOE
#define R3_Pin GPIO_PIN_14
#define R3_GPIO_Port GPIOE
#define R4_Pin GPIO_PIN_15
#define R4_GPIO_Port GPIOE
#define IN_1_1_Pin GPIO_PIN_15
#define IN_1_1_GPIO_Port GPIOB
#define IN_1_2_Pin GPIO_PIN_9
#define IN_1_2_GPIO_Port GPIOD
#define IN_2_1_Pin GPIO_PIN_11
#define IN_2_1_GPIO_Port GPIOD
#define LED1_Pin GPIO_PIN_12
#define LED1_GPIO_Port GPIOD
#define Status_Led_Pin GPIO_PIN_13
#define Status_Led_GPIO_Port GPIOD
#define IN_2_2_Pin GPIO_PIN_15
#define IN_2_2_GPIO_Port GPIOD
#define IN_3_2_Pin GPIO_PIN_7
#define IN_3_2_GPIO_Port GPIOC
#define IN_3_1_Pin GPIO_PIN_9
#define IN_3_1_GPIO_Port GPIOC
#define R_EN_Pin GPIO_PIN_15
#define R_EN_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
