/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32f0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
// #include "interrupts.h"
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
#define MotorANA1_Pin GPIO_PIN_0
#define MotorANA1_GPIO_Port GPIOA
#define MotorANA2_Pin GPIO_PIN_1
#define MotorANA2_GPIO_Port GPIOA
#define MotorANA3_Pin GPIO_PIN_2
#define MotorANA3_GPIO_Port GPIOA
#define MotorEN1_Pin GPIO_PIN_3
#define MotorEN1_GPIO_Port GPIOA
#define MotorEN2_Pin GPIO_PIN_4
#define MotorEN2_GPIO_Port GPIOA
#define MotorEN3_Pin GPIO_PIN_5
#define MotorEN3_GPIO_Port GPIOA
#define MotorPWM3_Pin GPIO_PIN_6
#define MotorPWM3_GPIO_Port GPIOA
#define MotorPWM2_Pin GPIO_PIN_7
#define MotorPWM2_GPIO_Port GPIOA
#define MotorPWM1_Pin GPIO_PIN_1
#define MotorPWM1_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
