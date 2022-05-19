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
#define GetCurrent_Pin GPIO_PIN_0
#define GetCurrent_GPIO_Port GPIOA
#define StartSensor_Pin GPIO_PIN_1
#define StartSensor_GPIO_Port GPIOA
#define StartSensor_EXTI_IRQn EXTI1_IRQn
#define Sensor1_Pin GPIO_PIN_3
#define Sensor1_GPIO_Port GPIOA
#define Sensor2_Pin GPIO_PIN_4
#define Sensor2_GPIO_Port GPIOA
#define Sensor3_Pin GPIO_PIN_6
#define Sensor3_GPIO_Port GPIOA
#define Sensor4_Pin GPIO_PIN_7
#define Sensor4_GPIO_Port GPIOA
#define StartSensor2_Pin GPIO_PIN_12
#define StartSensor2_GPIO_Port GPIOB
#define StartSensor2_EXTI_IRQn EXTI15_10_IRQn
#define Pwm_Pin GPIO_PIN_8
#define Pwm_GPIO_Port GPIOA
#define Sen1_Pin GPIO_PIN_7
#define Sen1_GPIO_Port GPIOB
#define Sen1_EXTI_IRQn EXTI9_5_IRQn
#define Pwm3_Pin GPIO_PIN_8
#define Pwm3_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
