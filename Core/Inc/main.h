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
#define pIO_BP_Pin GPIO_PIN_13
#define pIO_BP_GPIO_Port GPIOC
#define pIO_CP_Pin GPIO_PIN_0
#define pIO_CP_GPIO_Port GPIOC
#define pIO_CN_Pin GPIO_PIN_1
#define pIO_CN_GPIO_Port GPIOC
#define pIO_DP_Pin GPIO_PIN_2
#define pIO_DP_GPIO_Port GPIOC
#define pIO_DN_EXT13_Pin GPIO_PIN_3
#define pIO_DN_EXT13_GPIO_Port GPIOC
#define pIO_DN_EXT13_EXTI_IRQn EXTI3_IRQn
#define pWUP_Pin GPIO_PIN_0
#define pWUP_GPIO_Port GPIOB
#define pWDN_Pin GPIO_PIN_1
#define pWDN_GPIO_Port GPIOB
#define pSD_Pin GPIO_PIN_2
#define pSD_GPIO_Port GPIOB
#define CAN_ID_Pin GPIO_PIN_12
#define CAN_ID_GPIO_Port GPIOB
#define pIO_AP_Pin GPIO_PIN_13
#define pIO_AP_GPIO_Port GPIOB
#define pIO_AN_Pin GPIO_PIN_14
#define pIO_AN_GPIO_Port GPIOB
#define pIO_BN_Pin GPIO_PIN_12
#define pIO_BN_GPIO_Port GPIOC
#define pRY1_Pin GPIO_PIN_2
#define pRY1_GPIO_Port GPIOD
#define pRY2_Pin GPIO_PIN_4
#define pRY2_GPIO_Port GPIOB
#define pRY3_Pin GPIO_PIN_5
#define pRY3_GPIO_Port GPIOB
#define pRY4_Pin GPIO_PIN_6
#define pRY4_GPIO_Port GPIOB
#define pRY5_Pin GPIO_PIN_7
#define pRY5_GPIO_Port GPIOB
#define pRY6_Pin GPIO_PIN_8
#define pRY6_GPIO_Port GPIOB
#define pRY7_Pin GPIO_PIN_9
#define pRY7_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
