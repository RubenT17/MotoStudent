/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#define B1_USER_Pin GPIO_PIN_13
#define B1_USER_GPIO_Port GPIOC
#define B1_USER_EXTI_IRQn EXTI15_10_IRQn
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define TempDS2_Pin GPIO_PIN_4
#define TempDS2_GPIO_Port GPIOC
#define TempDS1_Pin GPIO_PIN_5
#define TempDS1_GPIO_Port GPIOC
#define IMD_OK_Pin GPIO_PIN_12
#define IMD_OK_GPIO_Port GPIOB
#define IMD_OK_EXTI_IRQn EXTI15_10_IRQn
#define CONTACTOR_IN_Pin GPIO_PIN_14
#define CONTACTOR_IN_GPIO_Port GPIOB
#define CONTACTOR_IN_EXTI_IRQn EXTI15_10_IRQn
#define BMS_OK_Pin GPIO_PIN_15
#define BMS_OK_GPIO_Port GPIOB
#define BMS_OK_EXTI_IRQn EXTI15_10_IRQn
#define Contactor_out_Pin GPIO_PIN_7
#define Contactor_out_GPIO_Port GPIOC
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
