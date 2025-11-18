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
#include "stm32c0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#ifdef DEBUG
#include <stdio.h>
#endif
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;

extern FDCAN_HandleTypeDef hfdcan1;

extern I2C_HandleTypeDef hi2c1;

extern TIM_HandleTypeDef htim3;

extern UART_HandleTypeDef huart1;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
// Update cycles by TIM3 overflow interrupt handler
#define EG_UPDATE_CYCLES   2
#define ADC_UPDATE_CYCLES 32
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
#define DEBUG_OUT_Pin GPIO_PIN_9
#define DEBUG_OUT_GPIO_Port GPIOB
#define ADC_A_Pin GPIO_PIN_0
#define ADC_A_GPIO_Port GPIOA
#define ADC_D_Pin GPIO_PIN_1
#define ADC_D_GPIO_Port GPIOA
#define ADC_S_Pin GPIO_PIN_2
#define ADC_S_GPIO_Port GPIOA
#define ADC_R_Pin GPIO_PIN_3
#define ADC_R_GPIO_Port GPIOA
#define ADC_D0_Pin GPIO_PIN_4
#define ADC_D0_GPIO_Port GPIOA
#define ADC_S0_Pin GPIO_PIN_5
#define ADC_S0_GPIO_Port GPIOA
#define ADC_GATE_1_Pin GPIO_PIN_6
#define ADC_GATE_1_GPIO_Port GPIOA
#define ADC_GATE_2_Pin GPIO_PIN_7
#define ADC_GATE_2_GPIO_Port GPIOA
#define A3_IND_RED_Pin GPIO_PIN_1
#define A3_IND_RED_GPIO_Port GPIOB
#define A3_IND_BLUE_Pin GPIO_PIN_2
#define A3_IND_BLUE_GPIO_Port GPIOB
#define USER_SW_Pin GPIO_PIN_6
#define USER_SW_GPIO_Port GPIOC
#define IND_SHIFT_Pin GPIO_PIN_10
#define IND_SHIFT_GPIO_Port GPIOA
#define IND_GATE_2_Pin GPIO_PIN_15
#define IND_GATE_2_GPIO_Port GPIOA
#define IND_GATE_1_Pin GPIO_PIN_3
#define IND_GATE_1_GPIO_Port GPIOB
#define CAN_STB_Pin GPIO_PIN_4
#define CAN_STB_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
