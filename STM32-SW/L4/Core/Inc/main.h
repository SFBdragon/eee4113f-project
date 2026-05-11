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
#include "stm32l4xx_hal.h"

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
#define L0_LPUART1_RX_Pin GPIO_PIN_0
#define L0_LPUART1_RX_GPIO_Port GPIOC
#define L0_LPUART1_TX_Pin GPIO_PIN_1
#define L0_LPUART1_TX_GPIO_Port GPIOC
#define Wi_EN_Pin GPIO_PIN_2
#define Wi_EN_GPIO_Port GPIOC
#define Wi_RST_Pin GPIO_PIN_3
#define Wi_RST_GPIO_Port GPIOC
#define Wi_USART2_CTS_Pin GPIO_PIN_0
#define Wi_USART2_CTS_GPIO_Port GPIOA
#define Wi_USART2_RTS_Pin GPIO_PIN_1
#define Wi_USART2_RTS_GPIO_Port GPIOA
#define Wi_USART2_TX_Pin GPIO_PIN_2
#define Wi_USART2_TX_GPIO_Port GPIOA
#define Wi_USART2_RX_Pin GPIO_PIN_3
#define Wi_USART2_RX_GPIO_Port GPIOA
#define SD_GPIO_INPUT_Pin GPIO_PIN_4
#define SD_GPIO_INPUT_GPIO_Port GPIOA
#define SD_SPI1_SCK_Pin GPIO_PIN_5
#define SD_SPI1_SCK_GPIO_Port GPIOA
#define SD_SPI1_MISO_Pin GPIO_PIN_6
#define SD_SPI1_MISO_GPIO_Port GPIOA
#define SD_SPI1_MOSI_Pin GPIO_PIN_7
#define SD_SPI1_MOSI_GPIO_Port GPIOA
#define Wi_USART3_TX_Pin GPIO_PIN_4
#define Wi_USART3_TX_GPIO_Port GPIOC
#define Wi_USART3_RX_Pin GPIO_PIN_5
#define Wi_USART3_RX_GPIO_Port GPIOC
#define Wi_PWR_CTRL_Pin GPIO_PIN_0
#define Wi_PWR_CTRL_GPIO_Port GPIOB
#define Lo_PWR_CTRL_Pin GPIO_PIN_1
#define Lo_PWR_CTRL_GPIO_Port GPIOB
#define SD0_PWR_CTRL_Pin GPIO_PIN_2
#define SD0_PWR_CTRL_GPIO_Port GPIOB
#define Lo_GPIO_0_Pin GPIO_PIN_10
#define Lo_GPIO_0_GPIO_Port GPIOB
#define Lo_GPIO_1_Pin GPIO_PIN_11
#define Lo_GPIO_1_GPIO_Port GPIOB
#define Lo_GPIO_2_Pin GPIO_PIN_12
#define Lo_GPIO_2_GPIO_Port GPIOB
#define Wi_WAKE_Pin GPIO_PIN_6
#define Wi_WAKE_GPIO_Port GPIOC
#define Wi_GPIO_0_Pin GPIO_PIN_7
#define Wi_GPIO_0_GPIO_Port GPIOC
#define D_USART_CK_Pin GPIO_PIN_8
#define D_USART_CK_GPIO_Port GPIOA
#define D_USART_RX_Pin GPIO_PIN_9
#define D_USART_RX_GPIO_Port GPIOA
#define D_USART_TX_Pin GPIO_PIN_10
#define D_USART_TX_GPIO_Port GPIOA
#define D_WAKE_Pin GPIO_PIN_11
#define D_WAKE_GPIO_Port GPIOA
#define D_WAKE_EXTI_IRQn EXTI15_10_IRQn
#define D_GPIO0_Pin GPIO_PIN_12
#define D_GPIO0_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
