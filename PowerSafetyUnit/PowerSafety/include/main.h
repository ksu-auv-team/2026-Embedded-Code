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

#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"

void Error_Handler(void);

/* Pin definitions -----------------------------------------------------------*/
#define XTAL_P_Pin GPIO_PIN_0
#define XTAL_P_GPIO_Port GPIOF
#define XTAL_N_Pin GPIO_PIN_1
#define XTAL_N_GPIO_Port GPIOF
#define VCC1_ADC_Pin GPIO_PIN_0
#define VCC1_ADC_GPIO_Port GPIOA
#define OPAMP1_P_Pin GPIO_PIN_1
#define OPAMP1_P_GPIO_Port GPIOA
#define VCC2_ADC_Pin GPIO_PIN_2
#define VCC2_ADC_GPIO_Port GPIOA
#define OPAMP1_N_Pin GPIO_PIN_3
#define OPAMP1_N_GPIO_Port GPIOA
#define VCC3_ADC_Pin GPIO_PIN_4
#define VCC3_ADC_GPIO_Port GPIOA
#define OPAMP2_N_Pin GPIO_PIN_5
#define OPAMP2_N_GPIO_Port GPIOA
#define OPAMP2_P_Pin GPIO_PIN_7
#define OPAMP2_P_GPIO_Port GPIOA
#define OPAMP3_P_Pin GPIO_PIN_0
#define OPAMP3_P_GPIO_Port GPIOB
#define SDA_Pin GPIO_PIN_8
#define SDA_GPIO_Port GPIOA
#define SCL_Pin GPIO_PIN_9
#define SCL_GPIO_Port GPIOA
#define SIGNAL_Pin GPIO_PIN_10
#define SIGNAL_GPIO_Port GPIOA
#define DOUT_N_Pin GPIO_PIN_11
#define DOUT_N_GPIO_Port GPIOA
#define DOUT_P_Pin GPIO_PIN_12
#define DOUT_P_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define Q6Gate_Pin GPIO_PIN_15
#define Q6Gate_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define Q3Gate_Pin GPIO_PIN_4
#define Q3Gate_GPIO_Port GPIOB
#define Q4Gate_Pin GPIO_PIN_5
#define Q4Gate_GPIO_Port GPIOB
#define USART_TX_Pin GPIO_PIN_6
#define USART_TX_GPIO_Port GPIOB
#define USART_RX_Pin GPIO_PIN_7
#define USART_RX_GPIO_Port GPIOB

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */