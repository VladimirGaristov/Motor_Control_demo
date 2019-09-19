/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_ll_adc.h"
#include "stm32f1xx_ll_dma.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_system.h"
#include "stm32f1xx_ll_exti.h"
#include "stm32f1xx_ll_cortex.h"
#include "stm32f1xx_ll_utils.h"
#include "stm32f1xx_ll_pwr.h"
#include "stm32f1xx_ll_tim.h"
#include "stm32f1xx.h"
#include "stm32f1xx_ll_gpio.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
#define SLOW_ALARM 62500
#define MED_ALARM 31250
#define FAST_ALARM 15625
#define LOW_PITCH 1000
#define MED_PITCH 500
#define HIGH_PITCH 333

#define LED0_Pin LL_GPIO_PIN_13
#define LED0_GPIO_Port GPIOC
#define LED1_Pin LL_GPIO_PIN_14
#define LED1_GPIO_Port GPIOC
#define LED2_Pin LL_GPIO_PIN_15
#define LED2_GPIO_Port GPIOC
#define POT_Pin LL_GPIO_PIN_1
#define POT_GPIO_Port GPIOA
#define OPTO_Pin LL_GPIO_PIN_2
#define OPTO_GPIO_Port GPIOA
#define LED3_Pin LL_GPIO_PIN_3
#define LED3_GPIO_Port GPIOA
#define LED4_Pin LL_GPIO_PIN_4
#define LED4_GPIO_Port GPIOA
#define INH1_Pin LL_GPIO_PIN_5
#define INH1_GPIO_Port GPIOA
#define IS1_Pin LL_GPIO_PIN_7
#define IS1_GPIO_Port GPIOA
#define CLK_7SEG_Pin LL_GPIO_PIN_0
#define CLK_7SEG_GPIO_Port GPIOB
#define DATA_7SEG_Pin LL_GPIO_PIN_1
#define DATA_7SEG_GPIO_Port GPIOB
#define LATCH_7SEG_Pin LL_GPIO_PIN_4
#define LATCH_7SEG_GPIO_Port GPIOB
#define IN1_Pin LL_GPIO_PIN_8
#define IN1_GPIO_Port GPIOA
#define BUZZER_Pin LL_GPIO_PIN_6
#define BUZZER_GPIO_Port GPIOB
#ifndef NVIC_PRIORITYGROUP_0
#define NVIC_PRIORITYGROUP_0         ((uint32_t)0x00000007) /*!< 0 bit  for pre-emption priority,
                                                                 4 bits for subpriority */
#define NVIC_PRIORITYGROUP_1         ((uint32_t)0x00000006) /*!< 1 bit  for pre-emption priority,
                                                                 3 bits for subpriority */
#define NVIC_PRIORITYGROUP_2         ((uint32_t)0x00000005) /*!< 2 bits for pre-emption priority,
                                                                 2 bits for subpriority */
#define NVIC_PRIORITYGROUP_3         ((uint32_t)0x00000004) /*!< 3 bits for pre-emption priority,
                                                                 1 bit  for subpriority */
#define NVIC_PRIORITYGROUP_4         ((uint32_t)0x00000003) /*!< 4 bits for pre-emption priority,
                                                                 0 bit  for subpriority */
#endif

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

#define STEPS_PER_ROTATION 50
#define MAX_POWER 800
#define ADC_MAX 0xFFF
#define ALARM_THR_1 0xFF0
#define ALARM_THR_2 0xEF0
#define ALARM_THR_3 0xDF0

void calculate_speed(void);
void calculate_power(void);
void refresh_7segm(void);

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
