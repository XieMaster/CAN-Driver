/* USER CODE BEGIN Header */
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
  * COPYRIGHT(c) 2019 STMicroelectronics
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
#define PWM_FREQUENCE 50
#define PWM_RESOLUTION 10000
#define APB1_TIMER_CLOCKS 36000000
#define PWM_DEFAULT_DUTY 5000
#define APB2_TIMER_CLOCKS 72000000
#define TIM_PSC_APB1 ((APB1_TIMER_CLOCKS/PWM_FREQUENCE)/PWM_RESOLUTION -1)
#define TIM_PSC_APB2 ((APB2_TIMER_CLOCKS/PWM_FREQUENCE)/PWM_RESOLUTION -1)
#define GPIO1_Pin GPIO_PIN_0
#define GPIO1_GPIO_Port GPIOC
#define GPIO2_Pin GPIO_PIN_1
#define GPIO2_GPIO_Port GPIOC
#define GPIO3_Pin GPIO_PIN_2
#define GPIO3_GPIO_Port GPIOC
#define GPIO4_Pin GPIO_PIN_3
#define GPIO4_GPIO_Port GPIOC
#define BUTTON_AD_Pin GPIO_PIN_1
#define BUTTON_AD_GPIO_Port GPIOA
#define OUT1_Pin GPIO_PIN_4
#define OUT1_GPIO_Port GPIOA
#define OUT2_Pin GPIO_PIN_5
#define OUT2_GPIO_Port GPIOA
#define OUT3_Pin GPIO_PIN_6
#define OUT3_GPIO_Port GPIOA
#define OUT4_Pin GPIO_PIN_7
#define OUT4_GPIO_Port GPIOA
#define PWR_IN_Pin GPIO_PIN_4
#define PWR_IN_GPIO_Port GPIOC
#define ledG_Pin GPIO_PIN_0
#define ledG_GPIO_Port GPIOB
#define ledR_Pin GPIO_PIN_1
#define ledR_GPIO_Port GPIOB
#define S_OUT4_Pin GPIO_PIN_6
#define S_OUT4_GPIO_Port GPIOC
#define S_OUT3_Pin GPIO_PIN_7
#define S_OUT3_GPIO_Port GPIOC
#define S_OUT2_Pin GPIO_PIN_8
#define S_OUT2_GPIO_Port GPIOC
#define S_OUT1_Pin GPIO_PIN_9
#define S_OUT1_GPIO_Port GPIOC
#define PWM1_Pin GPIO_PIN_8
#define PWM1_GPIO_Port GPIOA
#define PWM2_Pin GPIO_PIN_9
#define PWM2_GPIO_Port GPIOA
#define PWM3_Pin GPIO_PIN_10
#define PWM3_GPIO_Port GPIOA
#define PWM4_Pin GPIO_PIN_11
#define PWM4_GPIO_Port GPIOA
#define OLED_DC_Pin GPIO_PIN_4
#define OLED_DC_GPIO_Port GPIOB
#define OLED_RST_Pin GPIO_PIN_6
#define OLED_RST_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))      //´®¿Ú×Ö·û´®³¤¶È
    
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
