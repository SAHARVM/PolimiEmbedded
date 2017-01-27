/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
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
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define TEMP_MOTOR_Pin GPIO_PIN_0
#define TEMP_MOTOR_GPIO_Port GPIOC
#define AN_IN_Pin GPIO_PIN_2
#define AN_IN_GPIO_Port GPIOC
#define SENSE3_Pin GPIO_PIN_0
#define SENSE3_GPIO_Port GPIOA
#define SENSE2_Pin GPIO_PIN_1
#define SENSE2_GPIO_Port GPIOA
#define SENSE1_Pin GPIO_PIN_2
#define SENSE1_GPIO_Port GPIOA
#define ADC_TEMP_Pin GPIO_PIN_3
#define ADC_TEMP_GPIO_Port GPIOA
#define LED_GREEN_Pin GPIO_PIN_4
#define LED_GREEN_GPIO_Port GPIOC
#define LED_RED_Pin GPIO_PIN_5
#define LED_RED_GPIO_Port GPIOC
#define BR_SO2_Pin GPIO_PIN_0
#define BR_SO2_GPIO_Port GPIOB
#define BR_SO1_Pin GPIO_PIN_1
#define BR_SO1_GPIO_Port GPIOB
#define DC_CAL_Pin GPIO_PIN_12
#define DC_CAL_GPIO_Port GPIOB
#define L3_Pin GPIO_PIN_13
#define L3_GPIO_Port GPIOB
#define L2_Pin GPIO_PIN_14
#define L2_GPIO_Port GPIOB
#define L1_Pin GPIO_PIN_15
#define L1_GPIO_Port GPIOB
#define VESC_TX_Pin GPIO_PIN_6
#define VESC_TX_GPIO_Port GPIOC
#define VESC_RX_Pin GPIO_PIN_7
#define VESC_RX_GPIO_Port GPIOC
#define H3_Pin GPIO_PIN_8
#define H3_GPIO_Port GPIOA
#define H2_Pin GPIO_PIN_9
#define H2_GPIO_Port GPIOA
#define H1_Pin GPIO_PIN_10
#define H1_GPIO_Port GPIOA
#define EN_GATE_Pin GPIO_PIN_10
#define EN_GATE_GPIO_Port GPIOC
#define HALL_3_Pin GPIO_PIN_11
#define HALL_3_GPIO_Port GPIOC
#define FAULT_Pin GPIO_PIN_12
#define FAULT_GPIO_Port GPIOC
#define SERVO_Pin GPIO_PIN_5
#define SERVO_GPIO_Port GPIOB
#define HALL_1_Pin GPIO_PIN_6
#define HALL_1_GPIO_Port GPIOB
#define HALL_2_Pin GPIO_PIN_7
#define HALL_2_GPIO_Port GPIOB
#define CAN_RX_Pin GPIO_PIN_8
#define CAN_RX_GPIO_Port GPIOB
#define CAN_TX_Pin GPIO_PIN_9
#define CAN_TX_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
