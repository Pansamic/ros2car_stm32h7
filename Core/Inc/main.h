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
#include "stm32h7xx_hal.h"

#include "stm32h7xx_ll_adc.h"
#include "stm32h7xx_ll_crc.h"
#include "stm32h7xx_ll_dma.h"
#include "stm32h7xx_ll_i2c.h"
#include "stm32h7xx_ll_iwdg.h"
#include "stm32h7xx_ll_lptim.h"
#include "stm32h7xx_ll_pwr.h"
#include "stm32h7xx_ll_system.h"
#include "stm32h7xx_ll_exti.h"
#include "stm32h7xx_ll_rcc.h"
#include "stm32h7xx_ll_crs.h"
#include "stm32h7xx_ll_bus.h"
#include "stm32h7xx_ll_cortex.h"
#include "stm32h7xx_ll_utils.h"
#include "stm32h7xx_ll_rng.h"
#include "stm32h7xx_ll_rtc.h"
#include "stm32h7xx_ll_spi.h"
#include "stm32h7xx_ll_tim.h"
#include "stm32h7xx_ll_usart.h"
#include "stm32h7xx_ll_gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cringbuf.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct uart_s
{
    USART_TypeDef* huart;
    DMA_TypeDef * dma_rx;
    DMA_TypeDef * dma_tx;
    uint32_t dma_rx_channel;
    uint32_t dma_tx_channel;
    size_t expect_dma_rx_len;
    size_t expect_dma_tx_len;
    ringbuf_t rx_ringbuf;
    ringbuf_t tx_ringbuf;
    volatile uint8_t tx_cplt_flag;
    uint8_t rx_buf[1024];
    uint8_t tx_buf[1024];
}uart_t;
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
int uart_open(uart_t* pcb);
int uart_close(uart_t* pcb);
int uart_send(uart_t* pcb, void* pdata, size_t length);
int uart_read(uart_t* pcb, void* pdata, size_t length, size_t* read_length);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ENCODER_TIM TIM13
#define MOTOR3_ENC_TIM LPTIM1
#define MOTOR4_ENC_TIM LPTIM2
#define ADC_TRIG_TIM LPTIM3
#define BUZZER_TIM TIM15
#define RGB_TIM TIM17
#define SERVO34_TIM TIM12
#define SERVO12_TIM TIM8
#define GPS_PPS_TIM TIM5
#define CAM_SYNC_TIM TIM4
#define MOTOR2_ENC_TIM TIM3_IP_HANDLE
#define MOTOR1_ENC_TIM TIM2_IP_HANDLE
#define SPI_CS_EXT_Pin LL_GPIO_PIN_13
#define SPI_CS_EXT_GPIO_Port GPIOC
#define PWM_RGB_Pin LL_GPIO_PIN_9
#define PWM_RGB_GPIO_Port GPIOB
#define ENCB_MOTOR1_Pin LL_GPIO_PIN_3
#define ENCB_MOTOR1_GPIO_Port GPIOB
#define ENCA_MOTOR1_Pin LL_GPIO_PIN_15
#define ENCA_MOTOR1_GPIO_Port GPIOA
#define FAULT_MOTOR1_Pin LL_GPIO_PIN_15
#define FAULT_MOTOR1_GPIO_Port GPIOC
#define IMU_INT2_Pin LL_GPIO_PIN_3
#define IMU_INT2_GPIO_Port GPIOE
#define FSYNC1_Pin LL_GPIO_PIN_6
#define FSYNC1_GPIO_Port GPIOB
#define TX_BLE_Pin LL_GPIO_PIN_5
#define TX_BLE_GPIO_Port GPIOD
#define EXT_RTC_INT_Pin LL_GPIO_PIN_4
#define EXT_RTC_INT_GPIO_Port GPIOE
#define ENCB_MOTOR3_Pin LL_GPIO_PIN_1
#define ENCB_MOTOR3_GPIO_Port GPIOE
#define ENCB_MOTOR2_Pin LL_GPIO_PIN_5
#define ENCB_MOTOR2_GPIO_Port GPIOB
#define RX_BLE_Pin LL_GPIO_PIN_6
#define RX_BLE_GPIO_Port GPIOD
#define OLED_RESET_Pin LL_GPIO_PIN_3
#define OLED_RESET_GPIO_Port GPIOD
#define TX_DEBUG_Pin LL_GPIO_PIN_9
#define TX_DEBUG_GPIO_Port GPIOA
#define SPI_CS_OLED_Pin LL_GPIO_PIN_1
#define SPI_CS_OLED_GPIO_Port GPIOH
#define PWM_BUZZER_Pin LL_GPIO_PIN_5
#define PWM_BUZZER_GPIO_Port GPIOE
#define BUTTON_Pin LL_GPIO_PIN_0
#define BUTTON_GPIO_Port GPIOE
#define OLED_DC_Pin LL_GPIO_PIN_4
#define OLED_DC_GPIO_Port GPIOD
#define PWM_MOTOR1_Pin LL_GPIO_PIN_8
#define PWM_MOTOR1_GPIO_Port GPIOA
#define RX_DEBUG_Pin LL_GPIO_PIN_10
#define RX_DEBUG_GPIO_Port GPIOA
#define ETH_RESET_Pin LL_GPIO_PIN_6
#define ETH_RESET_GPIO_Port GPIOE
#define PWM_SERVO2_Pin LL_GPIO_PIN_7
#define PWM_SERVO2_GPIO_Port GPIOC
#define ADC_MOTOR4_Pin LL_GPIO_PIN_0
#define ADC_MOTOR4_GPIO_Port GPIOC
#define ADC_VCC_Pin LL_GPIO_PIN_3
#define ADC_VCC_GPIO_Port GPIOC
#define PWM_SERVO1_Pin LL_GPIO_PIN_6
#define PWM_SERVO1_GPIO_Port GPIOC
#define GPS_PPS_Pin LL_GPIO_PIN_0
#define GPS_PPS_GPIO_Port GPIOA
#define SPI_CS_IMU_Pin LL_GPIO_PIN_4
#define SPI_CS_IMU_GPIO_Port GPIOA
#define FAULT_MOTOR3_Pin LL_GPIO_PIN_2
#define FAULT_MOTOR3_GPIO_Port GPIOB
#define CTRL_MOTOR4_Pin LL_GPIO_PIN_10
#define CTRL_MOTOR4_GPIO_Port GPIOE
#define PWM_MOTOR4_Pin LL_GPIO_PIN_14
#define PWM_MOTOR4_GPIO_Port GPIOE
#define STROBE2_Pin LL_GPIO_PIN_15
#define STROBE2_GPIO_Port GPIOD
#define ENCB_MOTOR4_Pin LL_GPIO_PIN_11
#define ENCB_MOTOR4_GPIO_Port GPIOD
#define PWM_SERVO4_Pin LL_GPIO_PIN_15
#define PWM_SERVO4_GPIO_Port GPIOB
#define CTRL_MOTOR3_Pin LL_GPIO_PIN_7
#define CTRL_MOTOR3_GPIO_Port GPIOE
#define PWM_MOTOR3_Pin LL_GPIO_PIN_11
#define PWM_MOTOR3_GPIO_Port GPIOE
#define FAULT_MOTOR4_Pin LL_GPIO_PIN_15
#define FAULT_MOTOR4_GPIO_Port GPIOE
#define STROBE1_Pin LL_GPIO_PIN_14
#define STROBE1_GPIO_Port GPIOD
#define IMU_INT1_Pin LL_GPIO_PIN_10
#define IMU_INT1_GPIO_Port GPIOD
#define PWM_SERVO3_Pin LL_GPIO_PIN_14
#define PWM_SERVO3_GPIO_Port GPIOB
#define ENCA_MOTOR2_Pin LL_GPIO_PIN_6
#define ENCA_MOTOR2_GPIO_Port GPIOA
#define ADC_MOTOR2_Pin LL_GPIO_PIN_0
#define ADC_MOTOR2_GPIO_Port GPIOB
#define CTRL_MOTOR2_Pin LL_GPIO_PIN_8
#define CTRL_MOTOR2_GPIO_Port GPIOE
#define FAULT_MOTOR2_Pin LL_GPIO_PIN_12
#define FAULT_MOTOR2_GPIO_Port GPIOE
#define ENCA_MOTOR4_Pin LL_GPIO_PIN_10
#define ENCA_MOTOR4_GPIO_Port GPIOB
#define SBUS_RX_Pin LL_GPIO_PIN_9
#define SBUS_RX_GPIO_Port GPIOD
#define FSYNC2_Pin LL_GPIO_PIN_13
#define FSYNC2_GPIO_Port GPIOD
#define ADC_MOTOR1_Pin LL_GPIO_PIN_3
#define ADC_MOTOR1_GPIO_Port GPIOA
#define ADC_MOTOR3_Pin LL_GPIO_PIN_1
#define ADC_MOTOR3_GPIO_Port GPIOB
#define CTRL_MOTOR1_Pin LL_GPIO_PIN_9
#define CTRL_MOTOR1_GPIO_Port GPIOE
#define PWM_MOTOR2_Pin LL_GPIO_PIN_13
#define PWM_MOTOR2_GPIO_Port GPIOE
#define SBUS_TX_Pin LL_GPIO_PIN_8
#define SBUS_TX_GPIO_Port GPIOD
#define ENCA_MOTOR3_Pin LL_GPIO_PIN_12
#define ENCA_MOTOR3_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

extern uart_t uart1_cb;
extern uart_t uart2_cb;
extern uart_t uart3_cb;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
