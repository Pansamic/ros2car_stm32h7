/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32h7xx_it.c
  * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32h7xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "syslog.h"
#include "drv8874.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
extern SD_HandleTypeDef hsd1;
extern TIM_HandleTypeDef htim16;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32H7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32h7xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 stream0 global interrupt.
  */
void DMA1_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream0_IRQn 0 */
  size_t len = 0;
  void* p = NULL;
  ringbuf_ret_t err = RINGBUF_OK;
  uint8_t ringbuf_lock_flag = 0;
  if(LL_DMA_IsActiveFlag_TC0(uart1_cb.dma_rx))
  {
    if(ringbuf_locked(&uart1_cb.rx_ringbuf))
    {
      ringbuf_lock_flag = 1;
      ringbuf_unlock(&uart1_cb.rx_ringbuf);
    }
    LL_DMA_ClearFlag_TC0(uart1_cb.dma_rx);
    LL_DMA_DisableStream(uart1_cb.dma_rx, uart1_cb.dma_rx_channel);
    len = uart1_cb.expect_dma_rx_len - LL_DMA_GetDataLength(uart1_cb.dma_rx, uart1_cb.dma_rx_channel);
    err = ringbuf_compensate_written(&uart1_cb.rx_ringbuf, len);
    if(err != RINGBUF_OK)
    {
      for(;;){}
    }
    err = ringbuf_get_free_continuous_block(&uart1_cb.rx_ringbuf, &p, &len);
    if(err != RINGBUF_OK)
    {
      for(;;){}
    }
    LL_DMA_SetMemoryAddress(uart1_cb.dma_rx, uart1_cb.dma_rx_channel, (uint32_t)p);
    LL_DMA_SetDataLength(uart1_cb.dma_rx, uart1_cb.dma_rx_channel, (uint32_t)len);
    uart1_cb.expect_dma_rx_len = len;
    LL_DMA_EnableIT_TC(uart1_cb.dma_rx, uart1_cb.dma_rx_channel);
    LL_DMA_EnableStream(uart1_cb.dma_rx, uart1_cb.dma_rx_channel);
    LL_USART_EnableIT_IDLE(uart1_cb.huart);
    if(ringbuf_lock_flag)
    {
      ringbuf_lock(&uart1_cb.rx_ringbuf);
    }
  }

  /* USER CODE END DMA1_Stream0_IRQn 0 */
  /* USER CODE BEGIN DMA1_Stream0_IRQn 1 */

  /* USER CODE END DMA1_Stream0_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream1 global interrupt.
  */
void DMA1_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream1_IRQn 0 */
  size_t len;
  size_t removed_len;
  ringbuf_ret_t err;
  if(LL_DMA_IsActiveFlag_TC1(uart1_cb.dma_tx))
  {
    LL_DMA_ClearFlag_TC1(uart1_cb.dma_tx);
    len = uart1_cb.expect_dma_tx_len - LL_DMA_GetDataLength(uart1_cb.dma_tx, uart1_cb.dma_tx_channel);
    err = ringbuf_remove_block(&uart1_cb.tx_ringbuf, len, &removed_len);
    if(err != RINGBUF_OK)
    {
      // logic error if reach here
      // for(;;){}
      LOG_ERROR("USART1 TX DMA error: ringbuf_remove_block failed: %d", err);
    }
    uart1_cb.expect_dma_tx_len = 0;
    uart1_cb.tx_cplt_flag = 1;
  }

  /* USER CODE END DMA1_Stream1_IRQn 0 */
  /* USER CODE BEGIN DMA1_Stream1_IRQn 1 */

  /* USER CODE END DMA1_Stream1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream2 global interrupt.
  */
void DMA1_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream2_IRQn 0 */
  size_t len = 0;
  void* p = NULL;
  ringbuf_ret_t err = RINGBUF_OK;
  uint8_t ringbuf_lock_flag = 0;
  if(LL_DMA_IsActiveFlag_TC2(uart2_cb.dma_rx))
  {
    if(ringbuf_locked(&uart2_cb.rx_ringbuf))
    {
      ringbuf_lock_flag = 1;
      ringbuf_unlock(&uart2_cb.rx_ringbuf);
    }
    LL_DMA_ClearFlag_TC2(uart2_cb.dma_rx);
    LL_DMA_DisableStream(uart2_cb.dma_rx, uart2_cb.dma_rx_channel);
    len = uart2_cb.expect_dma_rx_len - LL_DMA_GetDataLength(uart2_cb.dma_rx, uart2_cb.dma_rx_channel);
    err = ringbuf_compensate_written(&uart2_cb.rx_ringbuf, len);
    if(err != RINGBUF_OK)
    {
      for(;;){}
    }
    err = ringbuf_get_free_continuous_block(&uart2_cb.rx_ringbuf, &p, &len);
    if(err != RINGBUF_OK)
    {
      for(;;){}
    }
    LL_DMA_SetMemoryAddress(uart2_cb.dma_rx, uart2_cb.dma_rx_channel, (uint32_t)p);
    LL_DMA_SetDataLength(uart2_cb.dma_rx, uart2_cb.dma_rx_channel, (uint32_t)len);
    uart2_cb.expect_dma_rx_len = len;
    LL_DMA_EnableIT_TC(uart2_cb.dma_rx, uart2_cb.dma_rx_channel);
    LL_DMA_EnableStream(uart2_cb.dma_rx, uart2_cb.dma_rx_channel);
    LL_USART_EnableIT_IDLE(uart2_cb.huart);
    if(ringbuf_lock_flag)
    {
      ringbuf_lock(&uart2_cb.rx_ringbuf);
    }
  }

  /* USER CODE END DMA1_Stream2_IRQn 0 */
  /* USER CODE BEGIN DMA1_Stream2_IRQn 1 */

  /* USER CODE END DMA1_Stream2_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream3 global interrupt.
  */
void DMA1_Stream3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream3_IRQn 0 */
  size_t len;
  size_t removed_len;
  ringbuf_ret_t err;
  if(LL_DMA_IsActiveFlag_TC3(uart2_cb.dma_tx))
  {
    LL_DMA_ClearFlag_TC3(uart2_cb.dma_tx);
    len = uart2_cb.expect_dma_tx_len - LL_DMA_GetDataLength(uart2_cb.dma_tx, uart2_cb.dma_tx_channel);
    err = ringbuf_remove_block(&uart2_cb.tx_ringbuf, len, &removed_len);
    if(err != RINGBUF_OK)
    {
      // logic error if reach here
      // for(;;){}
      LOG_ERROR("USART2 TX DMA error: ringbuf_remove_block failed: %d", err);
    }
    uart2_cb.expect_dma_tx_len = 0;
    uart2_cb.tx_cplt_flag = 1;
  }
  /* USER CODE END DMA1_Stream3_IRQn 0 */
  /* USER CODE BEGIN DMA1_Stream3_IRQn 1 */

  /* USER CODE END DMA1_Stream3_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream4 global interrupt.
  */
void DMA1_Stream4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream4_IRQn 0 */
  size_t len = 0;
  void* p = NULL;
  ringbuf_ret_t err = RINGBUF_OK;
  uint8_t ringbuf_lock_flag = 0;
  if(LL_DMA_IsActiveFlag_TC4(uart3_cb.dma_rx))
  {
    if(ringbuf_locked(&uart3_cb.rx_ringbuf))
    {
      ringbuf_lock_flag = 1;
      ringbuf_unlock(&uart3_cb.rx_ringbuf);
    }
    LL_DMA_ClearFlag_TC4(uart3_cb.dma_rx);
    LL_DMA_DisableStream(uart3_cb.dma_rx, uart3_cb.dma_rx_channel);
    len = uart3_cb.expect_dma_rx_len - LL_DMA_GetDataLength(uart3_cb.dma_rx, uart3_cb.dma_rx_channel);
    err = ringbuf_compensate_written(&uart3_cb.rx_ringbuf, len);
    if(err != RINGBUF_OK)
    {
      for(;;){}
    }
    err = ringbuf_get_free_continuous_block(&uart3_cb.rx_ringbuf, &p, &len);
    if(err != RINGBUF_OK)
    {
      for(;;){}
    }
    LL_DMA_SetMemoryAddress(uart3_cb.dma_rx, uart3_cb.dma_rx_channel, (uint32_t)p);
    LL_DMA_SetDataLength(uart3_cb.dma_rx, uart3_cb.dma_rx_channel, (uint32_t)len);
    uart3_cb.expect_dma_rx_len = len;
    LL_DMA_EnableIT_TC(uart3_cb.dma_rx, uart3_cb.dma_rx_channel);
    LL_DMA_EnableStream(uart3_cb.dma_rx, uart3_cb.dma_rx_channel);
    LL_USART_EnableIT_IDLE(uart3_cb.huart);
    if(ringbuf_lock_flag)
    {
      ringbuf_lock(&uart3_cb.rx_ringbuf);
    }
  }
  /* USER CODE END DMA1_Stream4_IRQn 0 */
  /* USER CODE BEGIN DMA1_Stream4_IRQn 1 */

  /* USER CODE END DMA1_Stream4_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream5 global interrupt.
  */
void DMA1_Stream5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream5_IRQn 0 */
  size_t len;
  size_t removed_len;
  ringbuf_ret_t err;
  if(LL_DMA_IsActiveFlag_TC5(uart2_cb.dma_tx))
  {
    LL_DMA_ClearFlag_TC5(uart2_cb.dma_tx);
    len = uart2_cb.expect_dma_tx_len - LL_DMA_GetDataLength(uart2_cb.dma_tx, uart2_cb.dma_tx_channel);
    err = ringbuf_remove_block(&uart2_cb.tx_ringbuf, len, &removed_len);
    if(err != RINGBUF_OK)
    {
      // logic error if reach here
      // for(;;){}
      LOG_ERROR("USART3 TX DMA error: ringbuf_remove_block failed: %d", err);
    }
    uart2_cb.expect_dma_tx_len = 0;
    uart2_cb.tx_cplt_flag = 1;
  }
  /* USER CODE END DMA1_Stream5_IRQn 0 */
  /* USER CODE BEGIN DMA1_Stream5_IRQn 1 */

  /* USER CODE END DMA1_Stream5_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream6 global interrupt.
  */
void DMA1_Stream6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream6_IRQn 0 */

  /* USER CODE END DMA1_Stream6_IRQn 0 */
  /* USER CODE BEGIN DMA1_Stream6_IRQn 1 */

  /* USER CODE END DMA1_Stream6_IRQn 1 */
}

/**
  * @brief This function handles SPI1 global interrupt.
  */
void SPI1_IRQHandler(void)
{
  /* USER CODE BEGIN SPI1_IRQn 0 */

  /* USER CODE END SPI1_IRQn 0 */
  /* USER CODE BEGIN SPI1_IRQn 1 */

  /* USER CODE END SPI1_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
  size_t len = 0;
  void* p = NULL;
  ringbuf_ret_t err = RINGBUF_OK;
  uint8_t ringbuf_lock_flag = 0;
  if(LL_USART_IsActiveFlag_IDLE(uart1_cb.huart))
  {
    if(ringbuf_locked(&uart1_cb.rx_ringbuf))
    {
      ringbuf_lock_flag = 1;
      ringbuf_unlock(&uart1_cb.rx_ringbuf);
    }
    LL_USART_ClearFlag_IDLE(uart1_cb.huart);
    LL_DMA_DisableStream(uart1_cb.dma_rx, uart1_cb.dma_rx_channel);
    len = uart1_cb.expect_dma_rx_len - LL_DMA_GetDataLength(uart1_cb.dma_rx, uart1_cb.dma_rx_channel);
    err = ringbuf_compensate_written(&uart1_cb.rx_ringbuf, len);
    if(err != RINGBUF_OK)
    {
      // for(;;){}
      LOG_ERROR("USART1 RX DMA error: ringbuf_compensate_written() failed: %d", err);
    }
    err = ringbuf_get_free_continuous_block(&uart1_cb.rx_ringbuf, &p, &len);
    if(err != RINGBUF_OK)
    {
      // for(;;){}
      LOG_ERROR("USART1 RX DMA error: ringbuf_get_free_continuous_block() failed: %d", err);
    }
    LL_DMA_SetMemoryAddress(uart1_cb.dma_rx, uart1_cb.dma_rx_channel, (uint32_t)p);
    LL_DMA_SetDataLength(uart1_cb.dma_rx, uart1_cb.dma_rx_channel, (uint32_t)len);
    uart1_cb.expect_dma_rx_len = len;
    LL_DMA_EnableIT_TC(uart1_cb.dma_rx, uart1_cb.dma_rx_channel);
    LL_DMA_EnableStream(uart1_cb.dma_rx, uart1_cb.dma_rx_channel);
    LL_USART_EnableIT_IDLE(uart1_cb.huart);
    if(ringbuf_lock_flag)
    {
      ringbuf_lock(&uart1_cb.rx_ringbuf);
    }
  }
  // else if(LL_USART_IsActiveFlag_TC(uart1_cb.huart))
  // {
  //   if(ringbuf_locked(&uart1_cb.rx_ringbuf))
  //   {
  //     ringbuf_lock_flag = 1;
  //     ringbuf_unlock(&uart1_cb.rx_ringbuf);
  //   }
  //   LL_USART_ClearFlag_TC(uart1_cb.huart);
  //   err = ringbuf_get_free_size(&uart1_cb.rx_ringbuf, &len);
  //   if(err != RINGBUF_OK)
  //   {
  //     // for(;;){}
  //     LOG_ERROR("USART1 RX DMA error: ringbuf_get_free_size() failed: %d", err);
  //   }
  //   if(len>1)
  //   {
  //     err = ringbuf_write_byte(&uart1_cb.rx_ringbuf, (uint8_t)(uart1_cb.huart->RDR & 0x000000FF));
  //     if(err != RINGBUF_OK)
  //     {
  //       for(;;){}
  //     }
  //     err = ringbuf_get_free_continuous_block(&uart1_cb.rx_ringbuf, &p, &len);
  //     if(err != RINGBUF_OK)
  //     {
  //       for(;;){}
  //     }
  //     LL_DMA_SetMemoryAddress(uart1_cb.dma_rx, uart1_cb.dma_rx_channel, (uint32_t)p);
  //     LL_DMA_SetDataLength(uart1_cb.dma_rx, uart1_cb.dma_rx_channel, (uint32_t)len);
  //     uart1_cb.expect_dma_rx_len = len;
  //     LL_DMA_EnableIT_TC(uart1_cb.dma_rx, uart1_cb.dma_rx_channel);
  //     LL_DMA_EnableStream(uart1_cb.dma_rx, uart1_cb.dma_rx_channel);
  //     LL_USART_EnableIT_IDLE(uart1_cb.huart);        
  //   }
  //   else if(len==1)
  //   {
  //     err = ringbuf_write_byte(&uart1_cb.rx_ringbuf, (uint8_t)(uart1_cb.huart->RDR & 0x000000FF));
  //     if(err != RINGBUF_OK)
  //     {
  //       for(;;){}
  //     }
  //     LL_USART_EnableIT_TC(uart1_cb.huart);
  //   }
  //   else
  //   {
  //     LL_USART_EnableIT_TC(uart1_cb.huart);
  //   }
  //   if(ringbuf_lock_flag)
  //   {
  //     ringbuf_lock(&uart1_cb.rx_ringbuf);
  //   }
  // }

  /* USER CODE END USART1_IRQn 0 */
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */
  size_t len = 0;
  void* p = NULL;
  ringbuf_ret_t err = RINGBUF_OK;
  uint8_t ringbuf_lock_flag = 0;
  if(LL_USART_IsActiveFlag_IDLE(uart2_cb.huart))
  {
    if(ringbuf_locked(&uart2_cb.rx_ringbuf))
    {
      ringbuf_lock_flag = 1;
      ringbuf_unlock(&uart2_cb.rx_ringbuf);
    }
    LL_USART_ClearFlag_IDLE(uart2_cb.huart);
    LL_DMA_DisableStream(uart2_cb.dma_rx, uart2_cb.dma_rx_channel);
    len = uart2_cb.expect_dma_rx_len - LL_DMA_GetDataLength(uart2_cb.dma_rx, uart2_cb.dma_rx_channel);
    err = ringbuf_compensate_written(&uart2_cb.rx_ringbuf, len);
    if(err != RINGBUF_OK)
    {
      for(;;){}
    }
    err = ringbuf_get_free_continuous_block(&uart2_cb.rx_ringbuf, &p, &len);
    if(err != RINGBUF_OK)
    {
      for(;;){}
    }
    LL_DMA_SetMemoryAddress(uart2_cb.dma_rx, uart2_cb.dma_rx_channel, (uint32_t)p);
    LL_DMA_SetDataLength(uart2_cb.dma_rx, uart2_cb.dma_rx_channel, (uint32_t)len);
    uart2_cb.expect_dma_rx_len = len;
    LL_DMA_EnableIT_TC(uart2_cb.dma_rx, uart2_cb.dma_rx_channel);
    LL_DMA_EnableStream(uart2_cb.dma_rx, uart2_cb.dma_rx_channel);
    LL_USART_EnableIT_IDLE(uart2_cb.huart);
    if(ringbuf_lock_flag)
    {
      ringbuf_lock(&uart2_cb.rx_ringbuf);
    }
  }
  // else if(LL_USART_IsActiveFlag_TC(uart2_cb.huart))
  // {
  //   if(ringbuf_locked(&uart2_cb.rx_ringbuf))
  //   {
  //     ringbuf_lock_flag = 1;
  //     ringbuf_unlock(&uart2_cb.rx_ringbuf);
  //   }
  //   LL_USART_ClearFlag_TC(uart2_cb.huart);
  //   err = ringbuf_get_free_size(&uart2_cb.rx_ringbuf, &len);
  //   if(err != RINGBUF_OK)
  //   {
  //     for(;;){}
  //   }
  //   if(len>1)
  //   {
  //     err = ringbuf_write_byte(&uart2_cb.rx_ringbuf, (uint8_t)(uart2_cb.huart->RDR & 0x000000FF));
  //     if(err != RINGBUF_OK)
  //     {
  //       for(;;){}
  //     }
  //     err = ringbuf_get_free_continuous_block(&uart2_cb.rx_ringbuf, &p, &len);
  //     if(err != RINGBUF_OK)
  //     {
  //       for(;;){}
  //     }
  //     LL_DMA_SetMemoryAddress(uart2_cb.dma_rx, uart2_cb.dma_rx_channel, (uint32_t)p);
  //     LL_DMA_SetDataLength(uart2_cb.dma_rx, uart2_cb.dma_rx_channel, (uint32_t)len);
  //     uart2_cb.expect_dma_rx_len = len;
  //     LL_DMA_EnableIT_TC(uart2_cb.dma_rx, uart2_cb.dma_rx_channel);
  //     LL_DMA_EnableStream(uart2_cb.dma_rx, uart2_cb.dma_rx_channel);
  //     LL_USART_EnableIT_IDLE(uart2_cb.huart);        
  //   }
  //   else if(len==1)
  //   {
  //     err = ringbuf_write_byte(&uart2_cb.rx_ringbuf, (uint8_t)(uart2_cb.huart->RDR & 0x000000FF));
  //     if(err != RINGBUF_OK)
  //     {
  //       for(;;){}
  //     }
  //     LL_USART_EnableIT_TC(uart2_cb.huart);
  //   }
  //   else
  //   {
  //     LL_USART_EnableIT_TC(uart2_cb.huart);
  //   }
  //   if(ringbuf_lock_flag)
  //   {
  //     ringbuf_lock(&uart2_cb.rx_ringbuf);
  //   }
  // }
  /* USER CODE END USART2_IRQn 0 */
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */
  size_t len = 0;
  void* p = NULL;
  ringbuf_ret_t err = RINGBUF_OK;
  uint8_t ringbuf_lock_flag = 0;
  if(LL_USART_IsActiveFlag_IDLE(uart3_cb.huart))
  {
    if(ringbuf_locked(&uart3_cb.rx_ringbuf))
    {
      ringbuf_lock_flag = 1;
      ringbuf_unlock(&uart3_cb.rx_ringbuf);
    }
    LL_USART_ClearFlag_IDLE(uart3_cb.huart);
    LL_DMA_DisableStream(uart3_cb.dma_rx, uart3_cb.dma_rx_channel);
    len = uart3_cb.expect_dma_rx_len - LL_DMA_GetDataLength(uart3_cb.dma_rx, uart3_cb.dma_rx_channel);
    err = ringbuf_compensate_written(&uart3_cb.rx_ringbuf, len);
    if(err != RINGBUF_OK)
    {
      for(;;){}
    }
    err = ringbuf_get_free_continuous_block(&uart3_cb.rx_ringbuf, &p, &len);
    if(err != RINGBUF_OK)
    {
      for(;;){}
    }
    LL_DMA_SetMemoryAddress(uart3_cb.dma_rx, uart3_cb.dma_rx_channel, (uint32_t)p);
    LL_DMA_SetDataLength(uart3_cb.dma_rx, uart3_cb.dma_rx_channel, (uint32_t)len);
    uart3_cb.expect_dma_rx_len = len;
    LL_DMA_EnableIT_TC(uart3_cb.dma_rx, uart3_cb.dma_rx_channel);
    LL_DMA_EnableStream(uart3_cb.dma_rx, uart3_cb.dma_rx_channel);
    LL_USART_EnableIT_IDLE(uart3_cb.huart);
    if(ringbuf_lock_flag)
    {
      ringbuf_lock(&uart3_cb.rx_ringbuf);
    }
  }
  // else if(LL_USART_IsActiveFlag_TC(uart3_cb.huart))
  // {
  //   if(ringbuf_locked(&uart3_cb.rx_ringbuf))
  //   {
  //     ringbuf_lock_flag = 1;
  //     ringbuf_unlock(&uart3_cb.rx_ringbuf);
  //   }
  //   LL_USART_ClearFlag_TC(uart3_cb.huart);
  //   err = ringbuf_get_free_size(&uart3_cb.rx_ringbuf, &len);
  //   if(err != RINGBUF_OK)
  //   {
  //     for(;;){}
  //   }
  //   if(len>1)
  //   {
  //     err = ringbuf_write_byte(&uart3_cb.rx_ringbuf, (uint8_t)(uart3_cb.huart->RDR & 0x000000FF));
  //     if(err != RINGBUF_OK)
  //     {
  //       for(;;){}
  //     }
  //     err = ringbuf_get_free_continuous_block(&uart3_cb.rx_ringbuf, &p, &len);
  //     if(err != RINGBUF_OK)
  //     {
  //       for(;;){}
  //     }
  //     LL_DMA_SetMemoryAddress(uart3_cb.dma_rx, uart3_cb.dma_rx_channel, (uint32_t)p);
  //     LL_DMA_SetDataLength(uart3_cb.dma_rx, uart3_cb.dma_rx_channel, (uint32_t)len);
  //     uart3_cb.expect_dma_rx_len = len;
  //     LL_DMA_EnableIT_TC(uart3_cb.dma_rx, uart3_cb.dma_rx_channel);
  //     LL_DMA_EnableStream(uart3_cb.dma_rx, uart3_cb.dma_rx_channel);
  //     LL_USART_EnableIT_IDLE(uart3_cb.huart);        
  //   }
  //   else if(len==1)
  //   {
  //     err = ringbuf_write_byte(&uart3_cb.rx_ringbuf, (uint8_t)(uart3_cb.huart->RDR & 0x000000FF));
  //     if(err != RINGBUF_OK)
  //     {
  //       for(;;){}
  //     }
  //     LL_USART_EnableIT_TC(uart3_cb.huart);
  //   }
  //   else
  //   {
  //     LL_USART_EnableIT_TC(uart3_cb.huart);
  //   }
  //   if(ringbuf_lock_flag)
  //   {
  //     ringbuf_lock(&uart3_cb.rx_ringbuf);
  //   }
  // }
  /* USER CODE END USART3_IRQn 0 */
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles TIM8 update interrupt and TIM13 global interrupt.
  */
void TIM8_UP_TIM13_IRQHandler(void)
{
  /* USER CODE BEGIN TIM8_UP_TIM13_IRQn 0 */
  if(LL_TIM_IsActiveFlag_UPDATE(TIM13))
  {
    LL_TIM_ClearFlag_UPDATE(TIM13);
    motors[0].interval_timer_count += 0x10000;
    motors[1].interval_timer_count += 0x10000;
    motors[2].interval_timer_count += 0x10000;
    motors[3].interval_timer_count += 0x10000;
  }
  /* USER CODE END TIM8_UP_TIM13_IRQn 0 */
  /* USER CODE BEGIN TIM8_UP_TIM13_IRQn 1 */

  /* USER CODE END TIM8_UP_TIM13_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream7 global interrupt.
  */
void DMA1_Stream7_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream7_IRQn 0 */

  /* USER CODE END DMA1_Stream7_IRQn 0 */
  /* USER CODE BEGIN DMA1_Stream7_IRQn 1 */

  /* USER CODE END DMA1_Stream7_IRQn 1 */
}

/**
  * @brief This function handles SDMMC1 global interrupt.
  */
void SDMMC1_IRQHandler(void)
{
  /* USER CODE BEGIN SDMMC1_IRQn 0 */

  /* USER CODE END SDMMC1_IRQn 0 */
  HAL_SD_IRQHandler(&hsd1);
  /* USER CODE BEGIN SDMMC1_IRQn 1 */

  /* USER CODE END SDMMC1_IRQn 1 */
}

/**
  * @brief This function handles UART4 global interrupt.
  */
void UART4_IRQHandler(void)
{
  /* USER CODE BEGIN UART4_IRQn 0 */

  /* USER CODE END UART4_IRQn 0 */
  /* USER CODE BEGIN UART4_IRQn 1 */

  /* USER CODE END UART4_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream0 global interrupt.
  */
void DMA2_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream0_IRQn 0 */

  /* USER CODE END DMA2_Stream0_IRQn 0 */
  /* USER CODE BEGIN DMA2_Stream0_IRQn 1 */

  /* USER CODE END DMA2_Stream0_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream1 global interrupt.
  */
void DMA2_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream1_IRQn 0 */

  /* USER CODE END DMA2_Stream1_IRQn 0 */
  /* USER CODE BEGIN DMA2_Stream1_IRQn 1 */

  /* USER CODE END DMA2_Stream1_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream2 global interrupt.
  */
void DMA2_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream2_IRQn 0 */

  /* USER CODE END DMA2_Stream2_IRQn 0 */
  /* USER CODE BEGIN DMA2_Stream2_IRQn 1 */

  /* USER CODE END DMA2_Stream2_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream3 global interrupt.
  */
void DMA2_Stream3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream3_IRQn 0 */

  /* USER CODE END DMA2_Stream3_IRQn 0 */
  /* USER CODE BEGIN DMA2_Stream3_IRQn 1 */

  /* USER CODE END DMA2_Stream3_IRQn 1 */
}

/**
  * @brief This function handles USB On The Go FS End Point 1 Out global interrupt.
  */
void OTG_FS_EP1_OUT_IRQHandler(void)
{
  /* USER CODE BEGIN OTG_FS_EP1_OUT_IRQn 0 */

  /* USER CODE END OTG_FS_EP1_OUT_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
  /* USER CODE BEGIN OTG_FS_EP1_OUT_IRQn 1 */

  /* USER CODE END OTG_FS_EP1_OUT_IRQn 1 */
}

/**
  * @brief This function handles USB On The Go FS End Point 1 In global interrupt.
  */
void OTG_FS_EP1_IN_IRQHandler(void)
{
  /* USER CODE BEGIN OTG_FS_EP1_IN_IRQn 0 */

  /* USER CODE END OTG_FS_EP1_IN_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
  /* USER CODE BEGIN OTG_FS_EP1_IN_IRQn 1 */

  /* USER CODE END OTG_FS_EP1_IN_IRQn 1 */
}

/**
  * @brief This function handles USB On The Go FS global interrupt.
  */
void OTG_FS_IRQHandler(void)
{
  /* USER CODE BEGIN OTG_FS_IRQn 0 */

  /* USER CODE END OTG_FS_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
  /* USER CODE BEGIN OTG_FS_IRQn 1 */

  /* USER CODE END OTG_FS_IRQn 1 */
}

/**
  * @brief This function handles TIM16 global interrupt.
  */
void TIM16_IRQHandler(void)
{
  /* USER CODE BEGIN TIM16_IRQn 0 */

  /* USER CODE END TIM16_IRQn 0 */
  HAL_TIM_IRQHandler(&htim16);
  /* USER CODE BEGIN TIM16_IRQn 1 */

  /* USER CODE END TIM16_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
