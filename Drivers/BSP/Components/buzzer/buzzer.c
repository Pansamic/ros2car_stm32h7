/**
 * @file buzzer.c
 * @author pansamic (pansamic@foxmail.com)
 * @brief 
 * @version 0.1
 * @date 2025-03-26
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#include "main.h"
#include "buzzer.h"

#define BUZZER_TIM_FREQUENCY 240000000ULL

void buzzer_on(void)
{
    LL_TIM_CC_EnableChannel(BUZZER_TIM, LL_TIM_CHANNEL_CH1);
    LL_TIM_EnableCounter(BUZZER_TIM);
    LL_TIM_EnableAllOutputs(BUZZER_TIM);
}

void buzzer_off(void)
{
    LL_TIM_CC_DisableChannel(BUZZER_TIM, LL_TIM_CHANNEL_CH1);
    LL_TIM_DisableCounter(BUZZER_TIM);
    LL_TIM_DisableAllOutputs(BUZZER_TIM);
}

void buzzer_set_frequency(uint16_t frequency)
{
    uint32_t prescaler = LL_TIM_GetPrescaler(BUZZER_TIM) + 1;
    uint32_t arr = BUZZER_TIM_FREQUENCY / prescaler / frequency - 1;
    uint32_t period = (arr + 1) / 2 - 1;
    LL_TIM_SetAutoReload(BUZZER_TIM, BUZZER_TIM_FREQUENCY / prescaler / frequency - 1);
    LL_TIM_OC_SetCompareCH1(BUZZER_TIM, period);
}