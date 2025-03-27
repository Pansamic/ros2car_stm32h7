/**
 * @file battery.c
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2025-03-20
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "battery.h"

#ifdef cplusplus
extern "C" {
#endif

int16_t battery_adc = 0;

static xTimerHandle battery_monitor_timer;

static void battery_monitor(TimerHandle_t xTimer)
{
    float battery_voltage = 0.0;
    battery_voltage = (float)battery_adc * 3.3 * BATTERY_SCALE_FACTOR / 65535.0 ;
    BATTERY_LOG_INFO("Battery voltage: %f V\n", battery_voltage);
}

/**
 * @brief Start Timer
 * 
 * @return battery_err_t 
 */
battery_err_t battery_init(void)
{
    /* Run ADC self calibration */
    // Need to define which calib parameter has to be used
    LL_ADC_StartCalibration(ADC3, LL_ADC_CALIB_OFFSET, LL_ADC_SINGLE_ENDED);
    LL_ADC_StartCalibration(ADC3, LL_ADC_CALIB_OFFSET_LINEARITY, LL_ADC_SINGLE_ENDED);

    while (LL_ADC_IsCalibrationOnGoing(ADC3) != 0);
    
    /* Delay between ADC end of calibration and ADC enable. */
    vTaskDelay(pdMS_TO_TICKS(5));
    
    /* Enable ADC */
    LL_ADC_Enable(ADC3);

    while (LL_ADC_IsActiveFlag_ADRDY(ADC3) == 0);

    if ((LL_ADC_IsEnabled(ADC3) == 1)               &&
        (LL_ADC_IsDisableOngoing(ADC3) == 0)        &&
        (LL_ADC_REG_IsConversionOngoing(ADC3) == 0)   )
    {
        LL_ADC_REG_StartConversion(ADC3);
        LL_DMA_SetPeriphAddress(DMA2, LL_DMA_STREAM_3, LL_ADC_DMA_GetRegAddr(ADC3, LL_ADC_DMA_REG_REGULAR_DATA));
        LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_3, (uint32_t)&battery_adc);
        LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_3, 1);
        LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_3);
    }
    else
    {
        BATTERY_LOG_ERROR("Battery monitor init failed: ADC conversion start could not be performed.\n");
        return BATTERY_ERROR;
    }

    battery_monitor_timer = xTimerCreate("battery_monitor", pdMS_TO_TICKS(1000), pdTRUE, NULL, battery_monitor);
    xTimerStart(battery_monitor_timer, 0);

    return BATTERY_OK;
}


/**
 * @brief Read battery voltage
 * @return float Battery voltage 
 */
float battery_read()
{
    return (float)battery_adc * 3.3 * BATTERY_SCALE_FACTOR / 65535.0 ;
}
#ifdef cplusplus
}
#endif