
#ifndef __WS2812B_H__
#define __WS2812B_H__


#include <stdint.h>
#include "syslog.h"

#define WS2812B_AMOUNT 4

#define WS2812B_TIM_PERIOD 300

#define WS2812B_LOG_CRITICAL(...)   LOG_CRITICAL(__VA_ARGS__)
#define WS2812B_LOG_ERROR(...)      LOG_ERROR(__VA_ARGS__)
#define WS2812B_LOG_WARNING(...)    LOG_WARN(__VA_ARGS__)
#define WS2812B_LOG_INFO(...)       LOG_INFO(__VA_ARGS__)
#define WS2812B_LOG_DEBUG(...)      LOG_DEBUG(__VA_ARGS__)
#define WS2812B_LOG_TRACE(...)      LOG_TRACE(__VA_ARGS__)

typedef enum {
    WS2812B_OK = 0,
    WS2812B_ERROR = 1
} ws2812b_err_t;

/**
 * @brief Initialize PWM timer and DMA for WS2812B
 * 
 * @return ws2812b_err_t 
 */
ws2812b_err_t ws2812b_init(void);

/**
 * @brief Set color of WS2812B by writing duty value to PWM compare value buffer and start DMA transfer.
 * 
 * @param index Index of WS2812B, from 0 to WS2812B_AMOUNT-1
 * @param r Red value, from 0 to 255
 * @param g Green value, from 0 to 255
 * @param b Blue value, from 0 to 255
 * @return ws2812b_err_t 
 */
ws2812b_err_t ws2812b_set_color(uint8_t index, uint8_t r, uint8_t g, uint8_t b);

/**
 * @brief Flush the DMA transfer to update the WS2812B color.
 * 
 * @return ws2812b_err_t 
 */
ws2812b_err_t ws2812b_flush(void);

/**
 * @brief Disable the DMA transfer.
 * @note Use in WS2812B DMA transfer complete interrupt.
 */
void ws2812b_flush_complete(void);
#endif // __WS2812B_H__