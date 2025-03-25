
#ifndef __WS2812B_H__
#define __WS2812B_H__


#include <stdint.h>
#include "syslog.h"

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

ws2812b_err_t ws2812b_init(void);
ws2812b_err_t ws2812b_set_color(uint8_t index, uint8_t r, uint8_t g, uint8_t b);
ws2812b_err_t ws2812b_set_brightness(uint8_t index, uint8_t brightness);
ws2812b_err_t ws2812b_set_all_color(uint8_t r, uint8_t g, uint8_t b);
ws2812b_err_t ws2812b_set_all_brightness(uint8_t brightness);
ws2812b_err_t ws2812b_update(void);

#endif // __WS2812B_H__