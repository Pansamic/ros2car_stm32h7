
#include "ws2812b.h"

#ifdef cplusplus
extern "C" {
#endif

ws2812b_err_t ws2812b_init(void)
{
    return WS2812B_OK;
}

ws2812b_err_t ws2812b_set_color(uint8_t index, uint8_t r, uint8_t g, uint8_t b)
{
    return WS2812B_OK;
}

ws2812b_err_t ws2812b_set_brightness(uint8_t index, uint8_t brightness)
{
    return WS2812B_OK;
}

ws2812b_err_t ws2812b_set_all_color(uint8_t r, uint8_t g, uint8_t b)
{
    return WS2812B_OK;
}

ws2812b_err_t ws2812b_set_all_brightness(uint8_t brightness)
{
    return WS2812B_OK;
}

ws2812b_err_t ws2812b_update(void)
{
    return WS2812B_OK;
}

#ifdef cplusplus
}
#endif
