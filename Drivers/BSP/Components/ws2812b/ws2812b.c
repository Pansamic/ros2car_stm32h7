#include "main.h"
#include "ws2812b.h"
#include "FreeRTOS.h"
#include "task.h"

#ifdef cplusplus
extern "C" {
#endif

#define WS2812B_TIM_ONE_COUNT WS2812B_TIM_PERIOD * 2 / 3 - 1
#define WS2812B_TIM_ZERO_COUNT WS2812B_TIM_PERIOD / 3 - 1
#define WS2812B_RESET_SEQUENCE_AMOUNT 200

uint16_t ws2812b_buffer[WS2812B_AMOUNT * 3 * 8] = {WS2812B_TIM_ZERO_COUNT};

ws2812b_err_t ws2812b_init(void)
{
    LL_TIM_EnableCounter(RGB_TIM);
    LL_TIM_CC_EnableChannel(RGB_TIM, LL_TIM_CHANNEL_CH1);
    LL_TIM_EnableDMAReq_CC1(RGB_TIM);
    LL_TIM_OC_SetCompareCH1(RGB_TIM, 0);
    LL_TIM_EnableAllOutputs(RGB_TIM);
    LL_DMA_SetPeriphAddress(DMA2, LL_DMA_STREAM_4, (uint32_t)&(RGB_TIM->CCR1));
    LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_4, (uint32_t)&ws2812b_buffer);
    for(uint16_t* p = ws2812b_buffer ; p < ws2812b_buffer + sizeof(ws2812b_buffer) ; p++)
    {
        *p = WS2812B_TIM_ZERO_COUNT;
    }
    // LL_DMA_SetPeriphAddress(DMA2, LL_DMA_STREAM_4, (uint32_t)&(RGB_TIM->CCR1));
    // LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_4, (uint32_t)&ws2812b_buffer);
    // LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_4, sizeof(ws2812b_buffer)/sizeof(ws2812b_buffer[0]));
    // LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_4);
    return WS2812B_OK;
}

ws2812b_err_t ws2812b_set_color(uint8_t index, uint8_t r, uint8_t g, uint8_t b)
{
    if(index >= WS2812B_AMOUNT)
    {
        return WS2812B_ERROR;
    }
    for(uint8_t i=0 ; i<8 ; i++)
    {
        ws2812b_buffer[index * 24 + 0 * 8 + i] = (uint16_t)((g<<i) & 0x80 ? WS2812B_TIM_ONE_COUNT : WS2812B_TIM_ZERO_COUNT);
        ws2812b_buffer[index * 24 + 1 * 8 + i] = (uint16_t)((r<<i) & 0x80 ? WS2812B_TIM_ONE_COUNT : WS2812B_TIM_ZERO_COUNT);
        ws2812b_buffer[index * 24 + 2 * 8 + i] = (uint16_t)((b<<i) & 0x80 ? WS2812B_TIM_ONE_COUNT : WS2812B_TIM_ZERO_COUNT);
    }
    return WS2812B_OK;
}

ws2812b_err_t ws2812b_flush(void)
{
    LL_TIM_OC_SetCompareCH1(RGB_TIM, 0);
    vTaskDelay(1);
    if(LL_DMA_IsEnabledStream(DMA2, LL_DMA_STREAM_4))
    {
        WS2812B_LOG_WARNING("WS2812B set color error: DMA is busy\n");
        return WS2812B_ERROR;
    }
    SCB_CleanDCache_by_Addr((uint32_t*)&ws2812b_buffer, sizeof(ws2812b_buffer));
    LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_4, sizeof(ws2812b_buffer)/sizeof(ws2812b_buffer[0]));
    LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_4);
    LL_DMA_EnableIT_TE(DMA2, LL_DMA_STREAM_4);
    LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_4);
    return WS2812B_OK;
}

void ws2812b_flush_complete(void)
{
    LL_DMA_DisableIT_TC(DMA2, LL_DMA_STREAM_4);
    LL_DMA_DisableIT_TE(DMA2, LL_DMA_STREAM_4);
    LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_4);
    LL_TIM_OC_SetCompareCH1(RGB_TIM, 0);
}
#ifdef cplusplus
}
#endif
