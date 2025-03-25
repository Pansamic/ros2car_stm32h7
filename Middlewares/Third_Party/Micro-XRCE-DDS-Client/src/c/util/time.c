#include <stdint.h>
#include <uxr/client/util/time.h>
#include <uxr/client/config.h>
#include <time.h>

#include "stm32h743xx.h"

//==================================================================
//                             PUBLIC
//==================================================================
int64_t uxr_millis(void)
{
    int64_t second = (int64_t)ETH->MACSTSR;
    int64_t nano = (int64_t)ETH->MACSTNR;
    second *= 1000;
    nano *= 999999999ULL;
    nano /= 0x7FFFFFFFULL;
    nano /= 1000000;
    return second + nano;
}

int64_t uxr_nanos(void)
{
    int64_t second = (int64_t)ETH->MACSTSR;
    int64_t nano = (int64_t)ETH->MACSTNR;
    second *= 1000000000;
    nano *= 999999999ULL;
    nano /= 0x7FFFFFFFULL;
    return second + nano;
}
