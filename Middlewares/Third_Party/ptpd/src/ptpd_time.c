#include "ptpd.h"
#include "ptpd_port.h"
#include "ethernetif.h"

#if LWIP_PTPD

uint32_t ptpd_get_rand(uint32_t rand_max)
{
    while (!LL_RNG_IsActiveFlag_DRDY(RNG));
    return LL_RNG_ReadRandData32(RNG) % rand_max;
}

void ptpd_get_time(TimeInternal *time)
{
    ptptime_t ts;

    ethptp_get_time(&ts);
    time->seconds = ts.tv_sec;
    time->nanoseconds = ts.tv_nsec;
}

void ptpd_set_time(const TimeInternal *time)
{
    ptptime_t ts;

    DBG("resetting system clock to %d sec %d nsec\n", time->seconds, time->nanoseconds);

    ts.tv_sec = time->seconds;
    ts.tv_nsec = time->nanoseconds;
    ethptp_set_time(&ts);
}

bool ptpd_adj_freq(int32_t adj)
{
    DBGV("ptpd_adj_freq %d\n", adj);

    if (adj > ADJ_FREQ_MAX)
        adj = ADJ_FREQ_MAX;
    else if (adj < -ADJ_FREQ_MAX)
        adj = -ADJ_FREQ_MAX;

    /* Fine update method */
    ethptp_adj_freq(adj);

    return true;
}
#endif // LWIP_PTPD

