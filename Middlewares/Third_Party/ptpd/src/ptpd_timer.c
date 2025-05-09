#include "FreeRTOS.h"
#include "timers.h"
#include "ptpd.h"

#if LWIP_PTPD

// Static array of PTPD timers.
static TimerHandle_t ptpd_timer_id[TIMER_ARRAY_SIZE];
static bool ptpd_timers_expired[TIMER_ARRAY_SIZE];

// Static timer control block array.
static uint32_t ptpd_timer_cb[TIMER_ARRAY_SIZE][256];

// Callback for timers.
static void ptpd_timer_callback(TimerHandle_t timer)
{
    int index = (int)pvTimerGetTimerID(timer);

    // Sanity check the index.
    if (index < TIMER_ARRAY_SIZE)
    {
        // Mark the indicated timer as expired.
        ptpd_timers_expired[index] = true;

        // Notify the PTP thread of a pending operation.
        ptpd_alert();
    }
}

// Initialize PTPD timers.
void ptpd_timer_init(void)
{
    int32_t i;

    // Create the various timers used in the system.
    for (i = 0; i < TIMER_ARRAY_SIZE; i++)
    {
        // Initialize the timer attributes.

        // Mark the timer as not expired.
        ptpd_timers_expired[i] = false;

        // Create the timer.
        ptpd_timer_id[i] = xTimerCreateStatic("ptpd", pdMS_TO_TICKS(1), pdTRUE, (void *) i, ptpd_timer_callback, (StaticTimer_t*)&ptpd_timer_cb[i]);
    }
}

// Start the indexed timer with the given interval.
void ptpd_timer_start(int32_t index, uint32_t interval_ms)
{
    // Sanity check the index.
    if (index >= TIMER_ARRAY_SIZE) return;
    if (ptpd_timer_id[index] == 0) return;

    DBGV("PTPD: set timer %d to %u\n", index, interval_ms);

    // Reset the timer expired flag.
    ptpd_timers_expired[index] = false;

    // Start the timer with the specified duration.
    osTimerStart(ptpd_timer_id[index], pdMS_TO_TICKS(interval_ms));
}

// Stop the indexed timer.
void ptpd_timer_stop(int32_t index)
{
    // Sanity check the index.
    if (index >= TIMER_ARRAY_SIZE) return;

    DBGV("PTPD: stop timer %d\n", index);

    // Stop the timer.
    osTimerStop(ptpd_timer_id[index]);

    // Reset the expired flag.
    ptpd_timers_expired[index] = false;
}

// If the timer has expired, this function will reset the
// expired flag and return true, otherwise it will false.
bool ptpd_timer_expired(int32_t index)
{
    // Sanity check the index.
    if (index >= TIMER_ARRAY_SIZE) return false;

    DBGV("PTPD: timer %d %s\n", index,
                ptpd_timers_expired[index] ? "expired" : "not expired");

    // Return false if the timer hasn't expired.
    if (!ptpd_timers_expired[index]) return false;

    // We only return the timer expired once.
    ptpd_timers_expired[index] = false;

    // Return true since the timer expired.
    return true;
}

#endif // LWIP_PTPD
