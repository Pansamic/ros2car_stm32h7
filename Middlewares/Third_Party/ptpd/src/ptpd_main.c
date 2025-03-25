#include <stdlib.h>
#include <string.h>
#include "lwip/sys.h"
#include "lwip/tcpip.h"
#include "ptpd.h"
#include "syslog.h"

#if LWIP_PTPD

// Statically allocated run-time configuration data.
static bool ptpd_slave_only = true;
static PtpClock ptp_clock;
static ForeignMasterRecord ptp_foreign_records[DEFAULT_MAX_FOREIGN_RECORDS];
static sys_mbox_t ptp_alert_queue;

uint8_t network_is_up(void)
{
  // Lock the lwIP core mutex.
  LOCK_TCPIP_CORE();

  // Query if the network interface is up.
  uint8_t is_up = netif_is_up(netif_default) ? true : false;

  // Unlock the lwIP core mutex.
  UNLOCK_TCPIP_CORE();

  return is_up;
}

static void ptpd_thread(void *arg)
{
    (void)arg;
    // Initialize the main PTP datastructure.
    memset(&ptp_clock, 0, sizeof(ptp_clock));

    // Run the clock in slave only?
    ptp_clock.rtOpts.slaveOnly = ptpd_slave_only;

    // Initialize run-time options to default values.
    ptp_clock.rtOpts.announceInterval = DEFAULT_ANNOUNCE_INTERVAL;
    ptp_clock.rtOpts.syncInterval = DEFAULT_SYNC_INTERVAL;
    ptp_clock.rtOpts.clockQuality.clockAccuracy = DEFAULT_CLOCK_ACCURACY;
    ptp_clock.rtOpts.clockQuality.clockClass = DEFAULT_CLOCK_CLASS;
    ptp_clock.rtOpts.clockQuality.offsetScaledLogVariance = DEFAULT_CLOCK_VARIANCE; // 7.6.3.3
    ptp_clock.rtOpts.priority1 = DEFAULT_PRIORITY1;
    ptp_clock.rtOpts.priority2 = DEFAULT_PRIORITY2;
    ptp_clock.rtOpts.domainNumber = DEFAULT_DOMAIN_NUMBER;
    ptp_clock.rtOpts.currentUtcOffset = DEFAULT_UTC_OFFSET;
    ptp_clock.rtOpts.servo.noResetClock = DEFAULT_NO_RESET_CLOCK;
    ptp_clock.rtOpts.servo.noAdjust = NO_ADJUST;
    ptp_clock.rtOpts.inboundLatency.nanoseconds = DEFAULT_INBOUND_LATENCY;
    ptp_clock.rtOpts.outboundLatency.nanoseconds = DEFAULT_OUTBOUND_LATENCY;
    ptp_clock.rtOpts.servo.sDelay = DEFAULT_DELAY_S;
    ptp_clock.rtOpts.servo.sOffset = DEFAULT_OFFSET_S;
    ptp_clock.rtOpts.servo.ap = DEFAULT_AP;
    ptp_clock.rtOpts.servo.ai = DEFAULT_AI;
    ptp_clock.rtOpts.maxForeignRecords = sizeof(ptp_foreign_records) / sizeof(ptp_foreign_records[0]);
    ptp_clock.rtOpts.stats = PTP_TEXT_STATS;
    ptp_clock.rtOpts.delayMechanism = DEFAULT_DELAY_MECHANISM;

    // Initialize the foriegn records buffers.
    ptp_clock.foreignMasterDS.records = ptp_foreign_records;

    // See: 9.2.2
    if (ptp_clock.rtOpts.slaveOnly) ptp_clock.rtOpts.clockQuality.clockClass = DEFAULT_CLOCK_CLASS_SLAVE_ONLY;

    // No negative or zero attenuation.
    if (ptp_clock.rtOpts.servo.ap < 1) ptp_clock.rtOpts.servo.ap = 1;
    if (ptp_clock.rtOpts.servo.ai < 1) ptp_clock.rtOpts.servo.ai = 1;

    // Wait until the network interface is up.
    while (!network_is_up())
    {
        // Sleep for 500 milliseconds.
        sys_msleep(500);
    }

    // Enter state PTP_INITIALIZING.
    ptpd_protocol_to_state(&ptp_clock, PTP_INITIALIZING);

    // Loop forever.
    for (;;)
    {
        void *msg;

        // If network interface is not up, then hold everything.
        if (!network_is_up() || ip4_addr_isany_val(netif_default->ip_addr))
        {
            // Wait until the network interface comes up.
            while (!network_is_up() || ip4_addr_isany_val(netif_default->ip_addr)) sys_msleep(500);

            // Network interface is now up so reinitialize.
            ptpd_protocol_to_state(&ptp_clock, PTP_INITIALIZING);
        }
        
        // Process the current state.
        do
        {
            // ptpd_protocol_do_state() has a switch for the actions and events to be
            // checked for 'port_state'. The actions and events may or may not change
            // 'port_state' by calling ptpd_protocol_to_state(), but once they are done we loop around
            // again and perform the actions required for the new 'port_state'.
            ptpd_protocol_do_state(&ptp_clock);
        }
        while (ptpd_net_select(&ptp_clock.netPath, 0) > 0);
        
        // Wait up to 100ms for something to do, then do something anyway.
        sys_arch_mbox_fetch(&ptp_alert_queue, &msg, 100);
    }
}

// PTPD initialization.
void ptpd_init(bool slave_only)
{
    sys_thread_t t;

    // Save the slave only flag.
    ptpd_slave_only = slave_only;

    // Create the alert queue mailbox.
    if (sys_mbox_new(&ptp_alert_queue, 8) == ERR_OK)
    {
        // Create the PTP daemon thread.
        t = sys_thread_new("PTPD", ptpd_thread, NULL, DEFAULT_THREAD_STACKSIZE, osPriorityAboveNormal);

        // Validate the thread id.
        if (t == NULL)
        {
            // Log the error.
            LOG_ERROR("PTPD: cannot create ptpd thread\n");
        }
    }
    else
    {
        // Log the error.
        LOG_ERROR("PTPD: failed to create alert queue mailbox\n");
    }
}

// Notify the PTPD thread of a pending operation.
void ptpd_alert(void)
{
    // Send a message to the alert queue to wake up the PTP thread.
    sys_mbox_trypost(&ptp_alert_queue, NULL);
}

// Get the current PTPD state.
uint32_t ptpd_get_state(void)
{
    // Return the current PTPD state.
    return (uint32_t) ptp_clock.portDS.portState;
}

#endif // LWIP_PTPD
