#include "ptpd_port.h"
#include "stm32h7xx_hal_eth.h"
#include "rx8900ce.h"

/* Examples of subsecond increment and addend values using HCLK = 144 MHz

 Addend * Increment = 2^63 / HCLK

 ptp_tick = Increment * 10^9 / 2^31

 +-----------+-----------+------------+
 | ptp tick  | Increment | Addend     |
 +-----------+-----------+------------+
 |  119 ns   |   255     | 0x0EF8B863 |
 |  100 ns   |   215     | 0x11C1C8D5 |
 |   50 ns   |   107     | 0x23AE0D90 |
 |   20 ns   |    43     | 0x58C8EC2B |
 |   14 ns   |    30     | 0x7F421F4F |
 +-----------+-----------+------------+
*/

/* Examples of subsecond increment and addend values using HCLK = 168 MHz

 Addend * Increment = 2^63 / HCLK

 ptp_tick = Increment * 10^9 / 2^31

 +-----------+-----------+------------+
 | ptp tick  | Increment | Addend     |
 +-----------+-----------+------------+
 |  119 ns   |   255     | 0x0CD53055 |
 |  100 ns   |   215     | 0x0F386300 |
 |   50 ns   |   107     | 0x1E953032 |
 |   20 ns   |    43     | 0x4C19EF00 |
 |   14 ns   |    30     | 0x6D141AD6 |
 +-----------+-----------+------------+
*/

/* Examples of subsecond increment and addend values using HCLK = 240 MHz

 Addend * Increment = 2^63 / HCLK

 ptp_tick = Increment * 10^9 / 2^31

 +-----------+-----------+------------+
 | ptp tick  | Increment | Addend     |
 +-----------+-----------+------------+
 |   20 ns   |    43     | 0x35455A81 |
 +-----------+-----------+------------+
*/

#define ADJ_FREQ_BASE_ADDEND      0x35455A81
#define ADJ_FREQ_BASE_INCREMENT   43

// Conversion from hardware to PTP format.
static uint32_t subsecond_to_nanosecond(uint32_t subsecond_value)
{
  uint64_t val = subsecond_value * 1000000000ll;
  val >>= 31;
  return val;
}

// Conversion from PTP to hardware format.
static uint32_t nanosecond_to_subsecond(uint32_t subsecond_value)
{
  uint64_t val = subsecond_value * 0x80000000ll;
  val /= 1000000000;
  return val;
}

void ETH_PTPTime_GetTime(struct ptptime_t * timestamp)
{
    timestamp->tv_sec = ETH->MACSTSR;
    timestamp->tv_nsec = subsecond_to_nanosecond(ETH->MACSTNR);
    if(timestamp->tv_sec != (int32_t)ETH->MACSTSR)
    {
        timestamp->tv_sec = ETH->MACSTSR;
        timestamp->tv_nsec = subsecond_to_nanosecond(ETH->MACSTNR);
    }
}

/*******************************************************************************
* Function Name  : ETH_PTPTimeStampUpdateOffset
* Description    : Updates time base offset
* Input          : Time offset with sign
* Output         : None
* Return         : None
*******************************************************************************/
void ETH_PTPTime_UpdateOffset(struct ptptime_t * timeoffset)
{
    uint32_t Sign;
    uint32_t SecondValue;
    uint32_t NanoSecondValue;
    uint32_t SubSecondValue;
    uint32_t addend;

    /* determine sign and correct Second and Nanosecond values */
    if(timeoffset->tv_sec < 0 || (timeoffset->tv_sec == 0 && timeoffset->tv_nsec < 0))
    {
        Sign = ETH_PTP_NegativeTime;
        SecondValue = -timeoffset->tv_sec;
        NanoSecondValue = -timeoffset->tv_nsec;
    }
    else
    {
        Sign = ETH_PTP_PositiveTime;
        SecondValue = timeoffset->tv_sec;
        NanoSecondValue = timeoffset->tv_nsec;
    }

    /* convert nanosecond to subseconds */
    SubSecondValue = nanosecond_to_subsecond(NanoSecondValue);

    /* read old addend register value*/
    addend = ETH->MACTSAR;

    while(ETH_GetPTPFlagStatus(ETH_PTP_FLAG_TSSTU) == SET);
    while(ETH_GetPTPFlagStatus(ETH_PTP_FLAG_TSSTI) == SET);

    /* Write the offset (positive or negative) in the Time stamp update high and low registers. */
    ETH_SetPTPTimeStampUpdate(Sign, SecondValue, SubSecondValue);

    /* Set bit 3 (TSSTU) in the Time stamp control register. */
    ETH_EnablePTPTimeStampUpdate();

    /* The value in the Time stamp update registers is added to or subtracted from the system */
    /* time when the TSSTU bit is cleared. */
    while(ETH_GetPTPFlagStatus(ETH_PTP_FLAG_TSSTU) == SET);

    /* Write back old addend register value. */
    ETH_SetPTPTimeStampAddend(addend);
    ETH_EnablePTPTimeStampAddend();
}

/*******************************************************************************
* Function Name  : ETH_PTPTimeStampSetTime
* Description    : Initialize time base
* Input          : Time with sign
* Output         : None
* Return         : None
*******************************************************************************/
void ETH_PTPTime_SetTime(struct ptptime_t * timestamp)
{
    uint32_t Sign;
    uint32_t SecondValue;
    uint32_t NanoSecondValue;
    uint32_t SubSecondValue;

    /* determine sign and correct Second and Nanosecond values */
    if(timestamp->tv_sec < 0 || (timestamp->tv_sec == 0 && timestamp->tv_nsec < 0))
    {
        Sign = ETH_PTP_NegativeTime;
        SecondValue = -timestamp->tv_sec;
        NanoSecondValue = -timestamp->tv_nsec;
    }
    else
    {
        Sign = ETH_PTP_PositiveTime;
        SecondValue = timestamp->tv_sec;
        NanoSecondValue = timestamp->tv_nsec;
    }

    /* convert nanosecond to subseconds */
    SubSecondValue = nanosecond_to_subsecond(NanoSecondValue);

    /* Write the offset (positive or negative) in the Time stamp update high and low registers. */
    ETH_SetPTPTimeStampUpdate(Sign, SecondValue, SubSecondValue);
    /* Set Time stamp control register bit 2 (Time stamp init). */
    ETH_InitializePTPTimeStamp();
    /* The Time stamp counter starts operation as soon as it is initialized
     * with the value written in the Time stamp update register. */
    while(ETH_GetPTPFlagStatus(ETH_PTP_FLAG_TSSTI) == SET);

    struct ptptime_t timestamp2;
    ETH_PTPTime_GetTime(&timestamp2);
}

/*******************************************************************************
* Function Name  : ETH_PTPTimeStampAdjFreq
* Description    : Updates time stamp addend register
* Input          : Correction value in thousandth of ppm (Adj*10^9)
* Output         : None
* Return         : None
*******************************************************************************/
void ETH_PTPTime_AdjFreq(int32_t Adj)
{
    uint32_t addend;
    
    /* calculate the rate by which you want to speed up or slow down the system time
         increments */

    /* precise */
    /*
    int64_t addend;
    addend = Adj;
    addend *= ADJ_FREQ_BASE_ADDEND;
    addend /= 1000000000-Adj;
    addend += ADJ_FREQ_BASE_ADDEND;
    */

    /* 32bit estimation
    ADJ_LIMIT = ((1l<<63)/275/ADJ_FREQ_BASE_ADDEND) = 11258181 = 11 258 ppm*/
    if( Adj > 5120000) Adj = 5120000;
    if( Adj < -5120000) Adj = -5120000;

    addend = ((((275LL * Adj)>>8) * (ADJ_FREQ_BASE_ADDEND>>24))>>6) + ADJ_FREQ_BASE_ADDEND;
    
    /* Reprogram the Time stamp addend register with new Rate value and set ETH_TPTSCR */
    ETH_SetPTPTimeStampAddend((uint32_t)addend);
    ETH_EnablePTPTimeStampAddend();
}

/*---------------------------------  PTP  ------------------------------------*/

/**
  * @brief  Updated the PTP block for fine correction with the Time Stamp Addend register value.
  * @param  None
  * @retval None
  */
void ETH_EnablePTPTimeStampAddend(void)
{
    /* Enable the PTP block update with the Time Stamp Addend register value */
    ETH->MACTSCR |= ETH_MACTSCR_TSADDREG;
}

/**
  * @brief  Updated the PTP system time with the Time Stamp Update register value.
  * @param  None
  * @retval None
  */
void ETH_EnablePTPTimeStampUpdate(void)
{
    /* Enable the PTP system time update with the Time Stamp Update register value */
    ETH->MACTSCR |= ETH_MACTSCR_TSUPDT;
}

/**
  * @brief  Initialize the PTP Time Stamp
  * @param  None
  * @retval None
  */
void ETH_InitializePTPTimeStamp(void)
{
    /* Initialize the PTP Time Stamp */
    ETH->MACTSCR |= ETH_MACTSCR_TSINIT;
}

/**
  * @brief  Selects the PTP Update method
  * @param  UpdateMethod: the PTP Update method
  *   This parameter can be one of the following values:
  *     @arg ETH_PTP_FineUpdate   : Fine Update method
  *     @arg ETH_PTP_CoarseUpdate : Coarse Update method
  * @retval None
  */
void ETH_PTPUpdateMethodConfig(uint32_t UpdateMethod)
{
    /* Check the parameters */
    assert_param(IS_ETH_PTP_UPDATE(UpdateMethod));

    if (UpdateMethod != ETH_PTP_CoarseUpdate)
    {
        /* Enable the PTP Fine Update method */
        ETH->MACTSCR |= ETH_MACTSCR_TSCFUPDT;
    }
    else
    {
        /* Disable the PTP Fine Update method */
        ETH->MACTSCR &= (~(uint32_t)ETH_MACTSCR_TSCFUPDT);
    }
}

/**
  * @brief  Enables or disables the PTP time stamp for transmit and receive frames.
  * @param  NewState: new state of the PTP time stamp for transmit and receive frames
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void ETH_PTPTimeStampCmd(FunctionalState NewState)
{
    /* Check the parameters */
    assert_param(IS_FUNCTIONAL_STATE(NewState));

    if (NewState != DISABLE)
    {
        /* Enable the PTP time stamp for transmit and receive frames */
        ETH->MACTSCR |= ETH_MACTSCR_TSENA | ETH_MACTSCR_TSENALL;
        //ETH->MACTSCR |= ETH_MACTSCR_TSENA | ETH_MACTSCR_TSIPV4ENA | ETH_MACTSCR_TSIPV6ENA | ETH_MACTSCR_TSENALL;
    }
    else
    {
        /* Disable the PTP time stamp for transmit and receive frames */
        ETH->MACTSCR &= (~(uint32_t)ETH_MACTSCR_TSENA);
    }
}

/**
  * @brief  Checks whether the specified ETHERNET PTP flag is set or not.
  * @param  ETH_PTP_FLAG: specifies the flag to check.
  *   This parameter can be one of the following values:
  *     @arg ETH_PTP_FLAG_TSARU : Addend Register Update
  *     @arg ETH_PTP_FLAG_TSITE : Time Stamp Interrupt Trigger Enable
  *     @arg ETH_PTP_FLAG_TSSTU : Time Stamp Update
  *     @arg ETH_PTP_FLAG_TSSTI  : Time Stamp Initialize
  * @retval The new state of ETHERNET PTP Flag (SET or RESET).
  */
FlagStatus ETH_GetPTPFlagStatus(uint32_t ETH_PTP_FLAG)
{
  FlagStatus bitstatus = RESET;
  /* Check the parameters */
  assert_param(IS_ETH_PTP_GET_FLAG(ETH_PTP_FLAG));

  if ((ETH->MACTSCR & ETH_PTP_FLAG) != (uint32_t)RESET)
  {
    bitstatus = SET;
  }
  else
  {
    bitstatus = RESET;
  }
  return bitstatus;
}

/**
  * @brief  Sets the system time Sub-Second Increment value.
  * @param  SubSecondValue: specifies the PTP Sub-Second Increment Register value.
  * @retval None
  */
void ETH_SetPTPSubSecondIncrement(uint32_t SubSecondValue)
{
    /* Check the parameters */
    assert_param(IS_ETH_PTP_SUBSECOND_INCREMENT(SubSecondValue));
    /* Set the PTP Sub-Second Increment Register */
    ETH->MACSSIR = (SubSecondValue<<16);
}

/**
  * @brief  Sets the Time Stamp update sign and values.
  * @param  Sign: specifies the PTP Time update value sign.
  *   This parameter can be one of the following values:
  *     @arg ETH_PTP_PositiveTime : positive time value.
  *     @arg ETH_PTP_NegativeTime : negative time value.
  * @param  SecondValue: specifies the PTP Time update second value.
  * @param  SubSecondValue: specifies the PTP Time update sub-second value.
  *   This parameter is a 31 bit value, bit32 correspond to the sign.
  * @retval None
  */
void ETH_SetPTPTimeStampUpdate(uint32_t Sign, uint32_t SecondValue, uint32_t SubSecondValue)
{
    /* Check the parameters */
    assert_param(IS_ETH_PTP_TIME_SIGN(Sign));
    assert_param(IS_ETH_PTP_TIME_STAMP_UPDATE_SUBSECOND(SubSecondValue));

    /* Set the PTP Time Update High Register */
    ETH->MACSTSUR = SecondValue;

    /* Set the PTP Time Update Low Register with sign */
    ETH->MACSTNUR = Sign | SubSecondValue;
}

/**
  * @brief  Sets the Time Stamp Addend value.
  * @param  Value: specifies the PTP Time Stamp Addend Register value.
  * @retval None
  */
void ETH_SetPTPTimeStampAddend(uint32_t Value)
{
    /* Set the PTP Time Stamp Addend Register */
    ETH->MACTSAR = Value;
}

/**
  * @brief  Sets the Target Time registers values.
  * @param  HighValue: specifies the PTP Target Time High Register value.
  * @param  LowValue: specifies the PTP Target Time Low Register value.
  * @retval None
  */
void ETH_SetPTPTargetTime(uint32_t HighValue, uint32_t LowValue)
{
    /* Set the PTP Target Time High Register */
    ETH->MACSTSR = HighValue;
    /* Set the PTP Target Time Low Register */
    ETH->MACSTNR = LowValue;
}

/**
  * @brief  Sets the frequency of the PPS output.
  * @param  Freq: specifies the frequency of the PPS output in Hz as 2^Freq.
  * @retval None
  */
void ETH_PTPSetPPSFreq(uint8_t Freq)
{
    /* Check the parameters */
    assert_param(IS_PPS_FREQ(Freq));

    ETH->MACPPSCR = Freq;
}

// Update method is ETH_PTP_FineUpdate or ETH_PTP_CoarseUpdate.
void ethptp_start(uint32_t update_method)
{
  // Program Time stamp register bit 0 to enable time stamping.
  ETH_PTPTimeStampCmd(ENABLE);

  // Program the Subsecond increment register based on the PTP clock frequency.
  ETH_SetPTPSubSecondIncrement(ADJ_FREQ_BASE_INCREMENT); // to achieve 20 ns accuracy, the value is ~ 43

  if (update_method == ETH_PTP_FineUpdate)
  {
    // If you are using the Fine correction method, program the Time stamp addend register
    // and set Time stamp control register bit 5 (addend register update).
    ETH_SetPTPTimeStampAddend(ADJ_FREQ_BASE_ADDEND);
    ETH_EnablePTPTimeStampAddend();

    // Poll the Time stamp control register until bit 5 is cleared.
    while(ETH_GetPTPFlagStatus(ETH_PTP_FLAG_TSARU) == SET);
  }

  // To select the Fine correction method (if required),
  // program Time stamp control register  bit 1.
  ETH_PTPUpdateMethodConfig(update_method);

  // Get the current time from external RTC
  struct tm timestamp;
  rx8900_get_time(&timestamp);
  LOG_INFO("Initialize PTP time: %4d-%02d-%02d %02d:%02d:%02d\n", 2000+timestamp.tm_year%100, timestamp.tm_mon+1, timestamp.tm_mday, timestamp.tm_hour, timestamp.tm_min, timestamp.tm_sec);

  // Program the Time stamp high update and Time stamp low update registers
  // with the appropriate time value.
  ETH_SetPTPTimeStampUpdate(ETH_PTP_PositiveTime, mktime(&timestamp), 0);

  // Set Time stamp control register bit 2 (Time stamp init).
  ETH_InitializePTPTimeStamp();

  // The enhanced descriptor format is enabled and the descriptor size is
  // increased to 32 bytes (8 DWORDS). This is required when time stamping
  // is activated above.
  // XXX I believe that by default the enhanced descriptor size is always
  // XXX used in the HAL Ethernet driver. This assumpton will need to be
  // XXX verified. Otherwise HAL_ETH_ConfigDMA() may need to be called. */
  // ETH_EnhancedDescriptorCmd(ENABLE);

  // The Time stamp counter starts operation as soon as it is initialized
  // with the value written in the Time stamp update register.
}

// Get the PTP time.
void ethptp_get_time(ptptime_t *timestamp)
{
  int32_t hi_reg;
  int32_t lo_reg;
  int32_t hi_reg_after;

  // The problem is we are reading two 32-bit registers that form
  // a 64-bit value, but it's possible the high 32-bits of the value
  // rolls over before we read the low 32-bits of the value.  To avoid
  // this situation we read the high 32-bits twice and determine which
  // high 32-bits the low 32-bit are associated with.
  __disable_irq();
  hi_reg = ETH->MACSTSR;
  lo_reg = ETH->MACSTNR;
  hi_reg_after = ETH->MACSTSR;
  __enable_irq();

  // Did a roll over occur while reading?
  if (hi_reg != hi_reg_after)
  {
    // We now know a roll over occurred. If the rollover occured before
    // the reading of the low 32-bits we move the substitute the second
    // 32-bit high value for the first 32-bit high value.
    if (lo_reg < (INT_MAX / 2)) hi_reg = hi_reg_after;
  }

  // Now convert the raw registers values into timestamp values.
  timestamp->tv_nsec = subsecond_to_nanosecond(lo_reg);
  timestamp->tv_sec = hi_reg;
}

// Set the PTP time.
void ethptp_set_time(ptptime_t *timestamp)
{
  uint32_t sign;
  uint32_t second_value;
  uint32_t nanosecond_value;
  uint32_t subsecond_value;

  // Determine sign and correct second and nanosecond values.
  if (timestamp->tv_sec < 0 || (timestamp->tv_sec == 0 && timestamp->tv_nsec < 0))
  {
    sign = ETH_PTP_NegativeTime;
    second_value = -timestamp->tv_sec;
    nanosecond_value = -timestamp->tv_nsec;
  }
  else
  {
    sign = ETH_PTP_PositiveTime;
    second_value = timestamp->tv_sec;
    nanosecond_value = timestamp->tv_nsec;
  }

  // Convert nanosecond to subseconds.
  subsecond_value = nanosecond_to_subsecond(nanosecond_value);

  // Write the offset (positive or negative) in the Time stamp update
  // high and low registers.
  ETH_SetPTPTimeStampUpdate(sign, second_value, subsecond_value);

  // Set Time stamp control register bit 2 (Time stamp init).
  ETH_InitializePTPTimeStamp();

  // The Time stamp counter starts operation as soon as it is initialized
  // with the value written in the Time stamp update register.
  while (ETH_GetPTPFlagStatus(ETH_PTP_FLAG_TSSTI) == SET);
}

// Adjust the PTP system clock rate by the specified value in parts-per-billion.
void ethptp_adj_freq(int32_t adj_ppb)
{
  // Adjust the fixed base frequency by parts-per-billion.
  // addend = base + ((base * adj_ppb) / 1000000000);
  uint32_t addend = ADJ_FREQ_BASE_ADDEND + (int32_t) ((((int64_t) ADJ_FREQ_BASE_ADDEND) * adj_ppb) / 1000000000);

  // Set the time stamp addend register with new rate value and set ETH_TPTSCR.
  ETH_SetPTPTimeStampAddend(addend);
  ETH_EnablePTPTimeStampAddend();
}
