#ifndef PTPD_DEP_H_
#define PTPD_DEP_H_

#include "ptpd_datatypes.h"

#define flip16(x) htons(x)
#define flip32(x) htonl(x)

typedef struct ptptime_t {
    int32_t tv_sec;
    int32_t tv_nsec;
}ptptime_t;

/**--------------------------------------------------------------------------**/
/**
  * @brief                           Ethernet PTP defines
  */
/**--------------------------------------------------------------------------**/
/**
  * @}
  */

/** @defgroup ETH_PTP_time_update_method
  * @{
  */
#define ETH_PTP_FineUpdate        ((uint32_t)0x00000001)  /*!< Fine Update method */
#define ETH_PTP_CoarseUpdate      ((uint32_t)0x00000000)  /*!< Coarse Update method */
#define IS_ETH_PTP_UPDATE(UPDATE) (((UPDATE) == ETH_PTP_FineUpdate) || \
                                   ((UPDATE) == ETH_PTP_CoarseUpdate))

/**
  * @}
  */


/** @defgroup ETH_PTP_Flags
  * @{
  */
#define ETH_PTP_FLAG_TSARU        ((uint32_t)0x00000020)  /*!< Addend Register Update */
#define ETH_PTP_FLAG_TSSTU        ((uint32_t)0x00000008)  /*!< Time Stamp Update */
#define ETH_PTP_FLAG_TSSTI        ((uint32_t)0x00000004)  /*!< Time Stamp Initialize */

#define ETH_PTP_FLAG_TSTTR        ((uint32_t)0x10000002)  /* Time stamp target time reached */
#define ETH_PTP_FLAG_TSSO         ((uint32_t)0x10000001)  /* Time stamp seconds overflow */

#define IS_ETH_PTP_GET_FLAG(FLAG) (((FLAG) == ETH_PTP_FLAG_TSARU) || \
                                    ((FLAG) == ETH_PTP_FLAG_TSSTU) || \
                                    ((FLAG) == ETH_PTP_FLAG_TSSTI) || \
                                    ((FLAG) == ETH_PTP_FLAG_TSTTR) || \
                                    ((FLAG) == ETH_PTP_FLAG_TSSO))

/**
  * @brief  ETH PTP subsecond increment
  */
#define IS_ETH_PTP_SUBSECOND_INCREMENT(SUBSECOND) ((SUBSECOND) <= 0xFF)

/** @defgroup ETH_PTP_time_sign
  * @{
  */
#define ETH_PTP_PositiveTime      ((uint32_t)0x00000000)  /*!< Positive time value */
#define ETH_PTP_NegativeTime      ((uint32_t)0x80000000)  /*!< Negative time value */
#define IS_ETH_PTP_TIME_SIGN(SIGN) (((SIGN) == ETH_PTP_PositiveTime) || \
                                    ((SIGN) == ETH_PTP_NegativeTime))

/**
  * @brief  ETH PTP time stamp low update
  */
#define IS_ETH_PTP_TIME_STAMP_UPDATE_SUBSECOND(SUBSECOND) ((SUBSECOND) <= 0x7FFFFFFF)

/**
  * @brief  ETH PTP PPS frequency check
  */
#define IS_PPS_FREQ(FREQUENCY) ((FREQUENCY) <= 0x0F)

#define getFlag(flagField, mask) (bool)(((flagField)  & (mask)) == (mask))
#define setFlag(flagField, mask) (flagField) |= (mask)
#define clearFlag(flagField, mask) (flagField) &= ~(mask)

void ETH_PTPTime_SetTime(struct ptptime_t * timestamp);
void ETH_PTPTime_GetTime(struct ptptime_t * timestamp);
void ETH_PTPTime_UpdateOffset(struct ptptime_t * timeoffset);
void ETH_PTPTime_AdjFreq(int32_t Adj);
void ETH_EnablePTPTimeStampAddend(void);
void ETH_EnablePTPTimeStampInterruptTrigger(void);
void ETH_EnablePTPTimeStampUpdate(void);
void ETH_InitializePTPTimeStamp(void);
void ETH_PTPSetPPSFreq(uint8_t Freq);
void ETH_PTPUpdateMethodConfig(uint32_t UpdateMethod);
void ETH_PTPTimeStampCmd(FunctionalState NewState);
FlagStatus ETH_GetPTPFlagStatus(uint32_t ETH_PTP_FLAG);
void ETH_SetPTPSubSecondIncrement(uint32_t SubSecondValue);
void ETH_SetPTPTimeStampUpdate(uint32_t Sign, uint32_t SecondValue, uint32_t SubSecondValue);
void ETH_SetPTPTimeStampAddend(uint32_t Value);
void ETH_SetPTPTargetTime(uint32_t HighValue, uint32_t LowValue);

void ethptp_start(uint32_t update_method);
void ethptp_get_time(ptptime_t *timestamp);
void ethptp_set_time(ptptime_t *timestamp);
void ethptp_adj_freq(int32_t adj_ppb);


#endif /* PTPD_DEP_H_*/
