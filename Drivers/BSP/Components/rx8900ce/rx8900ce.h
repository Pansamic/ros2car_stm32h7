#ifndef __RX8900CE_H__
#define __RX8900CE_H__

#include <stdint.h>
#include <time.h>
#include "syslog.h"

// RX-8900 Basic Time and Calendar Register definitions
#define RX8900_REG_SEC					0x00
#define RX8900_REG_MIN					0x01
#define RX8900_REG_HOUR					0x02
#define RX8900_REG_WEEK					0x03
#define RX8900_REG_DAY					0x04
#define RX8900_REG_MONTH				0x05
#define RX8900_REG_YEAR					0x06
#define RX8900_REG_RAM					0x07
#define RX8900_REG_ALARM_MIN			0x08
#define RX8900_REG_ALARM_HOUR			0x09
#define RX8900_REG_ALARM_WEEK_OR_DAY	0x0A
#define RX8900_REG_TIMER_CNT_0			0x0B
#define RX8900_REG_TIMER_CNT_1			0x0C
#define RX8900_REG_EXT					0x0D
#define RX8900_REG_FLAG					0x0E
#define RX8900_REG_CTRL					0x0F
#define RX8900_REG_TEMP                 0x17
#define RX8900_REG_BACKUP               0x18

// RX-8900 Extension Register 1 definitions
#define RX8900_EXT_SEC					0x10
#define RX8900_EXT_MIN					0x11
#define RX8900_EXT_HOUR					0x12
#define RX8900_EXT_WEEK					0x13
#define RX8900_EXT_DAY					0x14
#define RX8900_EXT_MONTH				0x15
#define RX8900_EXT_YEAR					0x16
#define RX8900_EXT_TEMP					0x17
#define RX8900_EXT_BACKUP				0x18

#define RX8900_EXT_TIMER_CNT_0			0x1B
#define RX8900_EXT_TIMER_CNT_1			0x1C
#define RX8900_EXT_EXT					0x1D
#define RX8900_EXT_FLAG					0x1E
#define RX8900_EXT_CTRL					0x1F


// Flag RX8900_REG_EXT Register bit positions
#define RX8900_REG_EXT_TSEL0		(1 << 0)
#define RX8900_REG_EXT_TSEL1		(1 << 1)
#define RX8900_REG_EXT_FSEL0		(1 << 2)
#define RX8900_REG_EXT_FSEL1		(1 << 3) 
#define RX8900_REG_EXT_TE 			(1 << 4)
#define RX8900_REG_EXT_USEL			(1 << 5) 
#define RX8900_REG_EXT_WADA			(1 << 6)
#define RX8900_REG_EXT_TEST			(1 << 7)

// Flag RX8900_REG_FLAG Register bit positions
#define RX8900_REG_FLAG_VDET 		(1 << 0)
#define RX8900_REG_FLAG_VLF 		(1 << 1)

#define RX8900_REG_FLAG_AF 			(1 << 3)
#define RX8900_REG_FLAG_TF 			(1 << 4)
#define RX8900_REG_FLAG_UF 			(1 << 5)

// Flag RX8900_REG_FLAG Register bit positions
#define RX8900_REG_CTRL_RESET 		(1 << 0)


#define RX8900_REG_CTRL_AIE 		(1 << 3)
#define RX8900_REG_CTRL_TIE 		(1 << 4)
#define RX8900_REG_CTRL_UIE 		(1 << 5)
#define RX8900_REG_CTRL_CSEL0 		(1 << 6)
#define RX8900_REG_CTRL_CSEL1		(1 << 7)


#define RX8900_SLAVE_ADDR           0x32<<1

#define RX8900_LOG_CRITICAL(...)    
#define RX8900_LOG_ERROR(...)       
#define RX8900_LOG_WARN(...)     
#define RX8900_LOG_INFO(...)        
#define RX8900_LOG_DEBUG(...)
#define RX8900_LOG_TRACE(...)

void rx8900_init(void);
void rx8900_get_time(struct tm *dt);
void rx8900_set_time(struct tm *dt);
void ts2time(struct tm *time, uint32_t ts);
void ts2beijingtime(struct tm *time, uint32_t ts);
#endif // __RX8900CE_H__