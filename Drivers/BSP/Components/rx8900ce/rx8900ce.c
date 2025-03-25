#include "main.h"
#include "rx8900ce.h"
#include "syslog.h"

static inline uint8_t bin2bcd(uint8_t bin)
{
    uint8_t bcd = 0;
    uint8_t shift = 0;

    while (bin > 0)
    {
        bcd |= (bin % 10) << shift;
        bin /= 10;
        shift += 4;
    }

    return bcd;
}

static inline uint8_t bcd2bin(uint8_t bcd)
{
    return ((bcd >> 4) * 10) + (bcd & 0x0F);
}

static int rx8900_get_week_day( uint8_t reg_week_day )
{
    int i, tm_wday = -1;
    
    for ( i=0; i < 7; i++ )
    {
        if ( reg_week_day & 1 )
        {
            tm_wday = i;
            break;
        }
        reg_week_day >>= 1;
    }
    
    return 	tm_wday;
}

static void rx8900_i2c_read_reg(uint8_t reg, uint8_t* data, uint8_t len)
{
    while(LL_I2C_IsActiveFlag_BUSY(I2C1));
    LL_I2C_HandleTransfer(I2C1, RX8900_SLAVE_ADDR, LL_I2C_ADDRSLAVE_7BIT, 1, LL_I2C_MODE_SOFTEND, LL_I2C_GENERATE_START_WRITE);
    while(!LL_I2C_IsActiveFlag_TXIS(I2C1));
    LL_I2C_TransmitData8(I2C1, reg);
    while(!LL_I2C_IsActiveFlag_TC(I2C1));
    LL_I2C_HandleTransfer(I2C1, RX8900_SLAVE_ADDR, LL_I2C_ADDRSLAVE_7BIT, len, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_RESTART_7BIT_READ);
    for(uint8_t i=0 ; i<len ; i++)
    {
        while(!LL_I2C_IsActiveFlag_RXNE(I2C1));
        data[i] = LL_I2C_ReceiveData8(I2C1);
    }
    while(!LL_I2C_IsActiveFlag_STOP(I2C1));
    LL_I2C_ClearFlag_STOP(I2C1);
}

static void rx8900_i2c_write_reg(uint8_t reg, uint8_t* data, uint8_t len)
{
    while(LL_I2C_IsActiveFlag_BUSY(I2C1));
    LL_I2C_HandleTransfer(I2C1, RX8900_SLAVE_ADDR, LL_I2C_ADDRSLAVE_7BIT, len+1, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);
    while(!LL_I2C_IsActiveFlag_TXIS(I2C1));
    LL_I2C_TransmitData8(I2C1, reg);
    for(uint8_t i=0 ; i<len ; i++)
    {
        while(!LL_I2C_IsActiveFlag_TXIS(I2C1));
        LL_I2C_TransmitData8(I2C1, data[i]);
    }
    while(!LL_I2C_IsActiveFlag_STOP(I2C1));
    LL_I2C_ClearFlag_STOP(I2C1);
}


static uint8_t is_leap_year(uint16_t year)
{
    if(((year)%4==0 && (year)%100!=0) || (year)%400==0)
        return 1;
    return 0;
}

static void get_week(struct tm *time)
{
    uint16_t YY = 0;
    uint8_t MM = 0;
    if(time->tm_mon==1 || time->tm_mon==2)
    {
        MM = time->tm_mon + 12;
        YY = time->tm_year - 1;
    }else{
        MM = time->tm_mon;
        YY = time->tm_year;
    }
    time->tm_wday = ( (time->tm_mday+2*MM+3*(MM+1)/5+YY+YY/4-YY/100+YY/400)%7 ) + 1;
}

void rx8900_init(void)
{
    uint8_t flags;
    uint8_t need_reset = 0;
    uint8_t need_clear = 0;
    uint8_t data;

    // rx8900_i2c_write_reg()

    rx8900_i2c_read_reg(RX8900_REG_FLAG, &flags, 1);

    if ( flags & RX8900_REG_FLAG_VDET )
    {
        LOG_WARN("RX8900CE:Voltage detection is detected.\n");
        RX8900_LOG_WARN("RX8900CE:Temperature compensation is stop detected.\n");
        need_clear = 1;		
    }
    
    if ( flags & RX8900_REG_FLAG_VLF )
    {
        LOG_WARN("RX8900CE:Data loss is detected. All registers must be initialized.\n");
        need_reset = 1;	
        need_clear = 1;	
    }
    
    if ( flags & RX8900_REG_FLAG_AF )
    {
        LOG_WARN("RX8900CE:Alarm was detected.\n");
        need_clear = 1;
    }
    
    if ( flags & RX8900_REG_FLAG_TF )
    {
        LOG_WARN("RX8900CE:Timer was detected.\n");
        need_clear = 1;
    }
    
    if ( flags & RX8900_REG_FLAG_UF )
    {
        LOG_WARN("RX8900CE:Update was detected.\n");
        need_clear = 1;
    }

    if (need_reset)
    {
        //clear ctrl register
        data = RX8900_REG_CTRL_CSEL0;
        rx8900_i2c_write_reg(RX8900_REG_CTRL, &data, 1);

        //set second update
        data = 0x00;
        rx8900_i2c_write_reg(RX8900_REG_EXT, &data, 1);
    }
    
    if (need_clear)
    {
        //clear flag register
        data = 0x00;
        rx8900_i2c_write_reg(RX8900_REG_FLAG, &data, 1);
    }
}

void rx8900_get_time(struct tm *dt)
{
    uint8_t date[7];

    rx8900_i2c_read_reg(RX8900_REG_SEC, date, 7);

    dt->tm_sec  = bcd2bin(date[RX8900_REG_SEC] & 0x7f);
    dt->tm_min  = bcd2bin(date[RX8900_REG_MIN] & 0x7f);
    dt->tm_hour = bcd2bin(date[RX8900_REG_HOUR] & 0x3f);
    dt->tm_wday = rx8900_get_week_day( date[RX8900_REG_WEEK] & 0x7f );
    dt->tm_mday = bcd2bin(date[RX8900_REG_DAY] & 0x3f);
    dt->tm_mon  = bcd2bin(date[RX8900_REG_MONTH] & 0x1f) - 1;
    dt->tm_year = bcd2bin(date[RX8900_REG_YEAR]);

    /* struct tm year is from 1900 */
    dt->tm_year += 100;

    LOG_DEBUG("RX8900CE: read date %ds %dm %dh %dwd %dmd %dm %dy\n",
        dt->tm_sec, dt->tm_min, dt->tm_hour, dt->tm_wday,
        dt->tm_mday, dt->tm_mon, dt->tm_year);
}

/**
 * @brief Write calendar to RX8900CE
 * 
 * @param dt date and time
 * @note RX8900CE use BCD code.
 * @note Month is 0-11, Year is 0-99.
 */
void rx8900_set_time(struct tm *dt)
{
    uint8_t date[7];
    
    date[RX8900_REG_SEC]   = bin2bcd(dt->tm_sec);
    date[RX8900_REG_MIN]   = bin2bcd(dt->tm_min);
    date[RX8900_REG_HOUR]  = bin2bcd(dt->tm_hour);		
    date[RX8900_REG_WEEK]  = 1 << (dt->tm_wday);
    date[RX8900_REG_DAY]   = bin2bcd(dt->tm_mday);
    date[RX8900_REG_MONTH] = bin2bcd(dt->tm_mon);
    date[RX8900_REG_YEAR]  = bin2bcd(dt->tm_year % 100);
    LOG_DEBUG("RX8900CE set date: %dy %dm %dd %dh %dm %ds %dwd\n",
        dt->tm_year, dt->tm_mon, dt->tm_mday, dt->tm_hour, dt->tm_min, dt->tm_sec, dt->tm_wday);
    LOG_DEBUG("RX8900CE write 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",
        date[0], date[1], date[2], date[3], date[4], date[5], date[6]);

    rx8900_i2c_write_reg(RX8900_REG_SEC, date, 7);
}

static void utc2beijing(struct tm *utc_time, struct tm *beijing_time)
{
    beijing_time->tm_year  =  utc_time->tm_year;
    beijing_time->tm_mon =  utc_time->tm_mon;
    beijing_time->tm_mday  =  utc_time->tm_mday;
    beijing_time->tm_hour   = utc_time->tm_hour;
    beijing_time->tm_min = utc_time->tm_min;
    beijing_time->tm_sec = utc_time->tm_sec;

    if(utc_time->tm_hour+8 > 23)  // Beijing time is the next day
    {
        beijing_time->tm_hour = utc_time->tm_hour + 8 - 24;  // Actual Beijing time (hours)
        // Large month
        if((utc_time->tm_mon==1)||(utc_time->tm_mon==3)||(utc_time->tm_mon==5)||(utc_time->tm_mon==7)||(utc_time->tm_mon==8)||(utc_time->tm_mon==10)||(utc_time->tm_mon==12))
        {
            if(utc_time->tm_mday==31) // If it is the last day of a large month in UTC, then Beijing time is the 1st of the next month
            {
                beijing_time->tm_mday = 1;
                beijing_time->tm_mon++;
                if(beijing_time->tm_mon > 12) // If it is the last day of December in UTC, then Beijing time is January 1st of the next year
                {
                    beijing_time->tm_mon = 1;
                    beijing_time->tm_year++;
                }
            }else{ // If it is not the last day of a large month in UTC
                beijing_time->tm_mday++;
            }
        }
        // February
        else if(utc_time->tm_mon==2)
        {
            if((is_leap_year(utc_time->tm_year)==1&&utc_time->tm_mday==28) || (utc_time->tm_mday<28))  // If it is a leap year and the 28th in UTC, then Beijing time is the 29th or less than the 28th, then add one
            {
                beijing_time->tm_mday++;
            }
            else if((is_leap_year(utc_time->tm_year)==0&&beijing_time->tm_mday==28) || (beijing_time->tm_mday==29))  // If it is a common year and the 28th or 29th in UTC, then Beijing time is March 1st
            {
                beijing_time->tm_mday = 1;
                beijing_time->tm_mon++;
            }
        }
        // Small month
        else
        {
            if(utc_time->tm_mday==30) // If it is the last day of a small month in UTC, then Beijing time is the 1st of the next month
            {
                beijing_time->tm_mday = 1;
                beijing_time->tm_mon++;
            }else{ // If it is not the last day of a small month in UTC
                beijing_time->tm_mday++;
            }
        }
    }else{ // If UTC time and Beijing time are on the same day
        beijing_time->tm_hour = utc_time->tm_hour + 8;
    }

    get_week(beijing_time);
}

/**
 * @brief Timestamp to local date and time.
 * 
 * @param time pointer to `struct tm` to store the result.
 * @param ts timestamp.
 */
void ts2time(struct tm *time, uint32_t ts)
{

    uint16_t year = 1970;
    uint32_t counter = 0, counter_temp;
    uint8_t Month[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    uint8_t i;

    while(counter <= ts)
    {
        counter_temp = counter;
        counter += 31536000;
        if(is_leap_year(year))
        {
            counter += 86400;
        }
        year++;
    }
    time->tm_year = year - 1;
    Month[1] = (is_leap_year(time->tm_year)?29:28);
    counter = ts - counter_temp;
    counter_temp = counter/86400;
    counter -= counter_temp*86400;
    time->tm_hour = counter / 3600;
    time->tm_min = counter % 3600 / 60;
    time->tm_sec = counter % 60;
    for(i=0; i<12; i++)
    {
        if(counter_temp < Month[i])
        {
            time->tm_mon = i + 1;
            time->tm_mday = counter_temp + 1;
            break;
        }
        counter_temp -= Month[i];
    }
    get_week(time);
}

/**
 * @brief UTC timestamp to Beijing time
 * 
 * @param time Pointer to `struct tm` to store the result. 
 * @param ts UTC timestamp.
 */
void ts2beijingtime(struct tm *time, uint32_t ts)
{
    struct tm utc_time;
    ts2time(&utc_time, ts);
    utc2beijing(&utc_time, time);
}