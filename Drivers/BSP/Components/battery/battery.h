/**
 * @file battery.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2025-03-20
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef __BATTERY_H__
#define __BATTERY_H__

#include "syslog.h"

#define BATTERY_SCALE_FACTOR 10.0

#define BATTERY_LOG_CRITICAL(...)   LOG_CRITICAL(__VA_ARGS__)
#define BATTERY_LOG_ERROR(...)      LOG_ERROR(__VA_ARGS__)
#define BATTERY_LOG_WARN(...)       LOG_WARN(__VA_ARGS__)
#define BATTERY_LOG_INFO(...)       LOG_INFO(__VA_ARGS__)
#define BATTERY_LOG_DEBUG(...)      LOG_DEBUG(__VA_ARGS__)
#define BATTERY_LOG_TRACE(...)      LOG_TRACE(__VA_ARGS__)

typedef enum {
    BATTERY_OK = 0,
    BATTERY_ERROR = 1
} battery_err_t;

battery_err_t battery_init(void);
float get_battery_voltage();

#endif // __BATTERY_H__