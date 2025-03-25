/**
 * @file icm42688p.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2025-03-20
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef __ICM42688P_H__
#define __ICM42688P_H__

#include "syslog.h"

#define ICM42688P_LOG_CRITICAL(...)  LOG_CRITICAL(__VA_ARGS__)
#define ICM42688P_LOG_ERROR(...)     LOG_ERROR(__VA_ARGS__)
#define ICM42688P_LOG_WARNING(...)   LOG_WARN(__VA_ARGS__)
#define ICM42688P_LOG_INFO(...)      LOG_INFO(__VA_ARGS__)
#define ICM42688P_LOG_DEBUG(...)     LOG_DEBUG(__VA_ARGS__)
#define ICM42688P_LOG_TRACE(...)     LOG_TRACE(__VA_ARGS__)

typedef enum
{
    ICM42688P_OK = 0,
    ICM42688P_ERROR = 1,
    ICM42688P_TIMEOUT = 2
} icm42688p_err_t;

typedef struct icm42688p
{
    double temperature;
    double acceleration_x;
    double acceleration_y;
    double acceleration_z;
    double angular_velocity_x;
    double angular_velocity_y;
    double angular_velocity_z;
}icm42688p_t;

/**
 * @brief Initialize ICM-42688-P sensor.
 * 
 * @return icm42688p_err_t 
 */
icm42688p_err_t icm42688p_init(void);

/**
 * @brief Read acceleration and angular velocity data from ICM-42688-P sensor.
 * 
 * @param data Pointer to icm42688p_t structure.
 * @return icm42688p_err_t 
 */
icm42688p_err_t icm42688p_read(icm42688p_t *data);

#endif // __ICM42688P_H__