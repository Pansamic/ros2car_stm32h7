/**
 * @file drv8874.h
 * @author Pansamic (pansamic@foxmail.com)
 * @brief Driver code for Texas Instrument DRV8874 DC motor driver.
 * @version 0.1
 * @date 2025-03-19
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef __DRV8874_H__
#define __DRV8874_H__

#include "main.h"
#include "syslog.h"

#define DRV8874_LOG_CRITICAL(...)  LOG_CRITICAL(__VA_ARGS__)
#define DRV8874_LOG_ERROR(...)     LOG_ERROR(__VA_ARGS__)
#define DRV8874_LOG_WARNING(...)   LOG_WARN(__VA_ARGS__)
#define DRV8874_LOG_INFO(...)      LOG_INFO(__VA_ARGS__)
#define DRV8874_LOG_DEBUG(...)     LOG_DEBUG(__VA_ARGS__)
#define DRV8874_LOG_TRACE(...)     LOG_TRACE(__VA_ARGS__)

typedef enum {
    DRV8874_OK = 0,
    DRV8874_ERROR = 1
} drv8874_err_t;

typedef enum {
    DRV8874_DIRECTION_FORWARD = 0,
    DRV8874_DIRECTION_BACKWARD = 1
} drv8874_direction_t;

typedef struct {
    uint8_t reverse;
    GPIO_TypeDef* ctrl_gpio_port;
    uint32_t ctrl_gpio_pin;
    TIM_TypeDef* pwm_timer;
    uint32_t channel;
    TIM_TypeDef* encoder_timer;
    uint32_t pwm_peroid_count;
    uint16_t turndown_ratio;
    float max_velocity;
    float kp;
    float kd;
    float previous_error;
    float error;
    float control;
    volatile float target_velocity;
    volatile float current_velocity;
    volatile float target_position;
    volatile float current_position;
    volatile int32_t encoder_count;
    int32_t previous_encoder_count;
    uint32_t encoder_round_count;
    uint32_t timer_frequency;
    int32_t timer_count;
} drv8874_t;

extern drv8874_t motors[4];

/**
 * @brief Initialize 4 DRV8874 motor driver instances and create control task.
 * 
 * @param dev 
 * @return drv8874_err_t 
 */
drv8874_err_t drv8874_init(drv8874_t *dev);
drv8874_err_t drv8874_set_position_control(drv8874_t *dev, float kp, float kd);
drv8874_err_t drv8874_set_velecity_control(drv8874_t *dev, float kp, float kd);
drv8874_err_t drv8874_set_torque_control(drv8874_t *dev, float kp, float kd);
drv8874_err_t drv9974_set_position(drv8874_t *dev, float position);
drv8874_err_t drv8874_set_velocity(drv8874_t *dev, float velocity);
drv8874_err_t drv8874_set_torque(drv8874_t *dev, float torque);
drv8874_err_t drv8874_set_direction(drv8874_t *dev, drv8874_direction_t direction);
drv8874_err_t drv8874_start(drv8874_t *dev);
drv8874_err_t drv8874_brake(drv8874_t *dev);
drv8874_err_t drv8874_stop(drv8874_t *dev);

#endif // __DRV8874_H__
