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

typedef enum {
    DRV8874_MODE_POSITION = 0,
    DRV8874_MODE_VELOCITY = 1,
    DRV8874_MODE_TORQUE = 2,
    DRV8874_MODE_TORQUE_POSITION = 3,
    DRV8874_MODE_TORQUE_VELOCITY = 4
}drv8874_mode_t;

typedef struct {
    uint8_t id;

    /* @ref drv8874_mode_t */
    drv8874_mode_t mode;

    /* reverse=1 set motor rotates in reverse direction */
    uint8_t reverse;

    /* DRV8874 PH pin. 1 is forward, 0 is reverse */
    GPIO_TypeDef* ctrl_gpio_port;
    uint32_t ctrl_gpio_pin;

    /* Timer to generate PWM wave */
    void* pwm_timer;
    /* PWM channel generate PWM wave for this motor */
    uint32_t pwm_channel;
    /* PWM timer auto reload register value. Calculate feed forward with this parameter and `max_velocity`. */
    uint32_t pwm_tim_autoreload;

    /* Timer to keep encoder pulses */
    void* encoder_timer;

    int32_t encoder_count;
    int32_t previous_encoder_count;
    uint32_t encoder_round_count;

    /* Basic timer to calculate the velocity of the motor with time between controller updates. */
    void* interval_timer;
    uint32_t interval_timer_frequency;
    int32_t interval_timer_count;

    /* Motor torque coefficient */
    float torque_coefficient;

    /* Motor turndown ratio */
    uint16_t turndown_ratio;

    /* A proximate value of max velocity of motor, unit: rad/s.
     * Calculate feed forward with this parameter and `pwm_tim_autoreload`. */
    float max_velocity;

    /* Position PD control parameters */
    float kp_pos;
    float kd_pos;
    float prev_error_pos;
    float error_pos;
    float control_pos;

    /* Velocity PD control parameters */
    float kp_vel;
    float kd_vel;
    float prev_error_vel;
    float error_vel;
    float control_vel;

    /* Torque PD control parameters */
    float kp_torq;
    float kd_torq;
    float prev_error_torq;
    float error_torq;
    float control_torq;


    /* Target angular velocity, unit: rad/s */
    float target_velocity;
    /* Current angular velocity, unit: rad/s */
    float current_velocity;
    /* Target angular position, unit: rad */
    float target_position;
    /* Current angular position, unit:rad */
    float current_position;
    /* Target torque, unit: Nm */
    float target_torque;
    /* Current torque, unit: Nm */
    float current_torque;

} drv8874_t;

extern drv8874_t motors[4];

/**
 * @brief Initialize 4 DRV8874 motor driver instances and create control task.
 * 
 * @return drv8874_err_t 
 */
drv8874_err_t drv8874_init();
/**
 * @brief Set motor to position control mode.
 * 
 * @param dev Pointer to DRV8874 driver instance.
 * @param kp Kp value of PD position controller.
 * @param kd Kd value of PD position controller.
 * @return drv8874_err_t 
 */
drv8874_err_t drv8874_set_position_control(drv8874_t *dev, float kp, float kd);
drv8874_err_t drv8874_set_velecity_control(drv8874_t *dev, float kp, float kd);
drv8874_err_t drv8874_set_torque_control(drv8874_t *dev, float kp, float kd);
drv8874_err_t drv8874_set_torque_position_control(drv8874_t *dev, float kp, float kd);
drv8874_err_t drv8874_set_torque_velocity_control(drv8874_t *dev, float kp, float kd);
drv8874_err_t drv8874_set_position(drv8874_t *dev, float position);
drv8874_err_t drv8874_set_velocity(drv8874_t *dev, float velocity);
drv8874_err_t drv8874_set_torque(drv8874_t *dev, float torque);
drv8874_err_t drv8874_set_direction(drv8874_t *dev, drv8874_direction_t direction);
drv8874_err_t drv8874_start(drv8874_t *dev);
drv8874_err_t drv8874_brake(drv8874_t *dev);
drv8874_err_t drv8874_stop(drv8874_t *dev);
drv8874_err_t drv8874_update_pos_control(drv8874_t *dev);
drv8874_err_t drv8874_update_vel_control(drv8874_t *dev);
drv8874_err_t drv8874_update_torque_control(drv8874_t *dev);
drv8874_err_t drv8874_update_torq_pos_control(drv8874_t *dev);
drv8874_err_t drv8874_update_torq_vel_control(drv8874_t *dev);
drv8874_err_t drv8874_update_control(drv8874_t *dev);
#endif // __DRV8874_H__
