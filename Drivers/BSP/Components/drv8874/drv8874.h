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
#define DRV8874_LOG_WARN(...)      LOG_WARN(__VA_ARGS__)
#define DRV8874_LOG_INFO(...)      LOG_INFO(__VA_ARGS__)
#define DRV8874_LOG_DEBUG(...)     LOG_DEBUG(__VA_ARGS__)
#define DRV8874_LOG_TRACE(...)     LOG_TRACE(__VA_ARGS__)

#define DRV8874_MOTOR_AMOUNT 4

/* User can change the float precision by changing the following line. */
typedef double drv8874_float_t;

typedef enum drv8874_err_t{
    DRV8874_OK = 0,
    DRV8874_ERROR = 1
} drv8874_err_t;

typedef enum drv8874_direction_t{
    DRV8874_DIRECTION_FORWARD = 0,
    DRV8874_DIRECTION_BACKWARD = 1
} drv8874_direction_t;

typedef enum drv8874_mode_t{
    DRV8874_MODE_POSITION = 0,
    DRV8874_MODE_VELOCITY = 1,
    DRV8874_MODE_CURRENT = 2
}drv8874_mode_t;

typedef struct drv8874_t{
    /* ID for this motor, start from 1. */
    uint8_t id;

    /* Flag to indicate whether this motor is enabled or not */
    uint8_t enable;

    /* @ref drv8874_mode_t */
    drv8874_mode_t mode;

    /* reverse=1 set motor rotates in reverse direction */
    uint8_t reverse;

    /* 1 indicates reverse encoder value */
    uint8_t reverse_encoder;

    /* Motor turndown ratio */
    uint16_t turndown_ratio;

    /* Number of pulses when the encoder rotates one round, 
     * not the number of pulses when the shaft rotates one round. */
    uint32_t encoder_round_count;

    /* A proximate value of max velocity of motor, unit: rad/s.
     * Calculate feed forward with this parameter and `pwm_tim_autoreload`. */
    // drv8874_float_t max_velocity;

    /* back electromotive force constant of rotor, not motor reducer output shaft. unit: V/(rad/s) */
    drv8874_float_t back_emf_constant;

    /* armature resistance, unit:ohm */
    drv8874_float_t armature_res;

    /* External current sensing resistor value, unit: ohm */
    uint32_t cur_sense_res;

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

    /* Time of previous control updating */
    drv8874_float_t last_update_time;

    /* Position PD control parameters */
    drv8874_float_t kp_pos;
    drv8874_float_t ki_pos;
    drv8874_float_t kd_pos;
    drv8874_float_t integral_error_pos;
    drv8874_float_t prev_error_pos;
    drv8874_float_t error_pos;
    drv8874_float_t control_pos;

    /* Velocity PI control parameters */
    drv8874_float_t kp_vel;
    drv8874_float_t ki_vel;
    drv8874_float_t kd_vel;
    drv8874_float_t integral_error_vel;
    drv8874_float_t prev_error_vel;
    drv8874_float_t error_vel;
    drv8874_float_t control_vel;

    /* Current PI control parameters */
    drv8874_float_t kp_cur;
    drv8874_float_t ki_cur;
    drv8874_float_t kd_cur;
    drv8874_float_t integral_error_cur;
    drv8874_float_t prev_error_cur;
    drv8874_float_t error_cur;
    drv8874_float_t control_cur;


    /* Target angular velocity, unit: rad/s */
    drv8874_float_t target_velocity;
    /* Current angular velocity, unit: rad/s */
    drv8874_float_t current_velocity;
    /* Target angular position, unit: rad */
    drv8874_float_t target_position;
    /* Current angular position, unit:rad */
    drv8874_float_t current_position;
    /* Target current, unit: A */
    drv8874_float_t target_current;
    /* Current current, unit: A */
    drv8874_float_t current_current;

} drv8874_t;

extern drv8874_t motors[4];
extern volatile uint32_t drv8874_timer_count;

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
 * @return drv8874_err_t 
 */
drv8874_err_t drv8874_set_position_mode(drv8874_t *dev);
/**
 * @brief Set motor to velocity control mode.
 * 
 * @param dev Pointer to DRV8874 driver instance.
 * @return drv8874_err_t 
 */
drv8874_err_t drv8874_set_velocity_mode(drv8874_t *dev);
/**
 * @brief Set motor to current control mode.
 * 
 * @param dev Pointer to DRV8874 driver instance.
 * @return drv8874_err_t 
 */
drv8874_err_t drv8874_set_current_mode(drv8874_t *dev);
/**
 * @brief Set target position for the motor.
 * 
 * @param dev Pointer to DRV8874 driver instance.
 * @param position Target angular position in radians.
 * @return drv8874_err_t 
 */
drv8874_err_t drv8874_set_position(drv8874_t *dev, drv8874_float_t position);
/**
 * @brief Set target velocity for the motor.
 * 
 * @param dev Pointer to DRV8874 driver instance.
 * @param velocity Target angular velocity in radians per second.
 * @return drv8874_err_t 
 */
drv8874_err_t drv8874_set_velocity(drv8874_t *dev, drv8874_float_t velocity);
/**
 * @brief Set target current for the motor.
 * 
 * @param dev Pointer to DRV8874 driver instance.
 * @param current Target current in Newton-meters.
 * @return drv8874_err_t 
 */
drv8874_err_t drv8874_set_current(drv8874_t *dev, drv8874_float_t current);
/**
 * @brief Set the direction of motor rotation.
 * 
 * @param dev Pointer to DRV8874 driver instance.
 * @param direction Desired direction of rotation (forward or backward).
 *     @ref DRV8874_DIRECTION_FORWARD
 *     @ref DRV8874_DIRECTION_BACKWARD
 * @return drv8874_err_t 
 */
drv8874_err_t drv8874_set_direction(drv8874_t *dev, drv8874_direction_t direction);
/**
 * @brief Start the motor by enabling its control logic and PWM output.
 * @note This function will reset encoder count, current position, velocity, and current to zero.
 * @param dev Pointer to DRV8874 driver instance.
 * @return drv8874_err_t 
 */
drv8874_err_t drv8874_start(drv8874_t *dev);
/**
 * @brief Apply braking to the motor by setting PWM output to zero.
 * 
 * @param dev Pointer to DRV8874 driver instance.
 * @return drv8874_err_t 
 */
drv8874_err_t drv8874_brake(drv8874_t *dev);
/**
 * @brief Stop the motor by disabling its PWM output.
 * 
 * @param dev Pointer to DRV8874 driver instance.
 * @return drv8874_err_t 
 */
drv8874_err_t drv8874_stop(drv8874_t *dev);
drv8874_err_t drv8874_set_current_ctrl_param(drv8874_t *dev, drv8874_float_t kp, drv8874_float_t ki);
drv8874_err_t drv8874_set_velocity_ctrl_param(drv8874_t *dev, drv8874_float_t kp, drv8874_float_t ki);
drv8874_err_t drv8874_set_position_ctrl_param(drv8874_t *dev, drv8874_float_t kp, drv8874_float_t kd);
#endif // __DRV8874_H__
