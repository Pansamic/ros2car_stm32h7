/**
 * @file drv8874.c
 * @author Pansamic (pansamic@foxmail.com)
 * @brief Driver code for Texas Instrument DRV8874 DC motor driver.
 * @version 0.1
 * @date 2025-03-19
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "FreeRTOS_CLI.h"
#include "lwprintf.h"
#include "drv8874.h"

#ifdef cplusplus
extern "C" {
#endif

/* DRV8874 control block instance */
drv8874_t motors[4];

/* FreeRTOS timer handle for motor control loop timer */
static xTimerHandle motor_control_timer;


static void drv8874_update_timer(TimerHandle_t xTimer);
static drv8874_err_t drv8874_init_adc(void);
static portBASE_TYPE drv8874_control_command(int8_t * write_buffer, size_t write_buffer_len, const int8_t * command_string);

static const CLI_Command_Definition_t drv8874_control_command_def =
{
    ( const int8_t * const ) "drv8874", /* The command string to type. */
    ( const int8_t * const ) "drv8874 <ID> <mode> <target value>:\r\nSet DRV8874 motor control mode and parameters.\r\n\r\n",
    drv8874_control_command, /* The function to run. */
    -1 /* No parameters are expected. */
};

/**
 * @brief FreeRTOS timer task function for motor control loop.
 * 
 * @param xTimer FreeRTOS timer handle.
 */
static void drv8874_update_timer(TimerHandle_t xTimer)
{
    drv8874_update_control(&motors[0]);
    drv8874_update_control(&motors[1]);
    drv8874_update_control(&motors[2]);
    drv8874_update_control(&motors[3]);
}

/**
 * @brief Initialize ADC periodically convertion and DMA auto transfer.
 * 
 * @return drv8874_err_t 
 */
static drv8874_err_t drv8874_init_adc(void)
{
    if(!LL_LPTIM_IsEnabled(LPTIM3)){
        LL_LPTIM_Enable(LPTIM3);
    }
    return DRV8874_OK;
}

static portBASE_TYPE drv8874_control_command(int8_t * write_buffer, size_t write_buffer_len, const int8_t * command_string)
{
    int8_t *parameter_string;
    portBASE_TYPE parameter_string_length;
    int32_t motor_id;
    float target_value;
    drv8874_t *motor;
    char mode[16];

    // Parse motor ID
    parameter_string = (int8_t *) FreeRTOS_CLIGetParameter(command_string, 1, &parameter_string_length);
    if (parameter_string == NULL)
    {
        write_buffer_len -= lwsnprintf(write_buffer, write_buffer_len, "Error: Missing motor ID.\r\n");
        return pdFALSE;
    }
    motor_id = atoi((const char *) parameter_string);
    if (motor_id < 1 || motor_id > 4)
    {
        write_buffer_len -= lwsnprintf(write_buffer, write_buffer_len, "Error: Invalid motor ID. Must be between 1 and 4.\r\n");
        return pdFALSE;
    }
    motor = &motors[motor_id - 1];

    // Parse mode
    parameter_string = (int8_t *) FreeRTOS_CLIGetParameter(command_string, 2, &parameter_string_length);
    if (parameter_string == NULL)
    {
        write_buffer_len -= lwsnprintf(write_buffer, write_buffer_len, "Error: Missing mode.\r\n");
        return pdFALSE;
    }
    strncpy(mode, (const char *) parameter_string, parameter_string_length);
    mode[parameter_string_length] = '\0';

    // Parse target value
    parameter_string = (int8_t *) FreeRTOS_CLIGetParameter(command_string, 3, &parameter_string_length);
    if (parameter_string == NULL)
    {
        write_buffer_len -= lwsnprintf(write_buffer, write_buffer_len, "Error: Missing target value.\r\n");
        return pdFALSE;
    }
    target_value = atof((const char *) parameter_string);

    // Set motor control mode and target value
    if (strcmp(mode, "position") == 0)
    {
        drv8874_set_position_control(motor, motor->kp_pos, motor->kd_pos);
        drv8874_set_position(motor, target_value);
    }
    else if (strcmp(mode, "velocity") == 0)
    {
        drv8874_set_velecity_control(motor, motor->kp_vel, motor->kd_vel);
        drv8874_set_velocity(motor, target_value);
    }
    else if (strcmp(mode, "torque") == 0)
    {
        drv8874_set_torque_control(motor, motor->kp_torq, motor->kd_torq);
        drv8874_set_torque(motor, target_value);
    }
    else
    {
        write_buffer_len -= lwsnprintf(write_buffer, write_buffer_len, "Error: Invalid mode. Use 'position', 'velocity', or 'torque'.\r\n");
        return pdFALSE;
    }

    return pdTRUE;
}

drv8874_err_t drv8874_init()
{
    drv8874_err_t err = DRV8874_OK;

    memset(motors, 0, sizeof(motors));

    motors[0].id = 1;
    motors[0].mode = DRV8874_MODE_VELOCITY;
    motors[0].reverse = 0;
    motors[0].ctrl_gpio_port = CTRL_MOTOR1_GPIO_Port;
    motors[0].ctrl_gpio_pin = CTRL_MOTOR1_Pin;
    motors[0].pwm_timer = MOTOR_PWM_TIM;
    motors[0].pwm_channel = LL_TIM_CHANNEL_CH1;
    motors[0].pwm_tim_autoreload = 1000;
    motors[0].encoder_timer = ENCODER_TIM;
    motors[0].encoder_round_count = 13*4;
    motors[0].interval_timer = ENCODER_TIM;
    motors[0].interval_timer_frequency = 1000000;
    motors[0].torque_coefficient = 1.00f;
    motors[0].turndown_ratio = 30;
    motors[0].max_velocity = 6*2*3.1415926;
    motors[0].kp_pos = 100;
    motors[0].kd_pos = 8;
    motors[0].kp_vel = 100;
    motors[0].kd_vel = 8;
    motors[0].kp_torq = 100;
    motors[0].kd_torq = 8;

    motors[1].id = 2;
    motors[1].mode = DRV8874_MODE_VELOCITY;
    motors[1].reverse = 0;
    motors[1].ctrl_gpio_port = CTRL_MOTOR2_GPIO_Port;
    motors[1].ctrl_gpio_pin = CTRL_MOTOR2_Pin;
    motors[1].pwm_timer = MOTOR_PWM_TIM;
    motors[1].pwm_channel = LL_TIM_CHANNEL_CH3;
    motors[1].pwm_tim_autoreload = 1000;
    motors[1].encoder_timer = ENCODER_TIM;
    motors[1].encoder_round_count = 13*4;
    motors[1].interval_timer = ENCODER_TIM;
    motors[1].interval_timer_frequency = 1000000;
    motors[1].torque_coefficient = 1.00f;
    motors[1].turndown_ratio = 30;
    motors[1].max_velocity = 6*2*3.1415926;
    motors[1].kp_pos = 100;
    motors[1].kd_pos = 8;
    motors[1].kp_vel = 100;
    motors[1].kd_vel = 8;
    motors[1].kp_torq = 100;
    motors[1].kd_torq = 8;

    motors[2].id = 3;
    motors[2].mode = DRV8874_MODE_VELOCITY;
    motors[2].reverse = 0;
    motors[2].ctrl_gpio_port = CTRL_MOTOR3_GPIO_Port;
    motors[2].ctrl_gpio_pin = CTRL_MOTOR3_Pin;
    motors[2].pwm_timer = MOTOR_PWM_TIM;
    motors[2].pwm_channel = LL_TIM_CHANNEL_CH2;
    motors[2].pwm_tim_autoreload = 1000;
    motors[2].encoder_timer = ENCODER_TIM;
    motors[2].encoder_round_count = 13*4;
    motors[2].interval_timer = ENCODER_TIM;
    motors[2].interval_timer_frequency = 1000000;
    motors[2].torque_coefficient = 1.00f;
    motors[2].turndown_ratio = 30;
    motors[2].max_velocity = 6*2*3.1415926;
    motors[2].kp_pos = 100;
    motors[2].kd_pos = 8;
    motors[2].kp_vel = 100;
    motors[2].kd_vel = 8;
    motors[2].kp_torq = 100;
    motors[2].kd_torq = 8;

    motors[3].id = 4;
    motors[3].mode = DRV8874_MODE_VELOCITY;
    motors[3].reverse = 0;
    motors[3].ctrl_gpio_port = CTRL_MOTOR4_GPIO_Port;
    motors[3].ctrl_gpio_pin = CTRL_MOTOR4_Pin;
    motors[3].pwm_timer = MOTOR_PWM_TIM;
    motors[3].pwm_channel = LL_TIM_CHANNEL_CH4;
    motors[3].pwm_tim_autoreload = 1000;
    motors[3].encoder_timer = ENCODER_TIM;
    motors[3].encoder_round_count = 13*4;
    motors[3].interval_timer = ENCODER_TIM;
    motors[3].interval_timer_frequency = 1000000;
    motors[3].torque_coefficient = 1.00f;
    motors[3].turndown_ratio = 30;
    motors[3].max_velocity = 6*2*3.1415926;
    motors[3].kp_pos = 100;
    motors[3].kd_pos = 8;
    motors[3].kp_vel = 100;
    motors[3].kd_vel = 8;
    motors[3].kp_torq = 100;
    motors[3].kd_torq = 8;

    for (drv8874_t* motor=motors ; motor<motors+sizeof(motors)/sizeof(motors[0]); motor++)
    {
        DRV8874_LOG_DEBUG("DRV8874 motor %u initialization: reverse=%u, auto_reload=%u, round_count=%u, interval_timer_frequency=%u, torque_coeffecient=%f, turndown_ratio=%f, max_velocity=%f.\n",
            motor->id,
            motor->reverse,
            motor->pwm_tim_autoreload,
            motor->encoder_round_count,
            motor->interval_timer_frequency,
            motor->torque_coefficient,
            motor->turndown_ratio,
            motor->max_velocity);
    }

    err = drv8874_init_adc();
    if(err != DRV8874_OK)
    {
        return err;
    }

    LL_TIM_EnableCounter(MOTOR_PWM_TIM);
    LL_TIM_EnableCounter(MOTOR1_ENC_TIM);
    LL_TIM_EnableCounter(MOTOR2_ENC_TIM);
    LL_LPTIM_StartCounter(MOTOR3_ENC_TIM, LL_LPTIM_OPERATING_MODE_CONTINUOUS);
    LL_LPTIM_StartCounter(MOTOR4_ENC_TIM, LL_LPTIM_OPERATING_MODE_CONTINUOUS);
    LL_TIM_EnableCounter(ENCODER_TIM);
    LL_TIM_EnableIT_UPDATE(ENCODER_TIM);
    LL_TIM_EnableAllOutputs(MOTOR_PWM_TIM);

    /* Create FreeRTOS timer to handle motor control calculation. */
    motor_control_timer = xTimerCreate("motor_control", pdMS_TO_TICKS(10), pdTRUE, NULL, drv8874_update_timer);
    FreeRTOS_CLIRegisterCommand(&drv8874_control_command_def);
    return DRV8874_OK;
}
drv8874_err_t drv8874_set_position_control(drv8874_t *dev, float kp, float kd)
{
    dev->mode = DRV8874_MODE_POSITION;
    dev->kp_pos = kp;
    dev->kd_pos = kd;
    DRV8874_LOG_INFO("DRV8874 set position control: kp=%f, kd=%f", kp, kd);
    return DRV8874_OK;
}
drv8874_err_t drv8874_set_velecity_control(drv8874_t *dev, float kp, float kd)
{
    dev->mode = DRV8874_MODE_VELOCITY;
    dev->kp_vel = kp;
    dev->kd_vel = kd;
    DRV8874_LOG_INFO("DRV8874 set velocity control: kp=%f, kd=%f", kp, kd);
    return DRV8874_OK;
}
drv8874_err_t drv8874_set_torque_control(drv8874_t *dev, float kp, float kd)
{
    dev->mode = DRV8874_MODE_TORQUE;
    dev->kp_torq = kp;
    dev->kd_torq = kd;
    DRV8874_LOG_INFO("DRV8874 set torque control: kp=%f, kd=%f", kp, kd);
    return DRV8874_OK;
}
drv8874_err_t drv8874_set_torque_position_control(drv8874_t *dev, float kp, float kd)
{
    dev->mode = DRV8874_MODE_TORQUE_POSITION;
    dev->kp_pos = kp;
    dev->kd_pos = kd;
    DRV8874_LOG_INFO("DRV8874 set torque position control: kp=%f, kd=%f", kp, kd);
    return DRV8874_OK;
}
drv8874_err_t drv8874_set_torque_velocity_control(drv8874_t *dev, float kp, float kd)
{
    dev->mode = DRV8874_MODE_TORQUE_VELOCITY;
    dev->kp_vel = kp;
    dev->kd_vel = kd;
    DRV8874_LOG_INFO("DRV8874 set torque velocity control: kp=%f, kd=%f", kp, kd);
    return DRV8874_OK;
}
drv8874_err_t drv8874_set_position(drv8874_t *dev, float position)
{
    dev->target_position = position;
    DRV8874_LOG_INFO("DRV8874 set position: %f rad.", position);
    return DRV8874_OK;
}
drv8874_err_t drv8874_set_velocity(drv8874_t *dev, float velocity)
{
    dev->target_velocity = velocity;
    DRV8874_LOG_INFO("DRV8874 set velocity: %f rad/s.", velocity);
    return DRV8874_OK;
}
drv8874_err_t drv8874_set_torque(drv8874_t *dev, float torque)
{
    dev->target_torque = torque;
    DRV8874_LOG_INFO("DRV8874 set torque: %f Nm.", torque);
    return DRV8874_OK;
}
drv8874_err_t drv8874_set_direction(drv8874_t *dev, drv8874_direction_t direction)
{
    if(direction == DRV8874_DIRECTION_FORWARD)
    {
        LL_GPIO_SetOutputPin(dev->ctrl_gpio_port, dev->ctrl_gpio_pin);
        DRV8874_LOG_TRACE("DRV8874 position control: set direction backward");
    }
    else if(direction == DRV8874_DIRECTION_BACKWARD)
    {
        LL_GPIO_ResetOutputPin(dev->ctrl_gpio_port, dev->ctrl_gpio_pin);
        DRV8874_LOG_TRACE("DRV8874 position control: set direction forward");
    }
    return DRV8874_OK;
}
drv8874_err_t drv8874_start(drv8874_t *dev)
{
    LL_TIM_CC_EnableChannel(dev->pwm_timer, dev->pwm_channel);
    return DRV8874_OK;
}
drv8874_err_t drv8874_brake(drv8874_t *dev)
{
    switch (dev->pwm_channel)
    {
    case LL_TIM_CHANNEL_CH1:
        LL_TIM_OC_SetCompareCH1(dev->pwm_timer, 0);
        break;
    case LL_TIM_CHANNEL_CH2:
        LL_TIM_OC_SetCompareCH2(dev->pwm_timer, 0);
        break;
    case LL_TIM_CHANNEL_CH3:
        LL_TIM_OC_SetCompareCH3(dev->pwm_timer, 0);
        break;
    case LL_TIM_CHANNEL_CH4:
        LL_TIM_OC_SetCompareCH4(dev->pwm_timer, 0);
        break;
    default:
        DRV8874_LOG_ERROR("DRV8874 brake error: unknown pwm channel %d", dev->pwm_channel);
        return DRV8874_ERROR;
    }
    return DRV8874_OK;
}
drv8874_err_t drv8874_stop(drv8874_t *dev)
{
    LL_TIM_CC_DisableChannel(dev->pwm_timer, dev->pwm_channel);
    return DRV8874_OK;
}
drv8874_err_t drv8874_update_pos_control(drv8874_t *dev)
{
    return DRV8874_OK;
}
drv8874_err_t drv8874_update_vel_control(drv8874_t *dev)
{
    /* Output compare value to be written into PWM timer channel */
    uint32_t oc_value = 0;

    /* Calculate feed forward value with known maximum motor angular velocity and PWM timer auto-reload register value. */
    uint32_t feed_forward = (uint32_t)((float)dev->pwm_tim_autoreload * dev->target_velocity / dev->max_velocity);
    DRV8874_LOG_TRACE("DRV8874 position control: feed forward value %d", feed_forward);

    /* Calculate angular velocity error */
    dev->error_vel = dev->target_velocity - dev->current_velocity;
    DRV8874_LOG_TRACE("DRV8874 position control: velocity error %f", dev->error_vel);

    /* Apply PD control logic */
    dev->control_vel = (uint32_t)(dev->kp_vel * dev->error_vel + dev->kd_vel * (dev->error_vel - dev->prev_error_vel));
    DRV8874_LOG_TRACE("DRV8874 position control: control velocity %d", dev->control_vel);

    /* Update previous error with current error for next control update */
    dev->prev_error_vel = dev->error_vel;

    /* If sudden rotation direction reverse is needed, brake until motor velocity decrease to 0.1 rad/s
     * and then apply the reverse velocity to motor to avoid damage of H bridge inside the DRV8874 */
    if((dev->target_velocity * dev->current_velocity < 0) && (dev->current_velocity > 0.1 || dev->current_velocity < -0.1))
    {
        drv8874_brake(dev);
        DRV8874_LOG_TRACE("DRV8874 position control: brake to stop motor");
    }
    else
    {
        if((dev->control_vel < 0 && !dev->reverse) || (dev->control_vel > 0 && dev->reverse))
        {
            drv8874_set_direction(dev, DRV8874_DIRECTION_BACKWARD);
        }
        else if((dev->control_vel > 0 && !dev->reverse) || (dev->control_vel < 0 && dev->reverse))
        {
            drv8874_set_direction(dev, DRV8874_DIRECTION_FORWARD);
        }
    }

    /* Output compare value is the sum of feed forward value and feed back value. */
    oc_value = (uint32_t)(dev->control_vel) + feed_forward;
    DRV8874_LOG_TRACE("DRV8874 position control: raw output compare value %d", oc_value);

    /* Constrain the value of output comapre register */
    oc_value = oc_value > dev->pwm_tim_autoreload ? dev->pwm_tim_autoreload : oc_value;
    DRV8874_LOG_TRACE("DRV8874 position control: constrained output compare value %d", oc_value);

    switch(dev->pwm_channel)
    {
        case LL_TIM_CHANNEL_CH1:
            LL_TIM_OC_SetCompareCH1(dev->pwm_timer, oc_value);
            break;
        case LL_TIM_CHANNEL_CH2:
            LL_TIM_OC_SetCompareCH2(dev->pwm_timer, oc_value);
            break;
        case LL_TIM_CHANNEL_CH3:
            LL_TIM_OC_SetCompareCH3(dev->pwm_timer, oc_value);
            break;
        case LL_TIM_CHANNEL_CH4:
            LL_TIM_OC_SetCompareCH4(dev->pwm_timer, oc_value);
            break;
        default:
            DRV8874_LOG_ERROR("DRV8874 error: unknown pwm channel %d", dev->pwm_channel);
            return DRV8874_ERROR;
    }
    return DRV8874_OK;
}
drv8874_err_t drv8874_update_torque_control(drv8874_t *dev)
{
    return DRV8874_OK;
}
drv8874_err_t drv8874_update_torq_pos_control(drv8874_t *dev)
{
    return DRV8874_OK;
}
drv8874_err_t drv8874_update_torq_vel_control(drv8874_t *dev)
{
    return DRV8874_OK;
}
drv8874_err_t drv8874_update_control(drv8874_t *dev)
{
    drv8874_err_t err = DRV8874_OK;

    dev->encoder_count += (int16_t)LL_TIM_GetCounter(dev->encoder_timer);
    LL_TIM_SetCounter(dev->encoder_timer, 0);
    dev->current_position = (float)dev->encoder_count / (dev->encoder_round_count * dev->turndown_ratio) * 2 * 3.14159265f;

    /* `dev->interval` increases 65536 in update interrupt of interval timer */
    dev->interval_timer_count += LL_TIM_GetCounter(dev->interval_timer);
    LL_TIM_SetCounter(dev->interval_timer, 0);

    /* Calculate angular velocity */
    dev->current_velocity = 2 * 3.14159265f * 
        (float)dev->interval_timer_frequency * 
        (float)(dev->encoder_count - dev->previous_encoder_count) /
        (float)(dev->encoder_round_count*dev->turndown_ratio) /
        (float)(dev->interval_timer_count);
    
    /* Reset interval to count new interval time */
    dev->interval_timer_count = 0;

    dev->previous_encoder_count = dev->encoder_count;

    switch (dev->mode)
    {
    case DRV8874_MODE_POSITION:
        (void)drv8874_update_pos_control(dev);
        break;
    case DRV8874_MODE_VELOCITY:
        (void)drv8874_update_vel_control(dev);
        break;
    case DRV8874_MODE_TORQUE:
        (void)drv8874_update_torque_control(dev);
        break;
    case DRV8874_MODE_TORQUE_POSITION:
        (void)drv8874_update_torq_pos_control(dev);
        break;
    case DRV8874_MODE_TORQUE_VELOCITY:
        (void)drv8874_update_torq_vel_control(dev);
        break;
    default:
        err = DRV8874_ERROR;
        DRV8874_LOG_ERROR("DRV8874 error: unknown mode %d", dev->mode);
        break;
    }
    return err;
}

#ifdef cplusplus
}
#endif