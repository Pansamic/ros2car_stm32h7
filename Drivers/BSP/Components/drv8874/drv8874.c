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
#include "drv8874.h"

#ifdef cplusplus
extern "C" {
#endif

/* DRV8874 control block instance */
drv8874_t motors[4];

static drv8874_err_t drv8874_init_adc(void)
{
    return DRV8874_OK;
}

drv8874_err_t drv8874_init(drv8874_t *dev)
{
    drv8874_err_t err = DRV8874_OK;
    err = drv8874_init_adc();
    if(err != DRV8874_OK)
    {
        return err;
    }
    return DRV8874_OK;
}
drv8874_err_t drv8874_set_position_control(drv8874_t *dev, float kp, float kd)
{
    return DRV8874_OK;
}
drv8874_err_t drv8874_set_velecity_control(drv8874_t *dev, float kp, float kd)
{
    return DRV8874_OK;
}
drv8874_err_t drv8874_set_torque_control(drv8874_t *dev, float kp, float kd)
{
    return DRV8874_OK;
}
drv8874_err_t drv8874_set_velocity(drv8874_t *dev, float velocity)
{
    return DRV8874_OK;
}
drv8874_err_t drv8874_set_direction(drv8874_t *dev, drv8874_direction_t direction)
{
    return DRV8874_OK;
}
drv8874_err_t drv8874_start(drv8874_t *dev)
{
    return DRV8874_OK;
}
drv8874_err_t drv8874_brake(drv8874_t *dev)
{
    return DRV8874_OK;
}
drv8874_err_t drv8874_stop(drv8874_t *dev)
{
    return DRV8874_OK;
}

#ifdef cplusplus
}
#endif