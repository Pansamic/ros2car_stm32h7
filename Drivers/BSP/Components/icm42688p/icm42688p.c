/**
 * @file icm42688p.c
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2025-03-20
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#include "main.h"
#include "icm42688p.h"

#define icm42688p_read_reg(reg, data, len) icm42688p_spi_read(reg, data, len)
#define icm42688p_write_reg(reg, data, len) icm42688p_spi_write(reg, data, len)

static icm42688p_err_t icm42688p_spi_read(uint8_t reg, uint8_t *data, uint16_t len)
{
    reg |= 0x80;
    LL_SPI_SetTransferSize(SPI1, len+1);
    LL_SPI_Enable(SPI1);
    LL_SPI_StartMasterTransfer(SPI1);
    LL_GPIO_ResetOutputPin(SPI_CS_IMU_GPIO_Port, SPI_CS_IMU_Pin);
    while(!LL_SPI_IsActiveFlag_TXP(SPI1));
    LL_SPI_TransmitData8(SPI1, reg);
    for(uint16_t i = 0; i < len; i++)
    {
        while(!LL_SPI_IsActiveFlag_TXP(SPI1));
        LL_SPI_TransmitData8(SPI1, 0x00);
        while(!LL_SPI_IsActiveFlag_RXP(SPI1));
        data[i] = LL_SPI_ReceiveData8(SPI1);
    }
    while (LL_SPI_IsActiveFlag_EOT(SPI1) == RESET);
    LL_GPIO_SetOutputPin(SPI_CS_IMU_GPIO_Port, SPI_CS_IMU_Pin);
    LL_SPI_ClearFlag_EOT(SPI1);
    LL_SPI_ClearFlag_TXTF(SPI1);
    LL_SPI_Disable(SPI1);
    return 0;
}

static icm42688p_err_t icm42688p_spi_write(uint8_t reg, const uint8_t *data, uint16_t len)
{
    LL_SPI_SetTransferSize(SPI1, len+1);
    LL_SPI_Enable(SPI1);
    LL_SPI_StartMasterTransfer(SPI1);
    LL_GPIO_ResetOutputPin(SPI_CS_IMU_GPIO_Port, SPI_CS_IMU_Pin);
    while(!LL_SPI_IsActiveFlag_TXP(SPI1));
    LL_SPI_TransmitData8(SPI1, reg);
    for(uint16_t i = 0; i < len; i++)
    {
        while(!LL_SPI_IsActiveFlag_TXP(SPI1));
        LL_SPI_TransmitData8(SPI1, data[i]);
    }
    while (LL_SPI_IsActiveFlag_EOT(SPI1) == RESET);
    LL_GPIO_SetOutputPin(SPI_CS_IMU_GPIO_Port, SPI_CS_IMU_Pin);
    LL_SPI_ClearFlag_EOT(SPI1);
    LL_SPI_ClearFlag_TXTF(SPI1);
    LL_SPI_Disable(SPI1);
    return 0;
}

icm42688p_err_t icm42688p_init(void)
{
    uint8_t data;
    icm42688p_err_t ret;
    data = 0x01;
    ret = icm42688p_write_reg(UB0_REG_DEVICE_CONFIG, &data, 1);
    if(ret != ICM42688P_OK)
    {
        return ret;
    }
    data = 0x00;
    ret = icm42688p_write_reg(UB0_REG_DRIVE_CONFIG, &data, 1);
    if(ret != ICM42688P_OK)
    {
        return ret;
    }
    data = 0x00;
    ret = icm42688p_write_reg(UB0_REG_INT_CONFIG, &data, 1);
    if(ret != ICM42688P_OK)
    {
        return ret;
    }
    data = 0x00;
    ret = icm42688p_write_reg(UB0_REG_FIFO_CONFIG, &data, 1);
    if(ret != ICM42688P_OK)
    {
        return ret;
    }
    return ICM42688P_OK;
}

icm42688p_err_t icm42688p_read(icm42688p_t *data)
{
    uint8_t buf[14];
    icm42688p_err_t ret;
    ret = icm42688p_read_reg(UB0_REG_TEMP_DATA1, buf, 14);
    if(ret != ICM42688P_OK)
    {
        return ret;
    }
    data->temperature = (double)((int16_t)(buf[0]<<8|buf[1]))/132.48+25;
    data->acceleration_x = (double)((int16_t)(buf[2]<<8|buf[3]))/16384;
    data->acceleration_y = (double)((int16_t)(buf[4]<<8|buf[5]))/16384;
    data->acceleration_z = (double)((int16_t)(buf[6]<<8|buf[7]))/16384;
    data->angular_velocity_x = (double)((int16_t)(buf[8]<<8|buf[9]))/131.072;
    data->angular_velocity_y = (double)((int16_t)(buf[10]<<8|buf[11]))/131.072;
    data->angular_velocity_z = (double)((int16_t)(buf[12]<<8|buf[13]))/131.072;
    return ICM42688P_OK;
}