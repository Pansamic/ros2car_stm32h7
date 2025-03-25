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

#define UB0_REG_DEVICE_CONFIG  0x11
// break
#define UB0_REG_DRIVE_CONFIG  0x13
#define UB0_REG_INT_CONFIG    0x14
// break
#define UB0_REG_FIFO_CONFIG  0x16
// break
#define UB0_REG_TEMP_DATA1     0x1D
#define UB0_REG_TEMP_DATA0     0x1E
#define UB0_REG_ACCEL_DATA_X1  0x1F
#define UB0_REG_ACCEL_DATA_X0  0x20
#define UB0_REG_ACCEL_DATA_Y1  0x21
#define UB0_REG_ACCEL_DATA_Y0  0x22
#define UB0_REG_ACCEL_DATA_Z1  0x23
#define UB0_REG_ACCEL_DATA_Z0  0x24
#define UB0_REG_GYRO_DATA_X1   0x25
#define UB0_REG_GYRO_DATA_X0   0x26
#define UB0_REG_GYRO_DATA_Y1   0x27
#define UB0_REG_GYRO_DATA_Y0   0x28
#define UB0_REG_GYRO_DATA_Z1   0x29
#define UB0_REG_GYRO_DATA_Z0   0x2A
#define UB0_REG_TMST_FSYNCH    0x2B
#define UB0_REG_TMST_FSYNCL    0x2C
#define UB0_REG_INT_STATUS     0x2D
#define UB0_REG_FIFO_COUNTH    0x2E
#define UB0_REG_FIFO_COUNTL    0x2F
#define UB0_REG_FIFO_DATA      0x30
#define UB0_REG_APEX_DATA0     0x31
#define UB0_REG_APEX_DATA1     0x32
#define UB0_REG_APEX_DATA2     0x33
#define UB0_REG_APEX_DATA3     0x34
#define UB0_REG_APEX_DATA4     0x35
#define UB0_REG_APEX_DATA5     0x36
#define UB0_REG_INT_STATUS2    0x37
#define UB0_REG_INT_STATUS3    0x38
// break
#define UB0_REG_SIGNAL_PATH_RESET   0x4B
#define UB0_REG_INTF_CONFIG0        0x4C
#define UB0_REG_INTF_CONFIG1        0x4D
#define UB0_REG_PWR_MGMT0           0x4E
#define UB0_REG_GYRO_CONFIG0        0x4F
#define UB0_REG_ACCEL_CONFIG0       0x50
#define UB0_REG_GYRO_CONFIG1        0x51
#define UB0_REG_GYRO_ACCEL_CONFIG0  0x52
#define UB0_REG_ACCEFL_CONFIG1      0x53
#define UB0_REG_TMST_CONFIG         0x54
// break
#define UB0_REG_APEX_CONFIG0  0x56
#define UB0_REG_SMD_CONFIG    0x57
// break
#define UB0_REG_FIFO_CONFIG1  0x5F
#define UB0_REG_FIFO_CONFIG2  0x60
#define UB0_REG_FIFO_CONFIG3  0x61
#define UB0_REG_FSYNC_CONFIG  0x62
#define UB0_REG_INT_CONFIG0   0x63
#define UB0_REG_INT_CONFIG1   0x64
#define UB0_REG_INT_SOURCE0   0x65
#define UB0_REG_INT_SOURCE1   0x66
// break
#define UB0_REG_INT_SOURCE3  0x68
#define UB0_REG_INT_SOURCE4  0x69
// break
#define UB0_REG_FIFO_LOST_PKT0  0x6C
#define UB0_REG_FIFO_LOST_PKT1  0x6D
// break
#define UB0_REG_SELF_TEST_CONFIG  0x70
// break
#define UB0_REG_WHO_AM_I  0x75

// User Bank 1
#define UB1_REG_SENSOR_CONFIG0  0x03
// break
#define UB1_REG_GYRO_CONFIG_STATIC2   0x0B
#define UB1_REG_GYRO_CONFIG_STATIC3   0x0C
#define UB1_REG_GYRO_CONFIG_STATIC4   0x0D
#define UB1_REG_GYRO_CONFIG_STATIC5   0x0E
#define UB1_REG_GYRO_CONFIG_STATIC6   0x0F
#define UB1_REG_GYRO_CONFIG_STATIC7   0x10
#define UB1_REG_GYRO_CONFIG_STATIC8   0x11
#define UB1_REG_GYRO_CONFIG_STATIC9   0x12
#define UB1_REG_GYRO_CONFIG_STATIC10  0x13
// break
#define UB1_REG_XG_ST_DATA  0x5F
#define UB1_REG_YG_ST_DATA  0x60
#define UB1_REG_ZG_ST_DATA  0x61
#define UB1_REG_TMSTVAL0    0x62
#define UB1_REG_TMSTVAL1    0x63
#define UB1_REG_TMSTVAL2    0x64
// break
#define UB1_REG_INTF_CONFIG4  0x7A
#define UB1_REG_INTF_CONFIG5  0x7B
#define UB1_REG_INTF_CONFIG6  0x7C

// User Bank 2
#define UB2_REG_ACCEL_CONFIG_STATIC2  0x03
#define UB2_REG_ACCEL_CONFIG_STATIC3  0x04
#define UB2_REG_ACCEL_CONFIG_STATIC4  0x05
// break
#define UB2_REG_XA_ST_DATA  0x3B
#define UB2_REG_YA_ST_DATA  0x3C
#define UB2_REG_ZA_ST_DATA  0x3D

// User Bank 4
#define UB4_REG_APEX_CONFIG1  0x40
#define UB4_REG_APEX_CONFIG2  0x41
#define UB4_REG_APEX_CONFIG3  0x42
#define UB4_REG_APEX_CONFIG4  0x43
#define UB4_REG_APEX_CONFIG5  0x44
#define UB4_REG_APEX_CONFIG6  0x45
#define UB4_REG_APEX_CONFIG7  0x46
#define UB4_REG_APEX_CONFIG8  0x47
#define UB4_REG_APEX_CONFIG9  0x48
// break
#define UB4_REG_ACCEL_WOM_X_THR  0x4A
#define UB4_REG_ACCEL_WOM_Y_THR  0x4B
#define UB4_REG_ACCEL_WOM_Z_THR  0x4C
#define UB4_REG_INT_SOURCE6      0x4D
#define UB4_REG_INT_SOURCE7      0x4E
#define UB4_REG_INT_SOURCE8      0x4F
#define UB4_REG_INT_SOURCE9      0x50
#define UB4_REG_INT_SOURCE10     0x51
// break
#define UB4_REG_OFFSET_USER0  0x77
#define UB4_REG_OFFSET_USER1  0x78
#define UB4_REG_OFFSET_USER2  0x79
#define UB4_REG_OFFSET_USER3  0x7A
#define UB4_REG_OFFSET_USER4  0x7B
#define UB4_REG_OFFSET_USER5  0x7C
#define UB4_REG_OFFSET_USER6  0x7D
#define UB4_REG_OFFSET_USER7  0x7E
#define UB4_REG_OFFSET_USER8  0x7F

#define icm42688p_read_regs(reg, data, len) icm42688p_spi_read_regs(reg, data, len)
#define icm42688p_write_regs(reg, data, len) icm42688p_spi_write_regs(reg, data, len)
#define icm42688p_read_reg(reg) icm42688p_spi_read_reg(reg)
#define icm42688p_write_reg(reg, data) icm42688p_spi_write_reg(reg, data)

static void icm42688p_spi_read_regs(uint8_t reg, uint8_t *data, uint16_t len)
{
    reg |= 0x80;
    LL_SPI_SetTransferSize(SPI1, len+1);
    // LL_SPI_Enable(SPI1);
    LL_SPI_StartMasterTransfer(SPI1);
    // LL_GPIO_ResetOutputPin(SPI_CS_IMU_GPIO_Port, SPI_CS_IMU_Pin);
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
    // LL_GPIO_SetOutputPin(SPI_CS_IMU_GPIO_Port, SPI_CS_IMU_Pin);
    LL_SPI_ClearFlag_EOT(SPI1);
    LL_SPI_ClearFlag_TXTF(SPI1);
    // LL_SPI_Disable(SPI1);
}

static void icm42688p_spi_write_regs(uint8_t reg, const uint8_t *data, uint16_t len)
{
    LL_SPI_SetTransferSize(SPI1, len+1);
    // LL_SPI_Enable(SPI1);
    LL_SPI_StartMasterTransfer(SPI1);
    // LL_GPIO_ResetOutputPin(SPI_CS_IMU_GPIO_Port, SPI_CS_IMU_Pin);
    while(!LL_SPI_IsActiveFlag_TXP(SPI1));
    LL_SPI_TransmitData8(SPI1, reg);
    for(uint16_t i = 0; i < len; i++)
    {
        while(!LL_SPI_IsActiveFlag_TXP(SPI1));
        LL_SPI_TransmitData8(SPI1, data[i]);
    }
    while (LL_SPI_IsActiveFlag_EOT(SPI1) == RESET);
    // LL_GPIO_SetOutputPin(SPI_CS_IMU_GPIO_Port, SPI_CS_IMU_Pin);
    LL_SPI_ClearFlag_EOT(SPI1);
    LL_SPI_ClearFlag_TXTF(SPI1);
    // LL_SPI_Disable(SPI1);
}

static uint8_t icm42688p_spi_read_reg(uint8_t reg)
{
    uint8_t data = 0;
    reg |= 0x80;
    LL_SPI_SetTransferSize(SPI1, 2);
    // LL_SPI_Enable(SPI1);
    LL_SPI_StartMasterTransfer(SPI1);
    // LL_GPIO_ResetOutputPin(SPI_CS_IMU_GPIO_Port, SPI_CS_IMU_Pin);

    /* Transmit register address and receive dummy byte */
    while(!LL_SPI_IsActiveFlag_TXP(SPI1));
    LL_SPI_TransmitData8(SPI1, reg);
    while(!LL_SPI_IsActiveFlag_RXP(SPI1));
    data = LL_SPI_ReceiveData8(SPI1);

    /* Transmit dummy byte and receive one byte */
    while(!LL_SPI_IsActiveFlag_TXP(SPI1));
    LL_SPI_TransmitData8(SPI1, 0x00);
    while(!LL_SPI_IsActiveFlag_RXP(SPI1));
    data = LL_SPI_ReceiveData8(SPI1);
    
    while (LL_SPI_IsActiveFlag_EOT(SPI1) == RESET);
    // LL_GPIO_SetOutputPin(SPI_CS_IMU_GPIO_Port, SPI_CS_IMU_Pin);
    LL_SPI_ClearFlag_EOT(SPI1);
    LL_SPI_ClearFlag_TXTF(SPI1);
    // LL_SPI_Disable(SPI1);
    return data;
}

static void icm42688p_spi_write_reg(uint8_t reg, const uint8_t data)
{
    LL_SPI_SetTransferSize(SPI1, 2);
    // LL_SPI_Enable(SPI1);
    LL_SPI_StartMasterTransfer(SPI1);
    // LL_GPIO_ResetOutputPin(SPI_CS_IMU_GPIO_Port, SPI_CS_IMU_Pin);
    while(!LL_SPI_IsActiveFlag_TXP(SPI1));
    LL_SPI_TransmitData8(SPI1, reg);

    while(!LL_SPI_IsActiveFlag_TXP(SPI1));
    LL_SPI_TransmitData8(SPI1, data);
    
    while (LL_SPI_IsActiveFlag_EOT(SPI1) == RESET);
    // LL_GPIO_SetOutputPin(SPI_CS_IMU_GPIO_Port, SPI_CS_IMU_Pin);
    LL_SPI_ClearFlag_EOT(SPI1);
    LL_SPI_ClearFlag_TXTF(SPI1);
    // LL_SPI_Disable(SPI1);
}

icm42688p_err_t icm42688p_init(void)
{
    uint8_t data;
    icm42688p_err_t ret;

    /* Soft reset ICM-42688-P */
    icm42688p_write_reg(UB0_REG_DEVICE_CONFIG, 0x01);
    if(ret != ICM42688P_OK)
    {
        return ret;
    }
    /* Wait 10 ms for soft reset be effective. */
    vTaskDelay(pdMS_TO_TICKS(10));

    /* Set slew rate 20ns-60ns under SPI mode. */
    // icm42688p_write_reg(UB0_REG_DRIVE_CONFIG, 0x00);

    // icm42688p_write_reg(UB0_REG_INT_CONFIG, 0x00);

    // icm42688p_write_reg(UB0_REG_FIFO_CONFIG, 0x00);

    /* Enable GYRO, accelerometer in low noise mode and RC oscillator */
    icm42688p_write_reg(UB0_REG_PWR_MGMT0, 0x1F);

    /* Set GYRO to 200Hz data rate and 2000dps range. */
    icm42688p_write_reg(UB0_REG_GYRO_CONFIG0, 0x07);

    /* Set accelerometer to 200Hz data rate and 16g range. */
    icm42688p_write_reg(UB0_REG_ACCEL_CONFIG0, 0x07);

    /* Disable GYRO and accelerometer low pass filter. */
    icm42688p_write_reg(UB0_REG_GYRO_ACCEL_CONFIG0, 0x00);

    /* Connect FSYNC interrupt signal to INT1 pin */
    icm42688p_write_reg(UB0_REG_INT_SOURCE0, 0x40);

    /* Connect data-ready interrupt signal to INT2 pin */
    icm42688p_write_reg(UB0_REG_INT_SOURCE3, 0x08);
    return ICM42688P_OK;
}

icm42688p_err_t icm42688p_read(icm42688p_t *data)
{
    uint8_t buf[14];
    icm42688p_err_t ret;
    
    icm42688p_read_regs(UB0_REG_TEMP_DATA1, buf, 14);

    data->temperature = (double)((int16_t)(buf[0]<<8|buf[1]))/132.48+25;
    data->acceleration_x = (double)((int16_t)(buf[2]<<8|buf[3]))/16384;
    data->acceleration_y = (double)((int16_t)(buf[4]<<8|buf[5]))/16384;
    data->acceleration_z = (double)((int16_t)(buf[6]<<8|buf[7]))/16384;
    data->angular_velocity_x = (double)((int16_t)(buf[8]<<8|buf[9]))/131.072;
    data->angular_velocity_y = (double)((int16_t)(buf[10]<<8|buf[11]))/131.072;
    data->angular_velocity_z = (double)((int16_t)(buf[12]<<8|buf[13]))/131.072;
    return ICM42688P_OK;
}