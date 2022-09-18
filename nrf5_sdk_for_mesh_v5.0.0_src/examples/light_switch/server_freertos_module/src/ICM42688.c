
/*---------------------------------------------------
 * 
 * ICM42688六轴加速度角速度传感器驱动程序，基于IIC传输
 *
 *--------------------------------------------------*/



#include "ICM42688.h"


#define ALPHA 0.99             /*!< Weight for gyroscope */
#define RAD_TO_DEG 57.27272727 /*!< Radians to degrees */



esp_err_t icm42688_write_byte(uint8_t reg_addr, uint8_t data)
{
    esp_err_t ret;
    ret = twi_reg_write(ICM42688_I2C_ADDRESS, reg_addr, data);
    return ret;
}

esp_err_t icm42688_write(uint8_t reg_start_addr, uint8_t reg_num, uint8_t *data_buf)
{
    uint32_t i = 0;
    if (data_buf != NULL) {
        for(i=0; i<reg_num; i++) {
            icm42688_write_byte(reg_start_addr+i, data_buf[i]);
        }
        return ESP_OK;
    }
    return ESP_FAIL;
}

esp_err_t icm42688_read_byte( uint8_t reg, uint8_t *data)
{
    esp_err_t ret;
    ret = twi_reg_read(ICM42688_I2C_ADDRESS, reg, data, 1);
    return ret;
}

esp_err_t icm42688_read_consec(uint8_t reg_start_addr, uint8_t reg_num, uint8_t *data_buf) //按地址递增连续读取
{
    uint32_t i = 0;
    uint8_t data_t = 0;
    if (data_buf != NULL) {
        for(i=0; i<reg_num; i++){
            icm42688_read_byte(reg_start_addr+i, &data_t);
            data_buf[i] = data_t;
        }
        return ESP_OK;
    } 
    return ESP_FAIL;  
}


esp_err_t icm42688_read(uint8_t reg_addr, uint8_t *dest, uint8_t length) //单地址读取多个字节
{
    esp_err_t ret;
    ret = twi_reg_read(ICM42688_I2C_ADDRESS, reg_addr, dest, length);
    return ret;
}


esp_err_t icm42688_get_deviceid(uint8_t* deviceid)
{
    esp_err_t ret;
    uint8_t tmp;
    ret = icm42688_read_byte(ICM42688_WHO_AM_I, &tmp);
    *deviceid = tmp;
    return ret;
}

esp_err_t icm42688_init(void)  //传感器初始化，读取设备id验证
{
    esp_err_t ret;
    icm42688_write_byte(ICM42688_INTF_CONFIG1, ICM42688_CLOCK_SEL_PLL);
    icm42688_write_byte(ICM42688_DEVICE_CONFIG, ICM42688_PWR_RESET);
    nrf_delay_ms(1);
    icm42688_write_byte(ICM42688_INTF_CONFIG1, ICM42688_CLOCK_SEL_PLL);
    uint8_t id=0;
    if(icm42688_get_deviceid(&id) == false){
        return ESP_FAIL;
    }
    if(id != ICM42688_ID){
        return ESP_FAIL;
    }
    ret = icm42688_write_byte(ICM42688_PWR_MGMT0, ICM42688_SEN_ENABLE);
    ret = icm42688_write_byte(ICM42688_ACCEL_CONFIG0, ICM42688_ACCEL_FS_SEL_16G | ICM42688_ACCEL_ODR_32KHZ);
    ret = icm42688_write_byte(ICM42688_GYRO_CONFIG0, ICM42688_GYRO_FS_SEL_2000DPS | ICM42688_GYRO_ODR_32KHZ);
    nrf_delay_ms(2);
    return ret;
}


esp_err_t icm42688_pwr_mgmt(icm42688_pwr_t val)
{
    esp_err_t ret;
    ret = icm42688_write_byte(ICM42688_PWR_MGMT0, val);
    return ret;
}

/*
esp_err_t  icm42688_sleep(mpu6050_handle_t sensor)
{
    esp_err_t ret;
    uint8_t tmp;
    ret =  mpu6050_read_byte(sensor, MPU6050_PWR_MGMT_1, &tmp);
    if (ret == ESP_FAIL) {
        return ret;
    }
    tmp |= BIT6;
    ret =  mpu6050_write_byte(sensor, MPU6050_PWR_MGMT_1, tmp);
    return ret;
}
*/

esp_err_t icm42688_set_acce_fs(icm42688_acce_fs_t acce_fs)
{
    esp_err_t ret;
    uint8_t tmp;
    ret = icm42688_read_byte(ICM42688_ACCEL_CONFIG0, &tmp);
    if (ret == ESP_FAIL) {
        return ret;
    }
    tmp &= 0x1F; // 0001 1111b
    tmp |= (acce_fs << 5);
    ret =  icm42688_write_byte(ICM42688_ACCEL_CONFIG0, tmp);
    return ret;
}

esp_err_t  icm42688_set_gyro_fs(icm42688_gyro_fs_t gyro_fs)
{
    esp_err_t ret;
    uint8_t tmp;
    ret =  icm42688_read_byte(ICM42688_GYRO_CONFIG0, &tmp);
    if (ret == ESP_FAIL) {
        return ret;
    }
    tmp &= 0x1F; // 0001 1111b
    tmp |= (gyro_fs << 5);
    ret =  icm42688_write_byte(ICM42688_GYRO_CONFIG0, tmp);
    return ret;
}

esp_err_t icm42688_get_acce_fs(icm42688_acce_fs_t *acce_fs)
{
    esp_err_t ret;
    uint8_t tmp;
    ret = icm42688_read_byte(ICM42688_ACCEL_CONFIG0, &tmp);
    tmp = (tmp >> 5) & 0x07;
    *acce_fs = tmp;
    return ret;
}

esp_err_t icm42688_get_gyro_fs(icm42688_gyro_fs_t *gyro_fs)
{
    esp_err_t ret;
    uint8_t tmp;
    ret = icm42688_read_byte(ICM42688_GYRO_CONFIG0, &tmp);
    tmp = (tmp >> 5) & 0x07;
    *gyro_fs = tmp;
    return ret;
}

esp_err_t icm42688_get_raw_acce(icm42688_raw_acce_value_t *raw_acce_value)
{
    uint8_t data_rd[6] = {0};

    esp_err_t ret = icm42688_read(ICM42688_ACCEL_OUT, data_rd, 6);

    raw_acce_value->raw_acce_x = (int16_t)((data_rd[0] << 8) + (data_rd[1]));
    raw_acce_value->raw_acce_y = (int16_t)((data_rd[2] << 8) + (data_rd[3]));
    raw_acce_value->raw_acce_z = (int16_t)((data_rd[4] << 8) + (data_rd[5]));
    return ret;
}

esp_err_t icm42688_get_raw_gyro(icm42688_raw_gyro_value_t *raw_gyro_value)
{
    uint8_t data_rd[6] = {0};

    esp_err_t ret = icm42688_read(ICM42688_GYRO_OUT, data_rd, 6);

    raw_gyro_value->raw_gyro_x = (int16_t)((data_rd[0] << 8) + (data_rd[1]));
    raw_gyro_value->raw_gyro_y = (int16_t)((data_rd[2] << 8) + (data_rd[3]));
    raw_gyro_value->raw_gyro_z = (int16_t)((data_rd[4] << 8) + (data_rd[5]));
    return ret;
}


esp_err_t icm42688_get_temp(int16_t *temp)
{
    esp_err_t ret;
    uint8_t tmp[2];
    int16_t rawt;
    ret = icm42688_read_consec(ICM42688_TEMP_OUT, 2, tmp);
    rawt = (int16_t)(tmp[0]<<8) | tmp[1];
    float t = (float)rawt /132.48 + 25;
    *temp = (int16_t)(t * 10);
    return ret;
}


//esp_err_t icm42688_get_fifo_data(icm42688_handle_t sensor, size_t len, uint8_t *buffer)
//{
//    mpu6050_dev_t* sens = (mpu6050_dev_t*) sensor;
//    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
//    i2c_master_start(cmd);
//    i2c_master_write_byte(cmd, ( MPU6050_I2C_ADDRESS << 1 ) | WRITE_BIT, ACK_CHECK_EN);
//    i2c_master_write_byte(cmd, MPU6050_FIFO_R_W, ACK_CHECK_EN);
//    i2c_master_start(cmd);
//    i2c_master_write_byte(cmd, ( MPU6050_I2C_ADDRESS << 1 ) | READ_BIT, ACK_CHECK_EN);
//    if(len > 1){
//        i2c_master_read(cmd, buffer, len - 1, ACK_VAL);
//    }
//    i2c_master_read(cmd, buffer + len - 1, 1, NACK_VAL);
//    i2c_master_stop(cmd);
//    esp_err_t ret =  i2c_bus_cmd_begin(sens->bus, cmd, 1000 / portTICK_RATE_MS);
//    i2c_cmd_link_delete(cmd);
//    return ret;
//}
//
//esp_err_t icm42688_get_fifo_len(icm42688_handle_t sensor, uint16_t *len)
//{
//    esp_err_t ret;
//    uint8_t datard[2];
//    ret =  mpu6050_read(sensor, MPU6050_FIFO_COUNTH, 2,  datard);
//    *len = ((int16_t)datard[0] << 8) + datard[1];
//    return ret;
//}
//
//
//esp_err_t icm42688_set_dlpf(icm42688_handle_t sensor, uint8_t flags)
//{
//    esp_err_t ret;
//    ret = mpu6050_write_byte(sensor, MPU6050_CONFIG, flags);
//    return ret;
//}
//
//esp_err_t icm42688_set_smplrt_div(icm42688_handle_t sensor, uint8_t val)
//{
//    esp_err_t ret;
//    ret = mpu6050_write_byte(sensor, MPU6050_SMPLRT_DIV, val);
//    return ret;
//}
//

//esp_err_t icm42688_set_fifo_en(icm42688_handle_t sensor, uint8_t flags)
//{
//    esp_err_t ret;
//    ret = mpu6050_write_byte(sensor, MPU6050_FIFO_EN, flags);
//    ret = mpu6050_write_byte(sensor, MPU6050_USER_CTRL, 0x44);
//    return ret;
//}
//
//
