
/*---------------------------------------------------
 * 
 * ICM42688六轴加速度角速度传感器驱动程序，基于IIC传输
 *
 *--------------------------------------------------*/


#ifndef ICM42688_H_
#define ICM42688_H_


#include "custom_twi.h"
#include "nrf_delay.h"


//esp库移植宏
#define ESP_OK true
#define ESP_FAIL false
typedef bool esp_err_t;


#if 0
#define ICM42688_WHO_AM_I            0x75


#define ICM42688_PWR_RESET           0x80
#define ICM42688_INTF_CONFIG1        0x4D
#define ICM42688_CLOCK_SEL_PLL       0x01
#define ICM42688_PWR_MGMT0           0x4E
#define ICM42688_SEN_ENABLE          0x0F

/* ICM42688 register */
//#define ICM42688_SELF_TEST_X         0x0D
//#define ICM42688_SELF_TEST_Y         0x0E
//#define ICM42688_SELF_TEST_Z         0x0F
//#define ICM42688_SELF_TEST_A         0x10
#define ICM42688_DEVICE_CONFIG       0x11
#define ICM42688_DRIVE_CONFIG        0x13
//#define ICM42688_SMPLRT_DIV          0x19
//#define ICM42688_CONFIG              0x1A

//#define ICM42688_FIFO_EN             0x23
//#define ICM42688_I2C_MST_CTRL        0x24
//#define ICM42688_I2C_SLV0_ADDR       0x25
//#define ICM42688_I2C_SLV0_REG        0x26
//#define ICM42688_I2C_SLV0_CTRL       0x27
//#define ICM42688_I2C_SLV1_ADDR       0x28
//#define ICM42688_I2C_SLV1_REG        0x29
//#define ICM42688_I2C_SLV1_CTRL       0x2A
//#define ICM42688_I2C_SLV2_ADDR       0x2B
//#define ICM42688_I2C_SLV2_REG        0x2C
//#define ICM42688_I2C_SLV2_CTRL       0x2D
//#define ICM42688_I2C_SLV3_ADDR       0x2E
//#define ICM42688_I2C_SLV3_REG        0x2F
//#define ICM42688_I2C_SLV3_CTRL       0x30
//#define ICM42688_I2C_SLV4_ADDR       0x31
//#define ICM42688_I2C_SLV4_REG        0x32
//#define ICM42688_I2C_SLV4_DO         0x33
//#define ICM42688_I2C_SLV4_CTRL       0x34
//#define ICM42688_I2C_SLV4_DI         0x35
//#define ICM42688_I2C_MST_STATUS      0x36
//#define ICM42688_INT_PIN_CFG         0x37
//#define ICM42688_INT_ENABLE          0x38
//#define ICM42688_DMP_INT_STATUS      0x39
//#define ICM42688_INT_STATUS          0x3A

#define ICM42688_ACCEL_CONFIG0       0x50
#define ICM42688_ACCEL_XOUT_1        0x1F
#define ICM42688_ACCEL_XOUT_0        0x20
#define ICM42688_ACCEL_YOUT_1        0x21
#define ICM42688_ACCEL_YOUT_0        0x22
#define ICM42688_ACCEL_ZOUT_1        0x23
#define ICM42688_ACCEL_ZOUT_0        0x24
#define ICM42688_TEMP_OUT_H          0x1D
#define ICM42688_TEMP_OUT_L          0x1E

#define ICM42688_GYRO_CONFIG0        0x4F
#define ICM42688_GYRO_XOUT_1         0x25
#define ICM42688_GYRO_XOUT_0         0x26
#define ICM42688_GYRO_YOUT_1         0x27
#define ICM42688_GYRO_YOUT_0         0x28
#define ICM42688_GYRO_ZOUT_1         0x29
#define ICM42688_GYRO_ZOUT_0         0x2A
//#define ICM42688_EXT_SENS_DATA_00    0x49
//#define ICM42688_EXT_SENS_DATA_01    0x4A
//#define ICM42688_EXT_SENS_DATA_02    0x4B
//#define ICM42688_EXT_SENS_DATA_03    0x4C
//#define ICM42688_EXT_SENS_DATA_04    0x4D
//#define ICM42688_EXT_SENS_DATA_05    0x4E
//#define ICM42688_EXT_SENS_DATA_06    0x4F
//#define ICM42688_EXT_SENS_DATA_07    0x50
//#define ICM42688_EXT_SENS_DATA_08    0x51
//#define ICM42688_EXT_SENS_DATA_09    0x52
//#define ICM42688_EXT_SENS_DATA_10    0x53
//#define ICM42688_EXT_SENS_DATA_11    0x54
//#define ICM42688_EXT_SENS_DATA_12    0x55
//#define ICM42688_EXT_SENS_DATA_13    0x56
//#define ICM42688_EXT_SENS_DATA_14    0x57
//#define ICM42688_EXT_SENS_DATA_15    0x58
//#define ICM42688_EXT_SENS_DATA_16    0x59
//#define ICM42688_EXT_SENS_DATA_17    0x5A
//#define ICM42688_EXT_SENS_DATA_18    0x5B
//#define ICM42688_EXT_SENS_DATA_19    0x5C
//#define ICM42688_EXT_SENS_DATA_20    0x5D
//#define ICM42688_EXT_SENS_DATA_21    0x5E
//#define ICM42688_EXT_SENS_DATA_22    0x5F
//#define ICM42688_EXT_SENS_DATA_23    0x60
//#define ICM42688_I2C_SLV0_DO         0x63
//#define ICM42688_I2C_SLV1_DO         0x64
//#define ICM42688_I2C_SLV2_DO         0x65
//#define ICM42688_I2C_SLV3_DO         0x66
//#define ICM42688_I2C_MST_DELAY_CTRL  0x67
//#define ICM42688_SIGNAL_PATH_RESET   0x68
//#define ICM42688_USER_CTRL           0x6A
#define ICM42688_PWR_MGMT_0          0x4E
//#define ICM42688_PWR_MGMT_2          0x6C
//#define ICM42688_FIFO_COUNTH         0x72
//#define ICM42688_FIFO_COUNTL         0x73
//#define ICM42688_FIFO_R_W            0x74
#define ICM42688_REG_BANK_SEL        0x76
#endif


#define ICM42688_I2C_ADDRESS         0x68    /*!< slave address for ICM42688 sensor */
#define ICM42688_ID                  0x47

// ICM42688 registers
// BANK 0
#define ICM42688_ACCEL_OUT   0x1F
#define ICM42688_GYRO_OUT   0x25
#define ICM42688_TEMP_OUT   0x1D

#define ICM42688_ACCEL_CONFIG0   0x50
#define ICM42688_ACCEL_FS_SEL_2G   0x80  // TODO: 0x60 in datasheet
#define ICM42688_ACCEL_FS_SEL_4G   0x60  // TODO: 0x40 in datasheet
#define ICM42688_ACCEL_FS_SEL_8G   0x40  // TODO: 0x20 in datasheet
#define ICM42688_ACCEL_FS_SEL_16G   0x20 // TODO: 0x00 in datasheet
#define ICM42688_ACCEL_ODR_32KHZ   0x01
#define ICM42688_ACCEL_ODR_16KHZ   0x02
#define ICM42688_ACCEL_ODR_8KHZ   0x03
#define ICM42688_ACCEL_ODR_4KHZ   0x04
#define ICM42688_ACCEL_ODR_2KHZ   0x05
#define ICM42688_ACCEL_ODR_1KHZ   0x06
#define ICM42688_ACCEL_ODR_200HZ   0x07
#define ICM42688_ACCEL_ODR_100HZ   0x08
#define ICM42688_ACCEL_ODR_50HZ   0x09
#define ICM42688_ACCEL_ODR_25HZ   0x0A
#define ICM42688_ACCEL_ODR_12_5HZ   0x0B
#define ICM42688_ACCEL_ODR_6_25HZ   0x0C
#define ICM42688_ACCEL_ODR_3_125HZ   0x0D
#define ICM42688_ACCEL_ODR_1_5625HZ   0x0E
#define ICM42688_ACCEL_ODR_500HZ   0x0F

#define ICM42688_GYRO_CONFIG0   0x4F
#define ICM42688_GYRO_FS_SEL_15_625DPS   0xE0
#define ICM42688_GYRO_FS_SEL_31_25DPS   0xC0
#define ICM42688_GYRO_FS_SEL_62_5DPS   0xA0
#define ICM42688_GYRO_FS_SEL_125DPS   0x80
#define ICM42688_GYRO_FS_SEL_250DPS   0x60
#define ICM42688_GYRO_FS_SEL_500DPS   0x40
#define ICM42688_GYRO_FS_SEL_1000DPS   0x20
#define ICM42688_GYRO_FS_SEL_2000DPS   0x00
#define ICM42688_GYRO_ODR_32KHZ   0x01
#define ICM42688_GYRO_ODR_16KHZ   0x02
#define ICM42688_GYRO_ODR_8KHZ   0x03
#define ICM42688_GYRO_ODR_4KHZ   0x04
#define ICM42688_GYRO_ODR_2KHZ   0x05
#define ICM42688_GYRO_ODR_1KHZ   0x06
#define ICM42688_GYRO_ODR_200HZ   0x07
#define ICM42688_GYRO_ODR_100HZ   0x08
#define ICM42688_GYRO_ODR_50HZ   0x09
#define ICM42688_GYRO_ODR_25HZ   0x0A
#define ICM42688_GYRO_ODR_12_5HZ   0x0B
#define ICM42688_GYRO_ODR_500HZ   0x0F

#define ICM42688_INT_CONFIG  0x14
#define ICM42688_INT_HOLD_ANY  0x08
#define ICM42688_INT_PULSE_100us  0x03
#define ICM42688_INT_SOURCE0  0x65
#define ICM42688_RESET_DONE_INT1_EN  0x10
#define ICM42688_UI_DRDY_INT1_EN  0x10
#define ICM42688_INT_STATUS  0x2D

#define ICM42688_DEVICE_CONFIG  0x11
#define ICM42688_PWR_RESET  0x80
#define ICM42688_INTF_CONFIG1  0x4D
#define ICM42688_CLOCK_SEL_PLL  0x01
#define ICM42688_PWR_MGMT0  0x4E
#define ICM42688_SEN_ENABLE  0x0F

#define ICM42688_WHO_AM_I  0x75
#define ICM42688_FIFO_EN  0x23
#define ICM42688_FIFO_TEMP_EN  0x04
#define ICM42688_FIFO_GYRO  0x02
#define ICM42688_FIFO_ACCEL  0x01
#define ICM42688_FIFO_COUNT  0x2E
#define ICM42688_FIFO_DATA  0x30

#define ICM42688_BANK_SEL  0x76
#define ICM42688_BANK0  0x00
#define ICM42688_BANK1  0x01
#define ICM42688_BANK2  0x02
#define ICM42688_BANK3  0x03
#define ICM42688_BANK4  0x04

    // BANK 1
#define ICM42688_GYRO_CONFIG_STATIC2  0x0B
#define ICM42688_GYRO_NF_ENABLE  0x00
#define ICM42688_GYRO_NF_DISABLE  0x01
#define ICM42688_GYRO_AAF_ENABLE  0x00
#define ICM42688_GYRO_AAF_DISABLE  0x02

    // BANK 2
#define ICM42688_ACCEL_CONFIG_STATIC2  0x03
#define ICM42688_ACCEL_AAF_ENABLE  0x00
#define ICM42688_ACCEL_AAF_DISABLE  0x01


typedef enum {
    ACCE_FS_16G  = 0,     /*!< Accelerometer full scale range is +/- 16g */
    ACCE_FS_8G   = 1,     /*!< Accelerometer full scale range is +/- 8g */
    ACCE_FS_4G   = 2,     /*!< Accelerometer full scale range is +/- 4g */
    ACCE_FS_2G   = 3,     /*!< Accelerometer full scale range is +/- 2g */
} icm42688_acce_fs_t;

typedef enum {
    GYRO_FS_2000DPS  = 0,     /*!< Gyroscope full scale range is +/- 2000 degree per sencond */
    GYRO_FS_1000DPS  = 1,     /*!... */
    GYRO_FS_500DPS = 2,
    GYRO_FS_250DPS = 3,
    GYRO_FS_125DPS = 4,
    GYRO_FS_62_5DPS = 5,
    GYRO_FS_31_25DPS = 6,
    GYRO_FS_15_625DPS = 7,
} icm42688_gyro_fs_t;


typedef enum {
	PWR_TEMP_ON = 0x20,
	PWR_GYRO_STB = 0x04,
	PWR_GYRO_LN  = 0x0C,
	PWR_ACCE_LP  = 0x01,
	PWR_ACCE_LN  = 0x03
} icm42688_pwr_t;



//
//typedef enum{
//	FIFO_EN_XG = 0x40,
//	FIFO_EN_YG = 0x20,
//	FIFO_EN_ZG = 0x10
//
//} icm42688_fifo_en_t;
//
//typedef enum{
//	DLPF_DISABLE = 0,
//	DLPF_1 = 1,
//	DLPF_2 = 2,
//	DLPF_3 = 3,
//	DLPF_4 = 4
//} icm42688_dlpf_config;
//
typedef struct {
    int16_t raw_acce_x;
    int16_t raw_acce_y;
    int16_t raw_acce_z;
} icm42688_raw_acce_value_t;

typedef struct {
    int16_t raw_gyro_x;
    int16_t raw_gyro_y;
    int16_t raw_gyro_z;
} icm42688_raw_gyro_value_t;
//
//typedef struct {
//    float acce_x;
//    float acce_y;
//    float acce_z;
//} ICM42688_acce_value_t;
//
//typedef struct {
//    float gyro_x;
//    float gyro_y;
//    float gyro_z;
//} ICM42688_gyro_value_t;
//
//typedef struct {
//    float roll;
//    float pitch;
//} complimentary_angle_t;





/**
 * @brief Get device identification of ICM42688
 *
 * @param sensor object handle of ICM42688
 * @param deviceid a pointer of device ID
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t icm42688_get_deviceid(uint8_t* deviceid);


esp_err_t icm42688_init(void);  //传感器初始化，验证设备id


/**
 * @brief Wake up ICM42688
 *
 * @param sensor object handle of ICM42688
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t icm42688_pwr_mgmt(icm42688_pwr_t val);


/**
 * @brief Enter sleep mode
 *
 * @param sensor object handle of ICM42688
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
//esp_err_t iot_ICM42688_sleep(ICM42688_handle_t sensor);


/**
 * @brief Set accelerometer full scale range
 *
 * @param sensor object handle of ICM42688
 * @param acce_fs accelerometer full scale range
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t icm42688_set_acce_fs(icm42688_acce_fs_t acce_fs);


/**
 * @brief Set gyroscope full scale range
 *
 * @param sensor object handle of ICM42688
 * @param gyro_fs gyroscope full scale range
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t icm42688_set_gyro_fs(icm42688_gyro_fs_t gyro_fs);


/**
 * @brief Get accelerometer full scale range
 *
 * @param sensor object handle of ICM42688
 * @param acce_fs accelerometer full scale range
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t icm42688_get_acce_fs(icm42688_acce_fs_t *acce_fs);


/**
 * @brief Get gyroscope full scale range
 *
 * @param sensor object handle of ICM42688
 * @param gyro_fs gyroscope full scale range
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t icm42688_get_gyro_fs(icm42688_gyro_fs_t *gyro_fs);


/**
 * @brief Get accelerometer sensitivity
 *
 * @param sensor object handle of ICM42688
 * @param acce_sensitivity accelerometer sensitivity
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
//esp_err_t iot_ICM42688_get_acce_sensitivity(ICM42688_handle_t sensor, float *acce_sensitivity);


/**
 * @brief Get gyroscope sensitivity
 *
 * @param sensor object handle of ICM42688
 * @param gyro_sensitivity gyroscope sensitivity
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
//esp_err_t iot_ICM42688_get_gyro_sensitivity(icm42688_handle_t sensor, float *gyro_sensitivity);


/**
 * @brief Read raw accelerometer measurements
 *
 * @param sensor object handle of ICM42688
 * @param acce_value raw accelerometer measurements
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t icm42688_get_raw_acce(icm42688_raw_acce_value_t *raw_acce_value);


/**
 * @brief Read raw gyroscope measurements
 *
 * @param sensor object handle of ICM42688
 * @param gyro_value raw gyroscope measurements
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t icm42688_get_raw_gyro(icm42688_raw_gyro_value_t *raw_gyro_value);


/**
 * @brief Read accelerometer measurements
 *
 * @param sensor object handle of ICM42688
 * @param acce_value accelerometer measurements
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
//esp_err_t ICM42688_get_acce(icm42688_handle_t sensor, icm42688_acce_value_t *acce_value);

/**
 * @brief Read gyro values
 *
 * @param sensor object handle of ICM42688
 * @param gyro_value gyroscope measurements
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
//esp_err_t ICM42688_get_gyro(icm42688_handle_t sensor, icm42688_gyro_value_t *gyro_value);

/**
 * @brief use complimentory filter to caculate roll and pitch
 *
 * @param acce_value accelerometer measurements
 * @param gyro_value gyroscope measurements
 * @param complimentary_angle complimentary angle
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
//esp_err_t ICM42688_complimentory_filter(ICM42688_handle_t sensor, ICM42688_acce_value_t *acce_value,
//                        ICM42688_gyro_value_t *gyro_value, complimentary_angle_t *complimentary_angle);


//esp_err_t ICM42688_get_fifo_data(ICM42688_handle_t sensor, size_t len, uint8_t *buffer);
//
//
//esp_err_t ICM42688_get_fifo_len(ICM42688_handle_t sensor, uint16_t *len);
//
//esp_err_t ICM42688_set_dlpf(ICM42688_handle_t sensor, uint8_t flags);
//
//esp_err_t ICM42688_set_smplrt_div(ICM42688_handle_t sensor, uint8_t val);
//
//esp_err_t ICM42688_set_fifo_en(ICM42688_handle_t sensor, uint8_t flags);

esp_err_t icm42688_get_temp(int16_t *temp);



#endif /* ICM42688_H_ */
