
/*-----------------------------------------
 * 
 * MAX30205温度传感器驱动程序，基于IIC传输
 *
 *----------------------------------------*/



#include "MAX30205_temp_drv.h"
#include "nrf_delay.h"

void temp_init(void)
{
    bool ret;
     
    ret = twi_reg_write(TEMP_DEV_ADDR, TEMP_CONFIG_ADDR, TEMP_CONFIG_SHUTDOWN);  //配置为shutdown模式
    
    ret = twi_reg_write(TEMP_DEV_ADDR, TEMP_TOS_ADDR, 0x00);

    ret = twi_reg_write(TEMP_DEV_ADDR, TEMP_THYST_ADDR, 0x00);

    if(true == ret){
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Temperature sensor initiated\n");
    }
}


float temp_read(void)
{
    bool ret;
    uint8_t rx_buff[2];
    float temp;

    uint8_t config_get;

    ret = twi_reg_write(TEMP_DEV_ADDR, TEMP_CONFIG_ADDR, (TEMP_CONFIG_SHUTDOWN|TEMP_CONFIG_ONESHOT));  //触发oneshot采集温度
    if(true == ret){
        //__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Start conversion temperature\n");
    }
    nrf_delay_ms(50);

    ret = twi_reg_read(TEMP_DEV_ADDR, TEMP_CONFIG_ADDR, &config_get, 1);

    //__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Configuration: 0x%02x\n", config_get);

    ret = twi_reg_read(TEMP_DEV_ADDR, TEMP_DATA_ADDR, rx_buff, 2);

    if(true == ret){
        //__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Convert temperature successfully\n");
    }

    //__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Temperature: 0x%02x 0x%02x\n",rx_buff[0], rx_buff[1]);

    int16_t raw = rx_buff[0] << 8 | rx_buff[1];  //combine two bytes
    temp = raw  * 0.00390625;     // convert to temperature
    return temp;
}