
/*-----------------------------------------
 * 
 * MAX30205温度传感器驱动程序，基于IIC传输
 *
 *----------------------------------------*/



#ifndef MAX30205_TEMP_DRV_H__
#define MAX30205_TEMP_DRV_H__


#include "custom_twi.h"

#define TEMP_DEV_ADDR     0x49    //max30205设备地址(7位LSB)
#define TEMP_CONFIG_ADDR  0x01    //config寄存器地址
#define TEMP_DATA_ADDR    0x00    //temp_data寄存器地址（16位数据）
#define TEMP_THYST_ADDR   0x02  //
#define TEMP_TOS_ADDR     0x03  

#define TEMP_CONFIG_SHUTDOWN  0x01 //配置shutdown模式
#define TEMP_CONFIG_ONESHOT   0x80 //启动单次采集


void temp_init(void);

float temp_read(void);

#endif  /*MAX30205_TEMP_DRV_H__*/