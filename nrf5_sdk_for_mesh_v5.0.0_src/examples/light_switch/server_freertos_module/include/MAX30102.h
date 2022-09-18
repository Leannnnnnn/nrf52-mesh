
/*----------------------------------------------
 * 
 * MAX30205心率血氧传感器驱动程序，基于IIC传输
 *
 *---------------------------------------------*/


#ifndef MAX30102_H__
#define MAX30102_H__

#include "custom_twi.h"
#include "nrf_delay.h"


#define PIN_INT 46  //P1.14，传感器INT触发引脚

#define MAX30102_I2C_WRITE_ADDR 0x57
#define MAX30102_I2C_READ_ADDR  0x57  //统一为设备地址

#define INTERRUPT_STATUS1 0X00
#define INTERRUPT_STATUS2 0X01
#define INTERRUPT_ENABLE1 0X02
#define INTERRUPT_ENABLE2 0X03

#define FIFO_WR_POINTER 0X04
#define FIFO_OV_COUNTER 0X05
#define FIFO_RD_POINTER 0X06
#define FIFO_DATA 0X07

#define FIFO_CONFIGURATION 0X08
#define MODE_CONFIGURATION 0X09
#define SPO2_CONFIGURATION 0X0A
#define LED1_PULSE_AMPLITUDE 0X0C
#define LED2_PULSE_AMPLITUDE 0X0D

#define MULTILED1_MODE 0X11
#define MULTILED2_MODE 0X12

#define TEMPERATURE_INTEGER 0X1F
#define TEMPERATURE_FRACTION 0X20
#define TEMPERATURE_CONFIG 0X21

#define VERSION_ID 0XFE
#define PART_ID 0XFF
#define EXPECTED_PARTID 0x15


/*------------------------------------移植接口---------------------------------------------------------*/
#define max30102_i2c_write(reg_addr, data)         twi_reg_write(MAX30102_I2C_WRITE_ADDR, reg_addr, data)
#define max30102_i2c_read(reg_addr, pdata, size)   twi_reg_read(MAX30102_I2C_READ_ADDR, reg_addr, pdata, size)
#define delay_ms(ms)                               nrf_delay_ms(ms)
/*----------------------------------------------------------------------------------------------------*/


bool max30102_init(void);
void max30102_fifo_read(float *data);
uint16_t max30102_getHeartRate(float *input_data,uint16_t cache_nums);
float max30102_getSpO2(float *ir_input_data,float *red_input_data,uint16_t cache_nums);

bool hr_flag_check(void);  //查询中断状态

void hr_flag_clear(void);  //清除中断状态


#endif /* MAX30102_H__ */