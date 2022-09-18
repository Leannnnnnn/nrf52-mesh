/*------------------------------------
 * 
 * 自定义twi（iic）初始化配置和传输函数
 *
 *-----------------------------------*/


#ifndef CUSTOM_TWI_H__
#define CUSTOM_TWI_H__

#include "boards.h"
#include "nrf_drv_twi.h"

/* Logging and RTT */
#include "log.h"
#include "rtt_input.h"


#define CUSTOM_SCL_PIN 44  //P1.12
#define CUSTOM_SDA_PIN 42  //P1.10

void twi_init(void);

bool twi_reg_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t value);  //写寄存器,8位值

bool twi_reg_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *dest, uint8_t length);  //读寄存器值,指定长度

#endif  /* CUSTOM_TWI_H__ */
