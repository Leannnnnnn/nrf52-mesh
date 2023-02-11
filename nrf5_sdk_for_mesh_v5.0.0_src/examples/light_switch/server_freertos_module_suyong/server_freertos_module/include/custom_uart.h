
/*---------------------------------
 * 
 * 自定义uart串口初始化配置和传输函数
 *
 *-------------------------------*/


#ifndef CUSTOM_H__
#define CUSTOM_H__



#include <stdint.h>

void uart_init(void);
void uart_send_str(char *str);  //发送字符串
void uart_send_int(int16_t num);  //发送整形数字




#endif  /* CUSTOM_H__ */