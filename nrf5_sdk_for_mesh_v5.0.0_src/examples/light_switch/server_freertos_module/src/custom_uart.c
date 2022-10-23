
/*-----------------------------------
 * 
 * 自定义uart串口初始化配置和传输函数
 *
 *----------------------------------*/



#include <string.h>
#include "boards.h"

#include "custom_uart.h"
#include "nrf_uart.h"
#include "sdk_errors.h"
#include "app_error.h"
#include "app_uart.h"

/* Logging and RTT */
#include "log.h"
#include "rtt_input.h"

void uart_evt_handle(app_uart_evt_t * p_event)  //串口事件回调函数
{

    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
    
}

void uart_init(void)
{
    uint32_t err_code;
    const app_uart_comm_params_t comm_params =
      {
          RX_PIN_NUMBER,
          TX_PIN_NUMBER,
          RTS_PIN_NUMBER,
          CTS_PIN_NUMBER,
          APP_UART_FLOW_CONTROL_DISABLED,  //流控制使能，需要开启串口RTS
          false, //parity,奇偶校验
          UART_BAUDRATE_BAUDRATE_Baud115200
      };

    APP_UART_FIFO_INIT(&comm_params,
                         16,
                         256,
                         uart_evt_handle,
                         APP_IRQ_PRIORITY_LOW_MID,
                         err_code);
    APP_ERROR_CHECK(err_code);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Uart init successfully! \n");
}

void uart_send_str(char *str)  //发送字符串
{
    uint16_t len = strlen(str);
    uint16_t i = 0;
    for(i = 0; i< len; i++){
        while (app_uart_put(str[i]) != NRF_SUCCESS);    
    }
} 

void uart_send_int(int16_t num)  //发送整形数字
{
    char buffer[10] = { 0 };
    sprintf(buffer, "%d", num);
    uart_send_str(buffer);
}