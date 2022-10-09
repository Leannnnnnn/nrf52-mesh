/*------------------------------------
 * 
 * 自定义twi（iic）初始化配置和传输函数
 *
 *-----------------------------------*/



#include "custom_twi.h"


/* TWI instance ID. */
#if TWI0_ENABLED
#define TWI_INSTANCE_ID     0
#elif TWI1_ENABLED
#define TWI_INSTANCE_ID     1
#endif

static bool m_xfer_done;

/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);


//TWI 事件处理函数
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    //判断TWI 事件类型
    switch (p_event->type)
    {
        //传输完成事件
        case NRF_DRV_TWI_EVT_DONE:
            m_xfer_done = true;//置位传输完成标志
            break;

        default:
            break;
    }
}


/**
 * @brief TWI initialization.
 */
void twi_init(void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_config = {
       .scl                = CUSTOM_SCL_PIN,
       .sda                = CUSTOM_SDA_PIN,
       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_LOW,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_config, NULL/*twi_handler*/, NULL);  //无回调函数，工作在阻塞模式
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "TWI0 initiated.\n");

}

bool twi_reg_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t value)  //写寄存器,8位值
{
    ret_code_t err_code;
    uint8_t tx_buff[2];

    tx_buff[0] = reg_addr;
    tx_buff[1] = value;

    m_xfer_done = false;

    err_code = nrf_drv_twi_tx(&m_twi, dev_addr, tx_buff, 2, false);   //发送，不开启no_stop
    //while(m_xfer_done == false){}
    if(NRF_SUCCESS != err_code){
        return false;
    }
    return true;
}

bool twi_reg_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *dest, uint8_t length)  //读寄存器值
{
    ret_code_t err_code;
    m_xfer_done = false;

    
    err_code = nrf_drv_twi_tx(&m_twi, dev_addr, &reg_addr, 1, true);   //发送，开启no_stop
    //while (m_xfer_done == false){}
    if(NRF_SUCCESS != err_code){
        return false;
    }

    m_xfer_done = false;

    err_code = nrf_drv_twi_rx(&m_twi, dev_addr, dest, length);
    //while (m_xfer_done == false){}
    if(NRF_SUCCESS != err_code){
        return false;
    }

    return true;
}