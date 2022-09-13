#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "hx_control_model_create.h"
#include "access_config.h"
#include "nrf_mesh_assert.h"

/* Logging and RTT */
#include "log.h"
#include "rtt_input.h"


/*****************************************************************************
* 红旭SERVER模型的回调处理函数
*****************************************************************************/
static void hx_model_server_tx_cb(const hx_model_server_t * p_self, const uint8_t* p_data,uint8_t length)
{
    uint8_t* message = (uint8_t*)malloc(sizeof(uint8_t)*length);
    memcpy(message,p_data,length);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Received Data: %s, Data Length: %d\n",
                                    message, length);
    free(message);                                       
}


static uint8_t* hx_model_server_get_cb(const hx_model_server_t * p_self)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "HX_MODEL_SERVER_OPCODE_GET Message is Received\n");
    
}


static hx_model_server_t m_hx_model_server[SERVER_MODEL_INSTANCE_COUNT];

hx_server_model_callbacks_t server_callbacks = {
    .get_handler = hx_model_server_get_cb,\
    .tx_handler = hx_model_server_tx_cb
};


/*****************************************************************************
* 红旭CLIENT模型的回调处理函数
*****************************************************************************/
static void hx_model_client_rx_cb(const hx_model_client_t *p_self,
                                            hx_model_client_status_t status,
                                            const uint8_t *p_data,
                                            uint16_t length,
                                            uint16_t src)
{
    uint8_t* message = (uint8_t*)malloc(sizeof(uint8_t)*length);
    memcpy(message,p_data,length);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Received Data: %s, Data Length: %d, Source Address: %x, Status: %d\n",
                                    message, length, src, status);
    free(message);                                       
}



static void hx_model_client_status_cb(const hx_model_client_t *p_self,
                                                hx_model_client_status_t status,
                                                const uint8_t *p_data,
                                                uint16_t length,
                                                uint16_t src)
{
    uint8_t* message = (uint8_t*)malloc(sizeof(uint8_t)*length);
    memcpy(message,p_data,length);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Response from HX SERVER MODEL: %s, length is %d, status is %d, Source Address: %4x\n", 
        message, length, status, src);
    free(message);
}



static hx_model_client_t m_hx_model_client[HX_CLIENT_MODEL_INSTANCE_COUNT];



hx_client_model_callbacks_t client_callbacks = {
    .status_handler = hx_model_client_status_cb,\
    .rx_handler = hx_model_client_rx_cb
};


/*****************************************************************************
* 红旭CONTROL模型的回调处理函数
*****************************************************************************/
static hx_model_control_t m_hx_model_control[HX_CONTROL_MODEL_INSTANCE_COUNT];


//初始化控制模型
/*
static void models_init_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Initializing and adding models\n");
    app_model_init();
    for(uint8_t i = 0; i <HX_CONTROL_MODEL_INSTANCE_COUNT; i++)
    {
        m_hx_model_control[i].server_model.p_callbacks = &server_callbacks;
        m_hx_model_control[i].client_model.p_callbacks = &client_callbacks;
        uint32_t status = hx_model_control_init(&m_hx_model_control[i],i);
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "HX Model Initializing and adding control model[%d] is %d\n",i,status);
    }        
    
}
*/


void hx_models_init(void)
{
    for(uint8_t i = 0; i <HX_CONTROL_MODEL_INSTANCE_COUNT; i++)
    {
        m_hx_model_control[i].server_model.p_callbacks = &server_callbacks;
        m_hx_model_control[i].client_model.p_callbacks = &client_callbacks;
        uint32_t status = hx_model_control_init(&m_hx_model_control[i],i);
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "HX Model Initializing and adding control model[%d] is %d\n",i,status);
    }  
}

void hx_client_tx(uint8_t *str)
{

  uint32_t status = hx_model_client_opcode_tx(&m_hx_model_control[0].client_model, str, strlen((const char*)str));
  __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Transmit data: %d\n",status);

}

void hx_client_tx_int(int16_t num)  //发送整形数字
{
    char buffer[100] = { 0 };
    sprintf(buffer, "%d", num);
    hx_client_tx(buffer);
}