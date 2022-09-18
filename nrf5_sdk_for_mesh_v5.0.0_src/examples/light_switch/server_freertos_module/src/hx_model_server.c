/** 
* @file         hx_model_client.c
* @brief        红旭服务端模型
* @details      该服务端模型主要实现在SIG Mesh网络中的自定义数据透传，
                接收Client发送的自定义数据或者来自于Provisioner的数据
* @author       临时工
* @par Copyright (c):  
*               红旭无线开发团队
* @par Website:  
*               www.wireless-tech.cn
* @par History:          
*               Ver0.0.1:
                    临时工, 2020/05/19, 初始化版本\n 
*/
/*
===========================
头文件包含
=========================== 
*/
#include "hx_model_control.h"
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "access.h"
#include "access_config.h"
#include "nrf_mesh_assert.h"

/*
===========================
变量定义
=========================== 
*/

/* 暂存TX或者TX_unreliable消息发送过来的数据及其长度 */
static uint16_t data_length = 0; 
static uint8_t message_from_opcode_tx[377];

/*
===========================
函数定义
=========================== 
*/

/*****************************************************************************
 * 私有函数
 *****************************************************************************/

/* HX_MODEL_CLIENT_OPCODE_TX消息的应答 */
static void tx_ack(const hx_model_server_t *p_server,
                   const access_message_rx_t *p_message)
{
  /* 应答的内容 */
  uint8_t acknowledgment[] = "OK!";
  access_message_tx_t reply;
  reply.opcode.opcode = HX_MODEL_OPCODE_STATUS;
  reply.opcode.company_id = SERVER_COMPANY_IDENTIFIER;
  reply.p_buffer = acknowledgment;
  reply.length = sizeof(acknowledgment) / sizeof(acknowledgment[0]);
  reply.force_segmented = false;
  reply.transmic_size = NRF_MESH_TRANSMIC_SIZE_DEFAULT;
  reply.access_token = nrf_mesh_unique_token_get();

  (void)access_model_reply(p_server->model_handle, p_message, &reply);  //调用处理回调函数+应答
}

/* HX_MODEL_CLIENT_OPCODE_GET消息的应答 */
static void get_reply(const hx_model_server_t *p_server,
                      const access_message_rx_t *p_message)
{
  access_message_tx_t reply;
  reply.opcode.opcode = HX_MODEL_OPCODE_RX;
  reply.opcode.company_id = SERVER_COMPANY_IDENTIFIER;
  if (data_length == 0)
  {
    reply.p_buffer = NULL;
    reply.length = 0;
  }
  else
  {
    reply.p_buffer = message_from_opcode_tx;
    reply.length = data_length;
  }

  /* 这里不强制分包，协议栈会自动处理是否分包 */
  reply.force_segmented = false;
  reply.transmic_size = NRF_MESH_TRANSMIC_SIZE_DEFAULT;
  reply.access_token = nrf_mesh_unique_token_get();

  (void)access_model_reply(p_server->model_handle, p_message, &reply);
}

/* HX_MODEL_OPCODE_TX操作码回调处理函数 */
static void hx_model_opcode_tx_cb(access_model_handle_t handle,
                                  const access_message_rx_t *p_message,
                                  void *p_args)
{ 
#if HX_MODEL_CONTROL_ENABLE
    hx_model_control_t* p_control = p_args;
    hx_model_server_t *p_server =  &p_control->server_model;
#else
    hx_model_server_t *p_server = p_args;
#endif
  
  NRF_MESH_ASSERT(p_server->p_callbacks->tx_handler!= NULL);
  data_length = p_message->length;
  memcpy(message_from_opcode_tx,p_message->p_data, data_length);
  /* 根据操作码携带的消息，执行相对应的操作 */
  p_server->p_callbacks->tx_handler(p_server, p_message->p_data, data_length); 

  tx_ack(p_server, p_message);

}

/* HX_MODEL_OPCODE_GET操作码回调处理函数 */
static void hx_model_opcode_get_cb(access_model_handle_t handle,
                                   const access_message_rx_t *p_message,
                                   void *p_args)
{
#if HX_MODEL_CONTROL_ENABLE
    hx_model_control_t* p_control = p_args;
    hx_model_server_t *p_server =  &p_control->server_model;
#else
    hx_model_server_t *p_server = p_args;
#endif    
  NRF_MESH_ASSERT(p_server->p_callbacks->get_handler != NULL);
  p_server->p_callbacks->get_handler(p_server);
  get_reply(p_server, p_message);
}

/* HX_MODEL_OPCODE_TX_UNRELIABLE操作码回调处理函数 */
static void hx_model_opcode_tx_unreliable_cb(access_model_handle_t handle,
                                             const access_message_rx_t *p_message,
                                             void *p_args)
{
#if HX_MODEL_CONTROL_ENABLE
    hx_model_control_t* p_control = p_args;
    hx_model_server_t *p_server =  &p_control->server_model;
#else
    hx_model_server_t *p_server = p_args;
#endif    
  NRF_MESH_ASSERT(p_server->p_callbacks->tx_handler != NULL);
  data_length = p_message->length;
  memcpy(message_from_opcode_tx,p_message->p_data, data_length);
  /* 根据操作码携带的消息，执行相对应的操作 */
  p_server->p_callbacks->tx_handler(p_server, p_message->p_data, data_length);         
}

/*****************************************************************************
 * 操作码与其对应的回调函数的映射
 *****************************************************************************/
const access_opcode_handler_t m_server_opcode_handlers[] =
{
  {ACCESS_OPCODE_VENDOR(HX_MODEL_CLIENT_OPCODE_TX, SERVER_COMPANY_IDENTIFIER), hx_model_opcode_tx_cb},
  {ACCESS_OPCODE_VENDOR(HX_MODEL_CLIENT_OPCODE_GET, SERVER_COMPANY_IDENTIFIER), hx_model_opcode_get_cb},
  {ACCESS_OPCODE_VENDOR(HX_MODEL_CLIENT_OPCODE_TX_UNRELIABLE, SERVER_COMPANY_IDENTIFIER), hx_model_opcode_tx_unreliable_cb}
};

/*****************************************************************************
 * 公开的函数
 *****************************************************************************/

/**
 * 初始化HX服务端模型并分配订阅列表.
 *
 *
 * @param[in] p_server      HX服务端模型的结构体指针
 * @param[in] element_index 在节点的哪个元素中增加该服务端模型
 *
 * @retval NRF_SUCCESS         模型初始化成功
 * @retval NRF_ERROR_NULL      传进形参的实参是空指针
 * @retval NRF_ERROR_NO_MEM    新添模型时，内存不足
 * @retval NRF_ERROR_FORBIDDEN 每个元素不允许多个模型实例存在
 * @retval NRF_ERROR_NOT_FOUND 无效的元素索引值
 */
uint32_t hx_model_server_init(hx_model_server_t *p_server, uint16_t element_index)
{
  if (p_server == NULL ||
      p_server->p_callbacks->tx_handler == NULL ||
      p_server->p_callbacks->get_handler == NULL)
  {
    return NRF_ERROR_NULL;
  }

  access_model_add_params_t init_params;
  init_params.element_index = element_index;
  init_params.model_id.model_id = HX_MODEL_SERVER_ID;
  init_params.model_id.company_id = SERVER_COMPANY_IDENTIFIER;
  init_params.p_opcode_handlers = &m_server_opcode_handlers[0];
  init_params.opcode_count = sizeof(m_server_opcode_handlers) / sizeof(m_server_opcode_handlers[0]);
  init_params.p_args = p_server;
  /* 因为我们用的非二值信号量的Vendor模型，所以暂不支持Publication功能 */
  init_params.publish_timeout_cb = NULL;
  uint32_t status = access_model_add(&init_params, &p_server->model_handle);
  if (status == NRF_SUCCESS)
  {
    status = access_model_subscription_list_alloc(p_server->model_handle);
  }
  return status;
}