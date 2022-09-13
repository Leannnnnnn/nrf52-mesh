/** 
* @file         hx_model_client.c
* @brief        红旭客户端模型
* @details      该客户端模型主要实现在SIG Mesh网络中的自定义数据透传，将数据发送给红旭服务端模型         
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
#include "access_reliable.h"
#include "nrf_mesh_assert.h"

/*
===========================
函数定义
=========================== 
*/

/*****************************************************************************
 * 私有函数
 *****************************************************************************/

/* 接收到Server发送的HX_MODEL_SERVER_OPCODE_STATUS消息时的回调，也就是HX_MODEL_OPCODE_TX的应答消息 */
static void hx_model_opcode_status_cb(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
#if HX_MODEL_CONTROL_ENABLE
    hx_model_control_t* p_control = p_args;
    hx_model_client_t *p_client =  &p_control->client_model;
#else
    hx_model_client_t *p_client = p_args;
#endif      
  NRF_MESH_ASSERT(p_client->p_callbacks->status_handler != NULL);

  /* 应用层处理Server发送过来的HX_MODEL_SERVER_OPCODE_STATUS消息 */
  p_client->p_callbacks->status_handler(p_client, 
                            HX_MODEL_CLIENT_STATUS_SUCCESS, 
                            p_message->p_data,
                            p_message->length, 
                            p_message->meta_data.src.value);
}

/* 接收到Server发送的HX_MODEL_SERVER_OPCODE_RX消息时的回调，也就是HX_MODEL_OPCODE_GET的应答消息 */
static void hx_model_opcode_rx_cb(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
#if HX_MODEL_CONTROL_ENABLE
    hx_model_control_t* p_control = p_args;
    hx_model_client_t *p_client =  &p_control->client_model;
#else
    hx_model_client_t *p_client = p_args;
#endif      
  NRF_MESH_ASSERT(p_client->p_callbacks->rx_handler != NULL);

  /* 应用层处理Server发送过来的HX_MODEL_SERVER_OPCODE_STATUS消息 */
  p_client->p_callbacks->rx_handler(p_client, 
                        HX_MODEL_CLIENT_STATUS_SUCCESS, 
                        p_message->p_data,
                        p_message->length, 
                        p_message->meta_data.src.value);
}

/*****************************************************************************
 * 操作码与其对应的回调函数的映射
 *****************************************************************************/
const access_opcode_handler_t m_client_opcode_handlers[] =
{
  {ACCESS_OPCODE_VENDOR(HX_MODEL_SERVER_OPCODE_STATUS, CLIENT_COMPANY_IDENTIFIER), hx_model_opcode_status_cb},
  {ACCESS_OPCODE_VENDOR(HX_MODEL_SERVER_OPCODE_RX, CLIENT_COMPANY_IDENTIFIER), hx_model_opcode_rx_cb}
};


/* hx_model_client_opcode_tx消息的状态回调函数 */
static void tx_reliable_status_cb(access_model_handle_t model_handle,
                               void *p_args,
                               access_reliable_status_t status)
{
#if HX_MODEL_CONTROL_ENABLE
    hx_model_control_t* p_control = p_args;
    hx_model_client_t *p_client =  &p_control->client_model;
#else
    hx_model_client_t *p_client = p_args;
#endif  
  
  NRF_MESH_ASSERT(p_client->p_callbacks->status_handler != NULL);

  p_client->state.reliable_transfer_active = false;
  switch (status)
  {
  case ACCESS_RELIABLE_TRANSFER_SUCCESS:
    /* Ignore */
    break;
  case ACCESS_RELIABLE_TRANSFER_TIMEOUT:
    p_client->p_callbacks->status_handler(p_client, HX_MODEL_CLIENT_STATUS_ERROR_NO_REPLY, NULL, 0, NRF_MESH_ADDR_UNASSIGNED);
    break;
  case ACCESS_RELIABLE_TRANSFER_CANCELLED:
    p_client->p_callbacks->status_handler(p_client, HX_MODEL_CLIENT_STATUS_CANCELLED, NULL, 0, NRF_MESH_ADDR_UNASSIGNED);
    break;
  default:
    /* Should not be possible. */
    NRF_MESH_ASSERT(false);
    break;
  }
}

/* hx_model_client_opcode_get消息的状态回调函数 */
static void get_reliable_status_cb(access_model_handle_t model_handle,
                               void *p_args,
                               access_reliable_status_t status)
{
#if HX_MODEL_CONTROL_ENABLE
    hx_model_control_t* p_control = p_args;
    hx_model_client_t *p_client =  &p_control->client_model;
#else
    hx_model_client_t *p_client = p_args;
#endif    
  NRF_MESH_ASSERT(p_client->p_callbacks->rx_handler != NULL);

  p_client->state.reliable_transfer_active = false;
  switch (status)
  {
  case ACCESS_RELIABLE_TRANSFER_SUCCESS:
    /* Ignore */
    break;
  case ACCESS_RELIABLE_TRANSFER_TIMEOUT:
    p_client->p_callbacks->rx_handler(p_client, HX_MODEL_CLIENT_STATUS_ERROR_NO_REPLY, NULL, 0, NRF_MESH_ADDR_UNASSIGNED);
    break;
  case ACCESS_RELIABLE_TRANSFER_CANCELLED:
    p_client->p_callbacks->rx_handler(p_client, HX_MODEL_CLIENT_STATUS_CANCELLED, NULL, 0, NRF_MESH_ADDR_UNASSIGNED);
    break;
  default:
    /* Should not be possible. */
    NRF_MESH_ASSERT(false);
    break;
  }
}

/* 发送带应答的TX消息 */
static uint32_t tx_reliable_message(const hx_model_client_t * p_client,
                                      hx_model_client_opcode_t opcode,
                                      const uint8_t * p_data,
                                      uint16_t length)
{
  access_reliable_t reliable;
  reliable.model_handle = p_client->model_handle;
  reliable.message.p_buffer = p_data;
  reliable.message.length = length;
  reliable.message.opcode.opcode = opcode;
  reliable.message.opcode.company_id = CLIENT_COMPANY_IDENTIFIER;

  /* 不强制分包，协议栈会自动处理是不是分包 */
  reliable.message.force_segmented = false;
  reliable.message.transmic_size = NRF_MESH_TRANSMIC_SIZE_DEFAULT;
  reliable.message.access_token = nrf_mesh_unique_token_get();
  reliable.reply_opcode.opcode = HX_MODEL_SERVER_OPCODE_STATUS;
  reliable.reply_opcode.company_id = CLIENT_COMPANY_IDENTIFIER;
  reliable.timeout = HX_MODEL_CLIENT_ACKED_TRANSACTION_TIMEOUT;
  reliable.status_cb = tx_reliable_status_cb;

  return access_model_reliable_publish(&reliable);
}

/* 发送带应答的GET消息 */
static uint32_t get_reliable_message(const hx_model_client_t * p_client,
                                      hx_model_client_opcode_t opcode)
{
  access_reliable_t reliable;
  reliable.model_handle = p_client->model_handle;
  reliable.message.p_buffer = NULL;
  reliable.message.length = 0;
  reliable.message.opcode.opcode = opcode;
  reliable.message.opcode.company_id = CLIENT_COMPANY_IDENTIFIER;
  reliable.message.force_segmented = false;
  reliable.message.transmic_size = NRF_MESH_TRANSMIC_SIZE_DEFAULT;
  reliable.message.access_token = nrf_mesh_unique_token_get();
  reliable.reply_opcode.opcode = HX_MODEL_SERVER_OPCODE_RX;
  reliable.reply_opcode.company_id = CLIENT_COMPANY_IDENTIFIER;
  reliable.timeout = HX_MODEL_CLIENT_ACKED_TRANSACTION_TIMEOUT;
  reliable.status_cb = get_reliable_status_cb;

  return access_model_reliable_publish(&reliable);
}

/*****************************************************************************
 * 公开的函数
 *****************************************************************************/

/**
 * 初始化HX客户端模型.
 *
 * @param[in] p_client         HX客户端模型的结构指针
 * @param[in] element_index    在节点的哪个元素中增加该客户端模型
 *
 * @retval NRF_SUCCESS         模型初始化成功
 * @retval NRF_ERROR_NULL      传进形参的实参是空指针
 * @retval NRF_ERROR_NO_MEM    新添模型时，内存不足
 * @retval NRF_ERROR_FORBIDDEN 每个元素不允许多个相同模型实例存在
 * @retval NRF_ERROR_NOT_FOUND 无效的元素索引值
 *
 */
uint32_t hx_model_client_init(hx_model_client_t *p_client, uint16_t element_index)
{
  if (p_client == NULL ||
      p_client->p_callbacks->rx_handler == NULL ||
      p_client->p_callbacks->status_handler == NULL )
  {
    return NRF_ERROR_NULL;
  }

  access_model_add_params_t init_params;
  init_params.model_id.model_id = HX_MODEL_CLIENT_ID;
  init_params.model_id.company_id = CLIENT_COMPANY_IDENTIFIER;
  init_params.element_index = element_index;
  init_params.p_opcode_handlers = &m_client_opcode_handlers[0];
  init_params.opcode_count = sizeof(m_client_opcode_handlers) / sizeof(m_client_opcode_handlers[0]);
  init_params.p_args = p_client;
  /* 因为我们用的非二值信号量的Vendor模型，所以暂不支持Publication功能 */
  init_params.publish_timeout_cb = NULL;
  return access_model_add(&init_params, &p_client->model_handle);
}

/**
 * 发送自定义的透传数据给红旭的服务端模型（带应答）
 *
 * @param[in,out] p_client HX客户端模型的结构指针
 * @param[in]     p_data   指向透传数据的指针
 * @param[in]     length   透传数据的总长度
 *
 * @retval NRF_SUCCESS              透传数据发送成功
 * @retval NRF_ERROR_NULL           指针为空
 * @retval NRF_ERROR_NO_MEM         内存空间不足
 * @retval NRF_ERROR_NOT_FOUND      模型句柄无效
 * @retval NRF_ERROR_INVALID_ADDR   元素索引值无效
 * @retval NRF_ERROR_INVALID_STATE  透传数据正在传输中，此时状态无效
 * @retval NRF_ERROR_INVALID_PARAM  对应的模型没有绑定appkey或者没有配置发布地址或者操作码格式不正确
 * 
 */
uint32_t hx_model_client_opcode_tx(hx_model_client_t *p_client, uint8_t *p_data, uint16_t length)
{
  if (p_client == NULL || p_client->p_callbacks->status_handler == NULL)
  {
    return NRF_ERROR_NULL;
  }
  else if (p_client->state.reliable_transfer_active)
  {
    return NRF_ERROR_INVALID_STATE;
  }

  uint32_t status = tx_reliable_message(p_client,
                                        HX_MODEL_OPCODE_TX,
                                        p_data,
                                        length);
  if (status == NRF_SUCCESS)
  {
    p_client->state.reliable_transfer_active = true;
  }
  return status;
}

/**
 * 发送自定义的透传数据给红旭的服务端模型（不带应答）
 *
 * @param[in,out] p_client HX客户端模型的结构指针
 * @param[in]     p_data   指向透传数据的指针
 * @param[in]     length   透传数据的总长度
 * @param[in]     repeats  发送该消息时的次数，因为没有应答所以需要多发几次
 *
 * @retval NRF_SUCCESS              透传数据发送成功
 * @retval NRF_ERROR_NULL           指针为空
 * @retval NRF_ERROR_NO_MEM         内存空间不足
 * @retval NRF_ERROR_NOT_FOUND      模型句柄无效
 * @retval NRF_ERROR_INVALID_ADDR   元素索引值无效
 * @retval NRF_ERROR_INVALID_PARAM  对应的模型没有绑定appkey或者没有配置发布地址或者操作码格式不正确
 * 
 */
uint32_t hx_model_client_opcode_tx_unreliable(hx_model_client_t *p_client, uint8_t *p_data, uint16_t length, uint8_t repeats)
{
  access_message_tx_t message;
  message.opcode.opcode = HX_MODEL_OPCODE_TX_UNRELIABLE;
  message.opcode.company_id = CLIENT_COMPANY_IDENTIFIER;
  message.p_buffer = p_data;
  message.length = length;

  /* 不强制分包，协议栈会自动处理是不是分包 */
  message.force_segmented = false;
  message.transmic_size = NRF_MESH_TRANSMIC_SIZE_DEFAULT;

  uint32_t status = NRF_SUCCESS;
  for (uint8_t i = 0; i < repeats; ++i)
  {
    message.access_token = nrf_mesh_unique_token_get();
    status = access_model_publish(p_client->model_handle, &message);
    if (status != NRF_SUCCESS)
    {
      break;
    }
  }
  return status;
}

/**
 * 获取红旭服务端模型收到的透传数据
 *
 * @note 红旭服务端模型将回复其上一次收到的透传数据
 *
 * @param[in,out] p_client HX客户端模型的结构指针
 *
 * @retval NRF_SUCCESS              GET消息被发送成功
 * @retval NRF_ERROR_NULL           指针为空
 * @retval NRF_ERROR_NO_MEM         内存空间不足
 * @retval NRF_ERROR_NOT_FOUND      模型句柄无效
 * @retval NRF_ERROR_INVALID_ADDR   元素索引值无效
 * @retval NRF_ERROR_INVALID_STATE  透传数据正在传输中，此时状态无效
 * @retval NRF_ERROR_INVALID_PARAM  对应的模型没有绑定appkey或者没有配置发布地址或者操作码格式不正确
 * 
 */
uint32_t hx_model_client_opcode_get(hx_model_client_t *p_client)
{
  if (p_client == NULL || p_client->p_callbacks->rx_handler == NULL)
  {
    return NRF_ERROR_NULL;
  }
  else if (p_client->state.reliable_transfer_active)
  {
    return NRF_ERROR_INVALID_STATE;
  }

  uint32_t status = get_reliable_message(p_client,
                                        HX_MODEL_OPCODE_GET);
  if (status == NRF_SUCCESS)
  {
    p_client->state.reliable_transfer_active = true;
  }
  return status;
}

/**
 * 停止正在发送带应答的消息传输
 *
 * @param[in,out] p_client HX客户端模型的结构指针
 */
void hx_model_client_pending_msg_cancel(hx_model_client_t *p_client)
{
  (void)access_model_reliable_cancel(p_client->model_handle);
}