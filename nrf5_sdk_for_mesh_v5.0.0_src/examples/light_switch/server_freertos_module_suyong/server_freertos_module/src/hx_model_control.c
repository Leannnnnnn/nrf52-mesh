/** 
* @file         hx_model_control.c
* @brief        红旭控制端模型
* @details      该模型同时实现client和server模型的功能，即可以同时处理client和server模型的消息
* @author       临时工
* @par Copyright (c):  
*               红旭无线开发团队
* @par Website:  
*               www.wireless-tech.cn
* @par History:          
*               Ver0.0.1:
                    临时工, 2020/06/03, 初始化版本\n 
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
#include <stdlib.h>

/*
===========================
变量定义
=========================== 
*/



/*
===========================
函数定义
=========================== 
*/

/*****************************************************************************
 * 私有函数
 *****************************************************************************/



/*****************************************************************************
 * 操作码与其对应的回调函数的映射
 *****************************************************************************/
static access_opcode_handler_t m_control_opcode_handlers[CONTROL_MODEL_OPCODE_COUNT];

/*****************************************************************************
 * 公开的函数
 *****************************************************************************/

/**
 * 初始化HX控制端模型.
 *
 *
 * @param[in] p_server      HX控制端模型的结构指针
 * @param[in] element_index 在节点的哪个元素中增加该服务端模型
 *
 * @retval NRF_SUCCESS         模型初始化成功
 * @retval NRF_ERROR_NULL      传进形参的实参是空指针
 * @retval NRF_ERROR_NO_MEM    新添模型时，内存不足
 * @retval NRF_ERROR_FORBIDDEN 每个元素不允许多个模型实例存在
 * @retval NRF_ERROR_NOT_FOUND 无效的元素索引值
 */
uint32_t hx_model_control_init(hx_model_control_t* p_control,uint16_t element_index)
{
  if (p_control == NULL || 
      p_control->server_model.p_callbacks->tx_handler == NULL ||
      p_control->server_model.p_callbacks->get_handler == NULL || 
      p_control->client_model.p_callbacks->status_handler == NULL ||
      p_control->client_model.p_callbacks->rx_handler == NULL)
  {
    return NRF_ERROR_NULL;
  }

  /* 操作码与回调函数填充 */  
  for(uint8_t i =0;i< SERVER_MODEL_OPCODE_COUNT;i++)
  {
    m_control_opcode_handlers[i].opcode  = m_server_opcode_handlers[i].opcode;
    m_control_opcode_handlers[i].handler = m_server_opcode_handlers[i].handler;
  }

  for(uint8_t i =0;i< CLIENT_MODEL_OPCODE_COUNT;i++)
  {
    m_control_opcode_handlers[i+SERVER_MODEL_OPCODE_COUNT].opcode  = m_client_opcode_handlers[i].opcode;
    m_control_opcode_handlers[i+SERVER_MODEL_OPCODE_COUNT].handler = m_client_opcode_handlers[i].handler;
  }

  access_model_add_params_t init_params;
  init_params.element_index = element_index;
  init_params.model_id.model_id = HX_MODEL_CONTROL_ID;
  init_params.model_id.company_id = SERVER_COMPANY_IDENTIFIER;
  init_params.p_opcode_handlers = m_control_opcode_handlers;
  init_params.opcode_count = CONTROL_MODEL_OPCODE_COUNT;
  init_params.p_args = p_control;
  /* 因为我们用的非二值信号量的Vendor模型，所以暂不支持Publication功能 */
  init_params.publish_timeout_cb = NULL;
  uint32_t status = access_model_add(&init_params, &p_control->model_handle);
  if (status == NRF_SUCCESS)
  {
    p_control->client_model.model_handle = p_control->model_handle;
    p_control->server_model.model_handle = p_control->model_handle;
    status = access_model_subscription_list_alloc(p_control->model_handle);
  }
  return status;
}