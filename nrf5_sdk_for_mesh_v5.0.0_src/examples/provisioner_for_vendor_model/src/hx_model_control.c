/** 
* @file         hx_model_control.c
* @brief        ������ƶ�ģ��
* @details      ��ģ��ͬʱʵ��client��serverģ�͵Ĺ��ܣ�������ͬʱ����client��serverģ�͵���Ϣ
* @author       ��ʱ��
* @par Copyright (c):  
*               �������߿����Ŷ�
* @par Website:  
*               www.wireless-tech.cn
* @par History:          
*               Ver0.0.1:
                    ��ʱ��, 2020/06/03, ��ʼ���汾\n 
*/
/*
===========================
ͷ�ļ�����
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
��������
=========================== 
*/



/*
===========================
��������
=========================== 
*/

/*****************************************************************************
 * ˽�к���
 *****************************************************************************/



/*****************************************************************************
 * �����������Ӧ�Ļص�������ӳ��
 *****************************************************************************/
static access_opcode_handler_t m_control_opcode_handlers[CONTROL_MODEL_OPCODE_COUNT];

/*****************************************************************************
 * �����ĺ���
 *****************************************************************************/

/**
 * ��ʼ��HX���ƶ�ģ��.
 *
 *
 * @param[in] p_server      HX���ƶ�ģ�͵Ľṹָ��
 * @param[in] element_index �ڽڵ���ĸ�Ԫ�������Ӹ÷����ģ��
 *
 * @retval NRF_SUCCESS         ģ�ͳ�ʼ���ɹ�
 * @retval NRF_ERROR_NULL      �����βε�ʵ���ǿ�ָ��
 * @retval NRF_ERROR_NO_MEM    ����ģ��ʱ���ڴ治��
 * @retval NRF_ERROR_FORBIDDEN ÿ��Ԫ�ز�������ģ��ʵ������
 * @retval NRF_ERROR_NOT_FOUND ��Ч��Ԫ������ֵ
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

  /* ��������ص�������� */  
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
  /* ��Ϊ�����õķǶ�ֵ�ź�����Vendorģ�ͣ������ݲ�֧��Publication���� */
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