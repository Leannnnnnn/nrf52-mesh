/** 
* @file         hx_model_server.h
* @brief        红旭客户端模型相关的宏定义以及函数声明
* @details      声明红旭客户端模型所需的宏定义、枚举变量定义、回调处理函数声明、相关结构体的定义等等
* @author       临时工
* @par Copyright (c):  
*               红旭无线开发团队
* @par Website:  
*               www.wireless-tech.cn
* @par History:          
*               Ver0.0.1:
                     临时工, 2020/05/19, 初始化版本\n 
*/

#ifndef HX_MODEL_CLIENT_H__
#define HX_MODEL_CLIENT_H__

/*
===========================
头文件包含
=========================== 
*/
#include <stdint.h>
#include <stdbool.h>
#include "access.h"

/*
===========================
宏定义
=========================== 
*/

/* 操作码的定义 */
#define HX_MODEL_SERVER_OPCODE_STATUS              0x06|0xC0     /* 应答收到的透传自定义数据 */
#define HX_MODEL_SERVER_OPCODE_RX                  0x52|0xC0     /* 接收上一次发送的透传自定义数据 */

/* 标识符 */
#define CLIENT_COMPANY_IDENTIFIER                  0x0059        /* Bluetooth SIG分配给Nordic的Company identifier */
#define HX_MODEL_CLIENT_ID                         0x0001        /* HX客户端模型的Vendor-assigned Model ID */

/* 红旭的客户端模型的个数 */
#define HX_CLIENT_MODEL_INSTANCE_COUNT                (1)
/* 红旭的客户端操作码的个数 */
#define CLIENT_MODEL_OPCODE_COUNT                  (2)

/** 应答消息的超时时间 */
#ifndef HX_MODEL_CLIENT_ACKED_TRANSACTION_TIMEOUT
#define HX_MODEL_CLIENT_ACKED_TRANSACTION_TIMEOUT  (SEC_TO_US(25))
#endif

/*
===========================
外部变量声明
=========================== 
*/
extern const access_opcode_handler_t m_client_opcode_handlers[CLIENT_MODEL_OPCODE_COUNT];

/*
===========================
枚举变量定义
=========================== 
*/

/* 红旭客户端操作码 */
typedef enum
{
  HX_MODEL_OPCODE_TX            = 0x54|0xC0,                     /**< 透传自定义数据，带应答 */
  HX_MODEL_OPCODE_TX_UNRELIABLE = 0x74|0xC0,                     /**< 透传自定义数据，不带应答 */
  HX_MODEL_OPCODE_GET           = 0x47|0xC0,                     /**< 查询上一次透传的自定义数据 */
} hx_model_client_opcode_t;

/** 红旭客户端模型的状态码 */
typedef enum
{
  /** 透传数据发送成功 */
  HX_MODEL_CLIENT_STATUS_SUCCESS,
  /** HX服务端模型在规定的时间内没有应答客户端的TX和GET消息 */
  HX_MODEL_CLIENT_STATUS_ERROR_NO_REPLY,
  /** HX客户端的TX/GET操作码被停止执行 */
  HX_MODEL_CLIENT_STATUS_CANCELLED
} hx_model_client_status_t;

/*
===========================
结构体变量定义
=========================== 
*/
typedef struct hx_model_client hx_model_client_t;

/*
===========================
函数指针声明
=========================== 
*/
typedef void (*hx_model_opcode_status_handler_t)(const hx_model_client_t *p_self,
                                                 hx_model_client_status_t status,
                                                 const uint8_t *p_data,
                                                 uint16_t length,
                                                 uint16_t src);

typedef void (*hx_model_opcode_rx_handler_t)(const hx_model_client_t *p_self,
                                             hx_model_client_status_t status,
                                             const uint8_t *p_data,
                                             uint16_t length,
                                             uint16_t src);

typedef void (*hx_model_client_timeout_handler_t)(access_model_handle_t handle, void *p_self);

/*
===========================
结构体定义
=========================== 
*/
typedef struct
{
  /* 红旭服务端模型的STATUS操作码，所携带的消息处理函数 */
  hx_model_opcode_status_handler_t status_handler;
  /* 红旭服务端模型的RX操作码，所携带的消息处理函数 */
  hx_model_opcode_rx_handler_t rx_handler;
  /* 红旭客户端模型Publication的超时处理函数 */
  hx_model_client_timeout_handler_t timeout_handler;
} hx_client_model_callbacks_t;



struct hx_model_client
{
  access_model_handle_t model_handle;
  const hx_client_model_callbacks_t* p_callbacks;
  struct
  {
    bool reliable_transfer_active;
  } state;
};



/*
===========================
函数声明
=========================== 
*/

/**
 * 初始化HX客户端模型.
 *
 *
 * @param[in] p_client      HX客户端模型的结构指针
 * @param[in] element_index 在节点的哪个元素中增加该客户端模型
 *
 * @retval NRF_SUCCESS         模型初始化成功
 * @retval NRF_ERROR_NULL      传进形参的实参是空指针
 * @retval NRF_ERROR_NO_MEM    新添模型时，内存不足
 * @retval NRF_ERROR_FORBIDDEN 每个元素不允许多个相同模型实例存在
 * @retval NRF_ERROR_NOT_FOUND 无效的元素索引值
 *
 */
uint32_t hx_model_client_init(hx_model_client_t * p_client, uint16_t element_index);

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
uint32_t hx_model_client_opcode_tx(hx_model_client_t * p_client, uint8_t* p_data, uint16_t length);

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
uint32_t hx_model_client_opcode_tx_unreliable(hx_model_client_t * p_client, uint8_t* p_data, uint16_t length, uint8_t repeats);

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
uint32_t hx_model_client_opcode_get(hx_model_client_t * p_client);

/**
 * 停止正在发送带应答的消息传输
 *
 * @param[in,out] p_client HX客户端模型的结构指针
 */
void hx_model_client_pending_msg_cancel(hx_model_client_t * p_client);


#endif //HX_MODEL_CLIENT_H__