/** 
* @file         hx_model_server.h
* @brief        红旭服务端模型相关的宏定义以及函数声明
* @details      声明红旭服务端模型所需的宏定义、枚举变量定义、回调处理函数声明、相关结构体的定义等等
* @author       临时工
* @par Copyright (c):  
*               红旭无线开发团队
* @par Website:  
*               www.wireless-tech.cn
* @par History:          
*               Ver0.0.1:
                     临时工, 2020/05/19, 初始化版本\n 
*/
#ifndef HX_MODEL_SERVER_H__
#define HX_MODEL_SERVER_H__

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
#define HX_MODEL_CLIENT_OPCODE_TX                  0x54|0xC0                /* 透传自定义数据，带应答 */
#define HX_MODEL_CLIENT_OPCODE_TX_UNRELIABLE       0x74|0xC0                /* 透传自定义数据，不带应答 */
#define HX_MODEL_CLIENT_OPCODE_GET                 0x47|0xC0                /* 查询上一次透传的自定义数据 */

/* 标识符 */
#define SERVER_COMPANY_IDENTIFIER                  0x0059                   /* Bluetooth SIG分配给Nordic的Company identifier */
#define HX_MODEL_SERVER_ID                         0x0000                   /* HX服务端模型的Vendor-assigned Model ID */

/* 红旭的服务端操作码的个数 */
#define SERVER_MODEL_OPCODE_COUNT                  (3)

/* 红旭的服务端模型的个数 */
#define SERVER_MODEL_INSTANCE_COUNT                (1)
/*
===========================
外部变量声明
=========================== 
*/
extern const access_opcode_handler_t m_server_opcode_handlers[SERVER_MODEL_OPCODE_COUNT];
/*
===========================
枚举变量定义
=========================== 
*/

/* 红旭客户端操作码 */
typedef enum
{
  HX_MODEL_OPCODE_STATUS            = 0x06|0xC0,                            /**< 应答HX_MODEL_CLIENT_OPCODE_TX的消息 */
  HX_MODEL_OPCODE_RX                = 0x52|0xC0,                            /**< 接应答HX_MODEL_CLIENT_OPCODE_GET的消息 */
}hx_model_server_opcode_t;

/*
===========================
结构体变量定义
=========================== 
*/

typedef struct hx_model_server hx_model_server_t;

/*
===========================
函数指针声明
=========================== 
*/
typedef uint8_t* (*hx_model_opcode_get_handler)(const hx_model_server_t * p_self);        /* 定义处理HX_MODEL_OPCODE_GET的消息的函数 */ 
typedef void (*hx_model_opcode_tx_handler)(const hx_model_server_t * p_self, \
                                           const uint8_t* p_data,uint8_t length);         /* 定义处理HX_MODEL_OPCODE_TX或者HX_MODEL_OPCODE_TX_UNRELIABLE的消息的函数 */
/*
===========================
结构体定义
=========================== 
*/

typedef struct
{
  /* 红旭客户端的操作码TX，所携带的消息的处理函数 */
  hx_model_opcode_tx_handler      tx_handler;
  /* 红旭客户端的操作码GET，所携带的消息的处理函数 */
  hx_model_opcode_get_handler     get_handler;
} hx_server_model_callbacks_t;

/* HX模型的功能定义 */
struct hx_model_server
{
  access_model_handle_t           model_handle;
  /* 红旭客户端的操作码，所携带的消息的处理函数 */
  hx_server_model_callbacks_t*    p_callbacks;
};

/*
===========================
函数声明
=========================== 
*/
/**
 * 初始化HX服务端模型.
 *
 *
 * @param[in] p_server      HX服务端模型的结构指针
 * @param[in] element_index 在节点的哪个元素中增加该服务端模型
 *
 * @retval NRF_SUCCESS         模型初始化成功
 * @retval NRF_ERROR_NULL      传进形参的实参是空指针
 * @retval NRF_ERROR_NO_MEM    新添模型时，内存不足
 * @retval NRF_ERROR_FORBIDDEN 每个元素不允许多个模型实例存在
 * @retval NRF_ERROR_NOT_FOUND 无效的元素索引值
 */
uint32_t hx_model_server_init(hx_model_server_t* p_server,uint16_t element_index);

#endif  //HX_MODEL_SERVER_H__