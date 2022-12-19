/** 
* @file         hx_model_control.h
* @brief        红旭控制端模型相关的宏定义以及函数声明
* @details      声明红旭控制端模型所需的宏定义、枚举变量定义、回调处理函数声明、相关结构体的定义等等
* @author       临时工
* @par Copyright (c):  
*               红旭无线开发团队
* @par Website:  
*               www.wireless-tech.cn
* @par History:          
*               Ver0.0.1:
                     临时工, 2020/06/03, 初始化版本\n 
*/
#ifndef HX_MODEL_CONTROL_H__
#define HX_MODEL_CONTROL_H__

/*
===========================
头文件包含
=========================== 
*/
#include <stdint.h>
#include <stdbool.h>
#include "access.h"
#include "hx_model_server.h"  
#include "hx_model_client.h"
/*
===========================
宏定义
=========================== 
*/

/* 标识符 */
#define SERVER_COMPANY_IDENTIFIER                  0x0059                   /* Bluetooth SIG分配给Nordic的Company identifier */
#define HX_MODEL_CONTROL_ID                        0x0002                   /* HX控制端模型的Vendor-assigned Model ID */

/* 红旭的控制端操作码的个数 */
#define CONTROL_MODEL_OPCODE_COUNT                 (CLIENT_MODEL_OPCODE_COUNT+SERVER_MODEL_OPCODE_COUNT)

/* 红旭的控制端模型的个数 */
#define HX_CONTROL_MODEL_INSTANCE_COUNT               (1)
/** 当使用控制端模型时置1，否则置0 */
#ifndef HX_MODEL_CONTROL_ENABLE
#define HX_MODEL_CONTROL_ENABLE                    (1)
#endif
/*
===========================
枚举变量定义
=========================== 
*/

/* 红旭客户端操作码 */
typedef struct
{
  hx_model_server_opcode_t server_opcode;                                   /* Server模型操作码 */
  hx_model_client_opcode_t client_opcode;                                   /* Client模型操作码 */
}hx_model_control_opcode_t;

/*
===========================
结构体定义
=========================== 
*/

/* HX模型的功能定义 */
typedef struct 
{
  access_model_handle_t           model_handle;
  /* 红旭客户端模型 */
  hx_model_client_t               client_model;
  /* 红旭服务端模型 */
  hx_model_server_t               server_model;
}hx_model_control_t;

/*
===========================
函数声明
=========================== 
*/
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
uint32_t hx_model_control_init(hx_model_control_t* p_control,uint16_t element_index);



#endif  //HX_MODEL_CONTROL_H__