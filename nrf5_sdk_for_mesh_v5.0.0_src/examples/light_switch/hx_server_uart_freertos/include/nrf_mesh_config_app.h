/* Copyright (c) 2010 - 2020, Nordic Semiconductor ASA
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef NRF_MESH_CONFIG_APP_H__
#define NRF_MESH_CONFIG_APP_H__

/**
 * @defgroup NRF_MESH_CONFIG_APP nRF Mesh app config
 *
 * Application side configuration file. Should be copied into every
 * application, and customized to fit its requirements.
 * @{
 */

/**
 * @defgroup MESH_FREERTOS FreeRTOS/Mesh configuration
 *
 * @{
 */

/** Set this to 1 to resume the Mesh processing task in the FreeRTOS idle handler instead
 *  of resuming it in the bearer_event signal handler
 */
#define MESH_FREERTOS_IDLE_HANDLER_RESUME   0

/** The stack depth used for the Mesh processing task.
 *  The actual stack allocated is equal to MESH_FREERTOS_TASK_STACK_DEPTH * sizeof(StackType_t)
 *  May have to be increased for some use cases.
 */
#define MESH_FREERTOS_TASK_STACK_DEPTH      2048

/** The FreeRTOS task priority used for the Mesh processing task.
 *  Should not have to be modified.
 *  It is important that this is not lower than the priority of the SoftDevice Handler
 *  task in nrf_sdh_freertos.c as that will cause issues with GATT provisioning.
 */
#define MESH_FREERTOS_TASK_PRIO             4

/** @} end of MESH_FREERTOS */

/**
 * @defgroup DEVICE_CONFIG Device configuration
 *
 * @{
 */

/** Device company identifier. */
#define DEVICE_COMPANY_ID (ACCESS_COMPANY_ID_NORDIC)

/** Device product identifier. */
#define DEVICE_PRODUCT_ID (0x0000)

/** Device version identifier. */
#define DEVICE_VERSION_ID (0x0000)

/** @} end of DEVICE_CONFIG */

/**
 * @defgroup ACCESS_CONFIG Access layer configuration
 * @{
 */

/**
 * The default TTL value for the node.
 */
#define ACCESS_DEFAULT_TTL (4)

/**
 * The number of models in the application.
 *
 * @note To fit the configuration and health models, this value must equal at least
 * the number of models needed by the application plus two.
 */
#define ACCESS_MODEL_COUNT (/* Element 0:                                       */ \
                            1 + /* Config Server                                */ \
                            1 + /* Health Server                                */ \
                            1 + /* Generic OnOff Server                         */ \
                            1 + /* Default Transition Time Server               */ \
                            1 + /* Scene Server                                 */ \
                            1 + /* Scene Setup Server (extends Scene Server)    */ \
                            1   /* hx control model */ )

/**
 * The number of elements in the application.
 *
 * @warning If the application is to support _multiple instances_ of the _same_ model, these instances
 * cannot be in the same element and a separate element is needed for each new instance of the same model.
 */
#define ACCESS_ELEMENT_COUNT (1)

/**
 * The number of (root only) default transition time instances used by the application.
 */
#define GENERIC_DTT_SERVER_INSTANCES_MAX (1)

/**
 * The number of (root only) generic onoff instances used by the application.
 */
#define GENERIC_ONOFF_SERVER_INSTANCES_MAX (1)

/**
 * The number of scene setup server instances used by the application.
 */
#ifndef SCENE_SETUP_SERVER_INSTANCES_MAX
#define SCENE_SETUP_SERVER_INSTANCES_MAX (1)
#endif

/**
 * The number of allocated subscription lists for the application.
 *
 * @note This value must equal @ref ACCESS_MODEL_COUNT minus the number of
 * models operating on shared states.
 */
#define ACCESS_SUBSCRIPTION_LIST_COUNT (ACCESS_MODEL_COUNT)

/**
 * @defgroup ACCESS_RELIABLE_CONFIG Configuration of access layer reliable transfer
 * @{
 */

/** Number of the allowed parallel transfers (size of the internal context pool). */
#define ACCESS_RELIABLE_TRANSFER_COUNT (ACCESS_MODEL_COUNT)

/** @} end of ACCESS_RELIABLE_CONFIG */


/** @} end of ACCESS_CONFIG */


/**
 * @ingroup HEALTH_MODEL
 * @{
 */

/** The number of instances of the health server model. */
#define HEALTH_SERVER_ELEMENT_COUNT (1)

/** @} end of HEALTH_MODEL */


/**
 * @defgroup DSM_CONFIG Device State Manager configuration
 * Sizes for the internal storage of the Device State Manager.
 * @{
 */
/** Maximum number of subnetworks. */
#define DSM_SUBNET_MAX                                  (4)
/** Maximum number of applications. */
#define DSM_APP_MAX                                     (8)
/** Maximum number of device keys. */
#define DSM_DEVICE_MAX                                  (1)
/** Maximum number of virtual addresses. */
#define DSM_VIRTUAL_ADDR_MAX                            (1)
/** Maximum number of non-virtual addresses.
 * - Generic OnOff publication
 * - Health publication
 * - Subscription address
 */
#define DSM_NONVIRTUAL_ADDR_MAX                         (3)
/** @} end of DSM_CONFIG */

/** @} */

/**
 * @defgroup NRF_MESH_CONFIG_CORE Compile time configuration
 * Configuration of the compilation of the core mesh modules.
 * @ingroup CORE_CONFIG
 * @{
 */

/**
 * @defgroup MESH_CONFIG_GATT GATT configuration defines
 * @{
 */
/** PB-GATT feature. To be enabled only in combination with linking GATT files. */
#define MESH_FEATURE_PB_GATT_ENABLED                    (1)
/** GATT proxy feature. To be enabled only in combination with linking GATT proxy files. */
#define MESH_FEATURE_GATT_PROXY_ENABLED                 (1)
/** @} end of MESH_CONFIG_GATT */

/** Enable the Friend feature. */
#define MESH_FEATURE_FRIEND_ENABLED (1)

/**
 * @defgroup BLE_SOFTDEVICE_SUPPORT_CONFIG BLE SoftDevice support module configuration.
 * @ingroup MESH_API_GROUP_APP_SUPPORT
 * Configuration for compile time. Part of BLE SoftDevice support module.
 *
 * @{
 */
#define GAP_DEVICE_NAME                 "nRF5x Mesh Light hx"
/** @} end of BLE_SOFTDEVICE_SUPPORT_CONFIG */

/** @} end of NRF_MESH_CONFIG_CORE */

#endif /* NRF_MESH_CONFIG_APP_H__ */
