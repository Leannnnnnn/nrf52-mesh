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

#include <stdint.h>
#include <string.h>

/* HAL */
#include "boards.h"
#include "simple_hal.h"
#include "app_timer.h"
#include <nrfx_pdm.h>


/* FreeRTOS and dependencies */
#include "FreeRTOS.h"
#include "task.h"
#include "nrf_sdh_freertos.h"
#include "nrf_drv_clock.h"

/* Core */
#include "nrf_mesh_config_core.h"
#include "nrf_mesh_gatt.h"
#include "nrf_mesh_configure.h"
#include "nrf_mesh.h"
#include "mesh_stack.h"
#include "device_state_manager.h"
#include "access_config.h"
#include "proxy.h"
#include "bearer_event.h"

/* Provisioning and configuration */
#include "mesh_provisionee.h"
#include "mesh_app_utils.h"

/* Models */
#include "generic_onoff_server.h"

/* Logging and RTT */
#include "log.h"
#include "rtt_input.h"

/* Example specific includes */
#include "nrf_sdh_soc.h"
#include "app_config.h"
#include "example_common.h"
#include "nrf_mesh_config_examples.h"
#include "light_switch_example_common.h"
#include "app_onoff.h"
#include "ble_softdevice_support.h"


#include "app_dtt.h"
#include "app_scene.h"


/* Custom Libraries */
#include "custom_uart.h"
#include "custom_twi.h"
#include "MAX30205_temp_drv.h"
#include "MAX30102.h"
#include "max30102_fir.h"
#include "ICM42688.h"


/* ?????????????????? */
#define MAX30205_EN 0
#define MAX30102_EN 0
#define ICM42688_EN 0

#define MP34DT05    1


/* Custom static variables */
#define CACHE_NUMS 150//?????????
#define PPG_DATA_THRESHOLD 100000 	//????????????

float ppg_data_cache_RED[CACHE_NUMS]={0};  //?????????
float ppg_data_cache_IR[CACHE_NUMS]={0};  //?????????

static float temperature = 0.0;
float max30102_data[2],fir_output[2];

icm42688_raw_acce_value_t raw_acceXYZ;
icm42688_raw_gyro_value_t raw_gyroXYZ;

static uint8_t temp_str[30];


#define SENSOR_PROCESS_THREAD_STACK_SIZE     (512)
#define HEARTRATE_PROCESS_THREAD_STACK_SIZE  (512)
#define BUTTON_HANDLER_THREAD_STACK_SIZE     (512)

#define SENSOR_PROCESS_THREAD_PRIORITY       (1)
#define HEARTRATE_PROCESS_THREAD_PRIORITY    (2)
#define BUTTON_HANDLER_THREAD_PRIORITY       (1)

static TaskHandle_t m_mesh_process_task;  /* The handle for the Mesh processing task */
static TaskHandle_t m_button_handler_thread;  /* The handle for the button handler thread */
static TaskHandle_t m_heartRate_process_thread;  /* The handle for heartRate sensors processing thread */
static TaskHandle_t m_sensor_process_thread;  /* The handle for other sensors processing thread */


/*************************************************************************************************/




/*****************************************************************************
 * Definitions
 *****************************************************************************/
#define ONOFF_SERVER_0_LED          (BSP_LED_0)
#define APP_ONOFF_ELEMENT_INDEX     (0)

/* Controls if the model instance should force all mesh messages to be segmented messages. */
#define APP_FORCE_SEGMENTATION      (false)
/* Controls the MIC size used by the model instance for sending the mesh messages. */
#define APP_MIC_SIZE                (NRF_MESH_TRANSMIC_SIZE_SMALL)


/*****************************************************************************
 * Forward declaration of static functions
 *****************************************************************************/
static void app_onoff_server_set_cb(const app_onoff_server_t * p_server, bool onoff);
static void app_onoff_server_get_cb(const app_onoff_server_t * p_server, bool * p_present_onoff);
static void app_onoff_server_transition_cb(const app_onoff_server_t * p_server,
                                                uint32_t transition_time_ms, bool target_onoff);

static void start(void * p_device_provisioned);
static void button_thread_notify(uint32_t button);

/*****************************************************************************
 * Static variables
 *****************************************************************************/
static bool m_device_provisioned;



/* Generic OnOff server structure definition and initialization */
APP_ONOFF_SERVER_DEF(m_onoff_server_0,
                     APP_FORCE_SEGMENTATION,
                     APP_MIC_SIZE,
                     app_onoff_server_set_cb,
                     app_onoff_server_get_cb,
                     app_onoff_server_transition_cb)

/* Callback for updating the hardware state */
static void app_onoff_server_set_cb(const app_onoff_server_t * p_server, bool onoff)
{
    /* Resolve the server instance here if required, this example uses only 1 instance. */

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Setting GPIO value: %d\n", onoff)

    hal_led_pin_set(ONOFF_SERVER_0_LED, onoff);
}

/* Callback for reading the hardware state */
static void app_onoff_server_get_cb(const app_onoff_server_t * p_server, bool * p_present_onoff)
{
    /* Resolve the server instance here if required, this example uses only 1 instance. */

    *p_present_onoff = hal_led_pin_get(ONOFF_SERVER_0_LED);
}

/* Callback for updating the hardware state */
static void app_onoff_server_transition_cb(const app_onoff_server_t * p_server,
                                                uint32_t transition_time_ms, bool target_onoff)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Transition time: %d, Target OnOff: %d\n",
                                       transition_time_ms, target_onoff);
}

static void app_model_init(void)
{
    /* Instantiate onoff server on element index APP_ONOFF_ELEMENT_INDEX */
    ERROR_CHECK(app_onoff_init(&m_onoff_server_0, APP_ONOFF_ELEMENT_INDEX));
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "App OnOff Model Handle: %d\n", m_onoff_server_0.server.model_handle);
}

/*************************************************************************************************/

static void node_reset(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- Node reset  -----\n");
    hal_led_blink_ms(LEDS_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_RESET);
    /* This function may return if there are ongoing flash operations. */
    mesh_stack_device_reset();
}

static void config_server_evt_cb(const config_server_evt_t * p_evt)
{
    if (p_evt->type == CONFIG_SERVER_EVT_NODE_RESET)
    {
        node_reset();
    }
}

#if NRF_MESH_LOG_ENABLE
static const char m_usage_string[] =
    "\n"
    "\t\t-------------------------------------------------------------------------------\n"
    "\t\t Button/RTT 1) LED state will toggle and inform clients about the state change.\n"
    "\t\t Button/RTT 4) Clear all the states to reset the node.\n"
    "\t\t-------------------------------------------------------------------------------\n";
#endif

static void button_event_handler(uint32_t button_number)
{
    /* Increase button number because the buttons on the board is marked with 1 to 4 */
    button_number++;
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Button %u pressed\n", button_number);
    switch (button_number)
    {
        /* Pressing SW1 on the Development Kit will result in LED state to toggle and trigger
        the STATUS message to inform client about the state change. This is a demonstration of
        state change publication due to local event. */
        case 1:
        {
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "User action \n");
            hal_led_pin_set(ONOFF_SERVER_0_LED, !hal_led_pin_get(ONOFF_SERVER_0_LED));
            app_onoff_status_publish(&m_onoff_server_0);
            break;
        }

        /* Initiate node reset */
        case 4:
        {
            /* Clear all the states to reset the node. */
            if (mesh_stack_is_device_provisioned())
            {
#if MESH_FEATURE_GATT_PROXY_ENABLED
                (void) proxy_stop();
#endif
                mesh_stack_config_clear();
                node_reset();
            }
            else
            {
                __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "The device is unprovisioned. Resetting has no effect.\n");
            }
            break;
        }

        default:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, m_usage_string);
            break;
    }
}

static void app_rtt_input_handler(int key)
{
    if (key >= '1' && key <= '4')
    {
        uint32_t button_number = key - '1';
        button_thread_notify(button_number);
    }
    else
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, m_usage_string);
    }
}

static void device_identification_start_cb(uint8_t attention_duration_s)
{
    hal_led_mask_set(LEDS_MASK, false);
    hal_led_blink_ms(BSP_LED_2_MASK  | BSP_LED_3_MASK,
                     LED_BLINK_ATTENTION_INTERVAL_MS,
                     LED_BLINK_ATTENTION_COUNT(attention_duration_s));
}

static void provisioning_aborted_cb(void)
{
    hal_led_blink_stop();
}

static void unicast_address_print(void)
{
    dsm_local_unicast_address_t node_address;
    dsm_local_unicast_addresses_get(&node_address);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Node Address: 0x%04x \n", node_address.address_start);
}

static void provisioning_complete_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Successfully provisioned\n");

#if MESH_FEATURE_GATT_ENABLED
    /* Restores the application parameters after switching from the Provisioning
     * service to the Proxy  */
    gap_params_init();
    conn_params_init();
#endif

    unicast_address_print();
    hal_led_blink_stop();
    hal_led_mask_set(LEDS_MASK, LED_MASK_STATE_OFF);
    hal_led_blink_ms(LEDS_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_PROV);
}

static void models_init_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Initializing and adding models\n");
    app_model_init();
}

static void mesh_init(void)
{
    mesh_stack_init_params_t init_params =
    {
        .core.irq_priority       = NRF_MESH_IRQ_PRIORITY_THREAD,
        .core.lfclksrc           = DEV_BOARD_LF_CLK_CFG,
        .core.p_uuid             = NULL,
        .models.models_init_cb   = models_init_cb,
        .models.config_server_cb = config_server_evt_cb
    };

    uint32_t status = mesh_stack_init(&init_params, &m_device_provisioned);
    switch (status)
    {
        case NRF_ERROR_INVALID_DATA:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Data in the persistent memory was corrupted. Device starts as unprovisioned.\n");
			__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Reset device before start provisioning.\n");
            break;
        case NRF_SUCCESS:
            break;
        default:
            ERROR_CHECK(status);
    }
}

/*************************************************************************************************/

static void clock_init(void)
{
    ret_code_t err_code = nrf_drv_clock_init();
    ERROR_CHECK(err_code);
}

/*************************************************************************************************/

static void button_thread_notify(uint32_t button)
{
    BaseType_t result = pdFAIL;
    BaseType_t switch_required = pdFALSE;

    result = xTaskNotifyFromISR(m_button_handler_thread, button, eSetValueWithoutOverwrite, &switch_required);

    if (result == pdPASS)
    {
        portYIELD_FROM_ISR(switch_required);
    }
}

static void button_handler_thread(void * p_arg)
{
    UNUSED_VARIABLE(p_arg);

    uint32_t notification = 0;

    for (;;)
    {
        if (xTaskNotifyWait(0, 0, &notification, portMAX_DELAY) == pdTRUE)
        {
            button_event_handler(notification);
        }
    }
}

#if !MESH_FREERTOS_IDLE_HANDLER_RESUME
static bool current_priority_is_rtos_safe(void)
{
    volatile IRQn_Type active_irq = hal_irq_active_get();
    if (active_irq != HAL_IRQn_NONE)
    {
        volatile uint32_t prio = NVIC_GetPriority(active_irq);
        return prio >= configMAX_SYSCALL_INTERRUPT_PRIORITY;
    }
    else
    {
        return true;
    }
}

/* bearer_event callback for resuming the Mesh processing task when events are received */
static void mesh_freertos_trigger_event_callback(void)
{
    BaseType_t yield = pdFALSE;

    APP_ERROR_CHECK_BOOL(current_priority_is_rtos_safe());

    vTaskNotifyGiveFromISR(m_mesh_process_task, &yield);

    /* Switch the task if required. */
    portYIELD_FROM_ISR(yield);
}
#endif

void vApplicationIdleHook( void )
{
#if MESH_FREERTOS_IDLE_HANDLER_RESUME
    (void) xTaskNotifyGive(m_mesh_process_task);
#endif
}

static void mesh_process_thread(void * arg)
{
    UNUSED_PARAMETER(arg);

    bool done = false;

    for (;;)
    {
        done = nrf_mesh_process();
        if (done)
        {
            (void) ulTaskNotifyTake(pdTRUE,         /* Clear the notification value before exiting (equivalent to the binary semaphore). */
                                    portMAX_DELAY); /* Block indefinitely (INCLUDE_vTaskSuspend has to be enabled).*/
        }
    }
}

static uint32_t nrf_mesh_process_thread_init(void)
{
    BaseType_t result;

    result = xTaskCreate(mesh_process_thread,
                         "MESH",
                         MESH_FREERTOS_TASK_STACK_DEPTH,
                         NULL,
                         MESH_FREERTOS_TASK_PRIO,
                         &m_mesh_process_task);

    if (result != pdPASS)
    {
        return NRF_ERROR_NO_MEM;
    }

#if !MESH_FREERTOS_IDLE_HANDLER_RESUME
    bearer_event_trigger_event_callback_set(APP_IRQ_PRIORITY_LOW, mesh_freertos_trigger_event_callback);
#endif

    return NRF_SUCCESS;
}


/**********************************Function thread program********************************************/
static void sensor_process_thread(void)
{
    static int i = 0;
    /*
#if MAX30102_EN 
    uint16_t cache_counter=0;  //???????????????
#endif
    */
    for(;;){
#if MAX30205_EN 
        i++;
        if(i == 100){  //??????????????????
            i = 0;
            temperature = temp_read();
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Temp is %d\n", (int)(temperature*100));
        }
#endif


#if ICM42688_EN
        if(true == icm42688_get_raw_acce(&raw_acceXYZ)){ 
            //__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "%d,%d,%d\n",raw_acceXYZ.raw_acce_x, raw_acceXYZ.raw_acce_y, raw_acceXYZ.raw_acce_z);
            sprintf(temp_str, "%d,%d,%d\n", raw_acceXYZ.raw_acce_x, raw_acceXYZ.raw_acce_y, raw_acceXYZ.raw_acce_z);
            //uart_send_str(temp_str);
        }
        else{
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "icm42688 not found!\n");
            //icm42688_init();  //???????????????
        }
        
        /*
        if(true == icm42688_get_raw_gyro(&raw_gyroXYZ)){ 
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "%d,%d,%d\n", raw_gyroXYZ.raw_gyro_x, raw_gyroXYZ.raw_gyro_y, raw_gyroXYZ.raw_gyro_z);
            sprintf(temp_str, "%d,%d,%d\n", raw_gyroXYZ.raw_gyro_x, raw_gyroXYZ.raw_gyro_y, raw_gyroXYZ.raw_gyro_z);
            uart_send_str(temp_str);
        }
        else{
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "icm42688 not found!\n");
        }
        */
        
#endif

#if MP34DT05



#endif

        vTaskDelay(10);  //????????????ms
    }

} 



static void heartRate_process_thread(void)
{
#if MAX30102_EN 
    uint16_t cache_counter=0;  //???????????????
#endif
    for(;;){
#if MAX30102_EN 
        if(nrf_gpio_pin_read(PIN_INT) == 0)			//??????????????????
        {
            hr_flag_clear();
            max30102_fifo_read(max30102_data);		//????????????
            max30102_i2c_write(INTERRUPT_ENABLE1,0xE0); //????????????
            ir_max30102_fir(&max30102_data[0],&fir_output[0]);
            red_max30102_fir(&max30102_data[1],&fir_output[1]);  //??????
    
            sprintf(temp_str, "%d,%d\n", (int32_t)(max30102_data[0]*100), (int32_t)(fir_output[0]*100));
            uart_send_str(temp_str);

            if((max30102_data[0]>PPG_DATA_THRESHOLD)&&(max30102_data[1]>PPG_DATA_THRESHOLD))  //???????????????????????????????????????
            {		
                ppg_data_cache_IR[cache_counter]=fir_output[0];
                ppg_data_cache_RED[cache_counter]=fir_output[1];
                cache_counter++;
            }
            else				//????????????
            {
                cache_counter=0;
                __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "No finger!\n");
            }

            if(cache_counter>=CACHE_NUMS)  //??????????????????
            {
                __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Heart rate: %d  /min\n ",max30102_getHeartRate(ppg_data_cache_IR,CACHE_NUMS));
                __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "SpO2: %d  %%\n", (int32_t)(max30102_getSpO2(ppg_data_cache_IR,ppg_data_cache_RED,CACHE_NUMS)*10));
                cache_counter=0;
            }
        }
#endif
        vTaskDelay(5);  //????????????ms
    }
}

/**********************************Function thread program********************************************/



#define PDM_BUF_SIZE 256

int16_t pdm_buf[PDM_BUF_SIZE];

void nrfx_pdm_event_handler(nrfx_pdm_evt_t const * const p_evt)
{
    if(p_evt->buffer_requested)
    {
        nrfx_pdm_buffer_set(pdm_buf, PDM_BUF_SIZE);
    }
    if(p_evt->buffer_released != 0)
    {
        //if(((uint16_t)pdm_buf[0]<10000)&&((uint16_t)pdm_buf[1]<10000)){
        sprintf(temp_str, "%d,%d\n",pdm_buf[0], pdm_buf[1]);
        uart_send_str(temp_str);
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, temp_str);
        //}
    }
}

static void pdm_init(void)
{
    nrfx_pdm_config_t pdm_config = /*NRFX_PDM_DEFAULT_CONFIG(3,4);*/
                                {                                                                     \
                                    .mode               = PDM_MODE_OPERATION_Mono,       \
                                    .edge               = PDM_MODE_EDGE_LeftRising,       \
                                    .pin_clk            = 3,                                   \
                                    .pin_din            = 4,               
                                                        \
                                    .clock_freq         = PDM_PDMCLKCTRL_FREQ_1280K, \
                                    .gain_l             = NRF_PDM_GAIN_DEFAULT,                       \
                                    .gain_r             = NRF_PDM_GAIN_DEFAULT,                       \
                                    .interrupt_priority = NRFX_PDM_CONFIG_IRQ_PRIORITY                \
                                };
    nrfx_pdm_init(&pdm_config, nrfx_pdm_event_handler);
   
}



static void initialize(void)
{
    __LOG_INIT(LOG_SRC_APP | LOG_SRC_FRIEND, LOG_LEVEL_DBG1, LOG_CALLBACK_DEFAULT);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- BLE Mesh Light Switch Server Demo -----\n");

    ERROR_CHECK(app_timer_init());
    hal_leds_init();
    clock_init();

#if BUTTON_BOARD
    ERROR_CHECK(hal_buttons_init(button_thread_notify));
#endif

    ble_stack_init();

#if MESH_FEATURE_GATT_ENABLED
    gap_params_init();
    conn_params_init();
#endif

    mesh_init();

    // Set up the Mesh processing task for FreeRTOS
    ERROR_CHECK(nrf_mesh_process_thread_init());

    uart_init();
    uart_send_str("Uart start.\r\n");
    twi_init();

#if MAX30205_EN
    temp_init();
#endif


#if MAX30102_EN
    if(true != max30102_init()){ 
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "max30102 not found!\n");
    }
    else{
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "max30102 initiated!\n");
    }
    max30102_fir_init();
#endif


#if ICM42688_EN
    
    if(true != icm42688_init()){ 
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "icm42688 not found!\n");
    }
    else{
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "icm42688 initiated!\n");
    }
#endif

#if MP34DT05
    pdm_init();  //pdm??????????????????

    nrfx_pdm_start();

#endif
    

    // Set up a task for processing button/RTT events
    if (pdPASS != xTaskCreate(button_handler_thread, 
                              "BTN", 
                              BUTTON_HANDLER_THREAD_STACK_SIZE, 
                              NULL, 
                              BUTTON_HANDLER_THREAD_PRIORITY, 
                              &m_button_handler_thread))
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }

    if (pdPASS != xTaskCreate(sensor_process_thread, 
                              "SENSOR", 
                              SENSOR_PROCESS_THREAD_STACK_SIZE, 
                              NULL, 
                              SENSOR_PROCESS_THREAD_PRIORITY, 
                              &m_sensor_process_thread))
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
    
    if (pdPASS != xTaskCreate(heartRate_process_thread, 
                              "HeartRate", 
                              HEARTRATE_PROCESS_THREAD_STACK_SIZE, 
                              NULL, 
                              HEARTRATE_PROCESS_THREAD_PRIORITY, 
                              &m_heartRate_process_thread))
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
    


    // Create a FreeRTOS task for handling SoftDevice events.
    // That task will call the start function when the task is started.
    nrf_sdh_freertos_init(start, &m_device_provisioned);
}

static void start(void * p_device_provisioned)
{
    bool device_provisioned = *(bool *)p_device_provisioned;

    rtt_input_enable(app_rtt_input_handler, RTT_INPUT_POLL_PERIOD_MS);
    
    if (!device_provisioned)
    {
        static const uint8_t static_auth_data[NRF_MESH_KEY_SIZE] = STATIC_AUTH_DATA;
        mesh_provisionee_start_params_t prov_start_params =
        {
            .p_static_data    = static_auth_data,
            .prov_sd_ble_opt_set_cb = NULL,
            .prov_complete_cb = provisioning_complete_cb,
            .prov_device_identification_start_cb = device_identification_start_cb,
            .prov_device_identification_stop_cb = NULL,
            .prov_abort_cb = provisioning_aborted_cb,
            .p_device_uri = EX_URI_LS_SERVER
        };
        ERROR_CHECK(mesh_provisionee_prov_start(&prov_start_params));
    }
    else
    {
        unicast_address_print();
    }

    mesh_app_uuid_print(nrf_mesh_configure_device_uuid_get());

    ERROR_CHECK(mesh_stack_start());

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, m_usage_string);

    hal_led_mask_set(LEDS_MASK, LED_MASK_STATE_OFF);
    hal_led_blink_ms(LEDS_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_START);
}

int main(void)
{
    initialize();

    // Start the FreeRTOS scheduler.
    vTaskStartScheduler();

    for (;;)
    {
        // The app should stay in the FreeRTOS scheduler loop and should never reach this point.
        APP_ERROR_HANDLER(NRF_ERROR_FORBIDDEN);
    }
}

