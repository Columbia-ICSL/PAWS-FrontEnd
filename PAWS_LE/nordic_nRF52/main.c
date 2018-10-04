/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 *
 * MIT License
 *
 * Copyright (c) 2018, Stephen Xia, Columbia Intelligent and Connected Systems Lab (ICSL), Columbia University
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the “Software”), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/** @file
 *
 * @defgroup ble_sdk_uart_over_ble_main main.c
 * @{
 * @ingroup  ble_sdk_app_nus_eval
 * @brief    UART over BLE application main file.
 *
 * This file contains the source code for a sample application that uses the Nordic UART service.
 * This application uses the @ref srvlib_conn_params module.
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "app_button.h"
#include "ble_nus.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "bsp.h"
#include "bsp_btn_ble.h"
#include "nrf_drv_pwm.h"
#include "nrf_drv_gpiote.h"

#define IS_SRVC_CHANGED_CHARACT_PRESENT 0                                           /**< Include the service_changed characteristic. If not enabled, the server's database cannot be changed for the lifetime of the device. */

#if (NRF_SD_BLE_API_VERSION == 3)
#define NRF_BLE_MAX_MTU_SIZE            GATT_MTU_SIZE_DEFAULT                       /**< MTU size used in the softdevice enabling and to reply to a BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST event. */
#endif

#define APP_FEATURE_NOT_SUPPORTED       BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2        /**< Reply when unsupported features are requested. */

#define CENTRAL_LINK_COUNT              0                                           /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT           1                                           /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#define DEVICE_NAME                     "Nordic_UART_0"                               /**< Name of device. Will be included in the advertising data. */
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      180                                         /**< The advertising timeout (in units of seconds). */

#define APP_TIMER_PRESCALER             0                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE         4                                           /**< Size of timer operation queues. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(7.5, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */

// Length of messages for the start and stop message to send to STM
#define START_MSG_LEN 5
#define END_MSG_LEN 5

// Constants for dealing with interfacing with xcorr chip
// Max sampling is around 30 kHz
#define XCORR_PWM_PIN 14                           // Pin for PWM clock
#define XCORR_PWM_INTERRUPT_PIN 11                 // Pin for reading the same PWM clock source as the xcorr chip for synchronized reads from the chip
#define XCORR_PWM_PIN_S0 15   // 0 signal pin
#define XCORR_PWM_PIN_S1 16   // 1 signal pin
#define XCORR_PWM_TOP_TICK 5 * 16
#define XCORR_PWM_TOGGLE_TICK 5 * 16/ 2
#define XCORR_NUM_TO_AVERAGE 750  // If reading samples at 30k, this corresponds to sending 20 samples per second
//#define XCORR_NUM_TO_AVERAGE 2500 // Number of values to average for one sample to transmit - with 1 MHz, 100 ticks per period yields txing 20 times per second
//#define XCORR_PWM_TOP_TICK 1600
//#define XCORR_PWM_TOGGLE_TICK 800
//#define XCORR_NUM_TO_AVERAGE 125 // Number of values to average for one sample to transmit - with 1 MHz, 100 ticks per period yields txing 20 times per second


#define XCORR_DATA_PIN_0 3
#define XCORR_DATA_PIN_1 4
#define XCORR_DATA_PIN_2 5
#define XCORR_DATA_PIN_3 6    
#define XCORR_DATA_PIN_4 7
#define XCORR_DATA_PIN_5 8
#define XCORR_DATA_PIN_6 18
#define XCORR_DATA_PIN_7 19

#define XCORR_PHI_PIN 25
#define XCORR_PHIB_PIN 26
#define XCORR_SC_IN_PIN 27
#define XCORR_SC_LOAD_PIN 28
#define XCORR_SC_i0O1_PIN 29
#define XCORR_SC_OUT_PIN 30
#define XCORR_RESETN_PIN 31

#define TX_WINDOW_SIZE 35// Number of bytes to send in one frame

// Pin number for controlling the state of the STM32f446re
#define STM_EXTI_PIN 17

// SCAN CHAIN INPUT
#define SC_size 42

#define SC_EXT_SEL 0,1 // 2
#define SC_DIV_SEL 0,0,0,0 // 4
#define SC_TAP_SEL 0,0,0,0 // 4
#define SC_CLK 0 // 1
#define SC_CLK_ST 0 // 1

#define SC_G 0,1,0 // 3
#define SC_FB_CTRL  0  // 1
#define SC_MAX_VAL 0,1,1,1,1,1,1,0 // 126
#define SC_RSTN 1 // 1

#define SC_FB 0 // 1
#define SC_ATDE_RST 0 // 1

#define SC_ES 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 // 16

static ble_nus_t                        m_nus;                                      /**< Structure to identify the Nordic UART Service. */
static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */

static ble_uuid_t                       m_adv_uuids[] = {{BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}};  /**< Universally unique service identifier. */

static uint8_t send = 0;   // Used to see if you can send or not (send = 0 if not connected, 1 if it is connected)
static uint8_t send_first_time = 0;  // Used to make sure the first time you send it begins with the delimiter

static uint8_t connection_char_desc[4] = {70, 82, 69, 68};   // descriptor of the connection characteristic name

// Test data
uint8_t test_data[1] = {65};

//static uint32_t       err_code;
// Start and end messages for STM
static uint8_t start_msg[START_MSG_LEN] = {83, 69, 85, 83, 83};  // SEUSS
static uint8_t end_msg[START_MSG_LEN] = {83, 69, 85, 83, 69};  // SEUSE

// Flags for determining when to start and stop pipeline 
static uint8_t start = 0;
static uint8_t stop = 0;
static uint8_t running = 0;

// Variables for controlling xcorr chip
static nrf_drv_pwm_t m_pwm0 = NRF_DRV_PWM_INSTANCE(0);
uint8_t pwm_polarity_flag = 0;
static uint8_t tx_window[TX_WINDOW_SIZE + 5] = {83, 69, 85, 83, 35, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8_t tx_delay[4] = {0, 0, 0, 0};    // The delay that is being transmitted
static uint32_t curr_delay[4] = {0, 0, 0, 0};  // The current delay that is being sampled and averaged
uint32_t sample_index = 0;
uint8_t tx_flag = 0;                // For determining if we should tx or not
uint8_t var_mux_state = 0;  // Variables for tracking mux state
uint8_t channel_idx = 0;    // Variable for keeping track of current delay channel being read.

//int ScanChain_input [SC_size] = {SC_ES,SC_RSTN,SC_EXT_SEL,SC_DIV_SEL,SC_TAP_SEL,SC_CLK,SC_CLK_ST,SC_MAX_VAL,SC_G,SC_FB,SC_ATDE_RST};
int ScanChain_input [SC_size] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,0,0,1,0,0,0};
int ScanChain_output [SC_size];

// Prototypes
static uint8_t ScanChain_rotate(int* input, int Size);
static void stm_exti_high(void);
static void stm_exti_low(void);
static void data_pins_init(void);
static void signal_pins_init(void);
static void S0_high(void);
static void S0_low(void);
static void S0_toggle(void);
static void S1_high(void);
static void S1_low(void);
static void S1_toggle(void);

/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
 *          the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
																					
		err_code = sd_ble_gap_tx_power_set(0);
		APP_ERROR_CHECK(err_code);
}

// Add custom rx characteristic
static uint32_t custom_rx_char_add(ble_nus_t * p_nus, const ble_nus_init_t * p_nus_init)
{
    /**@snippet [Adding proprietary characteristic to S110 SoftDevice] */
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&cccd_md, 0, sizeof(cccd_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);

    cccd_md.vloc = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.notify = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md;
    char_md.p_sccd_md         = NULL;

    ble_uuid.type = p_nus->uuid_type;
    ble_uuid.uuid = 0x1234;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

    attr_md.vloc    = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen    = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof(uint8_t);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = (GATT_MTU_SIZE_DEFAULT - 3);

    return sd_ble_gatts_characteristic_add(p_nus->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_nus->rx_handles);
    /**@snippet [Adding proprietary characteristic to S110 SoftDevice] */
}

// Handles the handshaking process between ble and stm as well as between ble and app whenever disconnect happens
static uint32_t handle_handshake_end(void)
{
	uint8_t i;
	send = 0;
	
	// Send stop message to STM32
	//stm_exti_low();
	
	return sd_ble_gap_disconnect(m_conn_handle,
                               BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
}

// Handles the handshaking process between ble and stm as well as between ble and app whenever connection happens
static uint32_t handle_handshake_start(void)
{
	uint8_t i;
	send = 1;
	
	// Send start message to STM32
	//stm_exti_high();
	
	return NRF_SUCCESS;
}

/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_nus    Nordic UART Service structure.
 * @param[in] p_data   Data to be send to UART module.
 * @param[in] length   Length of the data.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_t * p_nus, uint8_t * p_data, uint16_t length)
{
	  uint32_t err_code;
	  //send = send ^ 1;
	  // Parse Start and stop messages
		if(length >= 5) {
			
			// Start MESSAGE: SEUSS
			if(p_data[0] == 83 && p_data[1] == 69 && p_data[2] == 85 && p_data[3] == 83 && p_data[4] == 83) {
			  start = 1;
				
			}
			
			// Start MESSAGE: SEUSE
			if(p_data[0] == 83 && p_data[1] == 69 && p_data[2] == 85 && p_data[3] == 83 && p_data[4] == 69) {
			  stop = 1;
			}
		}
	
    /*for (uint32_t i = 0; i < length; i++)
    {
        while (app_uart_put(p_data[i]) != NRF_SUCCESS);
    }
    while (app_uart_put('\r') != NRF_SUCCESS);
    while (app_uart_put('\n') != NRF_SUCCESS);*/
}
/**@snippet [Handling the data received over BLE] */


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t       err_code;
    ble_nus_init_t nus_init;

    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
	
	  //err_code = custom_rx_char_add(&m_nus, &nus_init);
		//APP_ERROR_CHECK(err_code);
	
}


/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;
        default:
            break;
    }
}


/**@brief Function for the application's SoftDevice event handler.
 *
 * @param[in] p_ble_evt SoftDevice event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t                         err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break; // BLE_GAP_EVT_CONNECTED

        case BLE_GAP_EVT_DISCONNECTED:
            err_code = bsp_indication_set(BSP_INDICATE_IDLE);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break; // BLE_GAP_EVT_DISCONNECTED

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GAP_EVT_SEC_PARAMS_REQUEST

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_SYS_ATTR_MISSING

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTC_EVT_TIMEOUT

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_TIMEOUT

        case BLE_EVT_USER_MEM_REQUEST:
            err_code = sd_ble_user_mem_reply(p_ble_evt->evt.gattc_evt.conn_handle, NULL);
            APP_ERROR_CHECK(err_code);
            break; // BLE_EVT_USER_MEM_REQUEST

        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
        {
            ble_gatts_evt_rw_authorize_request_t  req;
            ble_gatts_rw_authorize_reply_params_t auth_reply;

            req = p_ble_evt->evt.gatts_evt.params.authorize_request;

            if (req.type != BLE_GATTS_AUTHORIZE_TYPE_INVALID)
            {
                if ((req.request.write.op == BLE_GATTS_OP_PREP_WRITE_REQ)     ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_NOW) ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL))
                {
                    if (req.type == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
                    }
                    else
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
                    }
                    auth_reply.params.write.gatt_status = APP_FEATURE_NOT_SUPPORTED;
                    err_code = sd_ble_gatts_rw_authorize_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                               &auth_reply);
                    APP_ERROR_CHECK(err_code);
                }
            }
        } break; // BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST

#if (NRF_SD_BLE_API_VERSION == 3)
        case BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST:
            err_code = sd_ble_gatts_exchange_mtu_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                       NRF_BLE_MAX_MTU_SIZE);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST
#endif

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for dispatching a SoftDevice event to all modules with a SoftDevice
 *        event handler.
 *
 * @details This function is called from the SoftDevice event interrupt handler after a
 *          SoftDevice event has been received.
 *
 * @param[in] p_ble_evt  SoftDevice event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    ble_conn_params_on_ble_evt(p_ble_evt);
    ble_nus_on_ble_evt(&m_nus, p_ble_evt);
    on_ble_evt(p_ble_evt);
	
	  if(p_ble_evt->header.evt_id == BLE_GAP_EVT_TIMEOUT) {
			ble_advertising_start(BLE_ADV_MODE_FAST);
		}
		else {
			ble_advertising_on_ble_evt(p_ble_evt);
		}
    bsp_btn_ble_on_ble_evt(p_ble_evt);
	
	  // Check and see if we are connected or disconnected
		/*if(p_ble_evt->header.evt_id == BLE_GAP_EVT_CONNECTED)
		{
			send = 1;
			send_first_time = 1;
		}*/
	  /*if(p_ble_evt->header.evt_id == BLE_GAP_EVT_CONNECTED)
		{
			send = 1;
		}*/
		if(p_ble_evt->header.evt_id == BLE_GAP_EVT_DISCONNECTED)
		{
			stop = 1;
		}

}


/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;

    //nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;

    // Initialize SoftDevice.
    //SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);
	  SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LF_SRC_RC, NULL);
	
    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
	  //ble_enable_params.common_enable_params.p_conn_bw_counts->tx_counts.high_count = 1;
		//ble_enable_params.common_enable_params.p_conn_bw_counts->tx_counts.mid_count = 0;
		//ble_enable_params.common_enable_params.p_conn_bw_counts->tx_counts.low_count = 0;
		//ble_enable_params.common_enable_params.p_conn_bw_counts->rx_counts.high_count = 1;
		//ble_enable_params.common_enable_params.p_conn_bw_counts->rx_counts.mid_count = 0;
		//ble_enable_params.common_enable_params.p_conn_bw_counts->rx_counts.low_count = 0;
    APP_ERROR_CHECK(err_code);

    //Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);

    // Enable BLE stack.
#if (NRF_SD_BLE_API_VERSION == 3)
    ble_enable_params.gatt_enable_params.att_mtu = NRF_BLE_MAX_MTU_SIZE;
#endif
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Subscribe for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);
		
		/** ble options **/
		/*ble_opt_t ble_opt;
		ble_opt.common_opt.conn_bw.role = BLE_GAP_ROLE_PERIPH;
		ble_opt.common_opt.conn_bw.conn_bw.conn_bw_tx = BLE_CONN_BW_HIGH;
		ble_opt.common_opt.conn_bw.conn_bw.conn_bw_rx = BLE_CONN_BW_HIGH;
		err_code = sd_ble_opt_set(BLE_COMMON_OPT_CONN_BW, &ble_opt);
		APP_ERROR_CHECK(err_code);*/
		
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;
    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist();
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break;

        default:
            break;
    }
}


/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to
 *          a string. The string will be be sent over BLE when the last character received was a
 *          'new line' i.e '\r\n' (hex 0x0D) or if the string has reached a length of
 *          @ref NUS_MAX_DATA_LENGTH.
 */
/**@snippet [Handling the data received over UART] */
void uart_event_handle(app_uart_evt_t * p_event)
{
    static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
    static uint8_t index = 0;
    uint32_t       err_code;
	  uint8_t i = 0;

    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
            index++;

						if (index >= (BLE_NUS_MAX_DATA_LEN))
            //if ((data_array[index - 1] == '\n') || (index >= (BLE_NUS_MAX_DATA_LEN)))
            {
							  /*for(i = 0; i < 20; i++)
								{
									while(app_uart_put(data_array[i++]) != NRF_SUCCESS);
								}*/
							
								/*if(send && send_first_time && data_array[1] == 82)
								{
									//err_code= ble_nus_string_send(&m_nus, data_array, 1);
									err_code = ble_nus_string_send(&m_nus, data_array, index);
									//printf("HI");
									if (err_code != NRF_ERROR_INVALID_STATE)
									{
											APP_ERROR_CHECK(err_code);
									}
									send_first_time = 0;
								}
								else if(send && !send_first_time)
								{
									err_code = ble_nus_string_send(&m_nus, data_array, index);
									//printf("HI");
									if (err_code != NRF_ERROR_INVALID_STATE)
									{
											APP_ERROR_CHECK(err_code);
									}
								}*/
								
								/*err_code = ble_nus_string_send(&m_nus, data_array, index);
								//printf("HI");
								if (err_code != NRF_ERROR_INVALID_STATE)
								{
										APP_ERROR_CHECK(err_code);
								}*/
								if(send)
								{
									err_code = ble_nus_string_send(&m_nus, data_array, index);
									if(err_code == BLE_ERROR_NO_TX_PACKETS) {
									}
									else if (err_code != NRF_SUCCESS) {
										APP_ERROR_CHECK(err_code);
									}
									else if (err_code != NRF_ERROR_INVALID_STATE)
									{
										APP_ERROR_CHECK(err_code);
									}
								}

                index = 0;
            }
            break;

        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}
/**@snippet [Handling the data received over UART] */

void uart_error_handle(app_uart_evt_t * p_event)
{
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
}

/**@brief  Function for initializing the UART module.
 */
/**@snippet [UART Initialization] */
static void uart_init(void)
{
    uint32_t                     err_code;
    const app_uart_comm_params_t comm_params =
    {
        RX_PIN_NUMBER,
        TX_PIN_NUMBER,
        RTS_PIN_NUMBER,
        CTS_PIN_NUMBER,
        APP_UART_FLOW_CONTROL_ENABLED,
        false,
        UART_BAUDRATE_BAUDRATE_Baud115200
    };

    APP_UART_FIFO_INIT( &comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_error_handle,
                       APP_IRQ_PRIORITY_LOW,
                       err_code);
    APP_ERROR_CHECK(err_code);
}
/**@snippet [UART Initialization] */


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t               err_code;
    ble_advdata_t          advdata;
    ble_advdata_t          scanrsp;
    ble_adv_modes_config_t options;

    // Build advertising data struct to pass into @ref ble_advertising_init.
    memset(&advdata, 0, sizeof(advdata));
    advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance = false;
    advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;

    memset(&scanrsp, 0, sizeof(scanrsp));
    scanrsp.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    scanrsp.uuids_complete.p_uuids  = m_adv_uuids;

    memset(&options, 0, sizeof(options));
    options.ble_adv_fast_enabled  = true;
    options.ble_adv_fast_interval = APP_ADV_INTERVAL;
    options.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = ble_advertising_init(&advdata, &scanrsp, &options, on_adv_evt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    bsp_event_t startup_event;

    uint32_t err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS,
                                 APP_TIMER_TICKS(100, APP_TIMER_PRESCALER),
                                 bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for placing the application in low power state while waiting for events.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}

// Handler for handling PWM interrupts
void xcorr_pwm_input_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
	uint8_t temp;
		/*if(action == NRF_GPIOTE_POLARITY_LOTOHI)
		{
				stm_exti_low();
		}
		else if(action == NRF_GPIOTE_POLARITY_HITOLO)
		{
				stm_exti_low();
		}
		else if(action == NRF_GPIOTE_POLARITY_TOGGLE)
		{
				stm_exti_high();
		}*/
	
	// Process
	if(pin == XCORR_PWM_INTERRUPT_PIN)
	{
		
		// Disable interrupt first
		//stm_exti_high();
	  nrf_drv_gpiote_in_event_disable(XCORR_PWM_INTERRUPT_PIN);
		
		// // Determine if we should read: According to Daniel, the first sequence of S0 = 0, S1 = 0 is not required apparently
		if((var_mux_state || !var_mux_state) && sample_index == XCORR_NUM_TO_AVERAGE - 1)
		{
			//temp = (((nrf_gpio_pin_read(XCORR_DATA_PIN_0) << 0) & 0x01) |
		  //      ((nrf_gpio_pin_read(XCORR_DATA_PIN_1) << 1) & 0x02) |
		  //      ((nrf_gpio_pin_read(XCORR_DATA_PIN_2) << 2) & 0x04) |
		  //      ((nrf_gpio_pin_read(XCORR_DATA_PIN_3) << 3) & 0x08) |
		  //      ((nrf_gpio_pin_read(XCORR_DATA_PIN_4) << 4) & 0x10) |
		  //      ((nrf_gpio_pin_read(XCORR_DATA_PIN_5) << 5) & 0x20) |
		  //      ((nrf_gpio_pin_read(XCORR_DATA_PIN_6) << 6) & 0x40) |
			//			((nrf_gpio_pin_read(XCORR_DATA_PIN_7) << 7) & 0x80));
			
			//curr_delay[channel_idx] = curr_delay[channel_idx] + (uint32_t)temp;
			////if(var_mux_state == 0) {
			//	 while(((NRF_GPIO->OUT >> XCORR_PWM_PIN_S0) & 1UL) || ((NRF_GPIO->OUT >> XCORR_PWM_PIN_S1) & 1UL));
			//}
			//else if(var_mux_state == 1) {
			//	 while(!((NRF_GPIO->OUT >> XCORR_PWM_PIN_S0) & 1UL) || !((NRF_GPIO->OUT >> XCORR_PWM_PIN_S1) & 1UL));
			//}
			//else if(var_mux_state == 2) {
			//	 while(((NRF_GPIO->OUT >> XCORR_PWM_PIN_S0) & 1UL) || !((NRF_GPIO->OUT >> XCORR_PWM_PIN_S1) & 1UL));
			//}
			//else if(var_mux_state == 3) {
			//	 while(!((NRF_GPIO->OUT >> XCORR_PWM_PIN_S0) & 1UL) || ((NRF_GPIO->OUT >> XCORR_PWM_PIN_S1) & 1UL));
			////}
			
			tx_delay[channel_idx] = (((nrf_gpio_pin_read(XCORR_DATA_PIN_0) << 0) & 0x01) |
		        ((nrf_gpio_pin_read(XCORR_DATA_PIN_1) << 1) & 0x02) |
		        ((nrf_gpio_pin_read(XCORR_DATA_PIN_2) << 2) & 0x04) |
		        ((nrf_gpio_pin_read(XCORR_DATA_PIN_3) << 3) & 0x08) |
		        ((nrf_gpio_pin_read(XCORR_DATA_PIN_4) << 4) & 0x10) |
		        ((nrf_gpio_pin_read(XCORR_DATA_PIN_5) << 5) & 0x20) |
		        ((nrf_gpio_pin_read(XCORR_DATA_PIN_6) << 6) & 0x40) |
						((nrf_gpio_pin_read(XCORR_DATA_PIN_7) << 7) & 0x80));
			channel_idx = channel_idx + 1;
		}
		
		// Change states
		if(!var_mux_state)
		{
			S0_high();
			S1_high();
			var_mux_state = var_mux_state + 1;
		}
		else if(var_mux_state == 1)
		{
			S0_low();
			S1_high();
			var_mux_state = var_mux_state + 1;
		}
		else if(var_mux_state == 2)
		{
			S0_high();
			S1_low();
			var_mux_state = var_mux_state + 1;
		}
		else if(var_mux_state == 3)
		{
			S0_low();
			S1_low();
			
			// Read from final channel - Get ready for next set of readings
			var_mux_state = 0;
			sample_index = sample_index + 1;
			channel_idx = 0;
		}
		
		// If we have the required number of samples per channel, then transmit and start reading next window.
		if(sample_index >= XCORR_NUM_TO_AVERAGE)
		{
			sample_index = 0;
			channel_idx = 0;
			//tx_delay[0] = curr_delay[0] / XCORR_NUM_TO_AVERAGE;
			//tx_delay[1] = curr_delay[1] / XCORR_NUM_TO_AVERAGE;
			//tx_delay[2] = curr_delay[2] / XCORR_NUM_TO_AVERAGE;
			//tx_delay[3] = curr_delay[3] / XCORR_NUM_TO_AVERAGE;
			//curr_delay[0] = 0;
			//curr_delay[1] = 0;
			//curr_delay[2] = 0;
			//curr_delay[3] = 0;
			tx_window[5] = tx_delay[1];
			tx_window[6] = tx_delay[2];
			tx_window[7] = tx_delay[3];
			tx_window[15] = tx_delay[0];
			//memcpy(&tx_window[5], &tx_delay[0], 1);
			//memcpy(&tx_window[6], &tx_delay[1], 1);
			//memcpy(&tx_window[7], &tx_delay[2], 1);
			//memcpy(&tx_window[15], &tx_delay[0], 1);
			tx_flag = 1;
		}
		
		else {
			// Re enable interrupt only if we are not going to be transmitting
			nrf_drv_gpiote_in_event_enable(XCORR_PWM_INTERRUPT_PIN, true);
		}
		
		//nrf_drv_gpiote_in_event_enable(XCORR_PWM_INTERRUPT_PIN, true);
	
	}
}

// Setup the input GPIO, receiving the PWM output from the nordic BLE chip (PWM generation and this receive is done on the same chip)
// For doing synchronous reads.
void xcorr_pwm_input_enable(void)
{
		ret_code_t err_code;

		// Initialize GPIO
    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);
	
		// Setup PIN
		nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
    //in_config.pull = NRF_GPIO_PIN_PULLUP;
		//in_config.is_watcher = true;

    err_code = nrf_drv_gpiote_in_init(XCORR_PWM_INTERRUPT_PIN, &in_config, xcorr_pwm_input_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(XCORR_PWM_INTERRUPT_PIN, true);
}

// Disable input GPIO for receiving PWM output from the nordic BLE chip
void xcorr_pwm_input_disable(void)
{
	nrf_drv_gpiote_in_event_disable(XCORR_PWM_INTERRUPT_PIN);
	nrf_drv_gpiote_in_uninit(XCORR_PWM_INTERRUPT_PIN);
}


// PWM intialization and enable for clock generation to source xcorr chip
static void xcorr_pwm_enable(void)
{	
		nrf_drv_pwm_config_t const config0 =
    {
        .output_pins =
        {
            XCORR_PWM_PIN, // channel 0
            NRF_DRV_PWM_PIN_NOT_USED,             // channel 1
            NRF_DRV_PWM_PIN_NOT_USED,             // channel 2
            NRF_DRV_PWM_PIN_NOT_USED,             // channel 3
        },
        .irq_priority = APP_IRQ_PRIORITY_LOWEST,
        .base_clock   = NRF_PWM_CLK_16MHz,
        .count_mode   = NRF_PWM_MODE_UP,
        .top_value    = XCORR_PWM_TOP_TICK,
        .load_mode    = NRF_PWM_LOAD_INDIVIDUAL,
        .step_mode    = NRF_PWM_STEP_AUTO
    };
		
    // Init PWM without error handler
    APP_ERROR_CHECK(nrf_drv_pwm_init(&m_pwm0, &config0, NULL));
		
		nrf_pwm_values_individual_t seq_values[] = {0, 0, 0, 0};
		
		// Set 50% duty cycle for now
		seq_values->channel_0 = XCORR_PWM_TOGGLE_TICK;
		
		nrf_pwm_sequence_t const seq =
		{
			.values.p_individual = seq_values,
			.length          = NRF_PWM_VALUES_LENGTH(seq_values),
			.repeats         = 0,
			.end_delay       = 0
		};
		
		nrf_drv_pwm_simple_playback(&m_pwm0, &seq, 1, NRF_DRV_PWM_FLAG_LOOP);
}

// PWM disable and uninitialization for clock generation to source xcorr chip
static void xcorr_pwm_disable(void)
{
	nrf_drv_pwm_uninit(&m_pwm0);
}

// Enable the pin used to wake up and cause the stm to sleep
static void stm_exti_init(void)
{
	nrf_gpio_cfg_output(STM_EXTI_PIN);
	/*nrf_gpio_cfg(
            STM_EXTI_PIN,
            NRF_GPIO_PIN_DIR_OUTPUT,
            NRF_GPIO_PIN_INPUT_DISCONNECT,
            NRF_GPIO_PIN_NOPULL,
            NRF_GPIO_PIN_H0H1,
            NRF_GPIO_PIN_NOSENSE);
	*/
	
	stm_exti_low();
}

// Pull stm pin high to generate sleep interrupt on stm
static void stm_exti_high(void)
{
	nrf_gpio_pin_set(STM_EXTI_PIN);
}

// Pull stm pin low to generate wake interrupt on stm
static void stm_exti_low(void)
{
	nrf_gpio_pin_clear(STM_EXTI_PIN);
}

/*
Returns 1 if output read from scanchain matches that of the input; 0 otherwise
*/
static uint8_t ScanChain_rotate(int* input, int Size)
{
  int k = 0;
  
	nrf_gpio_pin_clear(XCORR_SC_i0O1_PIN);
	nrf_gpio_pin_clear(XCORR_SC_LOAD_PIN);

	nrf_delay_us(5000);

  for (k = 0; k < Size; k++)
  {
		nrf_gpio_pin_clear(XCORR_PHI_PIN);
		nrf_gpio_pin_clear(XCORR_PHIB_PIN);

    nrf_delay_us(5000);
    nrf_gpio_pin_set(XCORR_PHI_PIN);
    nrf_gpio_pin_clear(XCORR_PHIB_PIN);
		nrf_delay_us(5000);
		
		if(input[k])
		{
			nrf_gpio_pin_set(XCORR_SC_IN_PIN);
		}
		else
		{
			nrf_gpio_pin_clear(XCORR_SC_IN_PIN);
		}

    nrf_delay_us(5000);
    nrf_gpio_pin_clear(XCORR_PHI_PIN);
    nrf_gpio_pin_clear(XCORR_PHIB_PIN);
    
    nrf_delay_us(1000);
    ScanChain_output[k] = nrf_gpio_pin_read(XCORR_SC_OUT_PIN);

    nrf_delay_us(5000);
    nrf_gpio_pin_clear(XCORR_PHI_PIN);
    nrf_gpio_pin_set(XCORR_PHIB_PIN);

    nrf_delay_us(5000);
    nrf_gpio_pin_clear(XCORR_PHI_PIN);
    nrf_gpio_pin_clear(XCORR_PHIB_PIN);
    

  }
  nrf_delay_us(10000);
  nrf_gpio_pin_set(XCORR_SC_LOAD_PIN);
  nrf_delay_us(10000);
  nrf_gpio_pin_clear(XCORR_SC_LOAD_PIN);
	nrf_delay_us(10000);
	
	// Print out the input and output
	/*for(k = 0; k < Size; k++)
	{
		printf("%x", input[k]);
	}
	printf("\n");
	for(k = 0; k < Size; k++)
	{
		printf("%x", ScanChain_output[k]);
	}*/
	
	
	// Check if input is equal to output
	for (k = 0; k < Size; k++)
	{
		if(ScanChain_output[k] == input[k])
		{
			stm_exti_high();
		}
		else
		{
			stm_exti_low();
			return 0;
			break;
		}
	}
	
	return 1;
}

// Initialize The data input pins + setup pins to the ASIC
static void data_pins_init(void)
{
	nrf_gpio_cfg_input(XCORR_DATA_PIN_0,NRF_GPIO_PIN_NOPULL);
	nrf_gpio_cfg_input(XCORR_DATA_PIN_1,NRF_GPIO_PIN_NOPULL);
	nrf_gpio_cfg_input(XCORR_DATA_PIN_2,NRF_GPIO_PIN_NOPULL);
	nrf_gpio_cfg_input(XCORR_DATA_PIN_3,NRF_GPIO_PIN_NOPULL);
	nrf_gpio_cfg_input(XCORR_DATA_PIN_4,NRF_GPIO_PIN_NOPULL);
	nrf_gpio_cfg_input(XCORR_DATA_PIN_5,NRF_GPIO_PIN_NOPULL);
	nrf_gpio_cfg_input(XCORR_DATA_PIN_6,NRF_GPIO_PIN_NOPULL);
	nrf_gpio_cfg_input(XCORR_DATA_PIN_7,NRF_GPIO_PIN_NOPULL);
	
	
	// Configure setup pins for the ASIC
	nrf_gpio_cfg_output(XCORR_PHI_PIN);
	nrf_gpio_cfg_output(XCORR_PHIB_PIN);
	nrf_gpio_cfg_output(XCORR_SC_IN_PIN);
	nrf_gpio_cfg_output(XCORR_SC_i0O1_PIN);
	nrf_gpio_cfg_output(XCORR_SC_LOAD_PIN);
	nrf_gpio_cfg_output(XCORR_RESETN_PIN);
	nrf_gpio_cfg_input(XCORR_SC_OUT_PIN, NRF_GPIO_PIN_NOPULL);
	
	// Configure input data pins
	/*nrf_gpio_cfg(
            XCORR_DATA_PIN_0,
            NRF_GPIO_PIN_DIR_INPUT,
            NRF_GPIO_PIN_INPUT_CONNECT,
            NRF_GPIO_PIN_PULLUP,
            NRF_GPIO_PIN_H0H1,
            NRF_GPIO_PIN_NOSENSE);
	
	nrf_gpio_cfg(
            XCORR_DATA_PIN_1,
            NRF_GPIO_PIN_DIR_INPUT,
            NRF_GPIO_PIN_INPUT_CONNECT,
            NRF_GPIO_PIN_PULLUP,
            NRF_GPIO_PIN_H0H1,
            NRF_GPIO_PIN_NOSENSE);
						
	nrf_gpio_cfg(
            XCORR_DATA_PIN_2,
            NRF_GPIO_PIN_DIR_INPUT,
            NRF_GPIO_PIN_INPUT_CONNECT,
            NRF_GPIO_PIN_PULLUP,
            NRF_GPIO_PIN_H0H1,
            NRF_GPIO_PIN_NOSENSE);

	nrf_gpio_cfg(
            XCORR_DATA_PIN_3,
            NRF_GPIO_PIN_DIR_INPUT,
            NRF_GPIO_PIN_INPUT_CONNECT,
            NRF_GPIO_PIN_PULLUP,
            NRF_GPIO_PIN_H0H1,
            NRF_GPIO_PIN_NOSENSE);
						
	nrf_gpio_cfg(
            XCORR_DATA_PIN_4,
            NRF_GPIO_PIN_DIR_INPUT,
            NRF_GPIO_PIN_INPUT_CONNECT,
            NRF_GPIO_PIN_PULLUP,
            NRF_GPIO_PIN_H0H1,
            NRF_GPIO_PIN_NOSENSE);
						
	nrf_gpio_cfg(
            XCORR_DATA_PIN_5,
            NRF_GPIO_PIN_DIR_INPUT,
            NRF_GPIO_PIN_INPUT_CONNECT,
            NRF_GPIO_PIN_PULLUP,
            NRF_GPIO_PIN_H0H1,
            NRF_GPIO_PIN_NOSENSE);
						
	nrf_gpio_cfg(
            XCORR_DATA_PIN_6,
            NRF_GPIO_PIN_DIR_INPUT,
            NRF_GPIO_PIN_INPUT_CONNECT,
            NRF_GPIO_PIN_PULLUP,
            NRF_GPIO_PIN_H0H1,
            NRF_GPIO_PIN_NOSENSE);
						
	nrf_gpio_cfg(
            XCORR_DATA_PIN_7,
            NRF_GPIO_PIN_DIR_INPUT,
            NRF_GPIO_PIN_INPUT_CONNECT,
            NRF_GPIO_PIN_PULLUP,
            NRF_GPIO_PIN_H0H1,
            NRF_GPIO_PIN_NOSENSE);*/
}

// Initialize the signal output pins
static void signal_pins_init(void)
{
	//nrf_gpio_cfg_output(XCORR_PWM_PIN_S0);
	//nrf_gpio_cfg_output(XCORR_PWM_PIN_S1);
	nrf_gpio_cfg(
            XCORR_PWM_PIN_S0,
            NRF_GPIO_PIN_DIR_OUTPUT,
            NRF_GPIO_PIN_INPUT_DISCONNECT,
            NRF_GPIO_PIN_NOPULL,
            NRF_GPIO_PIN_H0H1,
            NRF_GPIO_PIN_NOSENSE);
	
	nrf_gpio_cfg(
            XCORR_PWM_PIN_S1,
            NRF_GPIO_PIN_DIR_OUTPUT,
            NRF_GPIO_PIN_INPUT_DISCONNECT,
            NRF_GPIO_PIN_NOPULL,
            NRF_GPIO_PIN_H0H1,
            NRF_GPIO_PIN_NOSENSE);
	
	S0_low();
	S1_low();
}

static void S0_high(void) {
	nrf_gpio_pin_set(XCORR_PWM_PIN_S0);
}

static void S0_low(void) {
	nrf_gpio_pin_clear(XCORR_PWM_PIN_S0);
}

static void S0_toggle(void) {
	nrf_gpio_pin_toggle(XCORR_PWM_PIN_S0);
}

static void S1_high(void) {
	nrf_gpio_pin_set(XCORR_PWM_PIN_S1);
}

static void S1_low(void) {
	nrf_gpio_pin_clear(XCORR_PWM_PIN_S1);
}

static void S1_toggle(void) {
	nrf_gpio_pin_toggle(XCORR_PWM_PIN_S1);
}

// Tx current sample
static void tx_data(void) {
	
	uint32_t index;
	uint32_t tx_len;
	
	index = 0;
	tx_len = 0;
	
	// Keep transmitting until whole window is transmitted
	while(index < sizeof(tx_window)) {
		
		tx_len = BLE_NUS_MAX_DATA_LEN;
		if(index + tx_len >= sizeof(tx_window)) {
			tx_len = sizeof(tx_window) - index;
		}
		while(ble_nus_string_send(&m_nus, &tx_window[index], tx_len) != NRF_SUCCESS);
		index = index + tx_len;
		//ble_nus_string_send(&m_nus, &tx_window[index], tx_len);
	}
	
	// Re enable interrupt after transmission complete
	nrf_drv_gpiote_in_event_enable(XCORR_PWM_INTERRUPT_PIN, true);
}

// Setup and configure ASIC
static void asic_setup(void) {
	
	// trying to input scanchain until output matches input
	/*while(1)
	{
		nrf_delay_us(50000);
	
		// Low frequency input to start asic
		if(ScanChain_rotate(ScanChain_input, SC_size))
		{
			break;
		}
		
	}*/
	
	/*nrf_delay_us(5000);
	ScanChain_rotate(ScanChain_input, SC_size);
	nrf_gpio_pin_set(XCORR_RESETN_PIN);
	nrf_delay_us(5000);
	nrf_gpio_pin_clear(XCORR_RESETN_PIN);
	nrf_delay_us(5000);
	nrf_gpio_pin_set(XCORR_RESETN_PIN);
	
	
	nrf_delay_us(5000);
	ScanChain_rotate(ScanChain_input, SC_size);
	
	// Hard reset
	nrf_gpio_pin_set(XCORR_RESETN_PIN);
	nrf_delay_us(5000);
	nrf_gpio_pin_clear(XCORR_RESETN_PIN);
	nrf_delay_us(5000);
	nrf_gpio_pin_set(XCORR_RESETN_PIN);
	
	nrf_gpio_pin_clear(XCORR_PHI_PIN);
	nrf_gpio_pin_clear(XCORR_PHIB_PIN);
	nrf_gpio_pin_clear(XCORR_SC_IN_PIN);
	nrf_gpio_pin_clear(XCORR_SC_i0O1_PIN);
	nrf_gpio_pin_clear(XCORR_SC_LOAD_PIN);*/
	
	// Keep inputting scanchain until it is correct
	nrf_delay_us(5000);
	while(!ScanChain_rotate(ScanChain_input, SC_size))
	{
		nrf_delay_us(5000);
	}
	nrf_delay_us(5000);
	nrf_gpio_pin_set(XCORR_RESETN_PIN);
	nrf_delay_us(5000);
	nrf_gpio_pin_clear(XCORR_RESETN_PIN);
	nrf_delay_us(5000);
	nrf_gpio_pin_set(XCORR_RESETN_PIN);
	
	// MUX setup
	//S0_high();
	//S1_low();
	S0_high();
	S1_high();
}


/**@brief Application main function.
 */
int main(void)
{
    uint32_t err_code;
    bool erase_bonds;
	
	  // Start high frequency clock
		NRF_CLOCK->TASKS_HFCLKSTART = 1; 
    while(NRF_CLOCK->EVENTS_HFCLKSTARTED == 0); 

    // Initialize.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);
    //uart_init();
	  //printf("\r\nUART Start!\r\n");

    //buttons_leds_init(&erase_bonds);
    ble_stack_init();
    gap_params_init();
    services_init();
    advertising_init();
	  
		// Initialize reading from xcorr chip
		stm_exti_init();
		stm_exti_high();
		data_pins_init();
		signal_pins_init();
		
		nrf_gpio_pin_clear(XCORR_PHI_PIN);
		nrf_gpio_pin_clear(XCORR_PHIB_PIN);
		nrf_gpio_pin_clear(XCORR_SC_IN_PIN);
		nrf_gpio_pin_clear(XCORR_SC_i0O1_PIN);
		nrf_gpio_pin_clear(XCORR_SC_LOAD_PIN);
		nrf_gpio_pin_clear(XCORR_RESETN_PIN);
	
    //printf("\r\nUART Start!\r\n");
    err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
		
		/*nrf_delay_us(1000000);
		asic_setup();
		xcorr_pwm_enable();
		xcorr_pwm_input_enable();*/

    // Enter main loop.
    for (;;)
    {
      //power_manage();
			  
			// Start message received, wakeup STM to begin feature collection
			if(start)
			{
				start = 0;
				running = 1;
				asic_setup();
				xcorr_pwm_enable();
				xcorr_pwm_input_enable();
				//err_code = handle_handshake_start();
			  //APP_ERROR_CHECK(err_code);
			}
			
			// Stop message received, put STM to sleep
			else if(stop)
			{
				stop = 0;
				running = 0;
				xcorr_pwm_input_disable();
				xcorr_pwm_disable();
				err_code = handle_handshake_end();
			  APP_ERROR_CHECK(err_code);
			}
				//err_code = ble_nus_string_send(&m_nus, test_data, 1);
				
			else if(tx_flag) {
				tx_flag = 0;
				
				if(pwm_polarity_flag > 0)
				{
					//stm_exti_high();
					pwm_polarity_flag = 0;
				}
				else
				{
					//stm_exti_low();
					pwm_polarity_flag = 1;
				}
				
				tx_data();
			}
			
			//else if(!running) {
			//	power_manage();
			//}
    }
}


/**
 * @}
 */
