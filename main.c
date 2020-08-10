/**
 * Copyright (c) 2014 - 2018, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
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
 *
 */
/**
 * @brief BLE LED Button Service central and client application main file.
 *
 * This file contains the source code for a sample client application using the LED Button service.
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "nrf_pwr_mgmt.h"
#include "app_timer.h"
#include "boards.h"
#include "bsp.h"
//#include "bsp_btn_ble.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_db_discovery.h"
#include "ble_lbs_c.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_scan.h"


//added
#include "nrf_fstorage.h"
#include "ble_nus_c.h"
#include "nrf_delay.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define APP_BLE_CONN_CFG_TAG        1                                   /**< Tag that identifies the BLE configuration of the SoftDevice. */
#define APP_BLE_OBSERVER_PRIO       3                                   /**< BLE observer priority of the application. There is no need to modify this value. */
#define APP_SOC_OBSERVER_PRIO       1                                   /**< SoC observer priority of the application. There is no need to modify this value. */

#define CENTRAL_SCANNING_LED            BSP_BOARD_LED_0                     /**< Scanning LED will be on when the device is scanning. */
#define CENTRAL_CONNECTED_LED           BSP_BOARD_LED_1                     /**< Connected LED will be on when the device is connected. */
#define LEDBUTTON_LED                   BSP_BOARD_LED_2                     /**< LED to indicate a change of state of the the Button characteristic on the peer. */

#define SCAN_INTERVAL                   0x00A0                              /**< Determines scan interval in units of 0.625 millisecond. */
#define SCAN_WINDOW                     0x0050                              /**< Determines scan window in units of 0.625 millisecond. */
#define SCAN_DURATION                   0x0000                              /**< Timout when scanning. 0x0000 disables timeout. */

#define MIN_CONNECTION_INTERVAL         MSEC_TO_UNITS(7.5, UNIT_1_25_MS)    /**< Determines minimum connection interval in milliseconds. */
#define MAX_CONNECTION_INTERVAL         MSEC_TO_UNITS(30, UNIT_1_25_MS)     /**< Determines maximum connection interval in milliseconds. */
#define SLAVE_LATENCY                   0                                   /**< Determines slave latency in terms of connection events. */
#define SUPERVISION_TIMEOUT             MSEC_TO_UNITS(4000, UNIT_10_MS)     /**< Determines supervision time-out in units of 10 milliseconds. */

#define LEDBUTTON_BUTTON_PIN            BSP_BUTTON_0                        /**< Button that will write to the LED characteristic of the peer */
#define BUTTON_DETECTION_DELAY          APP_TIMER_TICKS(50)                 /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

// WR4119 cmds
#define TARGET_UUID                     0xFE9A 
#define WR4119_CMD_LENGHT               0x0007 //12
static uint8_t wr4119_cmd_pulse_start[7] = { 0xAB, 0x00, 0x04, 0xFF, 0x31, 0x09, 0x01 };
static uint8_t wr4119_cmd_pulse_stop[7] = { 0xAB, 0x00, 0x04, 0xFF, 0x31, 0x09, 0x00 };

//AB0004FF312101
static uint8_t wr4119_cmd_pressure_start[7] = { 0xAB, 0x00, 0x04, 0xFF, 0x31, 0x21, 0x01 };
static uint8_t wr4119_cmd_pressure_stop[7] = { 0xAB, 0x00, 0x04, 0xFF, 0x31, 0x21, 0x00 };


BLE_NUS_C_DEF(m_ble_nus_c); 
//BLE_LBS_C_DEF(m_ble_lbs_c);                                     /**< Main structure used by the LBS client module. */
NRF_BLE_GATT_DEF(m_gatt);                                       /**< GATT module instance. */
BLE_DB_DISCOVERY_DEF(m_db_disc);                                /**< DB discovery module instance. */
NRF_BLE_SCAN_DEF(m_scan);                                       /**< Scanning module instance. */

static uint16_t m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - OPCODE_LENGTH - HANDLE_LENGTH;

static bool wr4119_connected;
static bool wr4119_pulse_measured;
static bool wr4119_pressure_measured;

static bool                  m_whitelist_disabled;          /**< True if the whitelist is temporarily disabled. */
static bool                  m_memory_access_in_progress;

static ble_gap_scan_params_t const m_scan_param =                 /**< Scan parameters requested for scanning and connection. */
{
    .active        = 0x00,
    .interval      = NRF_BLE_SCAN_SCAN_INTERVAL,
    .window        = NRF_BLE_SCAN_SCAN_WINDOW,
    .filter_policy  = BLE_GAP_SCAN_FP_ACCEPT_ALL,
    .timeout       = NRF_BLE_SCAN_SCAN_DURATION,
    .scan_phys     = BLE_GAP_PHY_1MBPS,
    .extended      = 0,
};

typedef enum
{
    BLE_NO_SCAN,                                                  /**< No advertising running. */
    BLE_WHITELIST_SCAN,                                           /**< Advertising with whitelist. */
    BLE_FAST_SCAN,                                                /**< Fast advertising running. */
} ble_advertising_mode_t;

static char const m_target_periph_name[] = "WR4119";     /**< Name of the device we try to connect to. This name is searched in the scan report data*/
static bool is_connect_per_addr = true;            /**< If you want to connect to a peripheral with a given address, set this to true and put the correct address in the variable below. */

//.addr      = {0xEF, 0xCC, 0x8F, 0xF8, 0x87, 0x50}, but need to reverse
static ble_gap_addr_t const m_target_periph_addr =
{
    .addr_type = BLE_GAP_ADDR_TYPE_RANDOM_STATIC,
    .addr      = {0x50, 0x87, 0xF8, 0x8F, 0xCC, 0xEF} // reversed!
};

//bes337's custom whitelist (addrlist)
static bool wr_addrlist_is_running = false;
static ble_gap_addr_t wr_addrlist_addrs[BLE_GAP_WHITELIST_ADDR_MAX_COUNT]; // 8 devices
static ble_gap_addr_t const * wr_addrlist_addr_ptrs[BLE_GAP_WHITELIST_ADDR_MAX_COUNT]; // 8 devices
static uint8_t wr_addr_count = 0;

ret_code_t wr_ble_addrlist_add(ble_gap_addr_t *addr, uint8_t * addrlist_count)
{
    ret_code_t ret;
    if (wr_addr_count >= BLE_GAP_WHITELIST_ADDR_MAX_COUNT)
    {
        return NRF_ERROR_DATA_SIZE;
    }

    for (uint32_t i = 0; i < BLE_GAP_WHITELIST_ADDR_MAX_COUNT; i++)
    {
        if (memcmp(&wr_addrlist_addrs[i], addr, sizeof(ble_gap_addr_t))==0)
            return NRF_ERROR_INVALID_PARAM;
    }

    memcpy(&wr_addrlist_addrs[wr_addr_count], addr, sizeof(ble_gap_addr_t));
    wr_addr_count++;
    *addrlist_count = wr_addr_count;


    return NRF_SUCCESS;
}

ret_code_t wr_ble_addrlist_enable(void)
{
    ret_code_t ret;
    wr_addrlist_is_running = true;
    if (wr_addr_count == 0)
    {
        return NRF_ERROR_DATA_SIZE;
    }
    for (uint32_t i = 0; i < BLE_GAP_WHITELIST_ADDR_MAX_COUNT; i++)
    {
        wr_addrlist_addr_ptrs[i] = &wr_addrlist_addrs[i];
    }
    
    ret = sd_ble_gap_whitelist_set(wr_addrlist_addr_ptrs, wr_addr_count);
    APP_ERROR_CHECK(ret);
    return NRF_SUCCESS;
}

ret_code_t wr_ble_addrlist_clear(void)
{
    ret_code_t ret;
    memset(wr_addrlist_addrs, 0, sizeof(wr_addrlist_addrs) );
    wr_addr_count = 0;

    ret = sd_ble_gap_whitelist_set(NULL, 0);
    APP_ERROR_CHECK(ret);

    wr_addrlist_is_running = false;
    return ret;
}

// for debuging
static void wr_ble_addrlist_logshow(void)
{
    NRF_LOG_INFO("[ADDRLIST]: ");
    //for (uint32_t i = 0; i < BLE_GAP_WHITELIST_ADDR_MAX_COUNT; i++)
    //{
        NRF_LOG_INFO("%x", wr_addrlist_addr_ptrs[0]->addr[5]);
        NRF_LOG_INFO("%x", wr_addrlist_addr_ptrs[0]->addr[4]);
        NRF_LOG_INFO("%x", wr_addrlist_addr_ptrs[0]->addr[3]);
        NRF_LOG_INFO("%x", wr_addrlist_addr_ptrs[0]->addr[2]);
        NRF_LOG_INFO("%x", wr_addrlist_addr_ptrs[0]->addr[1]);
        NRF_LOG_INFO("%x", wr_addrlist_addr_ptrs[0]->addr[0]);
    //}
}

/**@brief Function to handle asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num     Line number of the failing ASSERT call.
 * @param[in] p_file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}

static void ble_nus_wr4119_processing(void);

/**@brief Function for the LEDs initialization.
 *
 * @details Initializes all LEDs used by the application.
 */
static void leds_init(void)
{
    bsp_board_init(BSP_INIT_LEDS);
}


/**@brief Function to start scanning.
 */
static void scan_start(void)
{
    ret_code_t err_code;
    // If there is any pending write to flash, defer scanning until it completes.
    if (nrf_fstorage_is_busy(NULL))
    {
        m_memory_access_in_progress = true;
        return;
    }
    err_code = nrf_ble_scan_start(&m_scan);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code;

    // For readability.
    ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;
    //ble_gap_evt_adv_report_t const * p_gap_rssi = &p_gap_evt->adv_report;

    switch (p_ble_evt->header.evt_id)
    {
        // Upon connection, check which peripheral has connected (HR or RSC), initiate DB
        // discovery, update LEDs status and resume scanning if necessary. */
        case BLE_GAP_EVT_CONNECTED:
        {
            NRF_LOG_INFO("Connected.");

            err_code = ble_nus_c_handles_assign(&m_ble_nus_c, p_ble_evt->evt.gap_evt.conn_handle, NULL);
            APP_ERROR_CHECK(err_code);

            //err_code = ble_lbs_c_handles_assign(&m_ble_lbs_c, p_gap_evt->conn_handle, NULL);
            //APP_ERROR_CHECK(err_code);

            err_code = ble_db_discovery_start(&m_db_disc, p_gap_evt->conn_handle);
            APP_ERROR_CHECK(err_code);

            wr4119_connected = true;
            

        } break;

        // Upon disconnection, reset the connection handle of the peer which disconnected, update
        // the LEDs status and start scanning again.
        case BLE_GAP_EVT_DISCONNECTED:
        {
            NRF_LOG_INFO("Disconnected.");
            wr4119_connected = false;
            wr4119_pulse_measured = false;
            wr4119_pressure_measured = false;
            scan_start();
        } break;

        case BLE_GAP_EVT_ADV_REPORT:
        {
            NRF_LOG_RAW_HEXDUMP_INFO (m_scan.scan_buffer.p_data, m_scan.scan_buffer.len);
            NRF_LOG_RAW_INFO ("RSSI: %d\r\n", p_gap_evt->params.adv_report.rssi);
            NRF_LOG_RAW_INFO ("\r\n");
        } break;


        case BLE_GAP_EVT_TIMEOUT:
        {
            // We have not specified a timeout for scanning, so only connection attemps can timeout.
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
                NRF_LOG_DEBUG("Connection request timed out.");
            }
        } break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
        {
            // Accept parameters requested by peer.
            err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                        &p_gap_evt->params.conn_param_update_request.conn_params);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTC_EVT_TIMEOUT:
        {
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTS_EVT_TIMEOUT:
        {
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
        } break;

        default:
            // No implementation needed.
            break;
    }
}

/**
 * @brief SoftDevice SoC event handler.
 *
 * @param[in] evt_id    SoC event.
 * @param[in] p_context Context.
 */
static void soc_evt_handler(uint32_t evt_id, void * p_context)
{
    switch (evt_id)
    {
        case NRF_EVT_FLASH_OPERATION_SUCCESS:
        /* fall through */
        case NRF_EVT_FLASH_OPERATION_ERROR:

            if (m_memory_access_in_progress)
            {
                m_memory_access_in_progress = false;
                scan_start();
            }
            break;

        default:
            // No implementation needed.
            break;
    }
}

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupts.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
    NRF_SDH_SOC_OBSERVER(m_soc_observer, APP_SOC_OBSERVER_PRIO, soc_evt_handler, NULL);
}


/**@brief Function for disabling the use of the whitelist for scanning.
 */
static void whitelist_disable(void)
{
    if (!m_whitelist_disabled)
    {
        NRF_LOG_INFO("Whitelist temporarily disabled.");
        m_whitelist_disabled = true;
        nrf_ble_scan_stop();
        scan_start();
    }
}



/**@brief Function for handling Scaning events.
 *
 * @param[in]   p_scan_evt   Scanning event.
 */
static void scan_evt_handler(scan_evt_t const * p_scan_evt)
{
    ret_code_t err_code;

    switch(p_scan_evt->scan_evt_id)
    {
        case NRF_BLE_SCAN_EVT_CONNECTING_ERROR:
            err_code = p_scan_evt->params.connecting_err.err_code;
            APP_ERROR_CHECK(err_code);
            break;
        default:
          break;
    }
}

/**@brief Function for handling characters received by the Nordic UART Service (NUS).
 *
 * @details This function takes a list of characters of length data_len and prints the characters out on UART.
 *          If @ref ECHOBACK_BLE_UART_DATA is set, the data is sent back to sender.
 */
static void ble_nus_chars_received_uart_print(uint8_t * p_data, uint16_t data_len)
{
    ret_code_t ret_val;

    NRF_LOG_INFO("Receiving data.");
    NRF_LOG_RAW_HEXDUMP_INFO(p_data, data_len);
    /*
    for (uint32_t i = 0; i < data_len; i++)
    {
        do
        {
            ret_val = app_uart_put(p_data[i]);
            if ((ret_val != NRF_SUCCESS) && (ret_val != NRF_ERROR_BUSY))
            {
                NRF_LOG_ERROR("app_uart_put failed for index 0x%04x.", i);
                APP_ERROR_CHECK(ret_val);
            }
        } while (ret_val == NRF_ERROR_BUSY);
    }
    if (p_data[data_len-1] == '\r')
    {
        while (app_uart_put('\n') == NRF_ERROR_BUSY);
    }
    */
    // ECHOBACK_BLE_UART_DATA (отправляет полученные данные обратно)
    /*
        //Turn on PULSE
        uint8_t * myp_data = wr4119_cmd_pulse_start;
        uint16_t mydata_len = WR4119_CMD_LENGHT;
        NRF_LOG_RAW_HEXDUMP_INFO(myp_data, mydata_len);

        // Send data back to the peripheral.
        
        do
        {
            
            ret_val = ble_nus_c_string_send(&m_ble_nus_c, myp_data, mydata_len);
            if ((ret_val != NRF_SUCCESS) && (ret_val != NRF_ERROR_BUSY))
            {
                NRF_LOG_ERROR("Failed sending NUS message. Error 0x%x. ", ret_val);
                APP_ERROR_CHECK(ret_val);
            }
            
        } while (ret_val == NRF_ERROR_BUSY);
    */
        
}

/**@brief Send shit on NUS
 */
static void ble_nus_wr4119_send_command(uint8_t * p_data, uint16_t data_len)
{
    ret_code_t ret_val;

    NRF_LOG_INFO("Sending data.");
    NRF_LOG_RAW_HEXDUMP_INFO(p_data, data_len);
    //Turn on PULSE
    //uint8_t * myp_data = wr4119_cmd_pulse_start;
    //uint16_t mydata_len = WR4119_CMD_LENGHT;
    //NRF_LOG_RAW_HEXDUMP_INFO(myp_data, mydata_len);

        // Send data back to the peripheral.
     
        do
        {
            
            ret_val = ble_nus_c_string_send(&m_ble_nus_c, p_data, data_len);
            if ((ret_val != NRF_SUCCESS) && (ret_val != NRF_ERROR_BUSY))
            {
                NRF_LOG_ERROR("Failed sending NUS message. Error 0x%x. ", ret_val);
                APP_ERROR_CHECK(ret_val);
            }
            
        } while (ret_val == NRF_ERROR_BUSY);
  
}



static void ble_nus_wr4119_processing(void)
{
    //wr4119_connected = false;
    //wr4119_pressure_measured = false; todo
    ble_nus_wr4119_send_command(wr4119_cmd_pulse_start, WR4119_CMD_LENGHT);
    NRF_LOG_INFO("delay."); //add delay
    nrf_delay_ms(60000);
    ble_nus_wr4119_send_command(wr4119_cmd_pulse_stop, WR4119_CMD_LENGHT);
    wr4119_pulse_measured = true;
    NRF_LOG_INFO("delay done.");
    ble_nus_wr4119_send_command(wr4119_cmd_pressure_start, WR4119_CMD_LENGHT);
    NRF_LOG_INFO("delay."); //add delay
    nrf_delay_ms(60000);
    ble_nus_wr4119_send_command(wr4119_cmd_pressure_stop, WR4119_CMD_LENGHT);
    wr4119_pressure_measured = true;
    NRF_LOG_INFO("delay done.");
}

/**@brief Callback handling Nordic UART Service (NUS) client events.
 *
 * @details This function is called to notify the application of NUS client events.
 *
 * @param[in]   p_ble_nus_c   NUS client handle. This identifies the NUS client.
 * @param[in]   p_ble_nus_evt Pointer to the NUS client event.
 */

/**@snippet [Handling events from the ble_nus_c module] */
static void ble_nus_c_evt_handler(ble_nus_c_t * p_ble_nus_c, ble_nus_c_evt_t const * p_ble_nus_evt)
{
    ret_code_t err_code;

    switch (p_ble_nus_evt->evt_type)
    {
        case BLE_NUS_C_EVT_DISCOVERY_COMPLETE:
        {
            NRF_LOG_INFO("Discovery complete.");
            err_code = ble_nus_c_handles_assign(p_ble_nus_c, p_ble_nus_evt->conn_handle, &p_ble_nus_evt->handles);
            APP_ERROR_CHECK(err_code);

            err_code = ble_nus_c_tx_notif_enable(p_ble_nus_c);
            APP_ERROR_CHECK(err_code);
            NRF_LOG_INFO("Connected to device with Nordic UART Service.");
            // замерить давление и пульс
            ble_nus_wr4119_processing();
         } break;

        case BLE_NUS_C_EVT_NUS_TX_EVT:
        {
            //NRF_LOG_INFO("Got NUS TX:");
            //NRF_LOG_RAW_HEXDUMP_INFO(p_ble_nus_evt->p_data, p_ble_nus_evt->data_len);
            //NRF_LOG_HEXDUMP_DEBUG(p_ble_nus_evt->p_data, p_ble_nus_evt->data_len);
            ble_nus_chars_received_uart_print(p_ble_nus_evt->p_data, p_ble_nus_evt->data_len);
        } break;

        case BLE_NUS_C_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected.");
            scan_start();
            break;
    }
}
/**@snippet [Handling events from the ble_nus_c module] */

/**@brief Function for initializing the Nordic UART Service (NUS) client. */
static void nus_c_init(void)
{
    ret_code_t       err_code;
    ble_nus_c_init_t init;

    init.evt_handler = ble_nus_c_evt_handler;

    err_code = ble_nus_c_init(&m_ble_nus_c, &init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling database discovery events.
 *
 * @details This function is callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function should forward the events
 *          to their respective services.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */
static void db_disc_handler(ble_db_discovery_evt_t * p_evt)
{
    ble_nus_c_on_db_disc_evt(&m_ble_nus_c, p_evt);
}

/**@brief Database discovery initialization.
 */
static void db_discovery_init(void)
{
    ret_code_t err_code = ble_db_discovery_init(db_disc_handler);
    APP_ERROR_CHECK(err_code);
}


static void whitelist_load()
{
    ret_code_t   ret;
    /*pm_peer_id_t peers[8];
    uint32_t     peer_cnt;

    memset(peers, PM_PEER_ID_INVALID, sizeof(peers));
    peer_cnt = (sizeof(peers) / sizeof(pm_peer_id_t));
    
    // Load all peers from the flash and whitelist them.
    peer_list_get(peers, &peer_cnt);

    ret = pm_whitelist_set(peers, peer_cnt);
    APP_ERROR_CHECK(ret);

    // Setup the list of device identities.
    // Some SoftDevices do not support this feature.
    ret = pm_device_identities_list_set(peers, peer_cnt);
    if (ret != NRF_ERROR_NOT_SUPPORTED)
    {
        APP_ERROR_CHECK(ret);
    }
    */
}

/**@brief Function for initializing the log.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing the timer.
 */
static void timer_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the Power manager. */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


static void scan_init(void)
{
    ret_code_t          err_code;
    nrf_ble_scan_init_t init_scan;

    memset(&init_scan, 0, sizeof(init_scan));

    init_scan.connect_if_match = true;
    init_scan.conn_cfg_tag     = APP_BLE_CONN_CFG_TAG;
    //init_scan.p_scan_param     = &m_scan_param;

    err_code = nrf_ble_scan_init(&m_scan, &init_scan, scan_evt_handler);
    APP_ERROR_CHECK(err_code);

    // установка фильтров
    //err_code = nrf_ble_scan_filter_set(&m_scan, SCAN_NAME_FILTER, m_target_periph_name);
    //APP_ERROR_CHECK(err_code);
    
    // NRF_BLE_SCAN_ADDR_FILTER NRF_BLE_SCAN_ALL_FILTER
    err_code = nrf_ble_scan_filters_enable(&m_scan, NRF_BLE_SCAN_ALL_FILTER, false);
    APP_ERROR_CHECK(err_code);
    
    if (is_connect_per_addr)
    {
        err_code = nrf_ble_scan_filter_set(&m_scan, SCAN_ADDR_FILTER, m_target_periph_addr.addr);
        if (err_code != NRF_SUCCESS) 
        {
            NRF_LOG_INFO("error-%d",err_code); // 7 = NRF_ERROR_INVALID_PARAM
        }
        APP_ERROR_CHECK(err_code);
    }
}

/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED)
    {
        NRF_LOG_INFO("ATT MTU exchange completed.");

        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        NRF_LOG_INFO("Ble NUS max data length set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
    }
}

/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    //ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);
    
    //dunno
    err_code = nrf_ble_gatt_att_mtu_central_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details Handle any pending log operation(s), then sleep until the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
    //NRF_LOG_FLUSH();
    //nrf_pwr_mgmt_run();
}

static void test_set_whitelist(void)
{
    ret_code_t ret;
    ble_gap_addr_t whitelist_addrs;
    uint8_t whitelist_num = 0;
    whitelist_addrs.addr_type = BLE_GAP_ADDR_TYPE_RANDOM_STATIC;
    whitelist_addrs.addr[5] = 0xcc;
    whitelist_addrs.addr[4] = 0xcc;
    whitelist_addrs.addr[3] = 0xcc;
    whitelist_addrs.addr[2] = 0xcc;
    whitelist_addrs.addr[1] = 0xcc;
    whitelist_addrs.addr[0] = 0xcc;
    ret = wr_ble_addrlist_add(&whitelist_addrs, &whitelist_num);
    NRF_LOG_INFO("addlist num: %u", whitelist_num);
    APP_ERROR_CHECK(ret);
}

int main(void)
{
    // Initialize.
    log_init();
    timer_init();
    leds_init();
    db_discovery_init();
    power_management_init();
    ble_stack_init();
    gatt_init();
    nus_c_init();
    scan_init();
    // led stuff are unused
    //lbs_c_init();

    //whitelist_load();

    ret_code_t ret;
    ret = wr_ble_addrlist_clear();
    APP_ERROR_CHECK(ret);
    
    test_set_whitelist();
    
    ret = wr_ble_addrlist_enable();
    APP_ERROR_CHECK(ret);
    wr_ble_addrlist_logshow();
    
    // Start execution.
    NRF_LOG_INFO("Blinky CENTRAL example started.");
       //test_set_whitelist();
    scan_start();

    // Turn on the LED to signal scanning.
    //bsp_board_led_on(CENTRAL_SCANNING_LED);

    // Enter main loop.
    for (;;)
    {
        idle_state_handle();
    }
}
