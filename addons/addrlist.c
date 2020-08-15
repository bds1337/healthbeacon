/**
 * @brief Addrlist (wishlist) addon for ibeacon project
 */

#include "sdk_common.h"

#include "sdk_config.h"
#include <stdlib.h>

#include <string.h>
#include "app_error.h"
#include "nrf_assert.h"
#include "sdk_macros.h"
#include "ble_advdata.h"

//for debug
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "addrlist.h"

// START OF ADDRLIST LIB
// a bit custom whitelist (addrlist)
static bool wr_addrlist_is_running = false;
static ble_gap_addr_t wr_addrlist_addrs[BLE_GAP_WHITELIST_ADDR_MAX_COUNT]; // 8 устройств максимум
static ble_gap_addr_t const * wr_addrlist_addr_ptrs[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];
static uint8_t wr_addr_count = 0;

ret_code_t wr_ble_addrlist_add(ble_gap_addr_t *addr, uint8_t * addrlist_count)
{
    ret_code_t ret;
    if (wr_addr_count >= BLE_GAP_WHITELIST_ADDR_MAX_COUNT)
        return NRF_ERROR_DATA_SIZE;

    for (uint32_t i = 0; i < BLE_GAP_WHITELIST_ADDR_MAX_COUNT; i++)
    {
        // запрещаю добавлять одинаковые адреса
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
        return NRF_ERROR_DATA_SIZE;

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

uint8_t wr_ble_addrlist_count(void)
{
    return wr_addr_count;
}

bool wr_ble_addrlist_is_running(void)
{
    return wr_addrlist_is_running;
}

// for debuging
void wr_ble_addrlist_logshow(void)
{
    NRF_LOG_INFO("[ADDRLIST]: ");
    for (uint32_t i = 0; i < wr_addr_count; i++)
    {
        NRF_LOG_INFO("%x", wr_addrlist_addr_ptrs[i]->addr[5]);
        NRF_LOG_INFO("%x", wr_addrlist_addr_ptrs[i]->addr[4]);
        NRF_LOG_INFO("%x", wr_addrlist_addr_ptrs[i]->addr[3]);
        NRF_LOG_INFO("%x", wr_addrlist_addr_ptrs[i]->addr[2]);
        NRF_LOG_INFO("%x", wr_addrlist_addr_ptrs[i]->addr[1]);
        NRF_LOG_INFO("%x", wr_addrlist_addr_ptrs[i]->addr[0]);
    }
}
