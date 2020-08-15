/**
 * @brief Module for handling the BLE whitelist.
 */

#ifndef ADDRLIST_H__
#define ADDRLIST_H__

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "ble.h"
#include "ble_gap.h"
#include "app_util.h"
#include "sdk_errors.h"
#include "sdk_config.h"

#ifdef __cplusplus
extern "C"
{
#endif

ret_code_t wr_ble_addrlist_add(ble_gap_addr_t *addr, uint8_t * count);

ret_code_t wr_ble_addrlist_enable(void);

ret_code_t wr_ble_addrlist_clear(void);

bool wr_ble_addrlist_is_running(void);

uint8_t wr_ble_addrlist_count(void);

void wr_ble_addrlist_logshow(void);


#ifdef __cplusplus
}
#endif

#endif // ADDRLIST_H__

/** @} */
