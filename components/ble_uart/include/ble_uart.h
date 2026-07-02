#pragma once

#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Initialise the NimBLE stack and start advertising as Nordic UART Service.
 * device_name — the BLE GAP name shown to scanners (e.g. "RTK-BASE"). */
esp_err_t ble_uart_init(const char *device_name);

/* Send data to a connected central as a NUS TX notification.
 * No-op when no client is connected. */
void ble_uart_send(const char *data, size_t len);

/* Return true while a central is connected. */
bool ble_uart_connected(void);

#ifdef __cplusplus
}
#endif
