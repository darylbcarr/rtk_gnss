#pragma once

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"
#include "driver/uart.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Called from the RX task for every packet received over LoRa. */
typedef void (*lora_rx_cb_t)(const uint8_t *buf, size_t len, void *ctx);

/*
 * Initialise the Ebyte E22-900T30D LoRa module.
 *
 * Configures the UART, drives M0/M1 to config mode, programs the module
 * registers (TX power, channel, air rate) via the C2 RAM-write command,
 * then returns to transparent (normal) mode and starts the RX task.
 *
 * cb may be NULL if only transmitting.
 */
esp_err_t lora_init(uart_port_t uart, int tx_pin, int rx_pin,
                    int m0_pin, int m1_pin, int aux_pin,
                    lora_rx_cb_t cb, void *ctx);

/*
 * Transmit len bytes.  Blocks until AUX goes high (module ready), then
 * writes to the UART.  Returns ESP_ERR_TIMEOUT if AUX does not go high
 * within 2 s.
 */
esp_err_t lora_send(const uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif
