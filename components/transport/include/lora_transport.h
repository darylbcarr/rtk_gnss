#pragma once

#include "transport.h"
#include "driver/uart.h"
#include "freertos/ringbuf.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uart_port_t uart;
    int         tx_pin, rx_pin;
    int         m0_pin, m1_pin, aux_pin;
} lora_transport_cfg_t;

typedef struct {
    transport_t          base;    /* vtable — MUST be first */
    lora_transport_cfg_t cfg;
    RingbufHandle_t      rxbuf;
} lora_transport_t;

/*
 * Wire up the vtable and copy cfg. Call before transport_init().
 * transport_init() will call lora_init() internally.
 */
void lora_transport_create(lora_transport_t *t, const lora_transport_cfg_t *cfg);

#ifdef __cplusplus
}
#endif
