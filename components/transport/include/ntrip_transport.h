#pragma once

#include "transport.h"
#include "ntrip.h"
#include "freertos/ringbuf.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    transport_t     base;    /* vtable — MUST be first */
    ntrip_cfg_t     cfg;
    RingbufHandle_t rxbuf;
} ntrip_transport_t;

/*
 * Wire up the vtable and copy cfg. Call before transport_init().
 * WiFi must be connected before transport_init() is called.
 */
void ntrip_transport_create(ntrip_transport_t *t, const ntrip_cfg_t *cfg);

#ifdef __cplusplus
}
#endif
