#include "ntrip_transport.h"
#include "ntrip.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "ntrip_tp";

/* ── RX callback: NTRIP task → ringbuffer ────────────────────────────── */

static void on_rtcm(const uint8_t *buf, size_t len, void *ctx)
{
    RingbufHandle_t rb = (RingbufHandle_t)ctx;
    if (xRingbufferSend(rb, buf, len, 0) != pdTRUE)
        ESP_LOGW(TAG, "rxbuf full — dropped %u RTCM bytes", (unsigned)len);
}

/* ── vtable implementation ───────────────────────────────────────────── */

static esp_err_t ntrip_tp_init(transport_t *t, const void *cfg)
{
    (void)cfg;
    ntrip_transport_t *nt = (ntrip_transport_t *)t;
    nt->rxbuf = xRingbufferCreate(4096, RINGBUF_TYPE_BYTEBUF);
    if (!nt->rxbuf) return ESP_ERR_NO_MEM;
    return ntrip_start(&nt->cfg, on_rtcm, nt->rxbuf);
}

static esp_err_t ntrip_tp_deinit(transport_t *t)
{
    ntrip_transport_t *nt = (ntrip_transport_t *)t;
    ntrip_stop();
    if (nt->rxbuf) {
        vRingbufferDelete(nt->rxbuf);
        nt->rxbuf = NULL;
    }
    return ESP_OK;
}

static int ntrip_tp_send(transport_t *t, const uint8_t *buf, size_t len)
{
    /* NTRIP client is receive-only for RTCM; GGA is sent internally. */
    (void)t; (void)buf; (void)len;
    return 0;
}

static int ntrip_tp_recv(transport_t *t, uint8_t *buf, size_t max_len, uint32_t timeout_ms)
{
    ntrip_transport_t *nt = (ntrip_transport_t *)t;
    size_t   item_size = 0;
    uint8_t *item = xRingbufferReceiveUpTo(nt->rxbuf, &item_size,
                                            pdMS_TO_TICKS(timeout_ms), max_len);
    if (!item) return 0;
    memcpy(buf, item, item_size);
    vRingbufferReturnItem(nt->rxbuf, item);
    return (int)item_size;
}

static int ntrip_tp_status(transport_t *t)
{
    (void)t;
    return ntrip_is_connected() ? TRANSPORT_CONNECTED : TRANSPORT_CONNECTING;
}

/* ── Public API ──────────────────────────────────────────────────────── */

void ntrip_transport_create(ntrip_transport_t *t, const ntrip_cfg_t *cfg)
{
    t->base.init   = ntrip_tp_init;
    t->base.deinit = ntrip_tp_deinit;
    t->base.send   = ntrip_tp_send;
    t->base.recv   = ntrip_tp_recv;
    t->base.status = ntrip_tp_status;
    t->base.priv   = NULL;
    t->cfg         = *cfg;
    t->rxbuf       = NULL;
}
