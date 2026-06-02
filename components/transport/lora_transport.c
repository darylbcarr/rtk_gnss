#include "lora_transport.h"
#include "lora.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "lora_tp";

/* ── RX callback: lora RX task → ringbuffer ─────────────────────────── */

static void on_lora_rx(const uint8_t *buf, size_t len, void *ctx)
{
    RingbufHandle_t rb = (RingbufHandle_t)ctx;
    if (xRingbufferSend(rb, buf, len, 0) != pdTRUE)
        ESP_LOGW(TAG, "rxbuf full — dropped %u LoRa bytes", (unsigned)len);
}

/* ── vtable implementation ───────────────────────────────────────────── */

static esp_err_t lora_tp_init(transport_t *t, const void *cfg)
{
    (void)cfg;
    lora_transport_t *lt = (lora_transport_t *)t;
    lt->rxbuf = xRingbufferCreate(4096, RINGBUF_TYPE_BYTEBUF);
    if (!lt->rxbuf) return ESP_ERR_NO_MEM;
    return lora_init(lt->cfg.uart,
                     lt->cfg.tx_pin, lt->cfg.rx_pin,
                     lt->cfg.m0_pin, lt->cfg.m1_pin, lt->cfg.aux_pin,
                     on_lora_rx, lt->rxbuf);
}

static esp_err_t lora_tp_deinit(transport_t *t)
{
    lora_transport_t *lt = (lora_transport_t *)t;
    if (lt->rxbuf) {
        vRingbufferDelete(lt->rxbuf);
        lt->rxbuf = NULL;
    }
    return ESP_OK;
}

static int lora_tp_send(transport_t *t, const uint8_t *buf, size_t len)
{
    (void)t;
    esp_err_t ret = lora_send(buf, len);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "lora_send failed: %s", esp_err_to_name(ret));
        return -1;
    }
    return (int)len;
}

static int lora_tp_recv(transport_t *t, uint8_t *buf, size_t max_len, uint32_t timeout_ms)
{
    lora_transport_t *lt = (lora_transport_t *)t;
    size_t   item_size = 0;
    uint8_t *item = xRingbufferReceiveUpTo(lt->rxbuf, &item_size,
                                            pdMS_TO_TICKS(timeout_ms), max_len);
    if (!item) return 0;
    memcpy(buf, item, item_size);
    vRingbufferReturnItem(lt->rxbuf, item);
    return (int)item_size;
}

static int lora_tp_status(transport_t *t)
{
    /* LoRa transparent mode has no connection state. */
    (void)t;
    return TRANSPORT_CONNECTED;
}

/* ── Public API ──────────────────────────────────────────────────────── */

void lora_transport_create(lora_transport_t *t, const lora_transport_cfg_t *cfg)
{
    t->base.init   = lora_tp_init;
    t->base.deinit = lora_tp_deinit;
    t->base.send   = lora_tp_send;
    t->base.recv   = lora_tp_recv;
    t->base.status = lora_tp_status;
    t->base.priv   = NULL;
    t->cfg         = *cfg;
    t->rxbuf       = NULL;
}
