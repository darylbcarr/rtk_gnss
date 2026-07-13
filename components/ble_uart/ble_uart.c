#include "ble_uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include <string.h>

static const char *TAG = "ble_uart";

/* ── Nordic UART Service UUIDs (128-bit, little-endian) ─────────────────
 * Service:  6E400001-B5A3-F393-E0A9-E50E24DCCA9E
 * RX char:  6E400002-...  (WRITE  — client → device, reserved for commands)
 * TX char:  6E400003-...  (NOTIFY — device → client, JSON status stream)   */
static const ble_uuid128_t s_svc_uuid =
    BLE_UUID128_INIT(0x9e,0xca,0xdc,0x24,0x0e,0xe5,0xa9,0xe0,
                     0x93,0xf3,0xa3,0xb5,0x01,0x00,0x40,0x6e);
static const ble_uuid128_t s_rx_uuid =
    BLE_UUID128_INIT(0x9e,0xca,0xdc,0x24,0x0e,0xe5,0xa9,0xe0,
                     0x93,0xf3,0xa3,0xb5,0x02,0x00,0x40,0x6e);
static const ble_uuid128_t s_tx_uuid =
    BLE_UUID128_INIT(0x9e,0xca,0xdc,0x24,0x0e,0xe5,0xa9,0xe0,
                     0x93,0xf3,0xa3,0xb5,0x03,0x00,0x40,0x6e);

static uint16_t s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t s_tx_val_handle;
static char     s_dev_name[32];

/* ── GATT characteristic access callbacks ────────────────────────────── */

static int tx_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                          struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    return 0;  /* TX is notify-only; READ returns empty */
}

static int rx_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                          struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        char buf[128];
        uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
        if (len >= (uint16_t)sizeof(buf)) len = (uint16_t)(sizeof(buf) - 1);
        os_mbuf_copydata(ctxt->om, 0, len, buf);
        buf[len] = '\0';
        ESP_LOGI(TAG, "rx cmd: %s", buf);
    }
    return 0;
}

/* ── GATT service table ──────────────────────────────────────────────── */

static const struct ble_gatt_svc_def s_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &s_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                /* TX: device → client */
                .uuid       = &s_tx_uuid.u,
                .access_cb  = tx_chr_access,
                .flags      = BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &s_tx_val_handle,
            },
            {
                /* RX: client → device (reserved for future commands) */
                .uuid      = &s_rx_uuid.u,
                .access_cb = rx_chr_access,
                .flags     = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
            },
            { 0 },
        },
    },
    { 0 },
};

/* ── GAP advertising + event handling ───────────────────────────────── */

static int gap_event(struct ble_gap_event *event, void *arg);

static void start_advertising(void)
{
    /*
     * Primary advertisement: flags + the NUS service UUID. Apps that
     * identify "generic NMEA over BLE" GNSS receivers (e.g. SW Maps) filter
     * scan results on this UUID rather than listing every nearby BLE
     * device, so it must be in the advertisement itself, not discovered
     * only after connecting. Legacy adv PDUs cap at 31 bytes and a 128-bit
     * UUID field already takes 18 of them, so the name goes in the
     * separate scan-response packet instead of here.
     */
    struct ble_hs_adv_fields f = {0};
    f.flags              = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    f.uuids128            = &s_svc_uuid;
    f.num_uuids128        = 1;
    f.uuids128_is_complete = 1;
    ble_gap_adv_set_fields(&f);

    struct ble_hs_adv_fields rsp = {0};
    rsp.name             = (const uint8_t *)s_dev_name;
    rsp.name_len         = (uint8_t)strlen(s_dev_name);
    rsp.name_is_complete = 1;
    ble_gap_adv_rsp_set_fields(&rsp);

    struct ble_gap_adv_params ap = {0};
    ap.conn_mode = BLE_GAP_CONN_MODE_UND;
    ap.disc_mode = BLE_GAP_DISC_MODE_GEN;
    int rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                                &ap, gap_event, NULL);
    if (rc != 0)
        ESP_LOGE(TAG, "adv_start failed: %d", rc);
    else
        ESP_LOGI(TAG, "advertising as '%s'", s_dev_name);
}

static int gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            s_conn_handle = event->connect.conn_handle;
            ESP_LOGI(TAG, "connected handle=%d", s_conn_handle);
        } else {
            ESP_LOGW(TAG, "connect failed status=%d", event->connect.status);
            start_advertising();
        }
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        ESP_LOGI(TAG, "disconnected reason=%d", event->disconnect.reason);
        start_advertising();
        break;

    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG, "MTU updated: conn=%d mtu=%d",
                 event->mtu.conn_handle, event->mtu.value);
        break;

    default:
        break;
    }
    return 0;
}

/* ── NimBLE host task ────────────────────────────────────────────────── */

static void nimble_host_task(void *arg)
{
    nimble_port_run();           /* blocks until nimble_port_stop() */
    nimble_port_freertos_deinit();
}

static void on_sync(void)
{
    ble_hs_util_ensure_addr(0);
    start_advertising();
}

static void on_reset(int reason)
{
    ESP_LOGE(TAG, "host reset reason=%d", reason);
}

/* ── Public API ──────────────────────────────────────────────────────── */

esp_err_t ble_uart_init(const char *device_name)
{
    strlcpy(s_dev_name, device_name, sizeof(s_dev_name));

    esp_err_t ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_init: %d", ret);
        return ret;
    }

    ble_hs_cfg.sync_cb         = on_sync;
    ble_hs_cfg.reset_cb        = on_reset;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    ble_svc_gap_init();
    ble_svc_gatt_init();

    int rc = ble_gatts_count_cfg(s_svcs);
    if (rc != 0) { ESP_LOGE(TAG, "count_cfg: %d", rc); return ESP_FAIL; }
    rc = ble_gatts_add_svcs(s_svcs);
    if (rc != 0) { ESP_LOGE(TAG, "add_svcs: %d", rc); return ESP_FAIL; }

    ble_svc_gap_device_name_set(s_dev_name);
    nimble_port_freertos_init(nimble_host_task);

    ESP_LOGI(TAG, "NUS ready, advertising as '%s'", s_dev_name);
    return ESP_OK;
}

bool ble_uart_connected(void)
{
    return s_conn_handle != BLE_HS_CONN_HANDLE_NONE;
}

void ble_uart_send(const char *data, size_t len)
{
    if (s_conn_handle == BLE_HS_CONN_HANDLE_NONE) return;

    struct os_mbuf *om = ble_hs_mbuf_from_flat(data, len);
    if (!om) {
        ESP_LOGW(TAG, "send: mbuf alloc failed");
        return;
    }
    int rc = ble_gatts_notify_custom(s_conn_handle, s_tx_val_handle, om);
    if (rc != 0 && rc != BLE_HS_ENOTCONN)
        ESP_LOGW(TAG, "notify: %d", rc);
}
