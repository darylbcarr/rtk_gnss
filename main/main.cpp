#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "lwip/inet.h"
#include "gnss.h"
#include "ntrip.h"
#include <cstring>

static const char *TAG = "app";

/* ZED-F9P wiring: ESP32-S3 TX→F9P RXD1, ESP32-S3 RX←F9P TXD1
 * GPIO 4 is used for TX — GPIOs 14–20 are reserved on this board. */
#define GNSS_UART   UART_NUM_1
#define GNSS_TX_PIN 4
#define GNSS_RX_PIN 17

/* ── WiFi ────────────────────────────────────────────────────────────── */

static EventGroupHandle_t s_wifi_eg;
static esp_netif_t       *s_sta_netif;
#define WIFI_UP_BIT BIT0

static void on_wifi(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START)
        esp_wifi_connect();
    else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED)
        esp_wifi_connect();
    else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        /* Force a known-working DNS server — DHCP may not supply one. */
        esp_netif_dns_info_t dns = {};
        dns.ip.u_addr.ip4.addr = inet_addr("8.8.8.8");
        dns.ip.type = ESP_IPADDR_TYPE_V4;
        esp_netif_set_dns_info(s_sta_netif, ESP_NETIF_DNS_MAIN, &dns);
        xEventGroupSetBits(s_wifi_eg, WIFI_UP_BIT);
    }
}

static void wifi_connect(void)
{
    s_wifi_eg = xEventGroupCreate();

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    s_sta_netif = esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,      on_wifi, NULL);
    esp_event_handler_register(IP_EVENT,   IP_EVENT_STA_GOT_IP,   on_wifi, NULL);

    wifi_config_t wcfg = {};
    strncpy((char *)wcfg.sta.ssid,     CONFIG_WIFI_SSID,     sizeof(wcfg.sta.ssid));
    strncpy((char *)wcfg.sta.password, CONFIG_WIFI_PASSWORD, sizeof(wcfg.sta.password));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wcfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Connecting to WiFi…");
    xEventGroupWaitBits(s_wifi_eg, WIFI_UP_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
    ESP_LOGI(TAG, "WiFi connected");
}

/* ── NTRIP callback ──────────────────────────────────────────────────── */

static void on_rtcm(const uint8_t *buf, size_t len, void *ctx)
{
    gnss_write_rtcm(buf, len);
}

/* ── Helpers ─────────────────────────────────────────────────────────── */

static const char *fix_name(uint8_t ft)
{
    static const char *const names[] = {
        "NO FIX", "DEAD REC", "2D", "3D", "GNSS+DR", "TIME ONLY"
    };
    return ft < 6 ? names[ft] : "?";
}

static const char *rtk_name(uint8_t cs)
{
    static const char *const names[] = { "", " +FLOAT", " +FIX" };
    return cs < 3 ? names[cs] : "";
}

/* ── app_main ────────────────────────────────────────────────────────── */

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "RTK GNSS — starting");

    wifi_connect();

    if (gnss_init(GNSS_UART, GNSS_TX_PIN, GNSS_RX_PIN) != ESP_OK) {
        ESP_LOGE(TAG, "GNSS init failed — halting");
        return;
    }

    static const ntrip_cfg_t ntrip_cfg = {
        .host       = CONFIG_NTRIP_HOST,
        .port       = CONFIG_NTRIP_PORT,
        .mountpoint = CONFIG_NTRIP_MOUNTPOINT,
        .user       = CONFIG_NTRIP_USER,
        .password   = CONFIG_NTRIP_PASSWORD,
    };
    ntrip_start(&ntrip_cfg, on_rtcm, NULL);

    gnss_pvt_t pvt;
    for (;;) {
        if (!gnss_read_pvt(&pvt, 2000)) {
            ESP_LOGW(TAG, "No NAV-PVT in 2 s — check antenna");
            continue;
        }

        ntrip_pos_t pos = {
            .lat      = pvt.lat,
            .lon      = pvt.lon,
            .alt_msl  = pvt.alt_msl,
            .i_tow    = pvt.i_tow,
            .pdop     = pvt.pdop,
            .fix_type = pvt.fix_type,
            .carr_soln= pvt.carr_soln,
            .num_sv   = pvt.num_sv,
        };
        ntrip_update_position(&pos);

        ESP_LOGI(TAG,
                 "%s%s  lat=%.7f lon=%.7f alt=%.2fm  hAcc=%.3fm vAcc=%.3fm  sv=%u pdop=%.1f",
                 fix_name(pvt.fix_type), rtk_name(pvt.carr_soln),
                 pvt.lat, pvt.lon, pvt.alt_msl,
                 pvt.h_acc, pvt.v_acc, pvt.num_sv, pvt.pdop);
    }
}
