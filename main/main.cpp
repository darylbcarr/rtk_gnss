#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "lwip/inet.h"
#include "driver/i2c_master.h"
#include "gnss.h"
#include "ntrip.h"
#include "imu.h"
#include "display.h"
#include "encoder.h"
#include "ui.h"
#include "config.h"
#include "transport.h"
#include "ntrip_transport.h"
#include "lora_transport.h"
#include "base_app.h"
#include <cstring>

static const char *TAG = "app";

/* ── Pin assignments ─────────────────────────────────────────────────── */

#define GNSS_UART    UART_NUM_1
#define GNSS_TX_PIN  6
#define GNSS_RX_PIN  1

#define I2C_PORT     I2C_NUM_0
#define I2C_SDA_PIN  8
#define I2C_SCL_PIN  9

#define OLED_ADDR    0x3C
#define IMU_ADDR     0x29

#define ENC_CLK_PIN  17
#define ENC_DT_PIN   18
#define ENC_SW_PIN   38

#define LORA_UART    UART_NUM_2
#define LORA_TX_PIN  4
#define LORA_RX_PIN  7
#define LORA_AUX_PIN 21
#define LORA_M0_PIN  15
#define LORA_M1_PIN  16

/* ── WiFi — only compiled when NTRIP correction source is selected ───── */

#if CONFIG_ROVER_CORRECTION_NTRIP

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

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    s_sta_netif = esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,    on_wifi, NULL);
    esp_event_handler_register(IP_EVENT,   IP_EVENT_STA_GOT_IP, on_wifi, NULL);

    wifi_config_t wcfg = {};
    strncpy((char *)wcfg.sta.ssid,     CONFIG_WIFI_SSID,     sizeof(wcfg.sta.ssid));
    strncpy((char *)wcfg.sta.password, CONFIG_WIFI_PASSWORD, sizeof(wcfg.sta.password));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wcfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Connecting to WiFi...");
    xEventGroupWaitBits(s_wifi_eg, WIFI_UP_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
    ESP_LOGI(TAG, "WiFi connected");
}

#endif /* CONFIG_ROVER_CORRECTION_NTRIP */

/* ── app_main ────────────────────────────────────────────────────────── */

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "RTK GNSS — starting");

    ESP_ERROR_CHECK(config_init());

    /* ── I2C bus ──── */
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port            = I2C_PORT,
        .sda_io_num          = (gpio_num_t)I2C_SDA_PIN,
        .scl_io_num          = (gpio_num_t)I2C_SCL_PIN,
        .clk_source          = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt   = 7,
        .flags = {
            .enable_internal_pullup = true,
        },
    };
    i2c_master_bus_handle_t i2c_bus;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &i2c_bus));

    /* ── I2C scan (diagnostic) ──── */
    ESP_LOGI(TAG, "I2C scan:");
    bool found_any = false;
    for (uint8_t addr = 8; addr < 120; addr++) {
        if (i2c_master_probe(i2c_bus, addr, 50) == ESP_OK) {
            ESP_LOGI(TAG, "  device at 0x%02X", addr);
            found_any = true;
        }
    }
    if (!found_any)
        ESP_LOGW(TAG, "  no devices found — check pull-ups and VCC");

    /* ── Peripherals ──── */
    if (display_init(i2c_bus, OLED_ADDR) != ESP_OK)
        ESP_LOGW(TAG, "OLED not found — continuing without display");

    display_puts(0, 0, "RTK GNSS starting");
    display_flush();

    bool imu_ok = (imu_init(i2c_bus, IMU_ADDR) == ESP_OK);
    if (!imu_ok)
        ESP_LOGW(TAG, "IMU not found — heading unavailable");

    encoder_init(ENC_CLK_PIN, ENC_DT_PIN, ENC_SW_PIN);

    /* ── Mode selection ──── */
    device_mode_t mode = config_get_device_mode();
    if (mode == DEVICE_MODE_UNSET)
        mode = ui_select_mode_blocking();
    ESP_LOGI(TAG, "Device mode: %s", mode == DEVICE_MODE_BASE ? "BASE" : "ROVER");

    ui_init(imu_ok, mode);

    /* ── LoRa transport (both modes) ──── */
    static lora_transport_t lora_tp;
    static const lora_transport_cfg_t lora_cfg = {
        .uart    = LORA_UART,
        .tx_pin  = LORA_TX_PIN,
        .rx_pin  = LORA_RX_PIN,
        .m0_pin  = LORA_M0_PIN,
        .m1_pin  = LORA_M1_PIN,
        .aux_pin = LORA_AUX_PIN,
    };
    lora_transport_create(&lora_tp, &lora_cfg);
    transport_t *lora_t = &lora_tp.base;

    bool lora_ok = (transport_init(lora_t, NULL) == ESP_OK);
    if (!lora_ok)
        ESP_LOGW(TAG, "LoRa transport init failed — continuing without LoRa");

    /* ── GNSS ──── */
    if (gnss_init(GNSS_UART, GNSS_TX_PIN, GNSS_RX_PIN) != ESP_OK) {
        ESP_LOGE(TAG, "GNSS init failed — halting");
        return;
    }

    /* ── Mode-specific transport ──── */
    transport_t *correction_t = NULL;   /* transport that delivers RTCM to this device */

    if (mode == DEVICE_MODE_ROVER) {

#if CONFIG_ROVER_CORRECTION_NTRIP
        wifi_connect();

        static ntrip_transport_t ntrip_tp;
        static const ntrip_cfg_t ntrip_cfg = {
            .host       = CONFIG_NTRIP_HOST,
            .port       = CONFIG_NTRIP_PORT,
            .mountpoint = CONFIG_NTRIP_MOUNTPOINT,
            .user       = CONFIG_NTRIP_USER,
            .password   = CONFIG_NTRIP_PASSWORD,
        };
        ntrip_transport_create(&ntrip_tp, &ntrip_cfg);
        if (transport_init(&ntrip_tp.base, NULL) == ESP_OK)
            correction_t = &ntrip_tp.base;
        else
            ESP_LOGW(TAG, "NTRIP transport init failed — no corrections");

#else  /* CONFIG_ROVER_CORRECTION_LORA */
        if (lora_ok) {
            correction_t = lora_t;
            ESP_LOGI(TAG, "Rover using LoRa corrections from local base");
            display_puts(0, 1, "CORR: LoRa base ");
            display_flush();
        } else {
            ESP_LOGE(TAG, "LoRa unavailable — rover has no correction source");
            display_puts(0, 1, "NO CORRECTION!  ");
            display_flush();
        }
#endif

    } else {
        /* Base: run survey-in then stream RTCM3 → LoRa */
        display_puts(0, 0, "BASE: SURVEY-IN");
        display_flush();

        transport_t *outbound = lora_ok ? lora_t : NULL;
        if (!outbound)
            ESP_LOGW(TAG, "LoRa unavailable — RTCM will not be forwarded");

        /* Survey-in defaults: 5-minute minimum, 2.0 m accuracy convergence. */
        base_app_start(outbound, 300, 2.0f);
    }

    /* ── Main loop ──── */
    gnss_pvt_t  pvt   = {};
    imu_data_t  imu_d = {};
    uint8_t     rtcm_buf[512];

    for (;;) {
        /* In base mode the LC29H outputs RTCM only — no NMEA GGA is emitted.
         * Skip the blocking PVT wait so the main loop yields CPU quickly. */
        if (mode == DEVICE_MODE_BASE) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        if (!gnss_read_pvt(&pvt, 2000)) {
            ESP_LOGW(TAG, "No GNSS data in 2 s — check antenna/wiring");
            display_puts(0, 0, "GNSS TIMEOUT    ");
            display_flush();
            continue;
        }

        if (imu_ok)
            imu_read(&imu_d);

        /* Drain incoming RTCM corrections and forward to GNSS */
        if (correction_t) {
            int n;
            while ((n = transport_recv(correction_t, rtcm_buf,
                                       sizeof(rtcm_buf), 0)) > 0)
                gnss_write_rtcm(rtcm_buf, (size_t)n);

#if CONFIG_ROVER_CORRECTION_NTRIP
            if (mode == DEVICE_MODE_ROVER) {
                ntrip_pos_t pos = {
                    .lat       = pvt.lat,
                    .lon       = pvt.lon,
                    .alt_msl   = pvt.alt_msl,
                    .i_tow     = pvt.i_tow,
                    .pdop      = pvt.pdop,
                    .fix_type  = pvt.fix_type,
                    .carr_soln = pvt.carr_soln,
                    .num_sv    = pvt.num_sv,
                };
                ntrip_update_position(&pos);
            }
#endif
        }

        ui_tick(&pvt, &imu_d);

        /* In base mode overlay survey-in progress on OLED lines 2–3 */
        if (mode == DEVICE_MODE_BASE) {
            base_state_t bst = base_app_get_state();
            if (bst == BASE_STATE_SURVEY_IN) {
                gnss_svin_t sv = base_app_get_svin();
                char ln2[22], ln3[22];
                snprintf(ln2, sizeof(ln2), "T:%4lus Acc:%.2fm",
                         (unsigned long)sv.dur_s, sv.mean_acc_m);
                snprintf(ln3, sizeof(ln3), "Sv:%2u  %s",
                         pvt.num_sv, sv.valid ? "VALID" : "ACTIVE");
                display_puts(0, 2, ln2);
                display_puts(0, 3, ln3);
                display_flush();
            } else if (bst == BASE_STATE_BROADCASTING) {
                display_puts(0, 2, "RTCM BROADCAST  ");
                display_flush();
            }
        }

        /* Log only on RTK state transitions so the monitor stays readable */
        static uint8_t s_last_carr;
        if (pvt.carr_soln != s_last_carr) {
            static const char *const RTK_STR[] = { "NONE", "FLOAT", "FIX" };
            ESP_LOGI(TAG, "RTK %s  sv=%u hAcc=%.3fm",
                     RTK_STR[pvt.carr_soln < 3 ? pvt.carr_soln : 0],
                     pvt.num_sv, pvt.h_acc);
            s_last_carr = pvt.carr_soln;
        }

        ESP_LOGD(TAG,
                 "sv=%u fix=%u rtk=%u  lat=%.7f lon=%.7f alt=%.2fm  "
                 "hAcc=%.3f pdop=%.1f  hdg=%.1f cal=%u",
                 pvt.num_sv, pvt.fix_type, pvt.carr_soln,
                 pvt.lat, pvt.lon, pvt.alt_msl,
                 pvt.h_acc, pvt.pdop, imu_d.heading, imu_d.sys_cal);
    }
}
