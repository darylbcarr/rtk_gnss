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
#include "lora.h"
#include <cstring>

static const char *TAG = "app";

/* ── Pin assignments ─────────────────────────────────────────────────── */

#define GNSS_UART    UART_NUM_1
#define GNSS_TX_PIN  6      /* ESP32 TX → LC29H RX (GPIO5 reserved on this board) */
#define GNSS_RX_PIN  1      /* ESP32 RX ← LC29H TX */

#define I2C_PORT     I2C_NUM_0
#define I2C_SDA_PIN  8
#define I2C_SCL_PIN  9

#define OLED_ADDR    0x3C   /* SA0 pin low */
#define IMU_ADDR     0x29   /* BNO055 ADR pin high */

#define ENC_CLK_PIN  17
#define ENC_DT_PIN   18
#define ENC_SW_PIN   38

/* LoRa (Ebyte E22-900T30D) — UART interface (not SPI), not yet wired
 * E22 pin names: AUX=output(idle high), TXD=out, RXD=in, M1=M0, M2=M1 */
#define LORA_UART    UART_NUM_2
#define LORA_TX_PIN  4    /* ESP TX → E22 RXD */
#define LORA_RX_PIN  7    /* ESP RX ← E22 TXD */
#define LORA_AUX_PIN 21   /* E22 AUX → ESP input (GPIO17 taken by encoder CLK) */
#define LORA_M0_PIN  15   /* ESP output → E22 M1 */
#define LORA_M1_PIN  16   /* ESP output → E22 M2 */

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

/* ── NTRIP callback ──────────────────────────────────────────────────── */

static void on_rtcm(const uint8_t *buf, size_t len, void *ctx)
{
    gnss_write_rtcm(buf, len);
}

/* ── app_main ────────────────────────────────────────────────────────── */

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "RTK GNSS — starting");

    /* ── I2C bus (shared by OLED and IMU) ──── */
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

    /* ── I2C scan (diagnostic — remove once hardware is verified) ──── */
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
    ui_init(imu_ok);

    /* ── LoRa ──── */
    if (lora_init(LORA_UART, LORA_TX_PIN, LORA_RX_PIN,
                  LORA_M0_PIN, LORA_M1_PIN, LORA_AUX_PIN,
                  NULL, NULL) != ESP_OK)
        ESP_LOGW(TAG, "LoRa init failed — continuing without LoRa");

    /* ── GNSS ──── */
    if (gnss_init(GNSS_UART, GNSS_TX_PIN, GNSS_RX_PIN) != ESP_OK) {
        /* Do not call display_flush here — a partial I2C command sent to the
         * SSD1306 while the bus is stressed blanks the display.  The "starting"
         * screen remains visible and the serial log has the full error. */
        ESP_LOGE(TAG, "GNSS init failed — halting");
        return;
    }

    /* ── WiFi + NTRIP ──── */
    wifi_connect();

    static const ntrip_cfg_t ntrip_cfg = {
        .host       = CONFIG_NTRIP_HOST,
        .port       = CONFIG_NTRIP_PORT,
        .mountpoint = CONFIG_NTRIP_MOUNTPOINT,
        .user       = CONFIG_NTRIP_USER,
        .password   = CONFIG_NTRIP_PASSWORD,
    };
    ntrip_start(&ntrip_cfg, on_rtcm, NULL);

    /* ── Main loop ──── */
    gnss_pvt_t pvt  = {};
    imu_data_t imu_d = {};

    for (;;) {
        if (!gnss_read_pvt(&pvt, 2000)) {
            ESP_LOGW(TAG, "No GNSS data in 2 s — check antenna/wiring");
            display_puts(0, 0, "GNSS TIMEOUT    ");
            display_flush();
            continue;
        }

        if (imu_ok)
            imu_read(&imu_d);

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
        ui_tick(&pvt, &imu_d);

        ESP_LOGI(TAG,
                 "sv=%u fix=%u rtk=%u  lat=%.7f lon=%.7f alt=%.2fm  "
                 "hAcc=%.3f pdop=%.1f  hdg=%.1f cal=%u",
                 pvt.num_sv, pvt.fix_type, pvt.carr_soln,
                 pvt.lat, pvt.lon, pvt.alt_msl,
                 pvt.h_acc, pvt.pdop, imu_d.heading, imu_d.sys_cal);
    }
}
