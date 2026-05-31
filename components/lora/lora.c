#include "lora.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include <string.h>

static const char *TAG = "lora";

/* ── Module state ────────────────────────────────────────────────────── */

static uart_port_t  s_uart;
static int          s_m0_pin;
static int          s_m1_pin;
static int          s_aux_pin;
static lora_rx_cb_t s_rx_cb;
static void        *s_rx_ctx;

/* ── E22 configuration ───────────────────────────────────────────────── */

/*
 * TX power register value:
 *   E22 encodes power as (30 - dBm) / 3, clamped to 0–3.
 *   21 dBm → (30-21)/3 = 3, 24→2, 27→1, 30→0.
 */
static uint8_t power_reg(int dbm)
{
    int v = (30 - dbm) / 3;
    if (v < 0) v = 0;
    if (v > 3) v = 3;
    return (uint8_t)v;
}

/*
 * C2 RAM-write command (10 bytes):
 *   0xC2  start_addr  length  REG0 REG1 REG2 REG3 REG4 CRYPTH CRYPTL
 *
 * REG0 = 0x64  → UART 9600 baud, air rate 9.6 kbps
 * REG1 = 0x00  → 240-byte sub-packet, RSSI ambient off, no power saving
 * REG2 = tx power (encoded)
 * REG3 = channel (0–83; 850.125 + ch MHz)
 * REG4 = 0x00  → transparent mode, no repeater, monitor off, RSSI byte off
 * CRYPT = 0x00 0x00  → encryption disabled
 */
static const uint8_t E22_UART_9600_AIR_96 = 0x64;
static const uint8_t E22_SUBPKT_240       = 0x00;
static const uint8_t E22_MODE_TRANSPARENT = 0x00;

/* ── GPIO helpers ────────────────────────────────────────────────────── */

static void set_mode(int m0, int m1)
{
    gpio_set_level(s_m0_pin, m0);
    gpio_set_level(s_m1_pin, m1);
    vTaskDelay(pdMS_TO_TICKS(2));
}

/* Poll AUX until high (module idle) or timeout. */
static esp_err_t wait_aux_high(uint32_t timeout_ms)
{
    uint32_t elapsed = 0;
    while (!gpio_get_level(s_aux_pin)) {
        vTaskDelay(pdMS_TO_TICKS(5));
        elapsed += 5;
        if (elapsed >= timeout_ms) {
            ESP_LOGW(TAG, "AUX timeout after %lu ms", timeout_ms);
            return ESP_ERR_TIMEOUT;
        }
    }
    return ESP_OK;
}

/* ── E22 register programming ────────────────────────────────────────── */

/* Baud rates to try when probing — 9600 is the E22 factory default */
static const int s_probe_bauds[] = { 9600, 115200, 57600, 38400, 19200 };
#define PROBE_BAUD_COUNT ((int)(sizeof(s_probe_bauds)/sizeof(s_probe_bauds[0])))

static esp_err_t e22_configure(void)
{
    /* Enter config mode: M0=1, M1=1 */
    set_mode(1, 1);
    esp_err_t ret = wait_aux_high(500);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "E22 did not go idle for config");
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(50));  /* settle after AUX high before sending */

    /* C1 probe at each candidate baud rate */
    const uint8_t c1_cmd[3] = { 0xC1, 0x00, 0x09 };
    int working_baud = 0;
    for (int bi = 0; bi < PROBE_BAUD_COUNT; bi++) {
        uart_flush_input(s_uart);
        /* Change UART baud rate for this probe */
        uart_set_baudrate(s_uart, s_probe_bauds[bi]);
        uart_write_bytes(s_uart, c1_cmd, sizeof(c1_cmd));
        uint8_t c1_resp[12] = {0};
        int c1_got = uart_read_bytes(s_uart, c1_resp, sizeof(c1_resp),
                                     pdMS_TO_TICKS(300));
        if (c1_got > 0) {
            working_baud = s_probe_bauds[bi];
            ESP_LOGI(TAG, "E22 responded at %d baud: %d bytes", working_baud, c1_got);
            for (int i = 0; i < c1_got; i++)
                ESP_LOGI(TAG, "  [%d] 0x%02X", i, c1_resp[i]);
            break;
        }
        ESP_LOGD(TAG, "no response at %d baud", s_probe_bauds[bi]);
    }
    if (!working_baud) {
        ESP_LOGE(TAG, "E22 no response at any baud rate — check TXD wiring");
        uart_set_baudrate(s_uart, 9600);
        /* continue — module may still TX even if we can't read config */
    } else if (working_baud != 9600) {
        /* Module is at a non-default baud; keep that rate to send the C2 command,
         * then C2 will reconfigure it to 9600 for normal operation. */
        ESP_LOGI(TAG, "E22 at non-default baud %d; will reconfigure to 9600", working_baud);
    }

    uint8_t cmd[10] = {
        0xC2,                                    /* RAM write (no flash wear) */
        0x00,                                    /* start address REG0 */
        0x07,                                    /* 7 bytes: REG0–REG4 + CRYPT×2 */
        E22_UART_9600_AIR_96,                    /* REG0 */
        E22_SUBPKT_240,                          /* REG1 */
        power_reg(CONFIG_LORA_TX_POWER_DBM),     /* REG2 */
        (uint8_t)(CONFIG_LORA_CHANNEL & 0xFF),   /* REG3 */
        E22_MODE_TRANSPARENT,                    /* REG4 */
        0x00, 0x00,                              /* CRYPT H/L */
    };

    uart_flush_input(s_uart);
    uart_write_bytes(s_uart, cmd, sizeof(cmd));

    /* E22 echoes the command back as confirmation */
    uint8_t resp[10] = {0};
    int got = uart_read_bytes(s_uart, resp, sizeof(resp), pdMS_TO_TICKS(500));
    if (got != (int)sizeof(resp) || memcmp(resp, cmd, sizeof(cmd)) != 0) {
        ESP_LOGW(TAG, "E22 C2 write: mismatch (got %d bytes)", got);
        for (int i = 0; i < got; i++)
            ESP_LOGW(TAG, "  [%d] 0x%02X", i, resp[i]);
    } else {
        ESP_LOGI(TAG, "E22 configured: %d dBm ch%d (%.3f MHz)",
                 CONFIG_LORA_TX_POWER_DBM,
                 CONFIG_LORA_CHANNEL,
                 850.125f + CONFIG_LORA_CHANNEL);
    }
    /* C2 set the module to 9600; match the ESP UART to that rate */
    uart_set_baudrate(s_uart, 9600);

    /* Return to transparent mode: M0=0, M1=0 */
    set_mode(0, 0);
    ret = wait_aux_high(500);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "E22 did not return to normal mode");
        return ret;
    }

    return ESP_OK;
}

/* ── RX task ─────────────────────────────────────────────────────────── */

#define RX_BUF_SIZE 512

static void lora_rx_task(void *arg)
{
    uint8_t *buf = malloc(RX_BUF_SIZE);
    if (!buf) {
        ESP_LOGE(TAG, "RX buffer alloc failed");
        vTaskDelete(NULL);
        return;
    }

    for (;;) {
        int len = uart_read_bytes(s_uart, buf, RX_BUF_SIZE, pdMS_TO_TICKS(100));
        if (len > 0 && s_rx_cb)
            s_rx_cb(buf, (size_t)len, s_rx_ctx);
    }
}

/* ── Public API ──────────────────────────────────────────────────────── */

esp_err_t lora_init(uart_port_t uart, int tx_pin, int rx_pin,
                    int m0_pin, int m1_pin, int aux_pin,
                    lora_rx_cb_t cb, void *ctx)
{
    s_uart   = uart;
    s_m0_pin = m0_pin;
    s_m1_pin = m1_pin;
    s_aux_pin = aux_pin;
    s_rx_cb  = cb;
    s_rx_ctx = ctx;

    /* M0, M1 as outputs — start HIGH (config mode) to prevent RF TX during init */
    gpio_config_t out_cfg = {
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
        .pin_bit_mask = (1ULL << m0_pin) | (1ULL << m1_pin),
    };
    ESP_ERROR_CHECK(gpio_config(&out_cfg));
    gpio_set_level(m0_pin, 1);
    gpio_set_level(m1_pin, 1);

    /* AUX as input */
    gpio_config_t in_cfg = {
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
        .pin_bit_mask = (1ULL << aux_pin),
    };
    ESP_ERROR_CHECK(gpio_config(&in_cfg));

    /* UART at 9600 to match E22 default (or previously configured rate) */
    const uart_config_t ucfg = {
        .baud_rate  = 9600,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_param_config(uart, &ucfg));
    ESP_ERROR_CHECK(uart_set_pin(uart, tx_pin, rx_pin,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(uart, RX_BUF_SIZE * 2, 0, 0, NULL, 0));

    /* Wait for module to power up (AUX HIGH = ready) */
    esp_err_t ret = wait_aux_high(2000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "E22 not responding — check wiring and power");
        return ret;
    }

    ret = e22_configure();
    if (ret != ESP_OK)
        return ret;

    xTaskCreate(lora_rx_task, "lora_rx", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "LoRa ready");
    return ESP_OK;
}

esp_err_t lora_send(const uint8_t *data, size_t len)
{
    esp_err_t ret = wait_aux_high(2000);
    if (ret != ESP_OK)
        return ret;

    int written = uart_write_bytes(s_uart, data, len);
    if (written < 0 || (size_t)written != len) {
        ESP_LOGE(TAG, "lora_send: wrote %d of %zu bytes", written, len);
        return ESP_FAIL;
    }
    return ESP_OK;
}
