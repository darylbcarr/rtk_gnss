#include "lora.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
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

/* E22 encodes TX power as (30 - dBm) / 3, clamped to 0–3.
 * 21 dBm → 3, 24 → 2, 27 → 1, 30 → 0. */
static uint8_t power_reg(int dbm)
{
    int v = (30 - dbm) / 3;
    if (v < 0) v = 0;
    if (v > 3) v = 3;
    return (uint8_t)v;
}

/*
 * C2 RAM-write, addresses 0x03–0x06 (REG3, REG4, CHAN, REG6).
 * ADDH/ADDL/NETID (0x00–0x02) left at factory defaults (broadcast 0xFFFF).
 * CRYPT (0x07–0x08) left at factory default 0x0000 (disabled) — the E22
 * does not echo those bytes in the C2 ACK so we omit them from the write.
 *
 * REG3 (0x03) = 0x64  → UART 9600 baud, 8N1, air rate 9.6 kbps
 * REG4 (0x04)         → sub-packet 240 B (bits 7:6=00) | TX power (bits 1:0)
 * CHAN (0x05)         → channel 0–83; frequency = 850.125 + ch MHz
 * REG6 (0x06) = 0x00  → transparent mode, no RSSI byte, no LBT
 *
 * E22 acknowledges a C2 write by echoing 0xC2 + the same addr/len/data.
 */
static const uint8_t E22_UART_9600_AIR_96 = 0x64;
static const uint8_t E22_SUBPKT_240       = 0x00;
static const uint8_t E22_MODE_TRANSPARENT = 0x00;

/* ── GPIO helpers ────────────────────────────────────────────────────── */

static void set_mode(int m0, int m1)
{
    gpio_set_level(s_m0_pin, m0);
    gpio_set_level(s_m1_pin, m1);
    vTaskDelay(pdMS_TO_TICKS(50));
}

/* Poll AUX until high (module idle) or timeout.
 * Uses esp_timer for a tick-rate-independent deadline. */
static esp_err_t wait_aux_high(uint32_t timeout_ms)
{
    int64_t deadline_us = esp_timer_get_time() + (int64_t)timeout_ms * 1000LL;
    while (!gpio_get_level(s_aux_pin)) {
        vTaskDelay(1);
        if (esp_timer_get_time() >= deadline_us) {
            ESP_LOGW(TAG, "AUX timeout after %lu ms", (unsigned long)timeout_ms);
            return ESP_ERR_TIMEOUT;
        }
    }
    return ESP_OK;
}

/* Read exactly `want` bytes within `timeout_ms`, looping for partial chunks. */
static int uart_read_all(uart_port_t uart, uint8_t *buf, int want, uint32_t timeout_ms)
{
    int64_t deadline_us = esp_timer_get_time() + (int64_t)timeout_ms * 1000LL;
    int got = 0;
    while (got < want && esp_timer_get_time() < deadline_us) {
        int n = uart_read_bytes(uart, buf + got, want - got, pdMS_TO_TICKS(10));
        if (n > 0) got += n;
    }
    return got;
}

/* ── E22 register programming ────────────────────────────────────────── */

static esp_err_t e22_configure(void)
{
    set_mode(0, 1);   /* CONFIG mode: M0=0, M1=1 */

    /* E22 must assert AUX LOW during the mode transition before going HIGH
     * (ready).  If AUX never dips LOW we raced — the module was already
     * HIGH and will not process C2 as a config command. */
    {
        bool saw_low = false;
        int64_t dl = esp_timer_get_time() + 300000LL;   /* 300 ms */
        while (!saw_low && esp_timer_get_time() < dl) {
            if (!gpio_get_level(s_aux_pin)) { saw_low = true; break; }
            vTaskDelay(1);
        }
        if (!saw_low)
            ESP_LOGW(TAG, "AUX never dipped LOW on CONFIG entry — M1/AUX wiring or timing suspect");
        else
            ESP_LOGD(TAG, "AUX dipped LOW: M1 & AUX wiring confirmed");
    }

    esp_err_t ret = wait_aux_high(3000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "E22 did not go idle for config");
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(50));   /* settle before sending */

    uint8_t cmd[7] = {
        0xC2,                                    /* RAM write (no flash wear) */
        0x03,                                    /* start address: REG3 */
        0x04,                                    /* 4 bytes: REG3, REG4, CHAN, REG6 */
        E22_UART_9600_AIR_96,                    /* REG3 (0x03): UART baud, parity, air rate */
        (uint8_t)(E22_SUBPKT_240 | power_reg(CONFIG_LORA_TX_POWER_DBM)), /* REG4 (0x04) */
        (uint8_t)(CONFIG_LORA_CHANNEL & 0xFF),   /* CHAN (0x05) */
        E22_MODE_TRANSPARENT,                    /* REG6 (0x06): transparent, no RSSI byte */
    };

    /* E22 may emit spontaneous bytes during CONFIG mode entry that arrive after
     * the flush — retry up to 3 times if the ACK is garbled. */
    bool ack_ok = false;
    for (int attempt = 0; attempt < 3 && !ack_ok; attempt++) {
        if (attempt > 0)
            vTaskDelay(pdMS_TO_TICKS(50));

        uart_flush_input(s_uart);
        uart_write_bytes(s_uart, cmd, sizeof(cmd));
        uart_wait_tx_done(s_uart, pdMS_TO_TICKS(50));

        uint8_t resp[7] = {0};
        int got = uart_read_all(s_uart, resp, sizeof(resp), 500);

        /* E22 firmware variants echo either 0xC1 or 0xC2 as the ACK byte */
        ack_ok = (got == (int)sizeof(resp)) &&
                 (resp[0] == 0xC1 || resp[0] == 0xC2) &&
                 (memcmp(resp + 1, cmd + 1, sizeof(cmd) - 1) == 0);

        if (!ack_ok) {
            ESP_LOGW(TAG, "E22 C2 write attempt %d: unexpected ACK (got %d bytes)",
                     attempt + 1, got);
            for (int i = 0; i < got; i++)
                ESP_LOGW(TAG, "  [%d] 0x%02X", i, resp[i]);
        }
    }

    if (ack_ok) {
        ESP_LOGI(TAG, "E22 configured: %d dBm ch%d (%.3f MHz)",
                 CONFIG_LORA_TX_POWER_DBM,
                 CONFIG_LORA_CHANNEL,
                 850.125f + CONFIG_LORA_CHANNEL);
    } else {
        /* REG6=0xFF enables fixed-point TX + RSSI-appended byte, breaking RTCM */
        ESP_LOGE(TAG, "E22 C2 write failed after 3 attempts — RTCM framing may be corrupted");
    }

    /* C1 readback — confirm actual register state regardless of ACK outcome.
     * Reads ADDH…CRYPT_L (0x00–0x08); key fields logged for wiring diagnosis. */
    {
        const uint8_t c1[3] = { 0xC1, 0x00, 0x09 };
        uint8_t rb[12] = {0};
        uart_flush_input(s_uart);
        uart_write_bytes(s_uart, c1, sizeof(c1));
        uart_wait_tx_done(s_uart, pdMS_TO_TICKS(50));
        vTaskDelay(pdMS_TO_TICKS(20));   /* E22 processing delay before response */
        int rb_got = uart_read_all(s_uart, rb, sizeof(rb), 500);
        if (rb_got == 12 && rb[0] == 0xC1) {
            /* rb[3..5]=ADDH/ADDL/NETID  rb[6]=REG3  rb[7]=REG4  rb[8]=CHAN  rb[9]=REG6 */
            ESP_LOGI(TAG, "E22 regs: addr=%02X%02X net=%02X "
                          "reg3=%02X reg4=%02X chan=%u reg6=%02X crypt=%02X%02X",
                     rb[3], rb[4], rb[5],
                     rb[6], rb[7], rb[8], rb[9],
                     rb[10], rb[11]);
            if (rb[8] != (uint8_t)(CONFIG_LORA_CHANNEL & 0xFF))
                ESP_LOGW(TAG, "E22 channel mismatch: have %u, want %u",
                         rb[8], CONFIG_LORA_CHANNEL & 0xFF);
            if (rb[9] & 0x80)
                ESP_LOGW(TAG, "E22 RSSI byte enabled — will corrupt RTCM framing");
            if (rb[9] & 0x40)
                ESP_LOGW(TAG, "E22 fixed-point mode enabled — not transparent");
        } else {
            ESP_LOGW(TAG, "E22 C1 readback failed (got %d bytes)", rb_got);
            for (int i = 0; i < rb_got; i++)
                ESP_LOGW(TAG, "  [%d] 0x%02X", i, rb[i]);
        }
    }

    set_mode(0, 0);   /* NORMAL mode: M0=0, M1=0 */
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
        if (len <= 0) continue;

        /* At 9600 baud the E22 UART FIFO flushes in ~92 ms chunks; packets
         * larger than ~120 bytes arrive split across two flushes.  Drain until
         * no more bytes arrive within one inter-flush window before calling the
         * callback, so the consumer always sees a complete message. */
        int more;
        do {
            more = uart_read_bytes(s_uart, buf + len, RX_BUF_SIZE - len, pdMS_TO_TICKS(150));
            if (more > 0) len += more;
        } while (more > 0 && len < RX_BUF_SIZE);

        if (s_rx_cb)
            s_rx_cb(buf, (size_t)len, s_rx_ctx);
    }
}

/* ── Public API ──────────────────────────────────────────────────────── */

esp_err_t lora_init(uart_port_t uart, int tx_pin, int rx_pin,
                    int m0_pin, int m1_pin, int aux_pin,
                    lora_rx_cb_t cb, void *ctx)
{
    s_uart    = uart;
    s_m0_pin  = m0_pin;
    s_m1_pin  = m1_pin;
    s_aux_pin = aux_pin;
    s_rx_cb   = cb;
    s_rx_ctx  = ctx;

    /* M0, M1 as outputs — start LOW (NORMAL mode) */
    gpio_config_t out_cfg = {
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
        .pin_bit_mask = (1ULL << m0_pin) | (1ULL << m1_pin),
    };
    ESP_ERROR_CHECK(gpio_config(&out_cfg));
    gpio_set_level(m0_pin, 0);
    gpio_set_level(m1_pin, 0);

    /* AUX as input — E22 drives AUX push-pull, no pullup needed */
    gpio_config_t in_cfg = {
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
        .pin_bit_mask = (1ULL << aux_pin),
    };
    ESP_ERROR_CHECK(gpio_config(&in_cfg));

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
    uart_wait_tx_done(s_uart, pdMS_TO_TICKS(500));
    return ESP_OK;
}
