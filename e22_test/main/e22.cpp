#include "e22.hpp"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/task.h"
#include <cstring>

static const char *TAG = "E22";

static constexpr int UART_BUF    = 512;
static constexpr int AUX_POLL    = 10;   // ms between AUX polls
static constexpr int MODE_SETTLE = 50;   // ms after driving M0/M1

// ── wait_aux ───────────────────────────────────────────────────────────────

esp_err_t E22::wait_aux(uint32_t timeout_ms)
{
    for (uint32_t elapsed = 0; gpio_get_level(_pins.aux) == 0; elapsed += AUX_POLL) {
        if (elapsed >= timeout_ms) {
            ESP_LOGE(TAG, "AUX timeout after %lu ms", (unsigned long)timeout_ms);
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(pdMS_TO_TICKS(AUX_POLL));
    }
    return ESP_OK;
}

// ── init ───────────────────────────────────────────────────────────────────

esp_err_t E22::init()
{
    gpio_config_t aux_cfg = {};
    aux_cfg.pin_bit_mask = 1ULL << _pins.aux;
    aux_cfg.mode         = GPIO_MODE_INPUT;
    aux_cfg.pull_up_en   = GPIO_PULLUP_ENABLE;
    ESP_RETURN_ON_ERROR(gpio_config(&aux_cfg), TAG, "AUX gpio_config");

    gpio_config_t mode_cfg = {};
    mode_cfg.pin_bit_mask = (1ULL << _pins.m0) | (1ULL << _pins.m1);
    mode_cfg.mode         = GPIO_MODE_OUTPUT;
    ESP_RETURN_ON_ERROR(gpio_config(&mode_cfg), TAG, "M0/M1 gpio_config");
    gpio_set_level(_pins.m0, 0);
    gpio_set_level(_pins.m1, 0);

    uart_config_t uart_cfg = {};
    uart_cfg.baud_rate  = 9600;
    uart_cfg.data_bits  = UART_DATA_8_BITS;
    uart_cfg.parity     = UART_PARITY_DISABLE;
    uart_cfg.stop_bits  = UART_STOP_BITS_1;
    uart_cfg.flow_ctrl  = UART_HW_FLOWCTRL_DISABLE;
    uart_cfg.source_clk = UART_SCLK_DEFAULT;
    ESP_RETURN_ON_ERROR(uart_param_config(_pins.uart, &uart_cfg), TAG, "uart_param_config");
    ESP_RETURN_ON_ERROR(uart_set_pin(_pins.uart, _pins.tx_pin, _pins.rx_pin,
                                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE), TAG, "uart_set_pin");
    ESP_RETURN_ON_ERROR(uart_driver_install(_pins.uart, UART_BUF, 0, 0, nullptr, 0),
                        TAG, "uart_driver_install");

    return wait_aux();
}

// ── set_mode ───────────────────────────────────────────────────────────────

esp_err_t E22::set_mode(E22Mode mode)
{
    ESP_RETURN_ON_ERROR(wait_aux(), TAG, "set_mode: pre-wait");

    const int v = static_cast<int>(mode);
    gpio_set_level(_pins.m0, (v >> 0) & 1);
    gpio_set_level(_pins.m1, (v >> 1) & 1);

    vTaskDelay(pdMS_TO_TICKS(MODE_SETTLE));
    ESP_RETURN_ON_ERROR(wait_aux(), TAG, "set_mode: post-wait");

    _mode = mode;
    return ESP_OK;
}

// ── read_config ────────────────────────────────────────────────────────────

esp_err_t E22::read_config(E22Config &cfg)
{
    ESP_RETURN_ON_ERROR(set_mode(E22Mode::CONFIG), TAG, "read_config: set mode");
    uart_flush(_pins.uart);

    const uint8_t cmd[] = {0xC1, 0x00, 0x09};
    uart_write_bytes(_pins.uart, cmd, sizeof(cmd));

    uint8_t r[12];
    int n = uart_read_bytes(_pins.uart, r, sizeof(r), pdMS_TO_TICKS(1000));
    if (n < 12 || r[0] != 0xC1 || r[1] != 0x00 || r[2] != 0x09) {
        ESP_LOGE(TAG, "read_config: bad reply (n=%d, head=%02X)", n, n > 0 ? r[0] : 0xFF);
        set_mode(E22Mode::NORMAL);
        return ESP_FAIL;
    }

    cfg.address    = (uint16_t)(r[3] << 8) | r[4];
    cfg.net_id     = r[5];
    cfg.uart_baud  = static_cast<E22Baud>   ((r[6] >> 5) & 0x07);
    cfg.air_rate   = static_cast<E22AirRate>( r[6]        & 0x07);
    cfg.sub_packet = (r[7] >> 6) & 0x03;
    cfg.rssi_noise = (r[7] >> 5) & 0x01;
    cfg.tx_power   = static_cast<E22TxPower>( r[7]        & 0x03);
    cfg.channel    = r[8];
    cfg.rssi_byte  = (r[9] >> 7) & 0x01;
    cfg.fixed_mode = (r[9] >> 6) & 0x01;
    cfg.lbt        = (r[9] >> 4) & 0x01;
    cfg.wor_cycle  =  r[9]        & 0x0F;
    cfg.crypt_key  = (uint16_t)(r[10] << 8) | r[11];

    return set_mode(E22Mode::NORMAL);
}

// ── write_config ───────────────────────────────────────────────────────────

esp_err_t E22::write_config(const E22Config &cfg)
{
    ESP_RETURN_ON_ERROR(set_mode(E22Mode::CONFIG), TAG, "write_config: set mode");
    uart_flush(_pins.uart);

    uint8_t cmd[12];
    cmd[0]  = 0xC0;   // 0xC0 = permanent (flash);  0xC2 = temporary
    cmd[1]  = 0x00;
    cmd[2]  = 0x09;
    cmd[3]  = (cfg.address >> 8) & 0xFF;
    cmd[4]  =  cfg.address       & 0xFF;
    cmd[5]  = cfg.net_id;
    cmd[6]  = (static_cast<uint8_t>(cfg.uart_baud) << 5)
            |  static_cast<uint8_t>(cfg.air_rate);
    cmd[7]  = ((cfg.sub_packet & 0x03) << 6)
            | (cfg.rssi_noise ? 0x20 : 0x00)
            |  static_cast<uint8_t>(cfg.tx_power);
    cmd[8]  = cfg.channel;
    cmd[9]  = (cfg.rssi_byte  ? 0x80 : 0x00)
            | (cfg.fixed_mode ? 0x40 : 0x00)
            | (cfg.lbt        ? 0x10 : 0x00)
            | (cfg.wor_cycle  & 0x0F);
    cmd[10] = (cfg.crypt_key >> 8) & 0xFF;
    cmd[11] =  cfg.crypt_key       & 0xFF;

    uart_write_bytes(_pins.uart, cmd, sizeof(cmd));

    // Module echoes cmd back as ACK (0xC1 header) then saves to flash
    uint8_t ack[12];
    int n = uart_read_bytes(_pins.uart, ack, sizeof(ack), pdMS_TO_TICKS(500));
    if (n < 12 || ack[0] != 0xC1) {
        ESP_LOGW(TAG, "write_config: unexpected ACK (n=%d)", n);
    }

    ESP_RETURN_ON_ERROR(wait_aux(), TAG, "write_config: post-wait");
    return set_mode(E22Mode::NORMAL);
}

// ── send / receive ─────────────────────────────────────────────────────────

esp_err_t E22::send(const void *data, size_t len)
{
    if (_mode != E22Mode::NORMAL) {
        ESP_LOGE(TAG, "send: must be in NORMAL mode");
        return ESP_ERR_INVALID_STATE;
    }
    ESP_RETURN_ON_ERROR(wait_aux(), TAG, "send: wait_aux");
    uart_write_bytes(_pins.uart, data, len);
    return ESP_OK;
}

int E22::receive(void *buf, size_t max_len, uint32_t timeout_ms)
{
    return uart_read_bytes(_pins.uart, buf, max_len, pdMS_TO_TICKS(timeout_ms));
}

// ── read_noise_rssi ────────────────────────────────────────────────────────
// Sends the 3-byte RSSI query command.  If the module supports it the
// response echoes those 3 bytes followed by the RSSI indicator byte.
// If the module does not support the command it will transmit the 3 bytes
// over the air instead and not respond on UART → we return INT_MIN.

int E22::read_noise_rssi()
{
    if (_mode != E22Mode::NORMAL) return INT_MIN;
    uart_flush(_pins.uart);
    const uint8_t cmd[] = {0xC0, 0xC1, 0xC2};
    uart_write_bytes(_pins.uart, cmd, sizeof(cmd));
    uint8_t resp[4];
    int n = uart_read_bytes(_pins.uart, resp, sizeof(resp), pdMS_TO_TICKS(500));
    if (n < 4 || resp[0] != 0xC0 || resp[1] != 0xC1 || resp[2] != 0xC2)
        return INT_MIN;
    return -(int)resp[3] / 2;
}
