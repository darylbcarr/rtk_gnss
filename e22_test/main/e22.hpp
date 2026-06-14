#pragma once
#include <cstdint>
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

// ── Pin bundle ─────────────────────────────────────────────────────────────
struct E22Pins {
    uart_port_t uart   = UART_NUM_1;
    gpio_num_t  tx_pin = GPIO_NUM_17;   // ESP TX → E22 RXD
    gpio_num_t  rx_pin = GPIO_NUM_18;   // ESP RX ← E22 TXD
    gpio_num_t  aux    = GPIO_NUM_4;
    gpio_num_t  m0     = GPIO_NUM_5;
    gpio_num_t  m1     = GPIO_NUM_21;
};

// ── Enums ──────────────────────────────────────────────────────────────────
enum class E22Mode : uint8_t {
    NORMAL = 0,   // M0=0 M1=0  transparent / fixed UART
    WOR    = 1,   // M0=1 M1=0  wake-on-radio sender
    CONFIG = 2,   // M0=0 M1=1  register read/write
    SLEEP  = 3,   // M0=1 M1=1  deep sleep / WOR receiver
};

enum class E22Baud : uint8_t {
    B1200   = 0,
    B2400   = 1,
    B4800   = 2,
    B9600   = 3,   // factory default
    B19200  = 4,
    B38400  = 5,
    B57600  = 6,
    B115200 = 7,
};

enum class E22AirRate : uint8_t {
    K0_3  = 0,
    K1_2  = 1,
    K2_4  = 2,   // factory default
    K4_8  = 3,
    K9_6  = 4,
    K19_2 = 5,
    K38_4 = 6,
    K62_5 = 7,
};

enum class E22TxPower : uint8_t {
    DBM30 = 0,   // factory default (max for T30D)
    DBM27 = 1,
    DBM24 = 2,
    DBM21 = 3,
};

// ── Config struct ──────────────────────────────────────────────────────────
// Mirrors the nine register bytes the module stores (ADDH…CRYPT_L).
// Frequency = 850.125 + channel MHz  →  channel 50 = 900.125 MHz
struct E22Config {
    uint16_t   address    = 0x0000;
    uint8_t    net_id     = 0x00;
    E22Baud    uart_baud  = E22Baud::B9600;
    E22AirRate air_rate   = E22AirRate::K2_4;
    uint8_t    sub_packet = 0;      // 0=200 B  1=128 B  2=64 B  3=32 B
    bool       rssi_noise = false;  // report ambient RSSI on AUX pin
    E22TxPower tx_power   = E22TxPower::DBM30;
    uint8_t    channel    = 50;
    bool       rssi_byte  = false;  // append RSSI byte to received frames
    bool       fixed_mode = false;  // true = fixed addressing (ADDR+CH prefix)
    bool       lbt        = false;  // listen-before-talk
    uint8_t    wor_cycle  = 3;      // 0–7
    uint16_t   crypt_key  = 0x0000;
};

// ── Driver class ───────────────────────────────────────────────────────────
class E22 {
public:
    explicit E22(E22Pins pins) : _pins(pins) {}

    // Block until AUX is high (module ready).
    esp_err_t wait_aux(uint32_t timeout_ms = 3000);

    esp_err_t init();
    esp_err_t set_mode(E22Mode mode);

    // Enter CONFIG mode, read all nine registers, return to NORMAL.
    esp_err_t read_config(E22Config &cfg);

    // Enter CONFIG mode, write all nine registers, return to NORMAL.
    // NOTE: if you change uart_baud you must also update the UART peripheral.
    esp_err_t write_config(const E22Config &cfg);

    // Send raw bytes (NORMAL mode, transparent or fixed addressing).
    esp_err_t send(const void *data, size_t len);

    // Read received bytes.  Returns byte count (0 if none within timeout).
    int receive(void *buf, size_t max_len, uint32_t timeout_ms = 0);

    // Request ambient channel RSSI (dBm) via the 0xC0 0xC1 0xC2 command.
    // Requires NORMAL mode and rssi_noise=true in config.
    // Returns INT_MIN if the module does not respond to the command.
    int read_noise_rssi();

private:
    E22Pins _pins;
    E22Mode _mode = E22Mode::NORMAL;
};
