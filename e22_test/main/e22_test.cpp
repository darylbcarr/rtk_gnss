#include <cstdio>
#include <cstring>
#include <climits>
#include <cstdint>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/uart.h"
#include "e22.hpp"

static const char *TAG  = "main";
static const char *RLOG = "RESULT";

// ── Pin assignments ────────────────────────────────────────────────────────

static const E22Pins PINS_A = {
    .uart = UART_NUM_1, .tx_pin = GPIO_NUM_17, .rx_pin = GPIO_NUM_18,
    .aux  = GPIO_NUM_4, .m0     = GPIO_NUM_5,  .m1     = GPIO_NUM_21,
};
static const E22Pins PINS_B = {
    .uart = UART_NUM_2, .tx_pin = GPIO_NUM_19, .rx_pin = GPIO_NUM_20,
    .aux  = GPIO_NUM_1, .m0     = GPIO_NUM_2,  .m1     = GPIO_NUM_38,
};

static E22 radio_a(PINS_A);
static E22 radio_b(PINS_B);
static TaskHandle_t g_responder;
// Set true to make the responder yield B's UART to the main task.
// Safe alternative to vTaskSuspend: avoids deadlock on the UART driver's
// internal rx_mux semaphore, which vTaskSuspend can freeze mid-hold.
static volatile bool g_responder_pause = false;

// ── Config factory ─────────────────────────────────────────────────────────

static E22Config make_config(E22AirRate rate = E22AirRate::K2_4)
{
    E22Config cfg;
    cfg.channel    = 50;
    cfg.tx_power   = E22TxPower::DBM21;
    cfg.air_rate   = rate;
    cfg.fixed_mode = false;
    cfg.rssi_byte  = true;
    return cfg;
}

// ── Responder task ─────────────────────────────────────────────────────────
// B echoes every received packet back verbatim (RSSI byte stripped).

static void responder_task(void *)
{
    uint8_t buf[256];
    while (true) {
        if (g_responder_pause) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        // Short timeout so the pause flag is checked frequently.
        int n = radio_b.receive(buf, sizeof(buf) - 1, 50);
        if (n > 0) {
            // Payloads >120 B span two UART FIFO flushes (~92 ms apart at 9600 baud).
            // Drain until no more bytes arrive within one inter-flush window.
            int more;
            do {
                more = radio_b.receive(buf + n, sizeof(buf) - 1 - n, 150);
                n += more;
            } while (more > 0 && n < (int)(sizeof(buf) - 1));
            n--;   // strip RSSI indicator byte
            radio_b.send(buf, n);
        }
    }
}

// ── Helpers ────────────────────────────────────────────────────────────────

// Apply cfg to both radios; pauses the responder so it does not hold
// B's UART while write_config uses it.
static esp_err_t apply_config(const E22Config &cfg)
{
    g_responder_pause = true;
    vTaskDelay(pdMS_TO_TICKS(100));  // wait for in-flight 50 ms receive to return
    uart_flush_input(PINS_B.uart);
    esp_err_t r = radio_a.write_config(cfg);
    if (r == ESP_OK) r = radio_b.write_config(cfg);
    g_responder_pause = false;
    return r;
}

// Send payload from A, wait for B's echo back to A.
// Returns received RSSI (dBm), or INT_MIN on timeout.
// Fills *rtt_us with the round-trip time in microseconds if non-null.
static int ping_echo(const void *payload, size_t len, int64_t *rtt_us = nullptr)
{
    uint8_t buf[256];
    radio_a.wait_aux();   // exclude AUX settle from RTT measurement
    int64_t t0 = esp_timer_get_time();
    if (radio_a.send(payload, len) != ESP_OK) return INT_MIN;
    int n = radio_a.receive(buf, sizeof(buf) - 1, 2000);
    if (n <= 0) return INT_MIN;
    if (rtt_us) *rtt_us = esp_timer_get_time() - t0;
    return -(int)buf[n - 1] / 2;
}

// ── Test 1: Packet Error Rate ──────────────────────────────────────────────

struct PerStats {
    int sent     = 0;
    int received = 0;
    int rssi_min = 0;      // any real RSSI will be < 0
    int rssi_max = -200;
    int rssi_sum = 0;
};

static PerStats run_per(int count)
{
    PerStats s;
    for (int i = 0; i < count; i++) {
        char msg[16];
        snprintf(msg, sizeof(msg), "P%04d", i);
        int rssi = ping_echo(msg, 5);
        s.sent++;
        if (rssi != INT_MIN) {
            s.received++;
            s.rssi_sum += rssi;
            if (rssi < s.rssi_min) s.rssi_min = rssi;
            if (rssi > s.rssi_max) s.rssi_max = rssi;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    return s;
}

// ── Test 2: Round-trip Latency ─────────────────────────────────────────────

struct LatStats {
    int     count     = 0;
    int64_t min_us    = INT64_MAX;
    int64_t max_us    = 0;
    int64_t sum_us    = 0;
    int64_t prev_us   = -1;
    int64_t jitter_us = 0;   // mean absolute deviation of consecutive RTTs
};

static LatStats run_latency(int count)
{
    LatStats s;
    for (int i = 0; i < count; i++) {
        int64_t rtt;
        if (ping_echo("LAT", 3, &rtt) != INT_MIN) {
            s.count++;
            s.sum_us += rtt;
            if (rtt < s.min_us) s.min_us = rtt;
            if (rtt > s.max_us) s.max_us = rtt;
            if (s.prev_us >= 0) {
                int64_t d = rtt - s.prev_us;
                s.jitter_us += d < 0 ? -d : d;
            }
            s.prev_us = rtt;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    return s;
}


// ── Test 4: Air Rate Sweep ─────────────────────────────────────────────────

struct RateResult {
    const char *label      = "";
    int         sent       = 0;
    int         received   = 0;
    int         rssi_avg   = 0;
    int         latency_ms = 0;
};

struct RateEntry { E22AirRate rate; const char *label; };
static const RateEntry k_rates[] = {
    { E22AirRate::K1_2, "1.2 kbps" },
    { E22AirRate::K2_4, "2.4 kbps" },
    { E22AirRate::K4_8, "4.8 kbps" },
    { E22AirRate::K9_6, "9.6 kbps" },
};
static constexpr int k_num_rates = sizeof(k_rates) / sizeof(k_rates[0]);

static void run_rate_sweep(RateResult results[k_num_rates])
{
    for (int i = 0; i < k_num_rates; i++) {
        ESP_LOGI(TAG, "  testing %s", k_rates[i].label);
        apply_config(make_config(k_rates[i].rate));
        vTaskDelay(pdMS_TO_TICKS(200));

        RateResult &r = results[i];
        r.label = k_rates[i].label;
        int rssi_sum = 0;
        int64_t lat_sum = 0;

        for (int j = 0; j < 20; j++) {
            char msg[16];
            snprintf(msg, sizeof(msg), "S%d_%02d", i, j);
            int64_t rtt;
            int rssi = ping_echo(msg, 5, &rtt);
            r.sent++;
            if (rssi != INT_MIN) {
                r.received++;
                rssi_sum += rssi;
                lat_sum  += rtt;
            }
            vTaskDelay(pdMS_TO_TICKS(50));
        }
        if (r.received > 0) {
            r.rssi_avg   = rssi_sum / r.received;
            r.latency_ms = (int)(lat_sum / 1000LL / r.received);
        }
    }
    apply_config(make_config());   // restore default air rate
}

// ── Test 5: Payload Sizes ──────────────────────────────────────────────────

static const int k_payload_sizes[] = { 1, 50, 100, 199 };
static constexpr int k_num_payloads = sizeof(k_payload_sizes) / sizeof(k_payload_sizes[0]);

static void run_payload(bool results[k_num_payloads])
{
    uint8_t tx[200], rx[256];
    for (int i = 0; i < k_num_payloads; i++) {
        int sz = k_payload_sizes[i];
        for (int j = 0; j < sz; j++) tx[j] = (uint8_t)(j & 0xFF);

        if (radio_a.send(tx, sz) != ESP_OK) { results[i] = false; continue; }
        // Same FIFO-split issue on the receive side: loop until we have all sz+1 bytes.
        const int expected = sz + 1;   // payload + RSSI byte
        const uint32_t timeout_ms = 3000 + (uint32_t)sz * 12;
        int n = 0;
        while (n < expected) {
            int r = radio_a.receive(rx + n, (int)sizeof(rx) - 1 - n,
                                    n == 0 ? timeout_ms : 200);
            if (r <= 0) break;
            n += r;
        }
        if (n < 1) { results[i] = false; continue; }
        n--;   // strip RSSI byte
        results[i] = (n == sz) && (memcmp(tx, rx, sz) == 0);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// ── Summary ────────────────────────────────────────────────────────────────

static void print_summary(const PerStats &per, const LatStats &lat,
                          int noise_a, int noise_b,
                          const RateResult rate[], const bool payload[])
{
    const float loss = per.sent ? 100.0f * (per.sent - per.received) / per.sent : 0.0f;
    const int rssi_avg = per.received ? per.rssi_sum / per.received : 0;

    ESP_LOGI(RLOG, "========================================");
    ESP_LOGI(RLOG, "           TEST SUMMARY");
    ESP_LOGI(RLOG, "========================================");

    ESP_LOGI(RLOG, "1. Packet Error Rate (%d packets)", per.sent);
    ESP_LOGI(RLOG, "   Received : %d / %d  (%.1f%% loss)", per.received, per.sent, loss);
    if (per.received)
        ESP_LOGI(RLOG, "   RSSI     : min=%d  avg=%d  max=%d dBm",
                 per.rssi_min, rssi_avg, per.rssi_max);

    if (lat.count > 0) {
        const int64_t avg = lat.sum_us / lat.count;
        const int64_t jit = lat.count > 1 ? lat.jitter_us / (lat.count - 1) : 0;
        ESP_LOGI(RLOG, "2. Round-trip Latency (%d samples)", lat.count);
        ESP_LOGI(RLOG, "   Min    : %lld ms", (long long)(lat.min_us / 1000));
        ESP_LOGI(RLOG, "   Max    : %lld ms", (long long)(lat.max_us / 1000));
        ESP_LOGI(RLOG, "   Avg    : %lld ms", (long long)(avg / 1000));
        ESP_LOGI(RLOG, "   Jitter : %lld ms", (long long)(jit / 1000));
    }

    ESP_LOGI(RLOG, "3. Noise Floor");
    if (noise_a != INT_MIN)
        ESP_LOGI(RLOG, "   Module A : %d dBm", noise_a);
    else
        ESP_LOGI(RLOG, "   Module A : command not supported by firmware");
    if (noise_b != INT_MIN)
        ESP_LOGI(RLOG, "   Module B : %d dBm", noise_b);
    else
        ESP_LOGI(RLOG, "   Module B : command not supported by firmware");

    ESP_LOGI(RLOG, "4. Air Rate Sweep (20 packets each)");
    ESP_LOGI(RLOG, "   %-10s  Rcvd/Sent   Loss    RSSI     RTT", "Rate");
    for (int i = 0; i < k_num_rates; i++) {
        const float rl = rate[i].sent
            ? 100.0f * (rate[i].sent - rate[i].received) / rate[i].sent : 0.0f;
        ESP_LOGI(RLOG, "   %-10s  %2d / %2d     %5.1f%%  %4d dBm  %4d ms",
                 rate[i].label, rate[i].received, rate[i].sent,
                 rl, rate[i].rssi_avg, rate[i].latency_ms);
    }

    ESP_LOGI(RLOG, "5. Payload Sizes");
    for (int i = 0; i < k_num_payloads; i++)
        ESP_LOGI(RLOG, "   %4d bytes : %s",
                 k_payload_sizes[i], payload[i] ? "PASS" : "FAIL");

    ESP_LOGI(RLOG, "========================================");
}

// ── app_main ───────────────────────────────────────────────────────────────

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Initialising radio A");
    ESP_ERROR_CHECK(radio_a.init());
    ESP_LOGI(TAG, "Initialising radio B");
    ESP_ERROR_CHECK(radio_b.init());

    ESP_LOGI(TAG, "Configuring radios");
    E22Config cfg = make_config();
    ESP_ERROR_CHECK(radio_a.write_config(cfg));
    ESP_ERROR_CHECK(radio_b.write_config(cfg));

    // Test 3 runs before the responder starts: no concurrency on either UART,
    // so uart_flush_input and read_noise_rssi are safe to call freely.
    ESP_LOGI(TAG, "--- Test 3: Noise Floor ---");
    int noise_a = INT_MIN, noise_b = INT_MIN;
    vTaskDelay(pdMS_TO_TICKS(200));
    noise_a = radio_a.read_noise_rssi();
    uart_flush_input(PINS_B.uart);
    vTaskDelay(pdMS_TO_TICKS(200));
    noise_b = radio_b.read_noise_rssi();
    uart_flush_input(PINS_A.uart);

    xTaskCreate(responder_task, "responder", 4096, nullptr, 5, &g_responder);
    vTaskDelay(pdMS_TO_TICKS(200));

    ESP_LOGI(TAG, "--- Test 1: Packet Error Rate (50 packets) ---");
    const PerStats per = run_per(50);

    ESP_LOGI(TAG, "--- Test 2: Round-trip Latency (30 samples) ---");
    const LatStats lat = run_latency(30);

    ESP_LOGI(TAG, "--- Test 4: Air Rate Sweep (4 rates x 20 packets) ---");
    RateResult rate_results[k_num_rates] = {};
    run_rate_sweep(rate_results);

    ESP_LOGI(TAG, "--- Test 5: Payload Sizes (1/50/100/199 bytes) ---");
    bool payload_results[k_num_payloads] = {};
    run_payload(payload_results);

    print_summary(per, lat, noise_a, noise_b, rate_results, payload_results);
}
