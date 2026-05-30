#include "gnss.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "gnss";

#define LC29H_BAUD         460800
#define GNSS_UART_BUF_SIZE 1024
#define NMEA_MAX_LINE      100

static uart_port_t       s_uart;
static SemaphoreHandle_t s_tx_mutex;

/* ── NMEA helpers ────────────────────────────────────────────────────── */

static bool nmea_valid(const char *line)
{
    if (line[0] != '$') return false;
    const char *star = strchr(line, '*');
    if (!star || !star[1] || !star[2]) return false;
    uint8_t cs = 0;
    for (const char *p = line + 1; p < star; p++) cs ^= (uint8_t)*p;
    char hex[3] = { star[1], star[2], '\0' };
    return cs == (uint8_t)strtol(hex, NULL, 16);
}

/* Split at commas and '*'; replaces separators with '\0' in place.
   Returns the number of fields written to f[]. */
static int nmea_split(char *line, char **f, int max)
{
    int n = 0;
    if (n < max) f[n++] = line;
    for (char *p = line; *p; p++) {
        if ((*p == ',' || *p == '*') && n < max) {
            *p = '\0';
            f[n++] = p + 1;
        }
    }
    return n;
}

/* DDmm.mmmmm → decimal degrees, with N/S/E/W sign. */
static double parse_latlon(const char *val, char dir)
{
    if (!val || !*val) return 0.0;
    double raw = atof(val);
    int    deg = (int)(raw / 100);
    double result = deg + (raw - deg * 100.0) / 60.0;
    return (dir == 'S' || dir == 'W') ? -result : result;
}

/* Parse HHMMSS[.SS] time field into milliseconds-of-day (for i_tow). */
static uint32_t parse_utc_ms(const char *t)
{
    if (!t || strlen(t) < 6) return 0;
    int hh = (t[0] - '0') * 10 + (t[1] - '0');
    int mm = (t[2] - '0') * 10 + (t[3] - '0');
    int ss = (t[4] - '0') * 10 + (t[5] - '0');
    return (uint32_t)(hh * 3600000 + mm * 60000 + ss * 1000);
}

/* ── Sentence parsers ────────────────────────────────────────────────── */

/* $xxGGA,HHMMSS,lat,N,lon,E,qual,sv,hdop,alt,M,...  */
static bool parse_gga(char *line, gnss_pvt_t *pvt)
{
    char *f[16];
    if (nmea_split(line, f, 16) < 10) return false;

    int quality = atoi(f[6]);
    if (quality == 0) {
        pvt->fix_type    = 0;
        pvt->carr_soln   = 0;
        pvt->gnss_fix_ok = false;
        return true;
    }

    pvt->lat     = parse_latlon(f[2], f[3][0]);
    pvt->lon     = parse_latlon(f[4], f[5][0]);
    pvt->alt_msl = atof(f[9]);
    pvt->num_sv  = (uint8_t)atoi(f[7]);
    pvt->i_tow   = parse_utc_ms(f[1]);

    /* Rough accuracy estimate from HDOP (adequate for display/logging). */
    float hdop  = atof(f[8]);
    pvt->h_acc  = hdop * 3.0f;
    pvt->v_acc  = hdop * 4.5f;

    /* GGA quality → carr_soln: 4=RTK Fixed→2, 5=RTK Float→1, else→0 */
    pvt->carr_soln   = (quality == 4) ? 2 : (quality == 5) ? 1 : 0;
    pvt->fix_type    = (quality == 6) ? 1 : 3;  /* 1=DR, 3=3D */
    pvt->gnss_fix_ok = true;
    return true;
}

/* $xxGSA,M,3,sv,sv,...,pdop,hdop,vdop  */
static void parse_gsa(char *line, gnss_pvt_t *pvt)
{
    char *f[20];
    if (nmea_split(line, f, 20) >= 16)
        pvt->pdop = atof(f[15]);
}

/* ── Public API ──────────────────────────────────────────────────────── */

esp_err_t gnss_init(uart_port_t port, int tx_pin, int rx_pin)
{
    s_uart = port;
    if (!s_tx_mutex) s_tx_mutex = xSemaphoreCreateMutex();

    uart_config_t cfg = {
        .baud_rate  = 115200,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    /* ── UART self-test ───────────────────────────────────────────────────
     * Send 4 bytes and try to read them back within 50 ms.
     * With TX shorted to RX this confirms the ESP UART hardware is working.
     * The test bytes are harmless garbage if the LC29H is connected instead. */
    uart_driver_delete(port);
    ESP_ERROR_CHECK(uart_driver_install(port, GNSS_UART_BUF_SIZE, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(port, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(port, tx_pin, rx_pin,
                                  UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    uart_flush(port);
    const uint8_t ping[] = { 0x55, 0xAA, 0x55, 0xAA };   /* alternating pattern */
    uart_write_bytes(port, (const char *)ping, sizeof(ping));
    uint8_t pong[4] = {0};
    int self_got = uart_read_bytes(port, pong, sizeof(pong), pdMS_TO_TICKS(50));
    if (self_got == 4 && memcmp(pong, ping, 4) == 0)
        ESP_LOGI(TAG, "UART self-test PASS — TX=%d→RX=%d loopback confirmed", tx_pin, rx_pin);
    else
        ESP_LOGI(TAG, "UART self-test: sent 4, got %d bytes on TX=%d RX=%d "
                 "(0 expected when LC29H connected, 4 when TX shorted to RX)",
                 self_got, tx_pin, rx_pin);

    /* ── Baud-rate scan ───────────────────────────────────────────────── */
    static const int try_bauds[] = { 460800, 115200, 9600, 38400, 57600, 19200, 4800 };
    int active_baud = 0;

    for (int bi = 0; bi < (int)(sizeof(try_bauds)/sizeof(try_bauds[0])) && !active_baud; bi++) {
        cfg.baud_rate = try_bauds[bi];
        uart_driver_delete(port);
        ESP_ERROR_CHECK(uart_driver_install(port, GNSS_UART_BUF_SIZE, 0, 0, NULL, 0));
        ESP_ERROR_CHECK(uart_param_config(port, &cfg));
        ESP_ERROR_CHECK(uart_set_pin(port, tx_pin, rx_pin,
                                      UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
        uart_flush(port);

        /* Send \r\n at each baud in case the module needs a prompt to start */
        uart_write_bytes(port, "\r\n", 2);

        ESP_LOGI(TAG, "trying %d baud on RX=%d TX=%d", try_bauds[bi], rx_pin, tx_pin);

        /* 2 s window — module outputs at 1 Hz so one sentence should arrive */
        int     rx_bytes = 0;
        char    line[NMEA_MAX_LINE];
        uint8_t li = 0;
        int64_t deadline = esp_timer_get_time() + 2000000LL;
        while (esp_timer_get_time() < deadline) {
            uint8_t b;
            if (uart_read_bytes(port, &b, 1, pdMS_TO_TICKS(10)) != 1) continue;
            rx_bytes++;
            if (b == '\n') {
                line[li] = '\0'; li = 0;
                if (line[0] == '$' && strlen(line) > 5) {
                    active_baud = try_bauds[bi];
                    ESP_LOGI(TAG, "LC29H detected at %d baud: %s", active_baud, line);
                }
            } else if (b != '\r' && li < NMEA_MAX_LINE - 1) {
                line[li++] = b;
            }
        }
        ESP_LOGI(TAG, "  %d baud: %d bytes rx'd, %s",
                 try_bauds[bi], rx_bytes, active_baud ? "NMEA found" : "no NMEA");
    }

    if (!active_baud) {
        ESP_LOGE(TAG, "LC29H not detected on RX=%d at any baud rate", rx_pin);
        ESP_LOGE(TAG, "Check: antenna connected? module powered? TX pin floating?");
        uart_driver_delete(port);
        return ESP_ERR_TIMEOUT;
    }

    /* If module was found at a non-default baud, reconfigure to 115200.
     * (Send PQTMCFGUART then switch ESP side — skipped for now since
     *  active_baud == LC29H_BAUD in the normal case.) */
    if (active_baud != LC29H_BAUD)
        ESP_LOGW(TAG, "LC29H running at %d baud (not %d) — RTCM may misbehave",
                 active_baud, LC29H_BAUD);

    ESP_LOGI(TAG, "LC29H ready — NMEA active on RX=%d TX=%d", rx_pin, tx_pin);
    return ESP_OK;
}

bool gnss_read_pvt(gnss_pvt_t *out, uint32_t timeout_ms)
{
    memset(out, 0, sizeof(*out));
    out->pdop = 99.9f;

    char    line[NMEA_MAX_LINE];
    uint8_t li = 0;
    bool    got_gga = false;
    bool    got_gsa = false;
    int64_t deadline = esp_timer_get_time() + (int64_t)timeout_ms * 1000;

    while (!(got_gga && got_gsa) && esp_timer_get_time() < deadline) {
        uint8_t b;
        if (uart_read_bytes(s_uart, &b, 1, pdMS_TO_TICKS(10)) != 1) continue;

        if (b == '\n') {
            line[li] = '\0';
            li = 0;

            if (!nmea_valid(line) || line[0] != '$') continue;

            /* Match any 2-letter talker + sentence type at chars [3..5]. */
            if (strncmp(line + 3, "GGA", 3) == 0) {
                parse_gga(line, out);
                got_gga = true;
            } else if (strncmp(line + 3, "GSA", 3) == 0 && !got_gsa) {
                parse_gsa(line, out);
                got_gsa = true;
            }
        } else if (b != '\r' && li < NMEA_MAX_LINE - 1) {
            line[li++] = b;
        }
    }
    return got_gga;
}

bool gnss_is_rtk_fix(const gnss_pvt_t *pvt)
{
    return pvt->gnss_fix_ok && pvt->carr_soln == 2;
}

void gnss_write_rtcm(const uint8_t *buf, size_t len)
{
    static uint32_t s_total = 0;
    static uint32_t s_last_log = 0;
    xSemaphoreTake(s_tx_mutex, portMAX_DELAY);
    uart_write_bytes(s_uart, (const char *)buf, len);
    s_total += len;
    uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
    if (now - s_last_log >= 10000) {
        ESP_LOGI(TAG, "RTCM → LC29H: %lu B total", s_total);
        s_last_log = now;
    }
    xSemaphoreGive(s_tx_mutex);
}
