#include "gnss.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/ringbuf.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

static const char *TAG = "gnss";

#define LC29H_BAUD          460800
#define GNSS_UART_BUF_SIZE  2048
#define NMEA_MAX_LINE       128
#define RTCM_MAX_FRAME      1029    /* 3 header + 1023 payload + 3 CRC */
#define RTCM_RINGBUF_SIZE   (RTCM_MAX_FRAME * 5 + 64)

static uart_port_t       s_uart;
static SemaphoreHandle_t s_tx_mutex;

/* Background reader queues */
static QueueHandle_t     s_pvt_queue;   /* depth-1 of gnss_pvt_t */
static RingbufHandle_t   s_rtcm_rb;     /* whole RTCM3 frames (NOSPLIT) */

/* Survey-in status, protected by mutex */
static gnss_svin_t       s_svin;
static SemaphoreHandle_t s_svin_mutex;
static bool              s_svin_updated;

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

static double parse_latlon(const char *val, char dir)
{
    if (!val || !*val) return 0.0;
    double raw = atof(val);
    int    deg = (int)(raw / 100);
    double result = deg + (raw - deg * 100.0) / 60.0;
    return (dir == 'S' || dir == 'W') ? -result : result;
}

static uint32_t parse_utc_ms(const char *t)
{
    if (!t || strlen(t) < 6) return 0;
    int hh = (t[0]-'0')*10 + (t[1]-'0');
    int mm = (t[2]-'0')*10 + (t[3]-'0');
    int ss = (t[4]-'0')*10 + (t[5]-'0');
    return (uint32_t)(hh*3600000 + mm*60000 + ss*1000);
}

/* ── Sentence parsers ────────────────────────────────────────────────── */

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

    float hdop  = atof(f[8]);
    pvt->h_acc  = hdop * 3.0f;
    pvt->v_acc  = hdop * 4.5f;

    pvt->carr_soln   = (quality == 4) ? 2 : (quality == 5) ? 1 : 0;
    pvt->fix_type    = (quality == 6) ? 1 : 3;
    pvt->gnss_fix_ok = true;
    return true;
}

static void parse_gsa(char *line, gnss_pvt_t *pvt)
{
    char *f[20];
    if (nmea_split(line, f, 20) >= 16)
        pvt->pdop = atof(f[15]);
}

/*
 * $PQTMSVINSTATUS,<active>,<valid>,<obs>,<meanX>,<meanY>,<meanZ>,<meanAcc>
 * meanAcc: mean 3D accuracy in cm (0.01 m units).
 */
static void parse_svinstatus(char *line)
{
    char *f[10];
    if (nmea_split(line, f, 10) < 8) return;

    gnss_svin_t sv;
    sv.active     = atoi(f[1]) != 0;
    sv.valid      = atoi(f[2]) != 0;
    sv.dur_s      = (uint32_t)atol(f[3]);
    sv.mean_acc_m = atof(f[7]) * 0.01f;   /* cm → m */

    xSemaphoreTake(s_svin_mutex, portMAX_DELAY);
    s_svin         = sv;
    s_svin_updated = true;
    xSemaphoreGive(s_svin_mutex);
}

/* ── CRC-24Q (RTCM3 integrity check) ────────────────────────────────── */

static uint32_t crc24q(const uint8_t *buf, size_t len)
{
    uint32_t crc = 0;
    while (len--) {
        crc ^= (uint32_t)(*buf++) << 16;
        for (int i = 0; i < 8; i++)
            crc = (crc << 1) ^ ((crc & 0x800000u) ? 0x1864CFBu : 0u);
    }
    return crc & 0xFFFFFFu;
}

/* ── NMEA checksum builder for PQTM commands ─────────────────────────── */

static void pqtm_send(const char *body)
{
    uint8_t cs = 0;
    for (const char *p = body; *p; p++) cs ^= (uint8_t)*p;
    char cmd[128];
    int  n = snprintf(cmd, sizeof(cmd), "$%s*%02X\r\n", body, cs);
    xSemaphoreTake(s_tx_mutex, portMAX_DELAY);
    uart_write_bytes(s_uart, cmd, n);
    xSemaphoreGive(s_tx_mutex);
    ESP_LOGD(TAG, "TX: %.*s", n - 2, cmd);   /* strip trailing \r\n */
}

/* ── Background stream reader task ──────────────────────────────────── */

/*
 * The LC29H outputs a mixed stream of NMEA sentences and RTCM3 binary
 * frames on the same UART.  This task owns the UART RX exclusively,
 * routes bytes to the appropriate consumer, and never blocks callers.
 *
 * Disambiguation:
 *   '$'  (0x24) → start of NMEA sentence
 *   0xD3        → start of RTCM3 frame
 *   Everything else while idle is silently discarded.
 */
static void gnss_reader_task(void *arg)
{
    /* NMEA accumulator */
    char    nmea_buf[NMEA_MAX_LINE];
    uint8_t nmea_idx = 0;
    bool    in_nmea  = false;

    /* Accumulated PVT (updated incrementally as sentences arrive) */
    gnss_pvt_t pvt = { .pdop = 99.9f };

    /* RTCM3 state machine */
    typedef enum { R_SYNC, R_LEN1, R_LEN2, R_PAYLOAD, R_CRC } rtcm_st_t;
    rtcm_st_t rtcm_st  = R_SYNC;
    uint8_t   rtcm_frame[RTCM_MAX_FRAME];
    uint16_t  rtcm_plen = 0;   /* payload length from header */
    uint16_t  rtcm_idx  = 0;   /* bytes accumulated into rtcm_frame */

    for (;;) {
        uint8_t b;
        if (uart_read_bytes(s_uart, &b, 1, pdMS_TO_TICKS(100)) != 1)
            continue;

        /* ── RTCM3 state machine (priority while frame in progress) ── */
        if (rtcm_st != R_SYNC) {
            if (rtcm_idx < RTCM_MAX_FRAME)
                rtcm_frame[rtcm_idx++] = b;
            else {
                /* Overrun — something went wrong, resync */
                ESP_LOGW(TAG, "RTCM frame overrun, resyncing");
                rtcm_st = R_SYNC; rtcm_idx = 0;
                /* Current byte discarded; next iteration classifies fresh */
                continue;
            }

            switch (rtcm_st) {
            case R_LEN1:
                /* bits[1:0] of byte 1 are the high 2 bits of the 10-bit length */
                rtcm_plen = (b & 0x03u) << 8;
                rtcm_st   = R_LEN2;
                break;
            case R_LEN2:
                rtcm_plen |= b;
                if (rtcm_plen > 1023) {
                    ESP_LOGW(TAG, "RTCM invalid length %u, resyncing", rtcm_plen);
                    rtcm_st = R_SYNC; rtcm_idx = 0;
                } else {
                    rtcm_st = R_PAYLOAD;
                }
                break;
            case R_PAYLOAD:
                /* Frame: [0xD3][len_hi][len_lo][payload×plen][CRC×3] */
                if (rtcm_idx >= 3 + rtcm_plen)
                    rtcm_st = R_CRC;
                break;
            case R_CRC:
                if (rtcm_idx >= 3 + rtcm_plen + 3) {
                    /* Complete frame — verify CRC-24Q */
                    uint16_t total = 3 + rtcm_plen + 3;
                    uint32_t computed = crc24q(rtcm_frame, 3 + rtcm_plen);
                    uint32_t stored   = ((uint32_t)rtcm_frame[3 + rtcm_plen]     << 16)
                                      | ((uint32_t)rtcm_frame[3 + rtcm_plen + 1] <<  8)
                                      |  (uint32_t)rtcm_frame[3 + rtcm_plen + 2];
                    if (computed == stored) {
                        if (xRingbufferSend(s_rtcm_rb, rtcm_frame, total, 0) != pdTRUE)
                            ESP_LOGW(TAG, "RTCM ringbuf full — dropped %u B frame", total);
                    } else {
                        ESP_LOGW(TAG, "RTCM CRC mismatch (got %06lX want %06lX)",
                                 (unsigned long)computed, (unsigned long)stored);
                    }
                    rtcm_st = R_SYNC; rtcm_idx = 0;
                }
                break;
            default:
                break;
            }
            continue;
        }

        /* ── Start RTCM3 frame ──── */
        if (b == 0xD3) {
            in_nmea        = false;
            rtcm_frame[0]  = 0xD3;
            rtcm_idx       = 1;
            rtcm_st        = R_LEN1;
            continue;
        }

        /* ── NMEA processing ─────── */
        if (b == '$') {
            nmea_idx = 0;
            in_nmea  = true;
            nmea_buf[nmea_idx++] = '$';
            continue;
        }
        if (!in_nmea) continue;

        if (b == '\n') {
            nmea_buf[nmea_idx] = '\0';
            nmea_idx  = 0;
            in_nmea   = false;

            if (!nmea_valid(nmea_buf)) continue;

            /* Sentence type at chars [3..5] (any 2-letter talker) */
            const char *type = nmea_buf + 3;
            if (strncmp(type, "GGA", 3) == 0) {
                parse_gga(nmea_buf, &pvt);
                /* Post on every GGA; GSA PDOP carries over */
                xQueueOverwrite(s_pvt_queue, &pvt);
            } else if (strncmp(type, "GSA", 3) == 0) {
                parse_gsa(nmea_buf, &pvt);
            } else if (strncmp(nmea_buf + 1, "PQTMSVINSTATUS", 14) == 0) {
                parse_svinstatus(nmea_buf);
            } else if (strncmp(nmea_buf + 1, "PQTM", 4) == 0) {
                /* Log all other PQTM responses so config ACKs are visible */
                ESP_LOGI(TAG, "%s", nmea_buf);
            }
        } else if (b != '\r' && nmea_idx < NMEA_MAX_LINE - 1) {
            nmea_buf[nmea_idx++] = b;
        }
    }
}

/* ── Public API ──────────────────────────────────────────────────────── */

esp_err_t gnss_init(uart_port_t port, int tx_pin, int rx_pin)
{
    s_uart = port;

    s_tx_mutex   = xSemaphoreCreateMutex();
    s_pvt_queue  = xQueueCreate(1, sizeof(gnss_pvt_t));
    s_svin_mutex = xSemaphoreCreateMutex();
    s_rtcm_rb    = xRingbufferCreate(RTCM_RINGBUF_SIZE, RINGBUF_TYPE_NOSPLIT);

    if (!s_tx_mutex || !s_pvt_queue || !s_svin_mutex || !s_rtcm_rb)
        return ESP_ERR_NO_MEM;

    uart_config_t cfg = {
        .baud_rate  = 115200,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    /* Baud-rate scan */
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
        uart_write_bytes(port, "\r\n", 2);

        ESP_LOGI(TAG, "trying %d baud on TX=%d RX=%d", try_bauds[bi], tx_pin, rx_pin);

        char    line[NMEA_MAX_LINE];
        uint8_t li = 0;
        int64_t deadline = esp_timer_get_time() + 2000000LL;
        while (esp_timer_get_time() < deadline) {
            uint8_t b;
            if (uart_read_bytes(port, &b, 1, pdMS_TO_TICKS(10)) != 1) continue;
            if (b == '\n') {
                line[li] = '\0'; li = 0;
                if (line[0] == '$' && strlen(line) > 5) {
                    active_baud = try_bauds[bi];
                    ESP_LOGI(TAG, "LC29H detected at %d baud: %s", active_baud, line);
                    break;
                }
            } else if (b != '\r' && li < NMEA_MAX_LINE - 1) {
                line[li++] = b;
            }
        }
        if (!active_baud)
            ESP_LOGI(TAG, "  no NMEA at %d baud", try_bauds[bi]);
    }

    if (!active_baud) {
        ESP_LOGE(TAG, "LC29H not detected on RX=%d at any baud", rx_pin);
        uart_driver_delete(port);
        return ESP_ERR_TIMEOUT;
    }

    if (active_baud != LC29H_BAUD)
        ESP_LOGW(TAG, "LC29H at %d baud — not at target %d", active_baud, LC29H_BAUD);

    /* Start background reader task */
    xTaskCreate(gnss_reader_task, "gnss_reader", 4096, NULL, 18, NULL);
    vTaskDelay(pdMS_TO_TICKS(50));  /* let task start so PQTM response lands in parser */

    /*
     * Set RTK rover mode.  Without this the LC29H default is standalone GNSS
     * and RTCM3 bytes arriving on the RX pin are silently discarded.
     * Expected response logged by reader: $PQTMCFGRCVRMODE,OK,1*XX
     */
    pqtm_send("PQTMCFGRCVRMODE,W,1");

    ESP_LOGI(TAG, "LC29H ready TX=%d RX=%d — RTK rover mode requested", tx_pin, rx_pin);
    return ESP_OK;
}

bool gnss_read_pvt(gnss_pvt_t *out, uint32_t timeout_ms)
{
    return xQueueReceive(s_pvt_queue, out, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
}

bool gnss_is_rtk_fix(const gnss_pvt_t *pvt)
{
    return pvt->gnss_fix_ok && pvt->carr_soln == 2;
}

void gnss_write_rtcm(const uint8_t *buf, size_t len)
{
    static uint32_t s_total = 0;
    static uint32_t s_last_log = 0;

    /*
     * Log each unique RTCM3 message type the first time it's seen.
     * This confirms what the caster is sending and that framing is valid.
     * RTCM3 message type = first 12 payload bits = buf[3]<<4 | buf[4]>>4.
     * Only works when buf[0]==0xD3 (frame-aligned chunk, typical for TCP recv).
     */
    if (len >= 6 && buf[0] == 0xD3) {
        uint16_t msg_type = ((uint16_t)(buf[3] & 0xFF) << 4) | (buf[4] >> 4);
        static uint16_t seen[20];
        static uint8_t  n_seen;
        bool dup = false;
        for (uint8_t i = 0; i < n_seen; i++) if (seen[i] == msg_type) { dup = true; break; }
        if (!dup && n_seen < 20) {
            seen[n_seen++] = msg_type;
            ESP_LOGI(TAG, "RTCM type %u (first seen, %u types total)", msg_type, n_seen);
        }
    }

    xSemaphoreTake(s_tx_mutex, portMAX_DELAY);
    uart_write_bytes(s_uart, (const char *)buf, len);
    s_total += len;
    uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
    if (now - s_last_log >= 10000) {
        ESP_LOGI(TAG, "RTCM → LC29H: %lu B total", (unsigned long)s_total);
        s_last_log = now;
    }
    xSemaphoreGive(s_tx_mutex);
}

/*
 * Start survey-in.
 * acc_limit in PQTM units = 0.0001 m (same as ZED-F9P TMODE3 convention).
 * Example: 2.0 m → 20000.
 */
esp_err_t gnss_start_survey_in(uint32_t min_dur_s, float acc_limit_m)
{
    (void)acc_limit_m;

    /*
     * The LC29H returns ERROR,3 for any W command sent while in base mode
     * (mode=2) — even PQTMCFGRTCM and PQTMSVIN.  gnss_init() leaves the
     * module in rover mode (W,1), so all configuration must be issued here
     * before switching to base mode.  PQTMCFGRCVRMODE,W,2 goes LAST.
     */

    /* Enable RTCM output (while still in rover mode). */
    pqtm_send("PQTMCFGRTCM,W,1");
    vTaskDelay(pdMS_TO_TICKS(200));

    /* Clear any stale survey-in state. */
    pqtm_send("PQTMSVIN,W,0");
    vTaskDelay(pdMS_TO_TICKS(200));

    /* Arm survey-in (takes effect when base mode is activated below). */
    char body[48];
    snprintf(body, sizeof(body), "PQTMSVIN,W,1,%lu", (unsigned long)min_dur_s);
    pqtm_send(body);
    vTaskDelay(pdMS_TO_TICKS(200));

    /* Switch to base mode — activates RTCM output and survey-in. */
    pqtm_send("PQTMCFGRCVRMODE,W,2");
    vTaskDelay(pdMS_TO_TICKS(500));

    /* Read back to confirm; response logged by reader task. */
    pqtm_send("PQTMCFGRCVRMODE,R");

    ESP_LOGI(TAG, "Base mode activated, RTCM enabled, survey-in armed (min=%lus)",
             (unsigned long)min_dur_s);
    return ESP_OK;
}

bool gnss_get_svin_status(gnss_svin_t *out)
{
    xSemaphoreTake(s_svin_mutex, portMAX_DELAY);
    bool updated = s_svin_updated;
    if (updated) {
        *out           = s_svin;
        s_svin_updated = false;
    }
    xSemaphoreGive(s_svin_mutex);
    return updated;
}

void gnss_poll_svin_status(void)
{
    pqtm_send("PQTMSVINSTATUS,R");
}

int gnss_read_rtcm(uint8_t *buf, size_t max_len, uint32_t timeout_ms)
{
    size_t   item_size = 0;
    uint8_t *item = xRingbufferReceive(s_rtcm_rb, &item_size,
                                        pdMS_TO_TICKS(timeout_ms));
    if (!item) return 0;
    size_t copy = (item_size < max_len) ? item_size : max_len;
    memcpy(buf, item, copy);
    vRingbufferReturnItem(s_rtcm_rb, item);
    if (item_size > max_len)
        ESP_LOGW(TAG, "RTCM frame truncated: %u → %u bytes", (unsigned)item_size, (unsigned)max_len);
    return (int)item_size;
}
