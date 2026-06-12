#include "gnss.h"
#include "driver/uart.h"
#include "driver/gpio.h"
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
#define RTCM_RINGBUF_SIZE   (RTCM_MAX_FRAME * 20 + 128)  /* ~20 s @ 1 kB/s */

static uart_port_t       s_uart;
static bool              s_booted_in_base;   /* RTCM detected at init — saved config intact */
static int               s_nreset_pin = -1;  /* GPIO wired to NRESET, or -1 if not connected */
static SemaphoreHandle_t s_tx_mutex;

/* Background reader queues */
static QueueHandle_t     s_pvt_queue;   /* depth-1 of gnss_pvt_t */
static RingbufHandle_t   s_rtcm_rb;     /* whole RTCM3 frames (NOSPLIT) */

/* Survey-in status, protected by mutex */
static gnss_svin_t       s_svin;
static SemaphoreHandle_t s_svin_mutex;
static bool              s_svin_updated;

/* RTCM ARP station ID seen by rover (0xFFFF = none yet) */
static uint16_t s_rtcm_arp_sid = 0xFFFFu;

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
    int nf = nmea_split(line, f, 16);
    if (nf < 10) return false;

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
    pvt->gga_quality = (uint8_t)quality;

    /*
     * Differential corrections diagnostic: GGA fields 13 (age) and 14 (station).
     * age=0-2 s means the LC29H is actively applying fresh corrections.
     * age > 10 s means the last correction was long ago (or retained from a
     * previous power cycle — which appears as quality=5 at boot before any
     * RTCM arrives).  Log when quality, station, or age-band changes so we
     * can see the exact moment fresh corrections kick in.
     */
    pvt->diff_age_s = (nf >= 14 && f[13][0] != '\0') ? atof(f[13]) : 0.0f;

    if (nf >= 15) {
        float diff_age     = pvt->diff_age_s;
        int   diff_station = atoi(f[14]);
        static float   s_last_age     = -999.0f;
        static int     s_last_station = -1;
        static int     s_last_quality = -1;
        float age_delta = diff_age - s_last_age;
        bool  age_band_changed = (age_delta > 5.0f || age_delta < -5.0f);
        if (quality      != s_last_quality  ||
            diff_station != s_last_station  ||
            age_band_changed) {
            ESP_LOGI(TAG, "GGA: quality=%d station=%d diff_age=%.1fs sv=%d",
                     quality, diff_station, diff_age, pvt->num_sv);
            s_last_age     = diff_age;
            s_last_station = diff_station;
            s_last_quality = quality;
        }
    }

    return true;
}

static void parse_gsa(char *line, gnss_pvt_t *pvt)
{
    char *f[20];
    if (nmea_split(line, f, 20) >= 16)
        pvt->pdop = atof(f[15]);
}

/*
 * Parse either format of SVINSTATUS into s_svin.
 *
 * Autonomous output: $PQTMSVINSTATUS,active,valid,obs,meanX,meanY,meanZ,meanAcc*cs
 *   base=1  (data starts at f[1])
 *
 * Query response:   $PQTM,PQTMSVINSTATUS,0,active,valid,obs,meanX,meanY,meanZ,meanAcc*cs
 *   base=3  (skip sentence name, command name, result code)
 *
 * meanAcc is in cm; multiply by 0.01 to get metres.
 */
static void parse_svinstatus(char *line, int base)
{
    char *f[12];
    if (nmea_split(line, f, 12) < base + 7) return;

    gnss_svin_t sv;
    sv.active     = atoi(f[base])     != 0;
    sv.valid      = atoi(f[base + 1]) != 0;
    sv.dur_s      = (uint32_t)atol(f[base + 2]);   /* obs count ≈ elapsed s at 1 Hz */
    sv.mean_acc_m = atof(f[base + 6]) * 0.01f;     /* cm → m */

    ESP_LOGI(TAG, "svin: active=%d valid=%d obs=%lu acc=%.2fm",
             (int)sv.active, (int)sv.valid,
             (unsigned long)sv.dur_s, sv.mean_acc_m);

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
                /* Autonomous output: $PQTMSVINSTATUS,active,valid,obs,...,meanAcc */
                parse_svinstatus(nmea_buf, 1);
            } else if (strncmp(nmea_buf + 1, "PQTM,PQTMSVINSTATUS,", 20) == 0) {
                /* Query response: $PQTM,PQTMSVINSTATUS,<result>,active,valid,obs,...,meanAcc */
                parse_svinstatus(nmea_buf, 3);
            } else if (strncmp(nmea_buf + 1, "PQTM", 4) == 0 ||
                       strncmp(nmea_buf + 1, "PAIR", 4) == 0) {
                /* All PQTM and PAIR responses at INFO so query replies are visible */
                ESP_LOGI(TAG, "%s", nmea_buf);
            }
        } else if (b != '\r' && nmea_idx < NMEA_MAX_LINE - 1) {
            nmea_buf[nmea_idx++] = b;
        }
    }
}

/* ── Public API ──────────────────────────────────────────────────────── */

esp_err_t gnss_init(uart_port_t port, int tx_pin, int rx_pin, int nreset_pin,
                    bool want_rover)
{
    s_uart       = port;
    s_nreset_pin = nreset_pin;

    if (nreset_pin >= 0) {
        gpio_config_t rst_cfg = {
            .pin_bit_mask = 1ULL << nreset_pin,
            .mode         = GPIO_MODE_OUTPUT,
            .pull_up_en   = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        gpio_config(&rst_cfg);
        gpio_set_level((gpio_num_t)nreset_pin, 1);   /* deassert — active low */
    }

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
        int     rtcm_d3 = 0;   /* count of 0xD3 bytes seen (module in base mode) */
        int64_t deadline = esp_timer_get_time() + 2000000LL;
        while (esp_timer_get_time() < deadline) {
            uint8_t b;
            if (uart_read_bytes(port, &b, 1, pdMS_TO_TICKS(10)) != 1) continue;
            if (b == '\n') {
                line[li] = '\0'; li = 0;
                /* The LC29H outputs a startup diagnostic like "$1,20381121,…"
                 * before entering normal mode — its second character is a digit.
                 * Log it (contains firmware build info) but keep scanning. */
                if (line[0] == '$' && line[1] >= '0' && line[1] <= '9')
                    ESP_LOGI(TAG, "LC29H startup: %s", line);
                /* Require a real GNSS NMEA talker prefix ($G, $P, $N, …). */
                if (line[0] == '$' && strlen(line) > 5 &&
                    line[1] >= 'A' && line[1] <= 'Z') {
                    active_baud = try_bauds[bi];
                    ESP_LOGI(TAG, "LC29H (NMEA) detected at %d baud: %s", active_baud, line);
                    break;
                }
            } else if (b == 0xD3) {
                /* RTCM3 preamble — module booted from saved base config */
                if (++rtcm_d3 >= 3) {
                    active_baud = try_bauds[bi];
                    s_booted_in_base = true;
                    ESP_LOGI(TAG, "LC29H (RTCM/base mode) detected at %d baud", active_baud);
                    break;
                }
            } else if (b != '\r' && li < NMEA_MAX_LINE - 1) {
                line[li++] = b;
            }
        }
        if (!active_baud)
            ESP_LOGI(TAG, "  no signal at %d baud", try_bauds[bi]);
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

    if (s_booted_in_base && want_rover) {
        /* Module has saved base-station config but app wants rover mode.
         * Switch it back to rover, save, and reset so it boots outputting NMEA. */
        ESP_LOGW(TAG, "LC29H booted in base mode but rover mode requested — switching");
        pqtm_send("PQTMCFGRCVRMODE,W,1");
        vTaskDelay(pdMS_TO_TICKS(300));
        pqtm_send("PQTMSAVEPAR");
        vTaskDelay(pdMS_TO_TICKS(500));
        if (s_nreset_pin >= 0) {
            gpio_set_level((gpio_num_t)s_nreset_pin, 0);
            vTaskDelay(pdMS_TO_TICKS(100));
            gpio_set_level((gpio_num_t)s_nreset_pin, 1);
            vTaskDelay(pdMS_TO_TICKS(2000));
        } else {
            pqtm_send("PQTMSRR");
            vTaskDelay(pdMS_TO_TICKS(5000));
        }
        s_booted_in_base = false;
        /* fall through to rover config below */
    }

    if (s_booted_in_base) {
        /* Module loaded its saved base-station config and is already outputting
         * RTCM.  Do NOT switch it to rover mode — that wipes the survey-in
         * accumulation that started at power-on.  gnss_start_survey_in() will
         * skip reconfiguration for the same reason. */
        ESP_LOGI(TAG, "LC29H ready TX=%d RX=%d — preserving saved base mode", tx_pin, rx_pin);
    } else {
        /* First boot or rover board: set RTK rover mode so incoming RTCM bytes
         * on the RX pin are accepted instead of discarded. */
        pqtm_send("PQTMVERNO");           /* firmware version query */
        vTaskDelay(pdMS_TO_TICKS(100));
        pqtm_send("PAIR002");             /* PAIR-protocol version query */
        vTaskDelay(pdMS_TO_TICKS(100));
        pqtm_send("PQTMCFGRTK,R");       /* read current RTK config */
        vTaskDelay(pdMS_TO_TICKS(100));
        pqtm_send("PQTMCFGRCVRMODE,W,1");
        vTaskDelay(pdMS_TO_TICKS(200));
        pqtm_send("PQTMCFGRCVRMODE,R");   /* read back — expect OK,1 */
        vTaskDelay(pdMS_TO_TICKS(100));
        pqtm_send("PQTMCFGRTK,W,1,1");   /* RTK enabled, corrections enabled */
        vTaskDelay(pdMS_TO_TICKS(200));
        pqtm_send("PQTMSAVEPAR");         /* persist rover config to flash */
        vTaskDelay(pdMS_TO_TICKS(500));
        pqtm_send("PQTMCFGRTK,R");       /* read back — expect OK,1,1 */
        ESP_LOGI(TAG, "LC29H ready TX=%d RX=%d — RTK rover mode requested", tx_pin, rx_pin);
    }
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
     * Stream-aware RTCM3 type and station-ID logger.  State persists across
     * calls so chunks that split across frame boundaries are handled correctly.
     *
     * Reads the first 3 payload bytes of every frame:
     *   payload[0..1] → 12-bit message type  (logged once per new type)
     *   payload[1..2] → 12-bit reference station ID  (logged on first change)
     *
     * If 1005/1006 ARP and MSM observation frames have different station IDs
     * the rover's RTK engine will not pair them and cannot resolve ambiguities.
     *
     * State machine: SYNC→L1→L2→P0→P1→P2→SKIP→SYNC
     * Reserved bits [7:2] of L1 byte filter false sync positions.
     */
    {
        typedef enum { RL_SYNC, RL_L1, RL_L2, RL_P0, RL_P1, RL_P2, RL_SKIP } rl_st_t;
        static rl_st_t  rl_state  = RL_SYNC;
        static uint16_t rl_plen;
        static uint8_t  rl_p0;
        static uint8_t  rl_p1;      /* saved so RL_P2 can compute station ID */
        static uint16_t rl_mt;      /* saved message type for RL_P2 */
        static uint16_t rl_remain;
        static uint16_t seen[20];
        static uint8_t  n_seen;

        for (size_t i = 0; i < len; i++) {
            uint8_t b = buf[i];
            switch (rl_state) {
            case RL_SYNC:
                if (b == 0xD3) rl_state = RL_L1;
                break;
            case RL_L1:
                if (b & 0xFCu) { rl_state = RL_SYNC; break; }
                rl_plen  = (uint16_t)(b & 0x03u) << 8;
                rl_state = RL_L2;
                break;
            case RL_L2:
                rl_plen |= b;
                rl_state  = (rl_plen >= 2) ? RL_P0 : RL_SYNC;
                break;
            case RL_P0:
                rl_p0    = b;
                rl_state = RL_P1;
                break;
            case RL_P1:
                rl_p1 = b;
                rl_mt = ((uint16_t)rl_p0 << 4) | (b >> 4);
                {
                    bool dup = false;
                    for (uint8_t j = 0; j < n_seen; j++) if (seen[j] == rl_mt) { dup = true; break; }
                    if (!dup && n_seen < 20) {
                        seen[n_seen++] = rl_mt;
                        ESP_LOGI(TAG, "RTCM type %u (first seen, %u types total)", rl_mt, n_seen);
                    }
                }
                if (rl_plen >= 3) {
                    rl_state = RL_P2;
                } else {
                    rl_remain = (uint16_t)(rl_plen - 2) + 3;
                    rl_state  = rl_remain ? RL_SKIP : RL_SYNC;
                }
                break;
            case RL_P2: {
                /* Reference Station ID = bits 12-23 of payload
                 * = lower nibble of payload[1] (rl_p1) + all of payload[2] (b) */
                uint16_t sid = ((uint16_t)(rl_p1 & 0x0Fu) << 8) | b;
                bool is_arp = (rl_mt == 1005 || rl_mt == 1006);
                bool is_msm = (rl_mt >= 1071 && rl_mt <= 1127);
                if (is_arp || is_msm) {
                    static uint16_t s_msm_sid = 0xFFFFu;
                    if (is_arp && sid != s_rtcm_arp_sid) {
                        s_rtcm_arp_sid = sid;
                        ESP_LOGI(TAG, "RTCM ARP  station ID: %u (type %u)", sid, rl_mt);
                    } else if (is_msm && sid != s_msm_sid) {
                        s_msm_sid = sid;
                        ESP_LOGI(TAG, "RTCM MSM  station ID: %u (from type %u)", sid, rl_mt);
                    }
                }
                /* Remaining to skip: (plen-3) payload bytes + 3 CRC = plen */
                rl_remain = rl_plen;
                rl_state  = rl_remain ? RL_SKIP : RL_SYNC;
                break;
            }
            case RL_SKIP:
                if (--rl_remain == 0) rl_state = RL_SYNC;
                break;
            }
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

bool gnss_rtcm_arp_seen(void)
{
    return s_rtcm_arp_sid != 0xFFFFu;
}

/*
 * Configure and start survey-in on the LC29H base station.
 * acc_limit_m is in metres; PQTMCFGSVIN expects centimetres, so 2.0 m → 200 cm.
 * Safe to call whether the module booted in rover or base mode.
 */
esp_err_t gnss_start_survey_in(uint32_t min_dur_s, float acc_limit_m)
{
    /*
     * If the module booted from saved base config, stay in base mode.
     * PQTMCFGSVIN,W is only accepted in base mode; switching to rover first
     * causes it to be silently rejected, after which PQTMCFGRCVRMODE,W,2 also
     * fails (ERROR,1) because there is no accepted SVIN config to transition to.
     * PAIR RTCM-enable commands work in both modes, so no rover dance is needed.
     */
    if (s_booted_in_base) {
        ESP_LOGI(TAG, "Booted in base mode — applying SVIN config in-place");
    }

    /* ── RTCM message enables ───────────────────────────────────────────── */
    /* MSM4 (compressed) instead of MSM7 (full): ~40% smaller frames,
     * sufficient for centimetre-level RTK, fits inside 9600-baud LoRa link. */
    pqtm_send("PAIR432,0");   /* disable MSM7 — overflows LoRa at 9.6 kbps */
    vTaskDelay(pdMS_TO_TICKS(200));
    pqtm_send("PAIR430,1");   /* enable MSM4 (1074/1084/1094/1124) */
    vTaskDelay(pdMS_TO_TICKS(200));
    pqtm_send("PAIR434,1");   /* 1005 ARP — required by rover for RTK */
    vTaskDelay(pdMS_TO_TICKS(200));
#if CONFIG_GNSS_BASE_RTCM_EPHEMERIS
    pqtm_send("PAIR436,1");   /* include ephemeris — may be required for RTK Fix on some builds */
#else
    pqtm_send("PAIR436,0");   /* disable ephemeris — rover tracks its own */
#endif
    vTaskDelay(pdMS_TO_TICKS(200));

    /* ── Survey-in configuration ────────────────────────────────────────────── */
    /*
     * PQTMCFGSVIN accuracy is in centimetres — the same unit as the meanAcc
     * field in $PQTMSVINSTATUS (which parse_svinstatus multiplies by 0.01 to
     * convert to metres).  Sending "2.0" means 2 cm, which standalone GNSS
     * can never achieve, so survey-in runs forever without converging.
     * Convert the caller's metres value to centimetres before sending.
     * Example: acc_limit_m=2.0 → 200 cm → survey-in converges outdoors in
     * roughly 5-10 minutes with a good sky view.
     *
     * PQTMCFGSVIN,W is only accepted in base mode (returns no response and is
     * silently ignored when the module is in rover mode).
     */
    uint32_t acc_cm = (uint32_t)(acc_limit_m * 100.0f + 0.5f);
    char body[72];
    snprintf(body, sizeof(body), "PQTMCFGSVIN,W,1,%lu,%lu,0,0,0",
             (unsigned long)min_dur_s, (unsigned long)acc_cm);
    pqtm_send(body);
    vTaskDelay(pdMS_TO_TICKS(200));

    /* ── Switch to base mode (only needed on first boot from rover) ─────── */
    if (!s_booted_in_base) {
        pqtm_send("PQTMCFGRCVRMODE,W,2");
        vTaskDelay(pdMS_TO_TICKS(300));
    }

    /* ── Save to flash ──────────────────────────────────────────────────── */
    pqtm_send("PQTMSAVEPAR");
    vTaskDelay(pdMS_TO_TICKS(500));

    /* ── Restart module to apply saved configuration ────────────────────── */
    if (s_nreset_pin >= 0) {
        /* Hardware reset: pulse NRESET low for 100 ms, then wait for boot */
        ESP_LOGI(TAG, "LC29H hardware reset via GPIO %d", s_nreset_pin);
        gpio_set_level((gpio_num_t)s_nreset_pin, 0);
        vTaskDelay(pdMS_TO_TICKS(100));
        gpio_set_level((gpio_num_t)s_nreset_pin, 1);
        vTaskDelay(pdMS_TO_TICKS(2000));   /* module boot time */
        s_booted_in_base = true;           /* module now running saved base config */
    } else {
        /* Software reset fallback — returns ERROR,3 on some firmware builds.
         * If it fails, power-cycle the base once to apply the saved config. */
        ESP_LOGI(TAG, "Requesting LC29H software reset (PQTMSRR)…");
        pqtm_send("PQTMSRR");
        vTaskDelay(pdMS_TO_TICKS(5000));
    }

    ESP_LOGI(TAG, "LC29H base init done — survey-in min=%lus acc=%.1fm",
             (unsigned long)min_dur_s, acc_limit_m);
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
