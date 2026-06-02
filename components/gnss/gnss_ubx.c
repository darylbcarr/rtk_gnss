#include "gnss.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>

static const char *TAG = "gnss";

/* ── UBX protocol constants ─────────────────────────────────────────── */

#define UBX_SYNC1  0xB5u
#define UBX_SYNC2  0x62u

#define UBX_CLASS_NAV  0x01u
#define UBX_CLASS_CFG  0x06u
#define UBX_CLASS_MON  0x0Au

#define UBX_NAV_PVT    0x07u
#define UBX_CFG_VALSET 0x8Au
#define UBX_MON_VER    0x04u

/* CFG-VALSET layer mask — RAM only; config reverts on power cycle */
#define UBX_LAYER_RAM  0x01u

/*
 * Configuration key IDs — ZED-F9P Interface Description HPG 1.30.
 * Key bits [31:28] encode the value width: 0x1=bit/bool, 0x2=U1,
 * 0x3=U2, 0x4=U4.  The send helpers below match each key's width.
 */
#define CFG_UART1_BAUDRATE           0x40520001u  /* U4 */
#define CFG_UART1OUTPROT_UBX         0x10740001u  /* L  */
#define CFG_UART1OUTPROT_NMEA        0x10740002u  /* L  */
#define CFG_UART1INPROT_RTCM3X       0x10730004u  /* L  — enable RTCM3 input */
#define CFG_MSGOUT_UBX_NAV_PVT_UART1 0x20910007u  /* U1 */
#define CFG_RATE_MEAS                0x30210001u  /* U2 — measurement period (ms) */

#define GNSS_BAUD_DEFAULT  38400
#define GNSS_BAUD_TARGET   460800
#define GNSS_UART_BUF_SIZE 2048
#define UBX_MAX_PAYLOAD    256

static uart_port_t       s_uart;
static SemaphoreHandle_t s_tx_mutex;

/* ── UBX-NAV-PVT payload (92 bytes, ZED-F9P IDD Table 3-20) ─────────── */

typedef struct __attribute__((packed)) {
    uint32_t iTOW;
    uint16_t year;
    uint8_t  month, day, hour, min, sec;
    uint8_t  valid;
    uint32_t tAcc;
    int32_t  nano;
    uint8_t  fixType;
    uint8_t  flags;      /* bit0=gnssFixOK, bits[7:6]=carrSoln */
    uint8_t  flags2;
    uint8_t  numSV;
    int32_t  lon;        /* 1e-7 deg */
    int32_t  lat;        /* 1e-7 deg */
    int32_t  height;     /* mm above ellipsoid */
    int32_t  hMSL;       /* mm above MSL */
    uint32_t hAcc;       /* mm */
    uint32_t vAcc;       /* mm */
    int32_t  velN, velE, velD;
    int32_t  gSpeed;
    int32_t  headMot;
    uint32_t sAcc;
    uint32_t headAcc;
    uint16_t pDOP;       /* 0.01 */
    uint8_t  flags3;
    uint8_t  reserved0[5];
    int32_t  headVeh;
    int16_t  magDec;
    uint16_t magAcc;
} ubx_nav_pvt_payload_t;

/* ── UBX frame parser ────────────────────────────────────────────────── */

typedef enum {
    S_SYNC1, S_SYNC2, S_CLASS, S_ID,
    S_LEN1,  S_LEN2,  S_PAYLOAD, S_CKA, S_CKB
} ubx_state_t;

typedef struct {
    ubx_state_t state;
    uint8_t  cls, id;
    uint16_t len, idx;
    uint8_t  buf[UBX_MAX_PAYLOAD];
    uint8_t  cka, ckb;   /* running Fletcher-8 checksum */
} ubx_parser_t;

static ubx_parser_t s_parser;   /* zero-initialised at startup → S_SYNC1 */

/* Feed one byte into the parser; returns true when a complete, valid
   frame has been received.  Frame fields are in s_parser.{cls,id,len,buf}. */
static bool ubx_push(ubx_parser_t *p, uint8_t b)
{
    switch (p->state) {
    case S_SYNC1:
        if (b == UBX_SYNC1) p->state = S_SYNC2;
        break;
    case S_SYNC2:
        p->state = (b == UBX_SYNC2) ? S_CLASS : S_SYNC1;
        break;
    case S_CLASS:
        p->cls = b; p->cka = b; p->ckb = b;
        p->state = S_ID;
        break;
    case S_ID:
        p->id = b; p->cka += b; p->ckb += p->cka;
        p->state = S_LEN1;
        break;
    case S_LEN1:
        p->len = b; p->cka += b; p->ckb += p->cka;
        p->state = S_LEN2;
        break;
    case S_LEN2:
        p->len |= (uint16_t)b << 8; p->cka += b; p->ckb += p->cka;
        p->idx = 0;
        p->state = p->len ? S_PAYLOAD : S_CKA;
        break;
    case S_PAYLOAD:
        if (p->idx < UBX_MAX_PAYLOAD) p->buf[p->idx] = b;
        p->idx++;
        p->cka += b; p->ckb += p->cka;
        if (p->idx >= p->len) p->state = S_CKA;
        break;
    case S_CKA:
        if (b != p->cka) { p->state = S_SYNC1; break; }
        p->state = S_CKB;
        break;
    case S_CKB:
        p->state = S_SYNC1;
        return b == p->ckb && p->len <= UBX_MAX_PAYLOAD;
    }
    return false;
}

/* ── UBX frame transmit helpers ──────────────────────────────────────── */

/* Wait for the F9P to ACK or NAK a message identified by (ack_cls, ack_id).
 * The ACK-ACK/ACK-NAK payload is [cls, id] of the acknowledged message.
 * Returns true on ACK, false on NAK or timeout. */
static bool ubx_await_ack(uint8_t ack_cls, uint8_t ack_id, uint32_t timeout_ms)
{
    ubx_parser_t p = {0};
    int64_t deadline = esp_timer_get_time() + (int64_t)timeout_ms * 1000;
    while (esp_timer_get_time() < deadline) {
        uint8_t b;
        if (uart_read_bytes(s_uart, &b, 1, pdMS_TO_TICKS(5)) != 1) continue;
        if (!ubx_push(&p, b)) continue;
        if (p.cls != 0x05u || p.len != 2) continue;          /* not ACK class  */
        if (p.buf[0] != ack_cls || p.buf[1] != ack_id) continue; /* wrong target */
        if (p.id == 0x01u) return true;                       /* ACK-ACK        */
        ESP_LOGW(TAG, "NAK received for %02X/%02X", ack_cls, ack_id);
        return false;                                          /* ACK-NAK        */
    }
    ESP_LOGW(TAG, "ACK timeout for %02X/%02X", ack_cls, ack_id);
    return false;
}

static void ubx_send(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t plen)
{
    uint8_t hdr[6] = {
        UBX_SYNC1, UBX_SYNC2, cls, id,
        (uint8_t)(plen & 0xFF), (uint8_t)(plen >> 8)
    };
    uint8_t cka = 0, ckb = 0;
    for (int i = 2; i < 6; i++) { cka += hdr[i]; ckb += cka; }
    for (uint16_t i = 0; i < plen; i++) { cka += payload[i]; ckb += cka; }

    uint8_t tail[2] = { cka, ckb };
    xSemaphoreTake(s_tx_mutex, portMAX_DELAY);
    uart_write_bytes(s_uart, (char *)hdr, 6);
    if (plen) uart_write_bytes(s_uart, (char *)payload, plen);
    uart_write_bytes(s_uart, (char *)tail, 2);
    xSemaphoreGive(s_tx_mutex);
}

/* CFG-VALSET helpers — each sends a single RAM-layer key-value pair. */

static void valset_u4(uint32_t key, uint32_t val)
{
    uint8_t p[12] = {
        0x00, UBX_LAYER_RAM, 0x00, 0x00,
        key & 0xFF, key >> 8, key >> 16, key >> 24,
        val & 0xFF, val >> 8, val >> 16, val >> 24
    };
    ubx_send(UBX_CLASS_CFG, UBX_CFG_VALSET, p, sizeof(p));
}

static void valset_u2(uint32_t key, uint16_t val)
{
    uint8_t p[10] = {
        0x00, UBX_LAYER_RAM, 0x00, 0x00,
        key & 0xFF, key >> 8, key >> 16, key >> 24,
        val & 0xFF, val >> 8
    };
    ubx_send(UBX_CLASS_CFG, UBX_CFG_VALSET, p, sizeof(p));
}

static void valset_u1(uint32_t key, uint8_t val)
{
    uint8_t p[9] = {
        0x00, UBX_LAYER_RAM, 0x00, 0x00,
        key & 0xFF, key >> 8, key >> 16, key >> 24,
        val
    };
    ubx_send(UBX_CLASS_CFG, UBX_CFG_VALSET, p, sizeof(p));
}

/* ── Public API ──────────────────────────────────────────────────────── */

esp_err_t gnss_init(uart_port_t port, int tx_pin, int rx_pin)
{
    s_uart = port;
    memset(&s_parser, 0, sizeof(s_parser));
    if (!s_tx_mutex) s_tx_mutex = xSemaphoreCreateMutex();

    uart_config_t cfg = {
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    /* Try 460800 first (previously-configured device), then 38400 (factory). */
    const int bauds[2] = { GNSS_BAUD_TARGET, GNSS_BAUD_DEFAULT };
    int active_baud = 0;

    for (int i = 0; i < 2 && !active_baud; i++) {
        uart_driver_delete(port);          /* no-op if not yet installed */
        cfg.baud_rate = bauds[i];
        ESP_ERROR_CHECK(uart_driver_install(port, GNSS_UART_BUF_SIZE, 0, 0, NULL, 0));
        ESP_ERROR_CHECK(uart_param_config(port, &cfg));
        ESP_ERROR_CHECK(uart_set_pin(port, tx_pin, rx_pin,
                                      UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
        uart_flush(port);

        /* Poll UBX-MON-VER; any valid UBX reply confirms we have the right baud. */
        ubx_send(UBX_CLASS_MON, UBX_MON_VER, NULL, 0);

        ubx_parser_t probe = {0};
        int64_t deadline = esp_timer_get_time() + 500000LL;   /* 500 ms */
        while (esp_timer_get_time() < deadline) {
            uint8_t b;
            if (uart_read_bytes(port, &b, 1, pdMS_TO_TICKS(5)) == 1) {
                if (ubx_push(&probe, b)) {
                    active_baud = bauds[i];
                    ESP_LOGI(TAG, "ZED-F9P detected at %d baud", active_baud);
                    break;
                }
            }
        }
    }

    if (!active_baud) {
        ESP_LOGE(TAG, "No response from ZED-F9P — check wiring on TX=%d RX=%d",
                 tx_pin, rx_pin);
        uart_driver_delete(port);
        return ESP_ERR_TIMEOUT;
    }

    /* Verify TX is reaching the F9P.  Detection can succeed from the F9P's
     * autonomous output even when TX is disconnected.  Send a harmless VALSET
     * (same baud value) and confirm the ACK to prove bidirectional comms. */
    valset_u4(CFG_UART1_BAUDRATE, (uint32_t)active_baud);
    if (!ubx_await_ack(UBX_CLASS_CFG, UBX_CFG_VALSET, 300)) {
        ESP_LOGE(TAG, "TX pin %d not reaching F9P — wire F9P RXD1 to GPIO %d",
                 tx_pin, tx_pin);
        uart_driver_delete(port);
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "Bidirectional comms confirmed at %d baud", active_baud);

    if (active_baud != GNSS_BAUD_TARGET) {
        ESP_LOGI(TAG, "Switching baud to %d", GNSS_BAUD_TARGET);
        valset_u4(CFG_UART1_BAUDRATE, GNSS_BAUD_TARGET);
        /* F9P sends ACK at the old rate then switches; wait for it to settle. */
        vTaskDelay(pdMS_TO_TICKS(150));
        ESP_ERROR_CHECK(uart_set_baudrate(port, GNSS_BAUD_TARGET));
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    uart_flush(port);

    valset_u1(CFG_UART1OUTPROT_NMEA, 0);
    ubx_await_ack(UBX_CLASS_CFG, UBX_CFG_VALSET, 200);

    valset_u1(CFG_UART1OUTPROT_UBX, 1);
    ubx_await_ack(UBX_CLASS_CFG, UBX_CFG_VALSET, 200);

    valset_u1(CFG_UART1INPROT_RTCM3X, 1);
    ubx_await_ack(UBX_CLASS_CFG, UBX_CFG_VALSET, 200);

    valset_u2(CFG_RATE_MEAS, 1000);
    ubx_await_ack(UBX_CLASS_CFG, UBX_CFG_VALSET, 200);

    valset_u1(CFG_MSGOUT_UBX_NAV_PVT_UART1, 1);
    ubx_await_ack(UBX_CLASS_CFG, UBX_CFG_VALSET, 200);

    ESP_LOGI(TAG, "GNSS ready — UBX NAV-PVT @ 1 Hz");
    return ESP_OK;
}

bool gnss_read_pvt(gnss_pvt_t *out, uint32_t timeout_ms)
{
    /* Poll for an immediate NAV-PVT.  The F9P always responds to a poll
     * regardless of the configured continuous-output rate, so we get data
     * even if the MSGOUT VALSET hasn't taken effect. */
    ubx_send(UBX_CLASS_NAV, UBX_NAV_PVT, NULL, 0);

    int64_t deadline = esp_timer_get_time() + (int64_t)timeout_ms * 1000;

    while (esp_timer_get_time() < deadline) {
        uint8_t buf[64];
        int n = uart_read_bytes(s_uart, buf, sizeof(buf), pdMS_TO_TICKS(10));
        for (int i = 0; i < n; i++) {
            if (!ubx_push(&s_parser, buf[i])) continue;
            if (s_parser.cls != UBX_CLASS_NAV || s_parser.id != UBX_NAV_PVT) continue;
            if (s_parser.len != sizeof(ubx_nav_pvt_payload_t)) continue;

            ubx_nav_pvt_payload_t pvt;
            memcpy(&pvt, s_parser.buf, sizeof(pvt));

            out->lat        = pvt.lat  * 1e-7;
            out->lon        = pvt.lon  * 1e-7;
            out->alt_msl    = pvt.hMSL * 1e-3;
            out->h_acc      = pvt.hAcc * 1e-3f;
            out->v_acc      = pvt.vAcc * 1e-3f;
            out->fix_type   = pvt.fixType;
            out->carr_soln  = (pvt.flags >> 6) & 0x03u;
            out->num_sv     = pvt.numSV;
            out->i_tow      = pvt.iTOW;
            out->pdop       = pvt.pDOP * 0.01f;
            out->gnss_fix_ok = pvt.flags & 0x01u;
            return true;
        }
    }
    return false;
}

bool gnss_is_rtk_fix(const gnss_pvt_t *pvt)
{
    return pvt->gnss_fix_ok && pvt->carr_soln == 2;
}

void gnss_write_rtcm(const uint8_t *buf, size_t len)
{
    xSemaphoreTake(s_tx_mutex, portMAX_DELAY);
    uart_write_bytes(s_uart, (const char *)buf, len);
    xSemaphoreGive(s_tx_mutex);
}

/* ZED-F9P base-station survey-in uses UBX-CFG-TMODE3 — not yet implemented. */
esp_err_t gnss_start_survey_in(uint32_t min_dur_s, float acc_limit_m)
{
    (void)min_dur_s; (void)acc_limit_m;
    ESP_LOGW("gnss", "gnss_start_survey_in not implemented for ZED-F9P driver");
    return ESP_ERR_NOT_SUPPORTED;
}

bool gnss_get_svin_status(gnss_svin_t *out)
{
    (void)out;
    return false;
}

int gnss_read_rtcm(uint8_t *buf, size_t max_len, uint32_t timeout_ms)
{
    (void)buf; (void)max_len; (void)timeout_ms;
    return 0;
}

void gnss_poll_svin_status(void) { /* UBX driver polls via NAV-SVIN poll, not yet implemented */ }
