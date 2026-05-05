#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "driver/uart.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    double   lat;        /* WGS84 decimal degrees */
    double   lon;
    double   alt_msl;    /* meters above MSL */
    float    h_acc;      /* horizontal 1-sigma (m) */
    float    v_acc;      /* vertical 1-sigma (m) */
    uint8_t  fix_type;   /* 0=none 1=DR 2=2D 3=3D 4=GNSS+DR 5=time */
    uint8_t  carr_soln;  /* 0=none 1=float 2=fix */
    uint8_t  num_sv;
    uint32_t i_tow;      /* GPS time of week (ms) */
    float    pdop;
    bool     gnss_fix_ok;
} gnss_pvt_t;

/*
 * Initialize GNSS on the given UART.  Auto-detects baud rate (tries
 * 460800 first, falls back to the factory default 38400 and switches).
 * Disables NMEA output and enables UBX NAV-PVT at 1 Hz.
 */
esp_err_t gnss_init(uart_port_t port, int tx_pin, int rx_pin);

/*
 * Block until a UBX-NAV-PVT frame is received or timeout_ms elapses.
 * Returns true and populates *out on success.
 */
bool gnss_read_pvt(gnss_pvt_t *out, uint32_t timeout_ms);

/* True when carr_soln == 2 (RTK fix) and gnss_fix_ok. */
bool gnss_is_rtk_fix(const gnss_pvt_t *pvt);

/* Forward RTCM3 correction bytes to the F9P receive port.
 * Thread-safe; may be called from any task. */
void gnss_write_rtcm(const uint8_t *buf, size_t len);

#ifdef __cplusplus
}
#endif
