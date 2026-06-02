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

/* Survey-in progress snapshot. */
typedef struct {
    bool     active;      /* survey-in is running */
    bool     valid;       /* position has converged */
    uint32_t dur_s;       /* elapsed observation seconds */
    float    mean_acc_m;  /* mean 3D position accuracy (m) */
} gnss_svin_t;

/*
 * Initialize GNSS on the given UART.  Starts a background reader task
 * that demultiplexes the NMEA/RTCM3 mixed stream.
 */
esp_err_t gnss_init(uart_port_t port, int tx_pin, int rx_pin);

/*
 * Block until a fresh position fix is available or timeout_ms elapses.
 * Returns true and populates *out on success.
 */
bool gnss_read_pvt(gnss_pvt_t *out, uint32_t timeout_ms);

/* True when carr_soln == 2 (RTK fix) and gnss_fix_ok. */
bool gnss_is_rtk_fix(const gnss_pvt_t *pvt);

/* Forward RTCM3 correction bytes to the module receive port.
 * Thread-safe; may be called from any task. */
void gnss_write_rtcm(const uint8_t *buf, size_t len);

/*
 * Start base-station survey-in.
 * min_dur_s    – minimum observation time before declaring convergence.
 * acc_limit_m  – 3D position accuracy threshold (m) for convergence.
 */
esp_err_t gnss_start_survey_in(uint32_t min_dur_s, float acc_limit_m);

/*
 * Return the most recently received survey-in status.
 * Returns true if at least one status update has been received.
 * Thread-safe.
 */
bool gnss_get_svin_status(gnss_svin_t *out);

/*
 * Request a fresh survey-in status from the module ($PQTMSVINSTATUS,R).
 * Use this when the module does not emit status autonomously.
 */
void gnss_poll_svin_status(void);

/*
 * Read one complete RTCM3 frame from the module output stream.
 * Blocks until a frame arrives or timeout_ms elapses.
 * Returns frame byte count on success, 0 on timeout, -1 on error.
 * buf must be at least 1029 bytes (max RTCM3 frame size).
 */
int gnss_read_rtcm(uint8_t *buf, size_t max_len, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif
