#pragma once

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    const char *host;
    uint16_t    port;
    const char *mountpoint;
    const char *user;
    const char *password;
} ntrip_cfg_t;

/* Position snapshot used to format NMEA GGA sentences sent to the caster.
 * Call ntrip_update_position() from the GNSS read loop. */
typedef struct {
    double   lat, lon, alt_msl;
    uint32_t i_tow;     /* GPS time of week (ms) — used for GGA timestamp */
    float    pdop;
    uint8_t  fix_type;  /* 0=none … 3=3D … */
    uint8_t  carr_soln; /* 0=none 1=float 2=fix */
    uint8_t  num_sv;
} ntrip_pos_t;

/* Called from the NTRIP task each time a chunk of RTCM3 data arrives. */
typedef void (*ntrip_rtcm_cb_t)(const uint8_t *buf, size_t len, void *ctx);

/*
 * Start the NTRIP client in a background FreeRTOS task.
 * Reconnects automatically on disconnect or timeout.
 */
esp_err_t ntrip_start(const ntrip_cfg_t *cfg, ntrip_rtcm_cb_t cb, void *ctx);
void      ntrip_stop(void);

/* Update the rover position used for GGA sentences.
 * Thread-safe; call after every successful gnss_read_pvt(). */
void ntrip_update_position(const ntrip_pos_t *pos);

#ifdef __cplusplus
}
#endif
