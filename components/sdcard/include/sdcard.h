#pragma once

#include <stdbool.h>
#include "esp_err.h"
#include "driver/spi_common.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int pin_mosi;
    int pin_miso;
    int pin_clk;
    int pin_cs;
    spi_host_device_t spi_host;   /* SPI2_HOST or SPI3_HOST */
} sdlog_cfg_t;

/*
 * Initialise the SPI SD card, mount a FAT filesystem at /sdcard, and open
 * an auto-numbered log file (<MODE>_NNN.CSV).  Writes a CSV header matched
 * to the mode string ("BASE" or "ROVER").  Starts a background task that
 * drains the write queue so callers never block on SD I/O.
 *
 * Returns ESP_OK on success.  On failure sdlog_writef() becomes a no-op.
 */
esp_err_t sdlog_init(const sdlog_cfg_t *cfg, const char *mode_name);

/*
 * Enqueue a formatted log line.  Non-blocking — if the queue is full the
 * line is silently dropped rather than stalling the caller.
 */
void sdlog_writef(const char *fmt, ...);

/* Request a flush of buffered data to the card. */
void sdlog_flush(void);

bool sdlog_is_ready(void);

#ifdef __cplusplus
}
#endif
