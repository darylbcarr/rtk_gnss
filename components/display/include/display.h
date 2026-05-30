#pragma once

#include <stdint.h>
#include <stdarg.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DISPLAY_COLS  21   /* characters per row (6px wide font) */
#define DISPLAY_ROWS   8   /* rows (8px tall font)                */

/* Attach SSD1306 128x64 to an already-created I2C master bus.
 * Typical I2C address: 0x3C (SA0 pin low) or 0x3D (SA0 pin high). */
esp_err_t display_init(i2c_master_bus_handle_t bus, uint8_t i2c_addr);

/* Clear the in-memory framebuffer (call display_flush to push to screen). */
void display_clear(void);

/* Write a null-terminated string at character position (col, row).
 * Clips at the right edge; does not wrap. */
void display_puts(uint8_t col, uint8_t row, const char *s);

/* printf-style helper; wraps display_puts. */
void display_printf(uint8_t col, uint8_t row, const char *fmt, ...);

/* Push the full framebuffer to the SSD1306 over I2C. */
void display_flush(void);

#ifdef __cplusplus
}
#endif
