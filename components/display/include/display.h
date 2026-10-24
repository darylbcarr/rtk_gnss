#pragma once

#include <stdint.h>
#include <stdarg.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DISPLAY_COLS  21   /* characters per row at 6 px/char */
#define DISPLAY_ROWS   7   /* visible text rows at 9 px/row (8px glyph + 1px gap) */

/* Attach SSD1306 128x64 to an already-created I2C master bus.
 * Typical I2C address: 0x3C (SA0 pin low) or 0x3D (SA0 pin high). */
esp_err_t display_init(i2c_master_bus_handle_t bus, uint8_t i2c_addr);

/* Clear the in-memory framebuffer (call display_flush to push to screen). */
void display_clear(void);

/* Write a null-terminated string at character position (col, row).
 * Clips at the right edge; does not wrap. */
void display_puts(uint8_t col, uint8_t row, const char *s);

/* Pixel graphics. Coordinates are in pixels (0,0 = top-left); out-of-range
 * pixels are clipped. Like display_puts they OR into the framebuffer, so call
 * display_clear() first each frame. */
void display_pixel(int x, int y);
void display_circle(int cx, int cy, int r);   /* outline */
void display_disc(int cx, int cy, int r);     /* filled */

/* printf-style helper; wraps display_puts. */
void display_printf(uint8_t col, uint8_t row, const char *fmt, ...);

/* Push the full framebuffer to the SSD1306 over I2C. */
void display_flush(void);

#ifdef __cplusplus
}
#endif
