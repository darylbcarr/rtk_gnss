#pragma once

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Install GPIO ISRs for a KY-040 rotary encoder.
 * CLK triggers on falling edge to detect rotation direction via DT level. */
esp_err_t encoder_init(int clk_pin, int dt_pin, int sw_pin);

/* Return accumulated step count since last call (positive = CW, negative = CCW).
 * Resets the counter to zero on each call. */
int  encoder_get_steps(void);

/* Return true once per button press (50 ms debounce). Clears the flag. */
bool encoder_get_press(void);

#ifdef __cplusplus
}
#endif
