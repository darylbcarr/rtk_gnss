#pragma once

#include <stdbool.h>
#include "gnss.h"
#include "imu.h"
#include "config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Call after encoder_init(). Stores the current mode for display. */
void ui_init(bool imu_available, device_mode_t mode);

/* Call once per GNSS epoch. Reads encoder, updates state, redraws display. */
void ui_tick(const gnss_pvt_t *pvt, const imu_data_t *imu);

/* Blocking first-boot mode select. Call before ui_init() when NVS has no
 * saved mode. Renders a selection screen, waits for encoder press, saves the
 * chosen mode to NVS, and returns it. */
device_mode_t ui_select_mode_blocking(void);

#ifdef __cplusplus
}
#endif
