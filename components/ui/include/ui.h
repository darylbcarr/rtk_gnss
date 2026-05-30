#pragma once

#include <stdbool.h>
#include "gnss.h"
#include "imu.h"

#ifdef __cplusplus
extern "C" {
#endif

void ui_init(bool imu_available);

/* Call once per GNSS epoch. Reads encoder, updates state, redraws display. */
void ui_tick(const gnss_pvt_t *pvt, const imu_data_t *imu);

#ifdef __cplusplus
}
#endif
