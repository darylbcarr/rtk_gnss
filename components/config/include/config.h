#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    DEVICE_MODE_UNSET = 0,
    DEVICE_MODE_ROVER = 1,
    DEVICE_MODE_BASE  = 2,
} device_mode_t;

/* Call once at startup — initialises NVS flash (erases on version mismatch). */
esp_err_t     config_init(void);

device_mode_t config_get_device_mode(void);
esp_err_t     config_set_device_mode(device_mode_t mode);

/* Level calibration: the accelerometer/mount offset of the IMU, in lean units
 * (sin of the tilt it adds along IMU X / Y). Returns ESP_ERR_NOT_FOUND when no
 * calibration has been saved. */
esp_err_t config_get_level_offset(float *x, float *y);
esp_err_t config_set_level_offset(float x, float y);

/* Pole length, antenna ARP to ground tip, in metres. Persisted so the last
 * session's value is the next default; 2.0 m if never set. */
float     config_get_pole_len_m(void);
esp_err_t config_set_pole_len_m(float metres);

#ifdef __cplusplus
}
#endif
