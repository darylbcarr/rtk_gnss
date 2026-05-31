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

#ifdef __cplusplus
}
#endif
