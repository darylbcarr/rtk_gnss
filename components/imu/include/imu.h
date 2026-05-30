#pragma once

#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float   heading;    /* 0–360 degrees, magnetic north reference */
    float   roll;       /* degrees */
    float   pitch;      /* degrees */
    uint8_t sys_cal;    /* 0–3; system calibration level (3 = fully calibrated) */
    bool    calibrated; /* true when sys_cal >= 1 */
} imu_data_t;

/* Attach BNO055 to an already-created I2C master bus.
 * I2C address 0x28 (ADR pin low) or 0x29 (ADR pin high). */
esp_err_t imu_init(i2c_master_bus_handle_t bus, uint8_t i2c_addr);

/* Read Euler angles and calibration status. Thread-safe: call from any task. */
esp_err_t imu_read(imu_data_t *out);

#ifdef __cplusplus
}
#endif
