#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    i2c_master_bus_handle_t bus;       /* shared with OLED/UPS — CS tied
                                         * permanently high on the LSM6DSVQTR
                                         * selects I2C mode over SPI. */
    uint16_t                i2c_addr;  /* 0x6A (SA0 low) or 0x6B (SA0 high) */
} imu_i2c_cfg_t;

typedef struct {
    float   heading;    /* yaw, degrees. Relative "game rotation vector" —
                          * the LSM6DSVQTR has no magnetometer, so this is
                          * NOT referenced to magnetic north and drifts slowly
                          * over time. Do not use as a compass heading. */
    float   roll;       /* degrees — gravity-referenced, stable */
    float   pitch;      /* degrees — gravity-referenced, stable */
    uint8_t sys_cal;    /* 0 = no fused sample yet, 1 = quaternion valid */
    bool    calibrated; /* true once the on-chip SFLP filter has produced a sample */
} imu_data_t;

typedef struct {
    float tilt_deg;  /* angle between the pole axis (IMU +Z) and vertical, 0 = plumb.
                      * Level-calibration offset already removed. */
    float lean_x;    /* up-vector component along IMU +X, unitless (= sin of the lean
                      * in that axis), offset removed. Positive means the "bubble"
                      * sits toward +X. */
    float lean_y;    /* same, along IMU +Y */
    float raw_x;     /* lean_x / lean_y before the calibration offset is removed; */
    float raw_y;     /* the level calibration routine samples these. */
    float accel_g;   /* magnitude of the latest raw sample, g. ~1.0 when still */
    bool  steady;    /* latest sample was within the stillness band around 1 g; when
                      * false the tilt values are held from the last steady sample */
} imu_tilt_t;

/* Attach the LSM6DSVQTR to an already-created I2C master bus and enable its
 * on-chip Sensor Fusion Low Power (SFLP) engine to compute a 6-axis game
 * rotation vector (accel + gyro only, no magnetometer). */
esp_err_t imu_init(const imu_i2c_cfg_t *cfg);

/* Drain the SFLP FIFO and report the most recent orientation. If no new
 * sample has arrived since the last call, *out is left untouched and
 * ESP_ERR_NOT_FOUND is returned. Thread-safe: call from any task. */
esp_err_t imu_read(imu_data_t *out);

/* Gravity-referenced tilt of the pole axis, from the raw accelerometer.
 * Needs no heading. Samples are low-pass filtered (~0.25 s) and only folded in
 * while the sample magnitude is near 1 g, so shaking the pole does not corrupt
 * the reading. Assumes IMU +Z lies along the pole axis (mount calibration is a
 * later step). Returns ESP_ERR_NOT_FOUND until the first steady sample.
 * Call from a single task (filter state is not locked). */
esp_err_t imu_read_tilt(imu_tilt_t *out);

/* Level-calibration offset, in lean units (sin of the tilt it adds along IMU
 * X / Y). imu_read_tilt() subtracts it. Defaults to zero; main loads the saved
 * value from NVS at boot. */
void imu_set_level_offset(float x, float y);
void imu_get_level_offset(float *x, float *y);

#ifdef __cplusplus
}
#endif
