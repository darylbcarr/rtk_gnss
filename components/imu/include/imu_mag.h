#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Melexis MLX90393 3-axis magnetometer, I2C mode, on the shared sensor bus.
 *
 * Runs in single-measurement mode on demand: with the interval set to 0 the
 * chip sits in IDLE (~2.4 µA typ). Each measurement costs about 2.3–3 mA for
 * the conversion time, so the rate is set by whoever needs the data.
 *
 * Readings are in the sensor's own axes with no hard-/soft-iron correction
 * and no alignment to the LSM6DSV frame yet. Do not use them as a compass
 * heading until those calibrations exist (see docs/IMU.md).
 */

typedef struct {
    i2c_master_bus_handle_t bus;       /* shared with OLED/UPS/IMU */
    uint16_t                i2c_addr;  /* 0x0C | (A1 << 1) | A0 -> 0x0C..0x0F */
} imu_mag_cfg_t;

typedef struct {
    float x_uT, y_uT, z_uT;   /* sensor frame, µT */
    float magnitude_uT;       /* |B|; Earth's field is roughly 25–65 µT */
    float temp_c;             /* die temperature, ±~few °C, not for calibration */
} imu_mag_t;

/* Probe, reset and configure the chip. ESP_ERR_NOT_FOUND if it does not
 * answer at cfg->i2c_addr. */
esp_err_t imu_mag_init(const imu_mag_cfg_t *cfg);

/* Time between measurement starts; 0 stops measuring (chip idles). Takes
 * effect on the next imu_mag_poll(). */
void imu_mag_set_interval_ms(uint32_t ms);

/* Non-blocking. Starts a measurement when one is due and collects it when the
 * conversion has finished. Returns ESP_OK and fills *out only when a new
 * sample arrived, ESP_ERR_NOT_FOUND when there is nothing new yet, and another
 * error if the bus transaction failed. Call from a single task. */
esp_err_t imu_mag_poll(imu_mag_t *out);

/* Pure conversion helpers, exposed for host tests. */
typedef struct {
    float xy_uT_per_lsb;
    float z_uT_per_lsb;
} imu_mag_sens_t;

/* raw = status byte followed by T, X, Y, Z as big-endian 16-bit words. */
void imu_mag_decode(const uint8_t raw[9], const imu_mag_sens_t *s, imu_mag_t *out);

#ifdef __cplusplus
}
#endif
