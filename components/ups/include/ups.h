#pragma once

#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float bus_voltage_V;  /* 3S battery voltage, ~9.0–12.6 V */
    float current_mA;     /* positive = charging, negative = discharging */
    float power_W;
    int   percent;        /* 0–100, derived from 3S voltage range */
    bool  charging;
    bool  present;        /* false if INA219 did not respond */
} ups_data_t;

/* Attach INA219 to an already-created I2C master bus and program calibration.
 * Typical I2C address for Waveshare UPS Module 3S: 0x41. */
esp_err_t ups_init(i2c_master_bus_handle_t bus, uint8_t i2c_addr);

/* Read current voltage, current, power, and derived percent.
 * Returns true on success; sets out->present = false on comms failure. */
bool ups_read(ups_data_t *out);

#ifdef __cplusplus
}
#endif
