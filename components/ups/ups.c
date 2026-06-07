#include "ups.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "ups";

/* INA219 register addresses */
#define REG_CONFIG  0x00
#define REG_SHUNT   0x01
#define REG_VOLTAGE 0x02
#define REG_POWER   0x03
#define REG_CURRENT 0x04
#define REG_CAL     0x05

/*
 * Config 0x399F: BRNG=32V, PGA=÷8 (±320mV shunt), BADC=12-bit 2-sample avg,
 * SADC=12-bit 2-sample avg, MODE=continuous shunt+bus.
 *
 * Calibration: shunt=0.1Ω, current LSB=100μA
 *   Cal = trunc(0.04096 / (100e-6 × 0.1)) = 4096
 */
#define INA219_CONFIG   0x399F
#define INA219_CAL      4096
#define CURRENT_LSB_MA  0.1f    /* mA per current-register LSB */
#define POWER_LSB_W     0.002f  /* W per power-register LSB    */

/* 3S Li-Ion voltage range: 3×3.0V = 9.0V (empty) to 3×4.2V = 12.6V (full) */
#define BATT_V_MIN  9.0f
#define BATT_V_MAX  12.6f

static i2c_master_dev_handle_t s_dev;
static bool                    s_ready = false;

static esp_err_t reg_write16(uint8_t reg, uint16_t val)
{
    uint8_t buf[3] = { reg, (uint8_t)(val >> 8), (uint8_t)(val & 0xFF) };
    return i2c_master_transmit(s_dev, buf, 3, 50);
}

static esp_err_t reg_read16(uint8_t reg, uint16_t *out)
{
    uint8_t raw[2];
    esp_err_t ret = i2c_master_transmit_receive(s_dev, &reg, 1, raw, 2, 50);
    if (ret == ESP_OK)
        *out = ((uint16_t)raw[0] << 8) | raw[1];
    return ret;
}

esp_err_t ups_init(i2c_master_bus_handle_t bus, uint8_t i2c_addr)
{
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = i2c_addr,
        .scl_speed_hz    = 100000,
    };
    esp_err_t ret = i2c_master_bus_add_device(bus, &dev_cfg, &s_dev);
    if (ret != ESP_OK) return ret;

    if (reg_write16(REG_CONFIG, INA219_CONFIG) != ESP_OK ||
        reg_write16(REG_CAL,    INA219_CAL)    != ESP_OK) {
        ESP_LOGW(TAG, "INA219 not found at 0x%02X", i2c_addr);
        return ESP_ERR_NOT_FOUND;
    }

    vTaskDelay(pdMS_TO_TICKS(5));  /* allow first conversion to complete */
    s_ready = true;
    ESP_LOGI(TAG, "INA219 ready at 0x%02X (3S UPS monitor)", i2c_addr);
    return ESP_OK;
}

bool ups_read(ups_data_t *out)
{
    if (!s_ready) {
        *out = (ups_data_t){ .present = false };
        return false;
    }

    uint16_t v_raw, c_raw, p_raw;
    if (reg_read16(REG_VOLTAGE, &v_raw) != ESP_OK ||
        reg_read16(REG_CURRENT, &c_raw) != ESP_OK ||
        reg_read16(REG_POWER,   &p_raw) != ESP_OK) {
        out->present = false;
        return false;
    }

    /* Bus voltage: bits 15:3, 4 mV/LSB */
    float bus_v = (float)(v_raw >> 3) * 0.004f;

    /* Current register is signed 16-bit, scaled by CURRENT_LSB_MA */
    float curr_mA = (float)(int16_t)c_raw * CURRENT_LSB_MA;

    /* Power register is unsigned, scaled by POWER_LSB_W */
    float power_W = (float)p_raw * POWER_LSB_W;

    float pct = (bus_v - BATT_V_MIN) / (BATT_V_MAX - BATT_V_MIN) * 100.0f;
    if (pct < 0.0f)   pct = 0.0f;
    if (pct > 100.0f) pct = 100.0f;

    out->bus_voltage_V = bus_v;
    out->current_mA    = curr_mA;
    out->power_W       = power_W;
    out->percent       = (int)pct;
    out->charging      = curr_mA > 50.0f;
    out->present       = true;
    return true;
}
