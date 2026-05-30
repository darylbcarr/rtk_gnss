#include "imu.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "imu";

/* BNO055 register map (datasheet section 4.3) */
#define REG_CHIP_ID    0x00   /* expected: 0xA0 */
#define REG_OPR_MODE   0x3D
#define REG_CALIB_STAT 0x35
#define REG_EUL_H_LSB  0x1A  /* 6 bytes: H_LSB,H_MSB,R_LSB,R_MSB,P_LSB,P_MSB */

#define MODE_CONFIGMODE 0x00
#define MODE_NDOF       0x0C  /* 9-DOF absolute orientation fusion */

static i2c_master_dev_handle_t s_dev;

static esp_err_t reg_write(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(s_dev, buf, 2, 50);
}

static esp_err_t reg_read(uint8_t reg, uint8_t *out, size_t len)
{
    esp_err_t ret = i2c_master_transmit_receive(s_dev, &reg, 1, out, len, 50);
    return ret;
}

esp_err_t imu_init(i2c_master_bus_handle_t bus, uint8_t i2c_addr)
{
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = i2c_addr,
        .scl_speed_hz    = 100000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &dev_cfg, &s_dev));

    uint8_t chip_id = 0;
    esp_err_t ret = reg_read(REG_CHIP_ID, &chip_id, 1);
    if (ret != ESP_OK || chip_id != 0xA0) {
        ESP_LOGE(TAG, "BNO055 not found at 0x%02X (chip_id=0x%02X, err=0x%x)",
                 i2c_addr, chip_id, ret);
        return ESP_ERR_NOT_FOUND;
    }

    reg_write(REG_OPR_MODE, MODE_CONFIGMODE);
    vTaskDelay(pdMS_TO_TICKS(25));
    reg_write(REG_OPR_MODE, MODE_NDOF);
    vTaskDelay(pdMS_TO_TICKS(20));

    ESP_LOGI(TAG, "BNO055 ready at 0x%02X (NDOF mode)", i2c_addr);
    return ESP_OK;
}

esp_err_t imu_read(imu_data_t *out)
{
    uint8_t buf[6];
    esp_err_t ret = reg_read(REG_EUL_H_LSB, buf, 6);
    if (ret != ESP_OK) return ret;

    /* Euler angles: 1 LSB = 1/16 degree (BNO055 datasheet Table 3-28) */
    int16_t raw_h = (int16_t)(buf[0] | ((uint16_t)buf[1] << 8));
    int16_t raw_r = (int16_t)(buf[2] | ((uint16_t)buf[3] << 8));
    int16_t raw_p = (int16_t)(buf[4] | ((uint16_t)buf[5] << 8));

    out->heading = raw_h / 16.0f;
    out->roll    = raw_r / 16.0f;
    out->pitch   = raw_p / 16.0f;

    uint8_t cal = 0;
    reg_read(REG_CALIB_STAT, &cal, 1);
    out->sys_cal    = (cal >> 6) & 0x03;
    out->calibrated = (out->sys_cal >= 1);
    return ESP_OK;
}
