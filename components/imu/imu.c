#include "imu.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <string.h>

static const char *TAG = "imu";

/* LSM6DSVQTR register map — main bank (datasheet + ST lsm6dsv_reg.h) */
#define REG_FUNC_CFG_ACCESS   0x01
#define REG_FIFO_CTRL4        0x0A
#define REG_WHO_AM_I          0x0F
#define REG_CTRL1              0x10   /* accel ODR / mode */
#define REG_CTRL2              0x11   /* gyro ODR / mode */
#define REG_CTRL3              0x12   /* BDU / IF_INC / SW_RESET */
#define REG_CTRL6              0x15   /* gyro full scale */
#define REG_CTRL8              0x17   /* accel full scale */
#define REG_FIFO_STATUS1       0x1B
#define REG_FIFO_STATUS2       0x1C
#define REG_FIFO_DATA_OUT_TAG  0x78   /* burst: tag + 6 data bytes */

#define WHO_AM_I_VAL           0x70

/* Embedded-function bank, reached by setting FUNC_CFG_ACCESS.emb_func_reg_access */
#define EMB_FUNC_EN_A           0x04   /* sflp_game_en   = bit1 */
#define EMB_FUNC_FIFO_EN_A      0x44   /* sflp_game_fifo_en = bit1 */
#define EMB_SFLP_ODR            0x5E   /* sflp_game_odr[2:0] */

#define FIFO_TAG_SFLP_GAME_ROTATION_VECTOR 0x13

static i2c_master_dev_handle_t s_dev;

/* ── Low-level register access ───────────────────────────────────────── */

static esp_err_t reg_write(uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = { reg, val };
    return i2c_master_transmit(s_dev, tx, sizeof(tx), 100);
}

static esp_err_t reg_read(uint8_t reg, uint8_t *out, size_t len)
{
    return i2c_master_transmit_receive(s_dev, &reg, 1, out, len, 100);
}

/* Toggle into/out of the embedded-function register page. */
static esp_err_t page_select(bool embedded)
{
    return reg_write(REG_FUNC_CFG_ACCESS, embedded ? 0x01 : 0x00);
}

/* ── IEEE-754 half → single precision, used to decode the SFLP quaternion.
 * Bit-manipulation algorithm from numpy's ieee754.c (BSD-3-Clause), the same
 * routine ST's own lsm6dsv driver uses for this conversion. */
static float half_to_float(uint16_t h)
{
    uint32_t h_exp = h & 0x7c00u;
    uint32_t f_sgn = ((uint32_t)h & 0x8000u) << 16u;
    uint32_t bits;

    if (h_exp == 0x0000u) {                        /* zero or subnormal */
        uint32_t h_sig = h & 0x03ffu;
        if (h_sig == 0u) {
            bits = f_sgn;
        } else {
            uint32_t exp = 0;
            h_sig <<= 1u;
            while ((h_sig & 0x0400u) == 0u) { h_sig <<= 1u; exp++; }
            uint32_t f_exp = (127u - 15u - exp) << 23u;
            uint32_t f_sig = (h_sig & 0x03ffu) << 13u;
            bits = f_sgn + f_exp + f_sig;
        }
    } else if (h_exp == 0x7c00u) {                  /* inf or NaN */
        bits = f_sgn + 0x7f800000u + (((uint32_t)h & 0x03ffu) << 13u);
    } else {                                        /* normalized */
        bits = f_sgn + ((((uint32_t)h & 0x7fffu) + 0x1c000u) << 13u);
    }

    float out;
    memcpy(&out, &bits, sizeof(out));
    return out;
}

/* SFLP game rotation vector FIFO frames carry only (x, y, z); w is recovered
 * from the unit-quaternion constraint (matches ST's sflp2q reference code). */
static void sflp_to_quat(const uint8_t data[6], float q[4])
{
    uint16_t hx, hy, hz;
    memcpy(&hx, &data[0], 2);
    memcpy(&hy, &data[2], 2);
    memcpy(&hz, &data[4], 2);

    q[0] = half_to_float(hx);
    q[1] = half_to_float(hy);
    q[2] = half_to_float(hz);

    float sumsq = q[0] * q[0] + q[1] * q[1] + q[2] * q[2];
    if (sumsq > 1.0f) {
        float n = sqrtf(sumsq);
        q[0] /= n; q[1] /= n; q[2] /= n;
        sumsq = 1.0f;
    }
    q[3] = sqrtf(1.0f - sumsq);
}

static void quat_to_euler_deg(const float q[4], float *roll, float *pitch, float *yaw)
{
    float x = q[0], y = q[1], z = q[2], w = q[3];

    float sinr_cosp = 2.0f * (w * x + y * z);
    float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
    *roll = atan2f(sinr_cosp, cosr_cosp) * (180.0f / (float)M_PI);

    float sinp = 2.0f * (w * y - z * x);
    if (sinp > 1.0f) sinp = 1.0f;
    if (sinp < -1.0f) sinp = -1.0f;
    *pitch = asinf(sinp) * (180.0f / (float)M_PI);

    float siny_cosp = 2.0f * (w * z + x * y);
    float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
    *yaw = atan2f(siny_cosp, cosy_cosp) * (180.0f / (float)M_PI);
}

/* ── Public API ───────────────────────────────────────────────────────── */

esp_err_t imu_init(const imu_i2c_cfg_t *cfg)
{
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = cfg->i2c_addr,
        .scl_speed_hz    = 400000,
    };
    esp_err_t ret = i2c_master_bus_add_device(cfg->bus, &dev_cfg, &s_dev);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "I2C device add failed: %s", esp_err_to_name(ret));
        return ret;
    }

    uint8_t who_am_i = 0;
    ret = reg_read(REG_WHO_AM_I, &who_am_i, 1);
    if (ret != ESP_OK || who_am_i != WHO_AM_I_VAL) {
        ESP_LOGE(TAG, "LSM6DSVQTR not found (who_am_i=0x%02X, err=%s)",
                 who_am_i, esp_err_to_name(ret));
        return ESP_ERR_NOT_FOUND;
    }

    /* Restore power-on defaults before configuring. */
    reg_write(REG_FUNC_CFG_ACCESS, 0x04 /* sw_por */);
    vTaskDelay(pdMS_TO_TICKS(30));

    reg_write(REG_CTRL3, 0x44);   /* BDU=1, IF_INC=1 */
    reg_write(REG_CTRL8, 0x01);   /* fs_xl = ±4 g */
    reg_write(REG_CTRL6, 0x02);   /* fs_g  = ±500 dps */
    reg_write(REG_FIFO_CTRL4, 0x06); /* fifo_mode = STREAM (continuous) */
    reg_write(REG_CTRL1, 0x05);   /* accel ODR 60 Hz, high-performance mode */
    reg_write(REG_CTRL2, 0x05);   /* gyro  ODR 60 Hz, high-performance mode */

    page_select(true);
    reg_write(EMB_FUNC_FIFO_EN_A, 0x02);  /* batch SFLP game rotation vector into FIFO */
    reg_write(EMB_SFLP_ODR, 0x02);        /* SFLP algorithm ODR = 60 Hz */
    reg_write(EMB_FUNC_EN_A, 0x02);       /* enable SFLP game rotation vector */
    page_select(false);

    vTaskDelay(pdMS_TO_TICKS(50));  /* let the filter produce its first sample */

    ESP_LOGI(TAG, "LSM6DSVQTR ready — SFLP game rotation vector @ 60 Hz");
    return ESP_OK;
}

esp_err_t imu_read(imu_data_t *out)
{
    uint8_t status[2];
    esp_err_t ret = reg_read(REG_FIFO_STATUS1, status, sizeof(status));
    if (ret != ESP_OK) return ret;

    uint16_t count = status[0] | (uint16_t)((status[1] & 0x01) << 8);
    if (count == 0) return ESP_ERR_NOT_FOUND;

    bool  found = false;
    float q[4]  = { 0, 0, 0, 1 };

    while (count-- > 0) {
        uint8_t frame[7];
        ret = reg_read(REG_FIFO_DATA_OUT_TAG, frame, sizeof(frame));
        if (ret != ESP_OK) return ret;

        uint8_t tag = frame[0] >> 3;
        if (tag == FIFO_TAG_SFLP_GAME_ROTATION_VECTOR) {
            sflp_to_quat(&frame[1], q);
            found = true;
        }
    }

    if (!found) return ESP_ERR_NOT_FOUND;

    quat_to_euler_deg(q, &out->roll, &out->pitch, &out->heading);
    out->sys_cal    = 1;
    out->calibrated = true;
    return ESP_OK;
}
