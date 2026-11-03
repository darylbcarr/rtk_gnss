#include "imu_mag.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <string.h>

static const char *TAG = "mag";

/* MLX90393 datasheet rev 012, section 15 (commands) and 16 (memory map). */
#define CMD_SM        0x30   /* start single measurement, | zyxt */
#define CMD_RM        0x40   /* read measurement, | zyxt */
#define CMD_RR        0x50   /* read register, then (addr << 2) */
#define CMD_WR        0x60   /* write register: hi, lo, (addr << 2) */
#define CMD_EX        0x80   /* exit mode */
#define CMD_RT        0xF0   /* reset */
#define ZYXT_ALL      0x0F

#define ST_SM_MODE    0x20
#define ST_ERROR      0x10

#define REG_CONF1     0x00   /* [6:4] GAIN_SEL, [3:0] HALLCONF */
#define REG_CONF3     0x02   /* [12:11] OSR2, [10:5] RES_Z/Y/X, [4:2] DIG_FILT, [1:0] OSR */

/* Chosen configuration. The power-on filter settings are documented as
 * illegal, so they are always rewritten. OSR 2 / DIG_FILT 4 gives ~14.5 ms
 * per XYZ+T conversion (table 19), a middle point between noise and the
 * ~2.5 mA drawn while converting. RES 0 keeps the output a signed 16-bit
 * word with the sensitivity from table 17. */
#define CFG_GAIN_SEL  7      /* finest sensitivity, ±4.9 mT range */
#define CFG_OSR       2
#define CFG_DIG_FILT  4
#define CFG_OSR2      0

/* Sensitivity in µT/LSB, HALLCONF = 0xC, RES = 0, 25 °C (table 17). */
static const float k_sens_xy[8] = { 0.751f, 0.601f, 0.451f, 0.376f, 0.300f, 0.250f, 0.200f, 0.150f };
static const float k_sens_z[8]  = { 1.210f, 0.968f, 0.726f, 0.605f, 0.484f, 0.403f, 0.323f, 0.242f };

#define TEMP_T25_LSB   46244.0f
#define TEMP_LSB_PER_C 45.2f

static i2c_master_dev_handle_t s_dev;
static imu_mag_sens_t          s_sens;
static uint32_t                s_wait_us;       /* conversion time + margin */
static uint32_t                s_interval_ms;

typedef enum { MAG_IDLE, MAG_MEASURING } mag_state_t;
static mag_state_t s_state;
static int64_t     s_started_us;
static int64_t     s_next_us;
static int         s_fail;

/* ── Bus helpers ─────────────────────────────────────────────────────── */

/* FreeRTOS here ticks at 100 Hz, so pdMS_TO_TICKS() of anything under 10 ms is
 * zero ticks, i.e. no delay at all. Sleep a whole tick or more. */
static void wait_ms(uint32_t ms)
{
    TickType_t ticks = pdMS_TO_TICKS(ms);
    vTaskDelay(ticks ? ticks : 1);
}

static esp_err_t xfer(const uint8_t *tx, size_t txn, uint8_t *rx, size_t rxn)
{
    return i2c_master_transmit_receive(s_dev, tx, txn, rx, rxn, 100);
}

static esp_err_t read_reg(uint8_t reg, uint16_t *val)
{
    uint8_t tx[2] = { CMD_RR, (uint8_t)(reg << 2) };
    uint8_t rx[3];
    esp_err_t ret = xfer(tx, sizeof(tx), rx, sizeof(rx));
    if (ret != ESP_OK) return ret;
    if (rx[0] & ST_ERROR) return ESP_FAIL;
    *val = (uint16_t)((rx[1] << 8) | rx[2]);
    return ESP_OK;
}

static esp_err_t write_reg(uint8_t reg, uint16_t val)
{
    uint8_t tx[4] = { CMD_WR, (uint8_t)(val >> 8), (uint8_t)val, (uint8_t)(reg << 2) };
    uint8_t st;
    esp_err_t ret = xfer(tx, sizeof(tx), &st, 1);
    if (ret != ESP_OK) return ret;
    return (st & ST_ERROR) ? ESP_FAIL : ESP_OK;
}

/* ── Conversion ──────────────────────────────────────────────────────── */

/* Total time for a single measurement of X, Y, Z and T (section 12): the fixed
 * phases at their datasheet maxima, plus three magnetic conversions and one
 * temperature conversion, padded for the +/-10 % oscillator spread and 1 ms
 * of bus slack. */
static uint32_t conversion_wait_us(int osr, int dig_filt, int osr2)
{
    uint32_t tconvm = 67u + 64u * (1u << osr) * (2u + (1u << dig_filt));
    uint32_t tconvt = 67u + 192u * (1u << osr2);
    uint32_t total  = 264u + 432u + 120u + 3u * tconvm + tconvt;
    return total + total / 8u + 1000u;
}

void imu_mag_decode(const uint8_t raw[9], const imu_mag_sens_t *s, imu_mag_t *out)
{
    uint16_t t = (uint16_t)((raw[1] << 8) | raw[2]);
    int16_t  x = (int16_t)((raw[3] << 8) | raw[4]);
    int16_t  y = (int16_t)((raw[5] << 8) | raw[6]);
    int16_t  z = (int16_t)((raw[7] << 8) | raw[8]);

    out->x_uT = x * s->xy_uT_per_lsb;
    out->y_uT = y * s->xy_uT_per_lsb;
    out->z_uT = z * s->z_uT_per_lsb;
    out->magnitude_uT = sqrtf(out->x_uT * out->x_uT + out->y_uT * out->y_uT +
                              out->z_uT * out->z_uT);
    out->temp_c = ((float)t - TEMP_T25_LSB) / TEMP_LSB_PER_C + 25.0f;
}

/* ── Public API ──────────────────────────────────────────────────────── */

esp_err_t imu_mag_init(const imu_mag_cfg_t *cfg)
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

    /* Leave any running mode, then warm-reset (datasheet 15.4.1.2). The
     * chip has no ID register; a NACK here means nothing is at this address. */
    uint8_t cmd = CMD_EX, st = 0;
    ret = xfer(&cmd, 1, &st, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MLX90393 not found at 0x%02X (%s)", cfg->i2c_addr, esp_err_to_name(ret));
        i2c_master_bus_rm_device(s_dev);
        s_dev = NULL;
        return ESP_ERR_NOT_FOUND;
    }
    wait_ms(10);                       /* datasheet: >1 ms between EX and RT */
    cmd = CMD_RT;
    xfer(&cmd, 1, &st, 1);

    /* The chip NACKs while it restarts (datasheet: >1.5 ms). Wait, then retry
     * the first register read a few times rather than trusting one delay. */
    uint16_t conf1 = 0;
    ret = ESP_FAIL;
    for (int attempt = 0; attempt < 5 && ret != ESP_OK; attempt++) {
        wait_ms(10);
        ret = read_reg(REG_CONF1, &conf1);
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "register read failed after reset (%s)", esp_err_to_name(ret));
        return ESP_FAIL;
    }

    /* Power-on defaults are GAIN_SEL 7 / HALLCONF 0xC. Anything else means
     * the non-volatile memory was changed or this is not an MLX90393. */
    uint8_t hallconf = conf1 & 0x0F;
    if ((conf1 & 0x7F) != ((7u << 4) | 0x0C))
        ESP_LOGW(TAG, "CONF1=0x%04X is not the datasheet default (0x007C)", conf1);

    conf1 = (uint16_t)((conf1 & ~0x0070u) | (CFG_GAIN_SEL << 4));
    uint16_t conf3 = (uint16_t)((CFG_OSR2 << 11) | (CFG_DIG_FILT << 2) | CFG_OSR);  /* RES_XYZ = 0 */
    if (write_reg(REG_CONF1, conf1) != ESP_OK || write_reg(REG_CONF3, conf3) != ESP_OK) {
        ESP_LOGE(TAG, "configuration write rejected");
        return ESP_FAIL;
    }
    uint16_t back1, back3;
    if (read_reg(REG_CONF1, &back1) != ESP_OK || read_reg(REG_CONF3, &back3) != ESP_OK ||
        back1 != conf1 || back3 != conf3) {
        ESP_LOGE(TAG, "configuration did not read back (CONF1 0x%04X/0x%04X, CONF3 0x%04X/0x%04X)",
                 back1, conf1, back3, conf3);
        return ESP_FAIL;
    }

    /* Table 17 is for HALLCONF = 0xC; 0x0 scales the sensitivity by 98/75. */
    float scale = (hallconf == 0x00) ? 98.0f / 75.0f : 1.0f;
    if (hallconf != 0x0C && hallconf != 0x00)
        ESP_LOGW(TAG, "HALLCONF 0x%X is not in the sensitivity table; using the 0xC values", hallconf);
    s_sens.xy_uT_per_lsb = k_sens_xy[CFG_GAIN_SEL] * scale;
    s_sens.z_uT_per_lsb  = k_sens_z[CFG_GAIN_SEL] * scale;
    s_wait_us = conversion_wait_us(CFG_OSR, CFG_DIG_FILT, CFG_OSR2);

    s_state       = MAG_IDLE;
    s_interval_ms = 0;
    s_fail        = 0;
    ESP_LOGI(TAG, "MLX90393 ready at 0x%02X — single-shot, %.1f ms per measurement, idle until requested",
             cfg->i2c_addr, s_wait_us / 1000.0f);
    return ESP_OK;
}

void imu_mag_set_interval_ms(uint32_t ms)
{
    if (ms != 0 && s_interval_ms == 0)
        s_next_us = 0;                       /* start straight away */
    s_interval_ms = ms;
}

esp_err_t imu_mag_poll(imu_mag_t *out)
{
    if (!s_dev) return ESP_ERR_INVALID_STATE;
    int64_t now = esp_timer_get_time();

    if (s_state == MAG_IDLE) {
        if (s_interval_ms == 0 || now < s_next_us) return ESP_ERR_NOT_FOUND;

        uint8_t cmd = CMD_SM | ZYXT_ALL, st;
        esp_err_t ret = xfer(&cmd, 1, &st, 1);
        if (ret == ESP_OK && (st & ST_ERROR || !(st & ST_SM_MODE)))
            ret = ESP_FAIL;
        if (ret != ESP_OK) {
            if (++s_fail >= 5) {                 /* get back to a known state */
                uint8_t ex = CMD_EX, s2;
                xfer(&ex, 1, &s2, 1);
                s_fail = 0;
            }
            s_next_us = now + (int64_t)s_interval_ms * 1000;
            return ret;
        }
        s_started_us = now;
        s_state = MAG_MEASURING;
        return ESP_ERR_NOT_FOUND;
    }

    if (now - s_started_us < (int64_t)s_wait_us) return ESP_ERR_NOT_FOUND;

    uint8_t cmd = CMD_RM | ZYXT_ALL;
    uint8_t rx[9];
    esp_err_t ret = xfer(&cmd, 1, rx, sizeof(rx));
    if (ret == ESP_OK && (rx[0] & ST_ERROR)) {
        /* Data not ready yet (the error bit flags a read while DRDY is low).
         * Try again next call, but give up if it never completes. */
        if (now - s_started_us < 4 * (int64_t)s_wait_us) return ESP_ERR_NOT_FOUND;
        ret = ESP_ERR_TIMEOUT;
    }
    s_state   = MAG_IDLE;
    s_next_us = s_started_us + (int64_t)s_interval_ms * 1000;
    if (ret != ESP_OK) {
        s_fail++;
        return ret;
    }
    s_fail = 0;
    imu_mag_decode(rx, &s_sens, out);
    return ESP_OK;
}
