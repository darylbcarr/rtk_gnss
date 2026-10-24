#include "config.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"

static const char *TAG      = "config";
static const char *NVS_NS   = "rtk_cfg";
static const char *KEY_MODE = "device_mode";
static const char *KEY_LVL_X = "lvl_off_x";   /* i32, micro-units */
static const char *KEY_LVL_Y = "lvl_off_y";
static const char *KEY_POLE  = "pole_mm";     /* u16 */

#define DEFAULT_POLE_MM 2000

esp_err_t config_init(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition full or version mismatch — erasing");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    return ret;
}

device_mode_t config_get_device_mode(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK)
        return DEVICE_MODE_UNSET;

    uint8_t val = DEVICE_MODE_UNSET;
    nvs_get_u8(h, KEY_MODE, &val);
    nvs_close(h);

    if (val != DEVICE_MODE_ROVER && val != DEVICE_MODE_BASE)
        return DEVICE_MODE_UNSET;
    return (device_mode_t)val;
}

esp_err_t config_set_device_mode(device_mode_t mode)
{
    nvs_handle_t h;
    esp_err_t ret = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (ret != ESP_OK) return ret;

    ret = nvs_set_u8(h, KEY_MODE, (uint8_t)mode);
    if (ret == ESP_OK)
        ret = nvs_commit(h);
    nvs_close(h);

    ESP_LOGI(TAG, "device_mode saved: %s",
             mode == DEVICE_MODE_ROVER ? "ROVER" : "BASE");
    return ret;
}

esp_err_t config_get_level_offset(float *x, float *y)
{
    nvs_handle_t h;
    esp_err_t ret = nvs_open(NVS_NS, NVS_READONLY, &h);
    if (ret != ESP_OK) return ESP_ERR_NOT_FOUND;

    int32_t ix, iy;
    ret = nvs_get_i32(h, KEY_LVL_X, &ix);
    if (ret == ESP_OK) ret = nvs_get_i32(h, KEY_LVL_Y, &iy);
    nvs_close(h);
    if (ret != ESP_OK) return ESP_ERR_NOT_FOUND;

    *x = ix * 1e-6f;
    *y = iy * 1e-6f;
    return ESP_OK;
}

esp_err_t config_set_level_offset(float x, float y)
{
    nvs_handle_t h;
    esp_err_t ret = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (ret != ESP_OK) return ret;

    ret = nvs_set_i32(h, KEY_LVL_X, (int32_t)(x * 1e6f));
    if (ret == ESP_OK) ret = nvs_set_i32(h, KEY_LVL_Y, (int32_t)(y * 1e6f));
    if (ret == ESP_OK) ret = nvs_commit(h);
    nvs_close(h);

    ESP_LOGI(TAG, "level offset saved: x=%.5f y=%.5f", x, y);
    return ret;
}

float config_get_pole_len_m(void)
{
    nvs_handle_t h;
    uint16_t mm = DEFAULT_POLE_MM;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) == ESP_OK) {
        nvs_get_u16(h, KEY_POLE, &mm);
        nvs_close(h);
    }
    return mm * 0.001f;
}

esp_err_t config_set_pole_len_m(float metres)
{
    nvs_handle_t h;
    esp_err_t ret = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (ret != ESP_OK) return ret;

    ret = nvs_set_u16(h, KEY_POLE, (uint16_t)(metres * 1000.0f + 0.5f));
    if (ret == ESP_OK) ret = nvs_commit(h);
    nvs_close(h);
    return ret;
}
