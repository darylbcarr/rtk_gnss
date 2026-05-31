#include "config.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"

static const char *TAG      = "config";
static const char *NVS_NS   = "rtk_cfg";
static const char *KEY_MODE = "device_mode";

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
