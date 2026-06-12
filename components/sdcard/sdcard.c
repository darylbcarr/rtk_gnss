#include "sdcard.h"
#include "esp_vfs_fat.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "sdmmc_cmd.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/message_buffer.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <errno.h>
#include <sys/stat.h>
#include <unistd.h>

static const char *TAG = "sdlog";

#define MOUNT_POINT     "/sdcard"
#define LOG_LINE_MAX        160
#define MSG_BUF_BYTES   (LOG_LINE_MAX * 48)   /* ~7.5 KB — holds ~48 queued lines */
#define FLUSH_INTERVAL  10                 /* flush after every N lines written */

static FILE                  *s_file          = NULL;
static bool                   s_ready         = false;
static MessageBufferHandle_t  s_msg_buf       = NULL;
static volatile bool          s_flush_req     = false;

/* ── Background write task ─────────────────────────────────────────────── */

static void sdlog_task(void *arg)
{
    char    line[LOG_LINE_MAX];
    int     unflushed = 0;

    for (;;) {
        /* Block up to 5 s for the next line; use the timeout as a flush tick */
        size_t n = xMessageBufferReceive(s_msg_buf, line, sizeof(line),
                                         pdMS_TO_TICKS(5000));
        if (n > 0 && s_file) {
            fwrite(line, 1, n, s_file);
            if (++unflushed >= FLUSH_INTERVAL || s_flush_req) {
                fflush(s_file);
                fsync(fileno(s_file));
                unflushed    = 0;
                s_flush_req  = false;
            }
        } else if (s_file && unflushed > 0) {
            /* Timeout with pending data — flush now */
            fflush(s_file);
            fsync(fileno(s_file));
            unflushed   = 0;
            s_flush_req = false;
        }
    }
}

/* ── Filename helper ───────────────────────────────────────────────────── */

static int next_file_num(const char *prefix)
{
    for (int i = 1; i <= 999; i++) {
        char path[64];
        snprintf(path, sizeof(path), "%s/%s_%03d.CSV", MOUNT_POINT, prefix, i);
        struct stat st;
        if (stat(path, &st) != 0)
            return i;
    }
    return 999;
}

/* ── Public API ────────────────────────────────────────────────────────── */

esp_err_t sdlog_init(const sdlog_cfg_t *cfg, const char *mode_name)
{
    /* ── SPI bus ── */
    spi_bus_config_t bus_cfg = {
        .mosi_io_num     = cfg->pin_mosi,
        .miso_io_num     = cfg->pin_miso,
        .sclk_io_num     = cfg->pin_clk,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = 4096,
    };
    esp_err_t ret = spi_bus_initialize(cfg->spi_host, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "SPI bus init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* ── SD slot ── */
    sdspi_device_config_t slot = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot.gpio_cs = (gpio_num_t)cfg->pin_cs;
    slot.host_id = cfg->spi_host;

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = cfg->spi_host;

    esp_vfs_fat_mount_config_t mount_cfg = {
        .format_if_mount_failed = false,
        .max_files              = 4,
        .allocation_unit_size   = 16 * 1024,
    };

    sdmmc_card_t *card;
    ret = esp_vfs_fat_sdspi_mount(MOUNT_POINT, &host, &slot, &mount_cfg, &card);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "SD card mount failed: %s — logging disabled",
                 esp_err_to_name(ret));
        spi_bus_free(cfg->spi_host);
        return ret;
    }

    ESP_LOGI(TAG, "SD card mounted — %s %.0f MB",
             card->cid.name,
             (double)((uint64_t)card->csd.capacity * card->csd.sector_size) / (1024 * 1024));

    /* ── Open log file ── */
    int num = next_file_num(mode_name);
    char path[64];
    snprintf(path, sizeof(path), "%s/%s_%03d.CSV", MOUNT_POINT, mode_name, num);

    s_file = fopen(path, "w");
    if (!s_file) {
        ESP_LOGE(TAG, "Cannot open %s: %s", path, strerror(errno));
        return ESP_FAIL;
    }

    /* Write CSV header matched to mode */
    if (strncmp(mode_name, "BASE", 4) == 0) {
        fprintf(s_file,
                "time_s,state,svin_dur_s,svin_acc_m,svin_valid,"
                "rtcm_frames,rtcm_1005_cnt,rtcm_errors,"
                "ups_v,ups_pct,ups_ma\n");
    } else {
        fprintf(s_file,
                "time_s,quality,carr_soln,sv_count,"
                "h_acc_m,diff_age_s,arp_seen,pdop,lora_rx_B\n");
    }
    fflush(s_file);
    fsync(fileno(s_file));

    /* ── Async write task ── */
    s_msg_buf = xMessageBufferCreate(MSG_BUF_BYTES);
    if (!s_msg_buf) {
        ESP_LOGE(TAG, "Message buffer alloc failed");
        fclose(s_file);
        s_file = NULL;
        return ESP_ERR_NO_MEM;
    }
    xTaskCreate(sdlog_task, "sdlog", 4096, NULL, 3, NULL);

    s_ready = true;
    ESP_LOGI(TAG, "Logging → %s", path);
    return ESP_OK;
}

void sdlog_writef(const char *fmt, ...)
{
    if (!s_ready || !s_msg_buf) return;
    char line[LOG_LINE_MAX];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);
    if (n > 0)
        xMessageBufferSend(s_msg_buf, line, (size_t)n, 0);
}

void sdlog_flush(void)
{
    s_flush_req = true;
}

bool sdlog_is_ready(void)
{
    return s_ready;
}
