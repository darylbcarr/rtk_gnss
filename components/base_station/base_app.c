#include "base_app.h"
#include "gnss.h"
#include "transport.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>

static const char *TAG = "base";

#define RTCM_BUF_SIZE  1029   /* max RTCM3 frame */
#define RTCM_TIMEOUT_MS 2000

/* Task state */
static volatile base_state_t s_state      = BASE_STATE_IDLE;
static transport_t           *s_transport = NULL;
static uint32_t               s_min_dur;
static float                  s_acc_limit;
static gnss_svin_t            s_last_svin;
static SemaphoreHandle_t      s_svin_lock;
static bool                   s_started   = false;

/* ── Survey-in phase ─────────────────────────────────────────────────── */

static void log_svin_progress(const gnss_svin_t *sv)
{
    ESP_LOGI(TAG, "svin: active=%d valid=%d t=%lus acc=%.3fm",
             sv->active, sv->valid,
             (unsigned long)sv->dur_s, sv->mean_acc_m);
}

static void wait_survey_in(void)
{
    s_state = BASE_STATE_SURVEY_IN;

    if (gnss_start_survey_in(s_min_dur, s_acc_limit) != ESP_OK) {
        ESP_LOGE(TAG, "gnss_start_survey_in failed");
        vTaskDelete(NULL);
    }

    /*
     * RTCM frames are already flowing from the LC29H.  Proceed to
     * broadcast_rtcm() immediately so the ring buffer doesn't overflow.
     * Survey-in progress is polled from inside the broadcast loop.
     */
    ESP_LOGI(TAG, "Base configured — starting RTCM broadcast immediately");
}

/* ── RTCM broadcast phase ────────────────────────────────────────────── */

static void broadcast_rtcm(void)
{
    s_state = BASE_STATE_BROADCASTING;

    /* Drain any frames that piled up during startup/configuration.
     * Sending a burst of old frames saturates the E22 and causes AUX
     * to stay LOW, blocking real-time forwarding. */
    {
        static uint8_t discard[RTCM_BUF_SIZE];
        int flushed = 0;
        while (gnss_read_rtcm(discard, sizeof(discard), 0) > 0) flushed++;
        if (flushed)
            ESP_LOGI(TAG, "Flushed %d stale RTCM frames before broadcast", flushed);
    }

    static uint8_t  buf[RTCM_BUF_SIZE];
    uint32_t        frames   = 0;
    uint32_t        bytes    = 0;
    uint32_t        errors      = 0;
    uint32_t        last_log    = (uint32_t)(esp_timer_get_time() / 1000);
    uint32_t        last_svin_s = 0;
    uint32_t        poll_ticks  = 0;

    for (;;) {
        int len = gnss_read_rtcm(buf, sizeof(buf), RTCM_TIMEOUT_MS);
        if (len == 0) {
            /* No frame this cycle — poll survey-in status while idle */
            if (++poll_ticks % 5 == 0)
                gnss_poll_svin_status();
            continue;
        }
        if (len < 0) { errors++; continue; }

        if (s_transport) {
            int sent = transport_send(s_transport, buf, (size_t)len);
            if (sent < 0) {
                errors++;
                ESP_LOGW(TAG, "LoRa send error for %d B frame", len);
            } else {
                frames++;
                bytes += (uint32_t)len;
            }
        }

        /* Periodic status logs */
        uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
        if (now - last_log >= 10000) {
            ESP_LOGI(TAG, "RTCM broadcast: %lu frames %lu B %lu errors",
                     (unsigned long)frames, (unsigned long)bytes, (unsigned long)errors);
            last_log = now;
        }

        /* Log survey-in progress when new status arrives */
        gnss_svin_t sv;
        if (gnss_get_svin_status(&sv)) {
            xSemaphoreTake(s_svin_lock, portMAX_DELAY);
            s_last_svin = sv;
            xSemaphoreGive(s_svin_lock);
            if (sv.dur_s != last_svin_s) {
                log_svin_progress(&sv);
                last_svin_s = sv.dur_s;
                if (sv.valid && s_state != BASE_STATE_BROADCASTING)
                    ESP_LOGI(TAG, "Survey-in converged at t=%lus, acc=%.3fm",
                             (unsigned long)sv.dur_s, sv.mean_acc_m);
            }
        }
    }
}

/* ── FreeRTOS task ───────────────────────────────────────────────────── */

static void base_task(void *arg)
{
    (void)arg;
    wait_survey_in();
    broadcast_rtcm();
    /* broadcast_rtcm never returns; task lives for the lifetime of the app */
}

/* ── Public API ──────────────────────────────────────────────────────── */

void base_app_start(transport_t *outbound, uint32_t min_dur_s, float acc_limit_m)
{
    if (s_started) return;
    s_started    = true;
    s_transport  = outbound;
    s_min_dur    = min_dur_s;
    s_acc_limit  = acc_limit_m;
    s_svin_lock  = xSemaphoreCreateMutex();
    xTaskCreate(base_task, "base_app", 4096, NULL, 12, NULL);
    ESP_LOGI(TAG, "Base task started (min=%lus acc=%.1fm transport=%s)",
             (unsigned long)min_dur_s, acc_limit_m,
             outbound ? "LoRa" : "none");
}

base_state_t base_app_get_state(void)
{
    return s_state;
}

gnss_svin_t base_app_get_svin(void)
{
    gnss_svin_t sv;
    xSemaphoreTake(s_svin_lock, portMAX_DELAY);
    sv = s_last_svin;
    xSemaphoreGive(s_svin_lock);
    return sv;
}
