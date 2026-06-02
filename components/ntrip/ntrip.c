#include "ntrip.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <sys/socket.h>
#include <netdb.h>
#include <unistd.h>
#include <errno.h>

static const char *TAG = "ntrip";

/* ── Base64 ──────────────────────────────────────────────────────────── */

static const char B64[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

static void base64_encode(const uint8_t *src, size_t len, char *dst)
{
    size_t di = 0;
    for (size_t si = 0; si < len; si += 3) {
        uint8_t a = src[si];
        uint8_t b = (si + 1 < len) ? src[si + 1] : 0;
        uint8_t c = (si + 2 < len) ? src[si + 2] : 0;
        dst[di++] = B64[a >> 2];
        dst[di++] = B64[((a & 0x03) << 4) | (b >> 4)];
        dst[di++] = (si + 1 < len) ? B64[((b & 0x0F) << 2) | (c >> 6)] : '=';
        dst[di++] = (si + 2 < len) ? B64[c & 0x3F] : '=';
    }
    dst[di] = '\0';
}

/* ── Task state ──────────────────────────────────────────────────────── */

static ntrip_cfg_t      s_cfg;
static ntrip_rtcm_cb_t  s_cb;
static void            *s_ctx;
static TaskHandle_t     s_task;
static volatile bool    s_running;
static volatile bool    s_connected;

static ntrip_pos_t       s_pos;
static bool              s_pos_valid;
static SemaphoreHandle_t s_pos_mutex;

/* ── GGA formatting ──────────────────────────────────────────────────── */

static int gga_quality(uint8_t fix_type, uint8_t carr_soln)
{
    if (fix_type < 2) return 0;
    if (carr_soln == 2) return 4;
    if (carr_soln == 1) return 5;
    return 1;
}

/* Write a complete NMEA GGA sentence (with $ and *CK\r\n) into out. */
static void build_gga(char *out, size_t outsz)
{
    ntrip_pos_t p;
    bool valid;
    xSemaphoreTake(s_pos_mutex, portMAX_DELAY);
    p     = s_pos;
    valid = s_pos_valid;
    xSemaphoreGive(s_pos_mutex);

    char body[128];
    if (valid) {
        uint32_t stow = (p.i_tow / 1000) % 86400;
        int hh = stow / 3600, mm = (stow % 3600) / 60, ss = stow % 60;
        double alat = fabs(p.lat), alon = fabs(p.lon);
        int lat_d = (int)alat, lon_d = (int)alon;
        double lat_m = (alat - lat_d) * 60.0, lon_m = (alon - lon_d) * 60.0;
        snprintf(body, sizeof(body),
            "GPGGA,%02d%02d%02d.00,%02d%08.5f,%c,%03d%08.5f,%c,%d,%02u,%.1f,%.2f,M,0.0,M,,",
            hh, mm, ss,
            lat_d, lat_m, (p.lat >= 0) ? 'N' : 'S',
            lon_d, lon_m, (p.lon >= 0) ? 'E' : 'W',
            gga_quality(p.fix_type, p.carr_soln), p.num_sv, (double)p.pdop, p.alt_msl);
    } else {
        strncpy(body, "GPGGA,000000.00,0000.00000,N,00000.00000,E,0,00,0.0,0.0,M,0.0,M,,",
                sizeof(body));
    }

    uint8_t ck = 0;
    for (int i = 0; body[i]; i++) ck ^= (uint8_t)body[i];
    snprintf(out, outsz, "$%s*%02X\r\n", body, ck);
}

/* ── Helpers ─────────────────────────────────────────────────────────── */

static bool send_gga(int fd)
{
    char gga[160];
    build_gga(gga, sizeof(gga));
    size_t n = strlen(gga);
    int sent = send(fd, gga, n, MSG_NOSIGNAL);
    if (sent != (int)n) {
        ESP_LOGW(TAG, "GGA send failed: sent=%d errno=%d", sent, errno);
        return false;
    }
    /* Log without trailing \r\n */
    size_t gl = n;
    while (gl > 0 && (gga[gl-1] == '\r' || gga[gl-1] == '\n')) gga[--gl] = '\0';
    ESP_LOGI(TAG, "GGA sent: %s", gga);
    return true;
}

static int tcp_connect(const char *host, uint16_t port)
{
    char port_str[8];
    snprintf(port_str, sizeof(port_str), "%u", port);

    struct addrinfo hints = { .ai_family = AF_INET, .ai_socktype = SOCK_STREAM };
    struct addrinfo *res  = NULL;

    if (getaddrinfo(host, port_str, &hints, &res) != 0 || !res) {
        ESP_LOGE(TAG, "DNS failed for %s", host);
        return -1;
    }

    int fd = socket(res->ai_family, res->ai_socktype, 0);
    if (fd >= 0) {
        struct timeval tv = { .tv_sec = 10 };
        setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv)); /* connect() timeout */
        tv.tv_sec = 15;
        setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)); /* recv() timeout */

        ESP_LOGI(TAG, "TCP connecting to %s:%u…", host, port);
        if (connect(fd, res->ai_addr, res->ai_addrlen) != 0) {
            ESP_LOGE(TAG, "connect to %s:%u failed: errno %d", host, port, errno);
            close(fd);
            fd = -1;
        } else {
            ESP_LOGI(TAG, "TCP connected, fd=%d", fd);
        }
    }
    freeaddrinfo(res);
    return fd;
}

static bool send_request(int fd, const ntrip_cfg_t *cfg)
{
    char creds[160];
    snprintf(creds, sizeof(creds), "%s:%s", cfg->user, cfg->password);
    char b64[((sizeof(creds) + 2) / 3) * 4 + 2];
    base64_encode((uint8_t *)creds, strlen(creds), b64);

    char req[512];
    int n = snprintf(req, sizeof(req),
        "GET /%s HTTP/1.0\r\n"
        "Host: %s:%u\r\n"
        "Ntrip-Version: Ntrip/1.0\r\n"
        "User-Agent: NTRIP ESP32/1.0\r\n"
        "Accept: */*\r\n"
        "Authorization: Basic %s\r\n"
        "\r\n",
        cfg->mountpoint, cfg->host, cfg->port, b64);

    return send(fd, req, n, 0) == n;
}

static bool read_response_ok(int fd)
{
    char first[128] = {0};
    size_t fi = 0;
    char tail[4] = {0};

    ESP_LOGI(TAG, "waiting for server response…");
    for (int i = 0; i < 4096; i++) {
        char c;
        int r = recv(fd, &c, 1, 0);
        if (r != 1) {
            ESP_LOGE(TAG, "recv in header: r=%d errno=%d after %d bytes (%.80s)",
                     r, errno, i, first);
            return false;
        }

        if (c != '\r' && c != '\n' && fi < sizeof(first) - 1) first[fi++] = c;

        /* ICY 200 OK responses (NTRIP 1.0) have no further headers — RTCM
         * starts immediately after the first \n.  Return as soon as we see
         * the status line rather than waiting for a \r\n\r\n that never comes. */
        if (c == '\n' && fi > 0) {
            bool ok = (strstr(first, "200") != NULL);
            if (ok)  ESP_LOGI(TAG, "caster accepted: %.80s", first);
            else     ESP_LOGE(TAG, "caster rejected: %.80s", first);
            return ok;
        }

        /* Also accept the standard HTTP double-CRLF header terminator. */
        memmove(tail, tail + 1, 3);
        tail[3] = c;
        if (memcmp(tail, "\r\n\r\n", 4) == 0) {
            bool ok = (strstr(first, "200") != NULL);
            if (ok)  ESP_LOGI(TAG, "caster accepted (HTTP): %.80s", first);
            else     ESP_LOGE(TAG, "caster rejected (HTTP): %.80s", first);
            return ok;
        }
    }
    ESP_LOGE(TAG, "no status line after 4096 bytes (%.80s)", first);
    return false;
}

/* ── Task ────────────────────────────────────────────────────────────── */

static void ntrip_task(void *arg)
{
    uint8_t buf[512];

    /* Wait up to 3 s for a valid GNSS position so the initial GGA
     * header carries real coordinates, not zeros. */
    for (int i = 0; i < 30 && !s_pos_valid; i++) vTaskDelay(pdMS_TO_TICKS(100));

    while (s_running) {
        ESP_LOGI(TAG, "connecting to %s:%u/%s", s_cfg.host, s_cfg.port, s_cfg.mountpoint);
        int fd = tcp_connect(s_cfg.host, s_cfg.port);
        if (fd < 0) { vTaskDelay(pdMS_TO_TICKS(10000)); continue; }

        if (!send_request(fd, &s_cfg)) {
            ESP_LOGE(TAG, "send request failed");
            close(fd);
            vTaskDelay(pdMS_TO_TICKS(10000));
            continue;
        }

        if (!read_response_ok(fd)) {
            close(fd);
            vTaskDelay(pdMS_TO_TICKS(30000));
            continue;
        }

        ESP_LOGI(TAG, "RTCM stream active — %s/%s", s_cfg.host, s_cfg.mountpoint);
        s_connected = true;

        /* RTK2go requires GGA within a few seconds; send immediately then every 30 s. */
        send_gga(fd);
        uint32_t rtcm_bytes = 0;
        TickType_t last_gga = xTaskGetTickCount();

        while (s_running) {
            int n = recv(fd, buf, sizeof(buf), 0);
            if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
                ESP_LOGW(TAG, "no RTCM for 15 s — reconnecting (received %lu B total)", rtcm_bytes);
                break;
            }
            if (n <= 0) {
                if (n == 0) ESP_LOGW(TAG, "caster closed after %lu B", rtcm_bytes);
                else        ESP_LOGW(TAG, "recv errno=%d after %lu B", errno, rtcm_bytes);
                break;
            }
            rtcm_bytes += (uint32_t)n;
            if (s_cb) s_cb(buf, (size_t)n, s_ctx);

            TickType_t now = xTaskGetTickCount();
            if ((now - last_gga) >= pdMS_TO_TICKS(30000)) {
                if (!send_gga(fd)) break;
                last_gga = now;
            }
        }

        s_connected = false;
        close(fd);
        if (s_running) {
            ESP_LOGI(TAG, "reconnecting in 5 s…");
            vTaskDelay(pdMS_TO_TICKS(5000));
        }
    }

    vTaskDelete(NULL);
}

bool ntrip_is_connected(void) { return s_connected; }

/* ── Public API ──────────────────────────────────────────────────────── */

esp_err_t ntrip_start(const ntrip_cfg_t *cfg, ntrip_rtcm_cb_t cb, void *ctx)
{
    s_cfg       = *cfg;
    s_cb        = cb;
    s_ctx       = ctx;
    s_running   = true;
    s_pos_valid = false;

    if (!s_pos_mutex) s_pos_mutex = xSemaphoreCreateMutex();

    BaseType_t ok = xTaskCreate(ntrip_task, "ntrip", 6144, NULL, 5, &s_task);
    return (ok == pdPASS) ? ESP_OK : ESP_ERR_NO_MEM;
}

void ntrip_stop(void)
{
    s_running = false;
}

void ntrip_update_position(const ntrip_pos_t *pos)
{
    xSemaphoreTake(s_pos_mutex, portMAX_DELAY);
    s_pos       = *pos;
    s_pos_valid = true;
    xSemaphoreGive(s_pos_mutex);
}
