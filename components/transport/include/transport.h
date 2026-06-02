#pragma once

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    TRANSPORT_DISCONNECTED = 0,
    TRANSPORT_CONNECTING,
    TRANSPORT_CONNECTED,
    TRANSPORT_ERROR,
} transport_status_t;

typedef struct transport_s transport_t;
struct transport_s {
    esp_err_t (*init)(transport_t *t, const void *cfg);
    esp_err_t (*deinit)(transport_t *t);

    /* Returns bytes written, or -1 on error. */
    int       (*send)(transport_t *t, const uint8_t *buf, size_t len);

    /* Returns bytes read (0 on timeout), or -1 on error. */
    int       (*recv)(transport_t *t, uint8_t *buf, size_t max_len, uint32_t timeout_ms);

    int       (*status)(transport_t *t);   /* returns transport_status_t */
    void      *priv;
};

/* Convenience macros so callers don't dereference the vtable manually. */
#define transport_init(t, cfg)              ((t)->init((t), (cfg)))
#define transport_deinit(t)                 ((t)->deinit((t)))
#define transport_send(t, buf, len)         ((t)->send((t), (buf), (len)))
#define transport_recv(t, buf, max, tmo)    ((t)->recv((t), (buf), (max), (tmo)))
#define transport_status(t)                 ((t)->status((t)))

#ifdef __cplusplus
}
#endif
