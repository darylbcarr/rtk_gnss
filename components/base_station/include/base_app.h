#pragma once

#include <stdint.h>
#include "gnss.h"
#include "transport.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    BASE_STATE_IDLE,
    BASE_STATE_SURVEY_IN,
    BASE_STATE_BROADCASTING,
} base_state_t;

/*
 * Start the base station task.  Begins survey-in, waits for convergence,
 * then forwards every RTCM3 frame via outbound transport.
 * Safe to call once; subsequent calls are no-ops.
 *
 * outbound    – LoRa (or other) transport; must already be initialised.
 * min_dur_s   – survey-in minimum observation duration (seconds).
 * acc_limit_m – survey-in 3D accuracy convergence threshold (meters).
 */
void base_app_start(transport_t *outbound, uint32_t min_dur_s, float acc_limit_m);

base_state_t base_app_get_state(void);

/* Returns the last received survey-in status snapshot. */
gnss_svin_t base_app_get_svin(void);

#ifdef __cplusplus
}
#endif
