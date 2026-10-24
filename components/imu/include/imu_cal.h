#pragma once

#include <stdbool.h>
#include "imu.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Level calibration: finds the fixed offset of the IMU's tilt reading by
 * sampling the device resting flat at four positions, each rotated 90° about
 * the vertical from the last. The table's own tilt rotates with the device in
 * the IMU frame while the sensor/mount offset does not, so the mean of the four
 * readings is the offset — no perfectly level surface needed.
 *
 * Pure logic, no I/O: the caller feeds it imu_tilt_t samples at its own rate.
 */

#define IMU_CAL_POSITIONS      4
#define IMU_CAL_SETTLE_SAMPLES 8     /* discarded after the operator presses start */
#define IMU_CAL_WINDOW_SAMPLES 25    /* averaged per position */
#define IMU_CAL_MOVE_DEG       0.5f  /* max spread within a window before it restarts */
#define IMU_CAL_MAX_RESID_DEG  0.5f  /* 0/180 and 90/270 estimates must agree this well */
#define IMU_CAL_MAX_OFFSET_DEG 5.0f  /* larger than this means the device was not flat */

typedef enum {
    IMU_CAL_SAMPLING,        /* collecting; keep feeding */
    IMU_CAL_POSITION_DONE,   /* this position finished; prompt for the next */
    IMU_CAL_MOVED,           /* device moved; the position must be retaken */
    IMU_CAL_ALL_DONE,        /* all four positions collected; call imu_cal_result() */
} imu_cal_status_t;

typedef enum {
    IMU_CAL_OK,
    IMU_CAL_INCONSISTENT,    /* opposite-pair estimates disagree: surface or handling */
    IMU_CAL_OFFSET_TOO_LARGE,
    IMU_CAL_INCOMPLETE,
} imu_cal_verdict_t;

typedef struct {
    float off_x, off_y;      /* lean units: subtract from raw readings */
    float resid_deg;         /* disagreement between the two pair estimates */
    float table_deg;         /* tilt of the surface the device sat on */
} imu_cal_result_t;

typedef struct {
    int   done;              /* positions completed */
    int   n;                 /* samples seen in the current attempt */
    float sum_x, sum_y;
    float min_x, max_x, min_y, max_y;
    float pt[IMU_CAL_POSITIONS][2];
} imu_cal_t;

void imu_cal_start(imu_cal_t *c);

/* Operator confirmed the device is placed: begin (or restart) the position. */
void imu_cal_begin_position(imu_cal_t *c);

imu_cal_status_t imu_cal_feed(imu_cal_t *c, const imu_tilt_t *t);

/* Progress of the current position, 0–100. */
int imu_cal_progress_pct(const imu_cal_t *c);

/* Offset and quality, valid after IMU_CAL_ALL_DONE. *r is filled even when the
 * verdict is not OK, so the caller can show why. */
imu_cal_verdict_t imu_cal_result(const imu_cal_t *c, imu_cal_result_t *r);

#ifdef __cplusplus
}
#endif
