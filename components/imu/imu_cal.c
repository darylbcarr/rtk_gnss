#include "imu_cal.h"
#include <math.h>
#include <string.h>

static float to_deg(float lean)
{
    if (lean > 1.0f) lean = 1.0f;
    if (lean < -1.0f) lean = -1.0f;
    return asinf(lean) * (180.0f / (float)M_PI);
}

void imu_cal_start(imu_cal_t *c)
{
    memset(c, 0, sizeof(*c));
}

void imu_cal_begin_position(imu_cal_t *c)
{
    c->n     = 0;
    c->sum_x = c->sum_y = 0.0f;
    c->min_x = c->min_y =  1e9f;
    c->max_x = c->max_y = -1e9f;
}

imu_cal_status_t imu_cal_feed(imu_cal_t *c, const imu_tilt_t *t)
{
    if (!t->steady)
        return IMU_CAL_MOVED;

    if (c->n < IMU_CAL_SETTLE_SAMPLES) {
        c->n++;
        return IMU_CAL_SAMPLING;
    }

    c->sum_x += t->raw_x;
    c->sum_y += t->raw_y;
    if (t->raw_x < c->min_x) c->min_x = t->raw_x;
    if (t->raw_x > c->max_x) c->max_x = t->raw_x;
    if (t->raw_y < c->min_y) c->min_y = t->raw_y;
    if (t->raw_y > c->max_y) c->max_y = t->raw_y;
    c->n++;

    float spread = fmaxf(c->max_x - c->min_x, c->max_y - c->min_y);
    if (to_deg(spread) > IMU_CAL_MOVE_DEG)
        return IMU_CAL_MOVED;

    int collected = c->n - IMU_CAL_SETTLE_SAMPLES;
    if (collected < IMU_CAL_WINDOW_SAMPLES)
        return IMU_CAL_SAMPLING;

    c->pt[c->done][0] = c->sum_x / (float)collected;
    c->pt[c->done][1] = c->sum_y / (float)collected;
    c->done++;
    return c->done >= IMU_CAL_POSITIONS ? IMU_CAL_ALL_DONE : IMU_CAL_POSITION_DONE;
}

int imu_cal_progress_pct(const imu_cal_t *c)
{
    int pct = c->n * 100 / (IMU_CAL_SETTLE_SAMPLES + IMU_CAL_WINDOW_SAMPLES);
    return pct > 100 ? 100 : pct;
}

imu_cal_verdict_t imu_cal_result(const imu_cal_t *c, imu_cal_result_t *r)
{
    memset(r, 0, sizeof(*r));
    if (c->done < IMU_CAL_POSITIONS)
        return IMU_CAL_INCOMPLETE;

    /* Opposite positions cancel the table tilt, leaving the offset. */
    float m02x = 0.5f * (c->pt[0][0] + c->pt[2][0]);
    float m02y = 0.5f * (c->pt[0][1] + c->pt[2][1]);
    float m13x = 0.5f * (c->pt[1][0] + c->pt[3][0]);
    float m13y = 0.5f * (c->pt[1][1] + c->pt[3][1]);

    r->off_x     = 0.5f * (m02x + m13x);
    r->off_y     = 0.5f * (m02y + m13y);
    r->resid_deg = to_deg(hypotf(m02x - m13x, m02y - m13y));

    float t02 = 0.5f * hypotf(c->pt[0][0] - c->pt[2][0], c->pt[0][1] - c->pt[2][1]);
    float t13 = 0.5f * hypotf(c->pt[1][0] - c->pt[3][0], c->pt[1][1] - c->pt[3][1]);
    r->table_deg = to_deg(0.5f * (t02 + t13));

    if (r->resid_deg > IMU_CAL_MAX_RESID_DEG)
        return IMU_CAL_INCONSISTENT;
    if (to_deg(hypotf(r->off_x, r->off_y)) > IMU_CAL_MAX_OFFSET_DEG)
        return IMU_CAL_OFFSET_TOO_LARGE;
    return IMU_CAL_OK;
}
