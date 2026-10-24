#include "ui.h"
#include "config.h"
#include "display.h"
#include "encoder.h"
#include "ups.h"
#include "imu.h"
#include "imu_cal.h"
#include "gnss.h"
#include "base_app.h"
#include "sdcard.h"
#include "esp_system.h"
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* ── Screen definitions ──────────────────────────────────────────────── */

typedef enum {
    SCREEN_LIVE,
    SCREEN_MENU,
    SCREEN_FIX_DETAIL,
    SCREEN_LEVEL,
    SCREEN_CAL,
    SCREEN_POLE,
    SCREEN_MODE_SELECT,
    SCREEN_CLEAR_CONFIRM,
} screen_t;

typedef struct {
    const char *label;
    screen_t    target;
} menu_item_t;

/* Forward declaration — defined with other statics below. */
static device_mode_t s_mode;

/* Rover has a "Fix detail" entry; Base omits it (no PVT in base mode). */
static const menu_item_t s_rover_menu[] = {
    { "Fix detail",  SCREEN_FIX_DETAIL    },
    { "Level",       SCREEN_LEVEL         },
    { "Calibrate lvl", SCREEN_CAL         },
    { "Pole length", SCREEN_POLE          },
    { "Device mode", SCREEN_MODE_SELECT   },
    { "Clear logs",  SCREEN_CLEAR_CONFIRM },
    { "Exit",        SCREEN_LIVE          },
};
static const menu_item_t s_base_menu[] = {
    { "Device mode", SCREEN_MODE_SELECT   },
    { "Clear logs",  SCREEN_CLEAR_CONFIRM },
    { "Exit",        SCREEN_LIVE          },
};

static const menu_item_t *cur_menu(void)
{
    return s_mode == DEVICE_MODE_BASE ? s_base_menu : s_rover_menu;
}
static int cur_menu_cnt(void)
{
    return s_mode == DEVICE_MODE_BASE
           ? (int)(sizeof(s_base_menu)  / sizeof(s_base_menu[0]))
           : (int)(sizeof(s_rover_menu) / sizeof(s_rover_menu[0]));
}

static screen_t      s_screen   = SCREEN_LIVE;
static int           s_menu_sel = 0;
static device_mode_t s_mode     = DEVICE_MODE_ROVER;
static int           s_mode_sel = 0;    /* cursor in SCREEN_MODE_SELECT: 0=ROVER 1=BASE 2=Back */
static int           s_clear_sel = 1;   /* cursor in SCREEN_CLEAR_CONFIRM: 0=Yes 1=No — defaults to No */

/* ── Shared helpers ──────────────────────────────────────────────────── */

static const char *fix_label(uint8_t fix_type, uint8_t carr_soln, uint8_t quality)
{
    if (fix_type == 0)  return "NO FIX  ";
    if (fix_type == 1)  return "DEAD REC";
    if (carr_soln == 2) return "RTK FIX ";
    if (carr_soln == 1) return "RTK FLT ";
    if (quality  == 2)  return "DGPS    ";
    return "3D      ";
}

static const char *mode_str(device_mode_t m)
{
    return m == DEVICE_MODE_BASE ? "BASE" : "ROVER";
}

/* ── Screen renderers ────────────────────────────────────────────────── */

static void render_rover_live(const gnss_pvt_t *pvt, const ups_data_t *ups,
                              uint32_t lora_rx_b)
{
    display_printf(0, 0, "%-8s SV:%-2u",
                   fix_label(pvt->fix_type, pvt->carr_soln, pvt->gga_quality),
                   pvt->num_sv);

    display_printf(0, 1, "%.7f%c",
                   pvt->lat >= 0 ?  pvt->lat : -pvt->lat,
                   pvt->lat >= 0 ? 'N' : 'S');
    display_printf(0, 2, "%.7f%c",
                   pvt->lon >= 0 ?  pvt->lon : -pvt->lon,
                   pvt->lon >= 0 ? 'E' : 'W');

    display_printf(0, 3, "Alt:%7.2fm hA:%.3f", pvt->alt_msl, pvt->h_acc);

    {
        bool     arp = gnss_rtcm_arp_seen();
        uint32_t kb  = lora_rx_b / 1024;
        if (pvt->diff_age_s > 0.0f)
            display_printf(0, 4, "ARP:%c Age:%4.1fs %3lukB",
                           arp ? 'Y' : 'N', pvt->diff_age_s, (unsigned long)kb);
        else
            display_printf(0, 4, "ARP:%c Age:  --s %3lukB",
                           arp ? 'Y' : 'N', (unsigned long)kb);
    }

    display_printf(0, 5, "PDOP:%-4.1f vAcc:%.3fm", pvt->pdop, pvt->v_acc);

    if (ups && ups->present)
        display_printf(0, 6, "%4.1fV %3d%% %+5.1fA %s",
                       ups->bus_voltage_V, ups->percent,
                       ups->current_mA / 1000.0f,
                       ups->charging ? "CHG" : "DSC");
    else
        display_puts(0, 6, "UPS:---             ");
}

static void render_base_live(const ups_data_t *ups)
{
    base_state_t bst = base_app_get_state();
    gnss_svin_t  sv  = base_app_get_svin();
    uint32_t frames, arpcnt, errors;
    base_app_get_rtcm_stats(&frames, &arpcnt, &errors);

    if (bst == BASE_STATE_SURVEY_IN)
        display_puts(0, 0, "BASE: SURVEY-IN      ");
    else if (bst == BASE_STATE_BROADCASTING)
        display_puts(0, 0, "BASE: BROADCASTING   ");
    else
        display_puts(0, 0, "BASE: INIT           ");

    display_printf(0, 1, "T:%4lus  Acc:%6.2fm  ",
                   (unsigned long)sv.dur_s, sv.mean_acc_m);
    if (bst == BASE_STATE_SURVEY_IN)
        display_puts(0, 2, sv.valid ? "CONVERGED            "
                                    : "ACTIVE...            ");
    else if (bst == BASE_STATE_BROADCASTING)
        display_printf(0, 2, "ARP:%-17s",
                       arpcnt > 0 ? "YES" : "waiting...");
    else
        display_puts(0, 2, "                     ");

    display_printf(0, 3, "RTCM:%6lu  Err:%4lu",
                   (unsigned long)frames, (unsigned long)errors);
    display_printf(0, 4, "1005:%6lu ARP:%-3s",
                   (unsigned long)arpcnt, arpcnt > 0 ? "YES" : "NO");

    if (ups && ups->present)
        display_printf(0, 5, "%4.1fV %3d%% %+5.1fA %s",
                       ups->bus_voltage_V, ups->percent,
                       ups->current_mA / 1000.0f,
                       ups->charging ? "CHG" : "DSC");
    else
        display_puts(0, 5, "UPS:---              ");

    display_printf(0, 6, "%-5s          [menu]", mode_str(s_mode));
}

static void render_menu(void)
{
    const menu_item_t *menu = cur_menu();
    int                cnt  = cur_menu_cnt();
    display_puts(0, 0, "      MENU           ");
    /* Rows 1..6 are items; scroll once the list is longer than that. */
    const int visible = DISPLAY_ROWS - 1;
    int top = s_menu_sel < visible ? 0 : s_menu_sel - visible + 1;
    for (int i = top; i < cnt && i < top + visible; i++)
        display_printf(0, 1 + i - top, "%c %-19s",
                       i == s_menu_sel ? '>' : ' ', menu[i].label);
}

static void render_fix_detail(const gnss_pvt_t *pvt)
{
    static const char *rtk[] = { "none", "Float", "Fixed" };
    uint8_t cs = pvt->carr_soln <= 2 ? pvt->carr_soln : 0;

    display_puts  (0, 0, "  FIX DETAIL         ");
    display_printf(0, 1, "Status:%-14s", fix_label(pvt->fix_type, pvt->carr_soln, pvt->gga_quality));
    display_printf(0, 2, "RTK:   %-14s",  rtk[cs]);
    display_printf(0, 3, "SV:    %-14u",  pvt->num_sv);
    display_printf(0, 4, "hAcc:  %.3f m", pvt->h_acc);
    display_printf(0, 5, "vAcc:  %.3f m", pvt->v_acc);
    display_printf(0, 6, "PDOP:  %-14.1f", pvt->pdop);
}

/* ── Level (bubble) screen ───────────────────────────────────────────── */

#define LEVEL_CX            32      /* bubble vial centre, px */
#define LEVEL_CY            32
#define LEVEL_R             30      /* vial radius, px */
#define LEVEL_FULLSCALE_DEG 8.0f    /* tilt at the vial's edge */
#define LEVEL_TOL_DEG       2.0f    /* "level enough" ring */
#define BUBBLE_R            3

/* Map IMU axes onto screen axes. Measured 2026-10-06 with the device facing the
 * operator: lifting the left edge moved the bubble up with the first mapping,
 * so IMU +Y runs to the operator's left and +X away from them (right-handed,
 * +Z up). The bubble should move toward the raised edge as seen from above:
 * screen-right = operator's right, screen-up = away. */
#define LEVEL_SWAP_XY 1
#define LEVEL_FLIP_X  1
#define LEVEL_FLIP_Y  0

/* Pole length, antenna ARP to ground tip. Loaded from NVS at ui_init and edited
 * from the Pole length screen. */
static float s_pole_len_m = 2.0f;

static int tilt_to_px(float lean)
{
    const float px_per_unit = (float)LEVEL_R / sinf(LEVEL_FULLSCALE_DEG * (float)M_PI / 180.0f);
    return (int)lroundf(lean * px_per_unit);
}

static void render_level(const imu_tilt_t *t)
{
    float ox, oy;
    imu_get_level_offset(&ox, &oy);
    display_puts(12, 0, (ox == 0.0f && oy == 0.0f) ? "TILT RAW" : "TILT");
    if (!t) {
        display_puts(12, 2, "IMU n/a");
        display_puts(12, 6, "[exit]");
        return;
    }

    /* Vial: outer edge, tolerance ring, dotted crosshair. */
    display_circle(LEVEL_CX, LEVEL_CY, LEVEL_R);
    display_circle(LEVEL_CX, LEVEL_CY,
                   tilt_to_px(sinf(LEVEL_TOL_DEG * (float)M_PI / 180.0f)));
    for (int i = -LEVEL_R + 2; i <= LEVEL_R - 2; i += 3) {
        display_pixel(LEVEL_CX + i, LEVEL_CY);
        display_pixel(LEVEL_CX, LEVEL_CY + i);
    }

    float lx = LEVEL_SWAP_XY ? t->lean_y : t->lean_x;
    float ly = LEVEL_SWAP_XY ? t->lean_x : t->lean_y;
    if (LEVEL_FLIP_X) lx = -lx;
    if (LEVEL_FLIP_Y) ly = -ly;

    int dx =  tilt_to_px(lx);
    int dy = -tilt_to_px(ly);
    float len = sqrtf((float)(dx * dx + dy * dy));
    float max = (float)(LEVEL_R - BUBBLE_R - 1);
    if (len > max) {
        dx = (int)lroundf(dx * max / len);
        dy = (int)lroundf(dy * max / len);
    }
    display_disc(LEVEL_CX + dx, LEVEL_CY + dy, BUBBLE_R);

    float err_cm = s_pole_len_m * sinf(t->tilt_deg * (float)M_PI / 180.0f) * 100.0f;
    display_printf(12, 1, "%4.1f deg", t->tilt_deg);
    display_printf(12, 2, "x:%+5.1f", asinf(t->lean_x) * (180.0f / (float)M_PI));
    display_printf(12, 3, "y:%+5.1f", asinf(t->lean_y) * (180.0f / (float)M_PI));
    display_printf(12, 4, "Err %3.0fcm", fabsf(err_cm));
    display_puts  (12, 5, !t->steady                  ? "MOVING"
                        : t->tilt_deg <= LEVEL_TOL_DEG ? "LEVEL"
                                                       : "TILTED");
    display_puts  (12, 6, "[exit]");
}

/* ── Level calibration screen ────────────────────────────────────────── */

typedef enum { CAL_PROMPT, CAL_SAMPLING, CAL_RESULT } cal_phase_t;

static imu_cal_t         s_cal;
static cal_phase_t       s_cal_phase;
static bool              s_cal_moved;     /* last attempt was restarted */
static imu_cal_verdict_t s_cal_verdict;
static imu_cal_result_t  s_cal_res;

static float lean_deg(float lean)
{
    return asinf(lean) * (180.0f / (float)M_PI);
}

static void render_cal(void)
{
    switch (s_cal_phase) {
    case CAL_PROMPT:
        display_puts  (0, 0, "LEVEL CALIBRATION");
        display_printf(0, 1, "Position %d of %d", s_cal.done + 1, IMU_CAL_POSITIONS);
        display_puts  (0, 2, "Rest flat and still");
        display_puts  (0, 3, s_cal.done == 0 ? "on a steady surface"
                                             : "Turn 90 deg from last");
        if (s_cal_moved)
            display_puts(0, 4, "Moved - try again");
        display_puts  (0, 5, "[press: start]");
        display_puts  (0, 6, "[turn: cancel]");
        break;

    case CAL_SAMPLING: {
        display_puts  (0, 0, "LEVEL CALIBRATION");
        display_printf(0, 1, "Position %d of %d", s_cal.done + 1, IMU_CAL_POSITIONS);
        display_puts  (0, 2, "Hold still...");
        int  pct  = imu_cal_progress_pct(&s_cal);
        char bar[22];
        int  fill = pct / 10;
        for (int i = 0; i < 10; i++) bar[i] = i < fill ? '#' : '.';
        bar[10] = '\0';
        display_printf(0, 3, "[%s] %3d%%", bar, pct);
        display_puts  (0, 6, "[turn: cancel]");
        break;
    }

    case CAL_RESULT:
        display_puts(0, 0, "CAL RESULT");
        if (s_cal_verdict == IMU_CAL_OK) {
            display_printf(0, 1, "Offset x:%+.2f deg", lean_deg(s_cal_res.off_x));
            display_printf(0, 2, "       y:%+.2f deg", lean_deg(s_cal_res.off_y));
            display_printf(0, 3, "Surface tilt %.1f deg", s_cal_res.table_deg);
            display_printf(0, 4, "Agreement %.2f deg", s_cal_res.resid_deg);
            display_puts  (0, 5, "[press: save]");
            display_puts  (0, 6, "[turn: discard]");
        } else {
            display_puts  (0, 1, s_cal_verdict == IMU_CAL_OFFSET_TOO_LARGE
                                 ? "Offset too large" : "Positions disagree");
            display_printf(0, 2, "(%.2f deg apart)", s_cal_res.resid_deg);
            display_puts  (0, 3, "Retry on a steady");
            display_puts  (0, 4, "surface, no bumps");
            display_puts  (0, 6, "[press: back]");
        }
        break;
    }
}

/* Returns true when the screen should return to the menu. */
static bool tick_cal(int steps, bool press, const imu_tilt_t *tilt)
{
    switch (s_cal_phase) {
    case CAL_PROMPT:
        if (steps != 0) return true;
        if (press) {
            imu_cal_begin_position(&s_cal);
            s_cal_moved = false;
            s_cal_phase = CAL_SAMPLING;
        }
        break;

    case CAL_SAMPLING:
        if (steps != 0) return true;
        if (!tilt) break;
        switch (imu_cal_feed(&s_cal, tilt)) {
        case IMU_CAL_SAMPLING:
            break;
        case IMU_CAL_MOVED:
            s_cal_moved = true;
            s_cal_phase = CAL_PROMPT;
            break;
        case IMU_CAL_POSITION_DONE:
            s_cal_moved = false;
            s_cal_phase = CAL_PROMPT;
            break;
        case IMU_CAL_ALL_DONE:
            s_cal_verdict = imu_cal_result(&s_cal, &s_cal_res);
            s_cal_phase   = CAL_RESULT;
            break;
        }
        break;

    case CAL_RESULT:
        if (steps != 0) return true;
        if (press) {
            if (s_cal_verdict == IMU_CAL_OK) {
                imu_set_level_offset(s_cal_res.off_x, s_cal_res.off_y);
                config_set_level_offset(s_cal_res.off_x, s_cal_res.off_y);
            }
            return true;
        }
        break;
    }
    return false;
}

/* ── Pole length screen ──────────────────────────────────────────────── */

#define POLE_MIN_MM   500
#define POLE_MAX_MM  4000
#define POLE_STEP_MM   10

static int s_pole_edit_mm;

static void render_pole(void)
{
    display_puts  (0, 0, "   POLE LENGTH       ");
    display_puts  (0, 1, "  antenna to ground  ");
    display_printf(0, 3, "      %d.%02d m", s_pole_edit_mm / 1000, (s_pole_edit_mm % 1000) / 10);
    display_puts  (0, 5, "[turn: +/- 1 cm]");
    display_puts  (0, 6, "[press: save]");
}

static void render_mode_select(void)
{
    display_puts  (0, 0, "   DEVICE MODE       ");
    display_printf(0, 1, "   Current: %-9s", mode_str(s_mode));
    display_puts  (0, 2, "                     ");
    display_printf(0, 3, " %c ROVER             ", s_mode_sel == 0 ? '>' : ' ');
    display_printf(0, 4, " %c BASE STATION      ", s_mode_sel == 1 ? '>' : ' ');
    display_printf(0, 5, " %c Back              ", s_mode_sel == 2 ? '>' : ' ');
    display_puts  (0, 6, "   [press: select]   ");
}

static void render_clear_confirm(void)
{
    display_puts  (0, 0, "   CLEAR LOGS?       ");
    display_puts  (0, 1, "  Deletes all .CSV   ");
    display_puts  (0, 2, "  and .LOG files     ");
    display_puts  (0, 3, "                     ");
    display_printf(0, 4, " %c Yes, delete       ", s_clear_sel == 0 ? '>' : ' ');
    display_printf(0, 5, " %c No, cancel        ", s_clear_sel == 1 ? '>' : ' ');
    display_puts  (0, 6, "   [press: select]   ");
}

/* ── Public API ──────────────────────────────────────────────────────── */

void ui_init(device_mode_t mode)
{
    s_mode     = mode;
    s_pole_len_m = config_get_pole_len_m();
    s_screen   = SCREEN_LIVE;
    s_menu_sel = 0;
}

bool ui_wants_fast_refresh(void)
{
    return s_screen == SCREEN_LEVEL || s_screen == SCREEN_CAL;
}

void ui_tick(const gnss_pvt_t *pvt, const ups_data_t *ups, uint32_t lora_rx_b,
             const imu_tilt_t *tilt)
{
    int  steps = encoder_get_steps();
    bool press = encoder_get_press();

    switch (s_screen) {
    case SCREEN_LIVE:
        if (press) { s_screen = SCREEN_MENU; s_menu_sel = 0; }
        break;

    case SCREEN_MENU: {
        int cnt = cur_menu_cnt();
        if (steps > 0)
            s_menu_sel = (s_menu_sel + 1) % cnt;
        else if (steps < 0)
            s_menu_sel = (s_menu_sel - 1 + cnt) % cnt;
        if (press) {
            screen_t target = cur_menu()[s_menu_sel].target;
            if (target == SCREEN_MODE_SELECT)
                s_mode_sel = (s_mode == DEVICE_MODE_BASE) ? 1 : 0;
            else if (target == SCREEN_CLEAR_CONFIRM)
                s_clear_sel = 1;   /* always default to No */
            else if (target == SCREEN_CAL) {
                imu_cal_start(&s_cal);
                s_cal_phase = CAL_PROMPT;
                s_cal_moved = false;
            } else if (target == SCREEN_POLE)
                s_pole_edit_mm = (int)lroundf(s_pole_len_m * 1000.0f);
            s_screen = target;
        }
        break;
    }

    case SCREEN_FIX_DETAIL:
    case SCREEN_LEVEL:
        if (press) s_screen = SCREEN_MENU;
        break;

    case SCREEN_CAL:
        if (tick_cal(steps, press, tilt)) s_screen = SCREEN_MENU;
        break;

    case SCREEN_POLE:
        s_pole_edit_mm += steps * POLE_STEP_MM;
        if (s_pole_edit_mm < POLE_MIN_MM) s_pole_edit_mm = POLE_MIN_MM;
        if (s_pole_edit_mm > POLE_MAX_MM) s_pole_edit_mm = POLE_MAX_MM;
        if (press) {
            s_pole_len_m = s_pole_edit_mm * 0.001f;
            config_set_pole_len_m(s_pole_len_m);
            s_screen = SCREEN_MENU;
        }
        break;

    case SCREEN_CLEAR_CONFIRM:
        if (steps != 0)
            s_clear_sel = (s_clear_sel + (steps > 0 ? 1 : -1) + 2) % 2;
        if (press) {
            if (s_clear_sel == 0) {
                int n = sdlog_clear_all();
                display_clear();
                if (n >= 0)
                    display_printf(0, 3, "  Cleared %d file%s   ", n, n == 1 ? "" : "s");
                else
                    display_puts(0, 3, "   Clear failed!     ");
                display_flush();
                vTaskDelay(pdMS_TO_TICKS(1200));
            }
            s_screen = SCREEN_MENU;
        }
        break;

    case SCREEN_MODE_SELECT:
        if (steps != 0)
            s_mode_sel = (s_mode_sel + (steps > 0 ? 1 : -1) + 3) % 3;
        if (press) {
            if (s_mode_sel == 2) {
                s_screen = SCREEN_MENU;
            } else {
                device_mode_t new_mode = (s_mode_sel == 0) ? DEVICE_MODE_ROVER
                                                            : DEVICE_MODE_BASE;
                if (new_mode == s_mode) {
                    s_screen = SCREEN_MENU;
                } else {
                    config_set_device_mode(new_mode);
                    display_clear();
                    display_puts(0, 3, "      Saved!         ");
                    display_puts(0, 4, "    Rebooting...     ");
                    display_flush();
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    esp_restart();
                }
            }
        }
        break;
    }

    display_clear();
    switch (s_screen) {
    case SCREEN_LIVE:
        if (s_mode == DEVICE_MODE_BASE)
            render_base_live(ups);
        else
            render_rover_live(pvt, ups, lora_rx_b);
        break;
    case SCREEN_MENU:          render_menu();           break;
    case SCREEN_FIX_DETAIL:    render_fix_detail(pvt);  break;
    case SCREEN_LEVEL:         render_level(tilt);      break;
    case SCREEN_CAL:           render_cal();            break;
    case SCREEN_POLE:          render_pole();           break;
    case SCREEN_MODE_SELECT:   render_mode_select();    break;
    case SCREEN_CLEAR_CONFIRM: render_clear_confirm();  break;
    }
    display_flush();
}

device_mode_t ui_select_mode_blocking(void)
{
    int sel = 0;

    for (;;) {
        display_clear();
        display_puts  (0, 0, "   SELECT MODE       ");
        display_puts  (0, 1, "   First boot:       ");
        display_puts  (0, 2, "   choose a role     ");
        display_puts  (0, 3, "                     ");
        display_printf(0, 4, " %c ROVER             ", sel == 0 ? '>' : ' ');
        display_printf(0, 5, " %c BASE STATION      ", sel == 1 ? '>' : ' ');
        display_puts  (0, 6, "   [press: confirm]  ");
        display_flush();

        vTaskDelay(pdMS_TO_TICKS(50));

        int  steps = encoder_get_steps();
        bool press = encoder_get_press();

        if (steps != 0)
            sel = (sel + (steps > 0 ? 1 : -1) + 2) % 2;
        if (press) {
            device_mode_t mode = (sel == 0) ? DEVICE_MODE_ROVER : DEVICE_MODE_BASE;
            config_set_device_mode(mode);
            display_clear();
            display_printf(0, 3, "  %-19s",
                           sel == 0 ? "Mode: ROVER" : "Mode: BASE STATION");
            display_puts  (0, 4, "      Saved!         ");
            display_flush();
            vTaskDelay(pdMS_TO_TICKS(800));
            return mode;
        }
    }
}
