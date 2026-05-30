#include "ui.h"
#include "display.h"
#include "encoder.h"

/* ── Screen definitions ──────────────────────────────────────────────── */

typedef enum {
    SCREEN_LIVE,
    SCREEN_MENU,
    SCREEN_FIX_DETAIL,
} screen_t;

typedef struct {
    const char *label;
    screen_t    target;
} menu_item_t;

static const menu_item_t s_menu[] = {
    { "Fix detail", SCREEN_FIX_DETAIL },
    { "Exit",       SCREEN_LIVE       },
};
#define MENU_COUNT ((int)(sizeof(s_menu) / sizeof(s_menu[0])))

static screen_t s_screen   = SCREEN_LIVE;
static int      s_menu_sel = 0;
static bool     s_imu_ok   = false;

/* ── Shared helpers ──────────────────────────────────────────────────── */

static const char *fix_label(uint8_t fix_type, uint8_t carr_soln)
{
    if (fix_type == 0)  return "NO FIX  ";
    if (fix_type == 1)  return "DEAD REC";
    if (carr_soln == 2) return "RTK FIX ";
    if (carr_soln == 1) return "RTK FLT ";
    return "3D      ";
}

/* ── Screen renderers ────────────────────────────────────────────────── */

static void render_live(const gnss_pvt_t *pvt, const imu_data_t *imu)
{
    /* Row 0: fix status + satellite count (21 chars: "%-8s SV:%-2u") */
    display_printf(0, 0, "%-8s SV:%-2u",
                   fix_label(pvt->fix_type, pvt->carr_soln), pvt->num_sv);

    /* Rows 1–2: lat/lon */
    display_printf(0, 1, "%.7f%c",
                   pvt->lat >= 0 ?  pvt->lat : -pvt->lat,
                   pvt->lat >= 0 ? 'N' : 'S');
    display_printf(0, 2, "%.7f%c",
                   pvt->lon >= 0 ?  pvt->lon : -pvt->lon,
                   pvt->lon >= 0 ? 'E' : 'W');

    /* Row 3: altitude + hAcc (21 chars: "Alt:%7.2fm hA:%.3f") */
    display_printf(0, 3, "Alt:%7.2fm hA:%.3f", pvt->alt_msl, pvt->h_acc);

    /* Row 4: heading + IMU calibration */
    if (s_imu_ok && imu->calibrated)
        display_printf(0, 4, "HDG:%5.1f  Cal:%u/3",
                       imu->heading, imu->sys_cal);
    else
        display_puts(0, 4, "HDG:---.-- Cal:--");

    /* Row 5: pitch / roll */
    display_printf(0, 5, "P:%+6.1f  R:%+6.1f", imu->pitch, imu->roll);

    /* Row 6: PDOP */
    display_printf(0, 6, "PDOP:%.1f", pvt->pdop);

    /* Row 7: hint */
    display_puts(0, 7, "[press for menu]");
}

static void render_menu(void)
{
    display_puts(0, 0, "      MENU      ");
    for (int i = 0; i < MENU_COUNT; i++)
        display_printf(0, 1 + i, "%c %-18s",
                       i == s_menu_sel ? '>' : ' ', s_menu[i].label);
}

static void render_fix_detail(const gnss_pvt_t *pvt)
{
    static const char *rtk[] = { "none", "Float", "Fixed" };
    uint8_t cs = pvt->carr_soln <= 2 ? pvt->carr_soln : 0;

    /* Each label is 7 chars; %-14s fills remaining 14 → 21 total */
    display_puts  (0, 0, "  FIX DETAIL    ");
    display_printf(0, 1, "Status:%-14s", fix_label(pvt->fix_type, pvt->carr_soln));
    display_printf(0, 2, "RTK:   %-14s",  rtk[cs]);
    display_printf(0, 3, "SV:    %-14u",  pvt->num_sv);
    display_printf(0, 4, "hAcc:  %.3f m", pvt->h_acc);
    display_printf(0, 5, "vAcc:  %.3f m", pvt->v_acc);
    display_printf(0, 6, "PDOP:  %-14.1f", pvt->pdop);
    display_puts  (0, 7, " [press: back]  ");
}

/* ── Public API ──────────────────────────────────────────────────────── */

void ui_init(bool imu_available)
{
    s_imu_ok   = imu_available;
    s_screen   = SCREEN_LIVE;
    s_menu_sel = 0;
}

void ui_tick(const gnss_pvt_t *pvt, const imu_data_t *imu)
{
    int  steps = encoder_get_steps();
    bool press = encoder_get_press();

    switch (s_screen) {
    case SCREEN_LIVE:
        if (press) { s_screen = SCREEN_MENU; s_menu_sel = 0; }
        break;

    case SCREEN_MENU:
        if (steps > 0)
            s_menu_sel = (s_menu_sel + 1) % MENU_COUNT;
        else if (steps < 0)
            s_menu_sel = (s_menu_sel - 1 + MENU_COUNT) % MENU_COUNT;
        if (press)
            s_screen = s_menu[s_menu_sel].target;
        break;

    case SCREEN_FIX_DETAIL:
        if (press) s_screen = SCREEN_MENU;
        break;
    }

    display_clear();
    switch (s_screen) {
    case SCREEN_LIVE:       render_live(pvt, imu);   break;
    case SCREEN_MENU:       render_menu();            break;
    case SCREEN_FIX_DETAIL: render_fix_detail(pvt);  break;
    }
    display_flush();
}
