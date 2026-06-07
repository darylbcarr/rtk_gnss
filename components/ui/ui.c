#include "ui.h"
#include "config.h"
#include "display.h"
#include "encoder.h"
#include "ups.h"
#include "base_app.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* ── Screen definitions ──────────────────────────────────────────────── */

typedef enum {
    SCREEN_LIVE,
    SCREEN_MENU,
    SCREEN_FIX_DETAIL,
    SCREEN_MODE_SELECT,
} screen_t;

typedef struct {
    const char *label;
    screen_t    target;
} menu_item_t;

/* Forward declaration — defined with other statics below. */
static device_mode_t s_mode;

/* Rover has a "Fix detail" entry; Base omits it (no PVT in base mode). */
static const menu_item_t s_rover_menu[] = {
    { "Fix detail",  SCREEN_FIX_DETAIL  },
    { "Device mode", SCREEN_MODE_SELECT },
    { "Exit",        SCREEN_LIVE        },
};
static const menu_item_t s_base_menu[] = {
    { "Device mode", SCREEN_MODE_SELECT },
    { "Exit",        SCREEN_LIVE        },
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
static bool          s_imu_ok   = false;
static device_mode_t s_mode     = DEVICE_MODE_ROVER;
static int           s_mode_sel = 0;    /* cursor in SCREEN_MODE_SELECT: 0=ROVER 1=BASE */

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

static void render_rover_live(const gnss_pvt_t *pvt, const imu_data_t *imu,
                              const ups_data_t *ups)
{
    display_printf(0, 0, "%-8s SV:%-2u",
                   fix_label(pvt->fix_type, pvt->carr_soln, pvt->gga_quality), pvt->num_sv);
    display_printf(0, 1, "%.7f%c",
                   pvt->lat >= 0 ?  pvt->lat : -pvt->lat,
                   pvt->lat >= 0 ? 'N' : 'S');
    display_printf(0, 2, "%.7f%c",
                   pvt->lon >= 0 ?  pvt->lon : -pvt->lon,
                   pvt->lon >= 0 ? 'E' : 'W');
    display_printf(0, 3, "Alt:%7.2fm hA:%.3f", pvt->alt_msl, pvt->h_acc);
    if (s_imu_ok && imu->calibrated)
        display_printf(0, 4, "HDG:%5.1f  Cal:%u/3",
                       imu->heading, imu->sys_cal);
    else
        display_puts(0, 4, "HDG:---.-- Cal:--");
    display_printf(0, 5, "PDOP:%-4.1f  P:%+5.1f", pvt->pdop, imu->pitch);

    if (ups && ups->present)
        display_printf(0, 6, "%4.1fV %3d%% %+5.1fA %s",
                       ups->bus_voltage_V, ups->percent,
                       ups->current_mA / 1000.0f,
                       ups->charging ? "CHG" : "DSC");
    else
        display_puts(0, 6, "UPS:---             ");

    display_printf(0, 7, "%-5s          [menu]", mode_str(s_mode));
}

static void render_base_live(const ups_data_t *ups)
{
    base_state_t bst = base_app_get_state();
    gnss_svin_t  sv  = base_app_get_svin();

    if (bst == BASE_STATE_SURVEY_IN) {
        display_puts  (0, 0, "BASE: SURVEY-IN      ");
        display_printf(0, 1, "T:%4lus  Acc:%.2fm   ",
                       (unsigned long)sv.dur_s, sv.mean_acc_m);
        display_puts  (0, 2, sv.valid ? "Status: CONVERGED    "
                                      : "Status: ACTIVE...    ");
    } else if (bst == BASE_STATE_BROADCASTING) {
        display_puts(0, 0, "BASE: BROADCAST      ");
        display_puts(0, 1, "RTCM streaming       ");
        display_puts(0, 2, "                     ");
    } else {
        display_puts(0, 0, "BASE: INITIALIZING   ");
        display_puts(0, 1, "                     ");
        display_puts(0, 2, "                     ");
    }

    display_puts(0, 3, "                     ");

    if (ups && ups->present)
        display_printf(0, 4, "%4.1fV %3d%% %+5.1fA %s",
                       ups->bus_voltage_V, ups->percent,
                       ups->current_mA / 1000.0f,
                       ups->charging ? "CHG" : "DSC");
    else
        display_puts(0, 4, "UPS:---              ");

    display_puts(0, 5, "                     ");
    display_puts(0, 6, "                     ");
    display_printf(0, 7, "%-5s          [menu]", mode_str(s_mode));
}

static void render_menu(void)
{
    const menu_item_t *menu = cur_menu();
    int                cnt  = cur_menu_cnt();
    display_puts(0, 0, "      MENU           ");
    for (int i = 0; i < cnt; i++)
        display_printf(0, 1 + i, "%c %-19s",
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
    display_puts  (0, 7, " [press: back]       ");
}

static void render_mode_select(void)
{
    display_puts  (0, 0, "   DEVICE MODE       ");
    display_printf(0, 1, "   Current: %-9s", mode_str(s_mode));
    display_puts  (0, 2, "                     ");
    display_printf(0, 3, " %c ROVER             ", s_mode_sel == 0 ? '>' : ' ');
    display_printf(0, 4, " %c BASE STATION      ", s_mode_sel == 1 ? '>' : ' ');
    display_puts  (0, 5, "                     ");
    display_puts  (0, 6, "                     ");
    display_puts  (0, 7, "   [press: select]   ");
}

/* ── Public API ──────────────────────────────────────────────────────── */

void ui_init(bool imu_available, device_mode_t mode)
{
    s_imu_ok   = imu_available;
    s_mode     = mode;
    s_screen   = SCREEN_LIVE;
    s_menu_sel = 0;
}

void ui_tick(const gnss_pvt_t *pvt, const imu_data_t *imu, const ups_data_t *ups)
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
            s_screen = target;
        }
        break;
    }

    case SCREEN_FIX_DETAIL:
        if (press) s_screen = SCREEN_MENU;
        break;

    case SCREEN_MODE_SELECT:
        if (steps != 0)
            s_mode_sel = (s_mode_sel + (steps > 0 ? 1 : -1) + 2) % 2;
        if (press) {
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
        break;
    }

    display_clear();
    switch (s_screen) {
    case SCREEN_LIVE:
        if (s_mode == DEVICE_MODE_BASE)
            render_base_live(ups);
        else
            render_rover_live(pvt, imu, ups);
        break;
    case SCREEN_MENU:        render_menu();           break;
    case SCREEN_FIX_DETAIL:  render_fix_detail(pvt); break;
    case SCREEN_MODE_SELECT: render_mode_select();    break;
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
        display_puts  (0, 6, "                     ");
        display_puts  (0, 7, "   [press: confirm]  ");
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
