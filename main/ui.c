#include "ui.h"

#include <stdio.h>
#include <string.h>
#include "lvgl.h"
#include <ctype.h>
#include <math.h>
#include <time.h>
#include "airlines.h"
#include "faflight.h"
#include "fonts.h"
#include "lang.h"
#include "settings.h"
#include "clock_fmt.h"
#include "geo_math.h"
#include "logos.h"
#include "flags.h"
#include "esp_timer.h"
#include "runways.h"
#include "airports.h"
#include "flight_task.h"
#include "flight_data.h"
#include "regcountry.h"
#include "routes.h"
#include "units.h"
#include "airspace.h"
#include "tilemap.h"
#include "rainviewer.h"
#include "trails.h"
#include "tz.h"
#include "ui_settings.h"
#include "mqtt_pub.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <assert.h>
#include "esp_heap_caps.h"
#include "lvgl_port.h"
#include "esp_log.h"
#include "waveshare_rgb_lcd_port.h"

#define LIST_W        UISX(310)
#define HEADER_H      UISY(48)
#define MAX_SHOWN     40

/* The panel is a fixed 800x480; the app build may register a larger LVGL
 * display (tablets), so structural sizes come from the live display. On the
 * device these evaluate to exactly 800/480. */
#define SCR_W  LV_HOR_RES
#define SCR_H  LV_VER_RES
/* Design height of the hand-composed right-panel blocks (detail/stats);
 * extra vertical space beyond it is distributed at runtime. */
#define DSN_H  UISY(432)
/* Inner content width of the right panel (16px side padding). Stretchable
 * elements (bars, grids, charts) span it; on the device it is 458. */
#define DTL_W  (SCR_W - LIST_W - UISX(32))
/* Vertical surplus of the right panel over the 432px design; the detail
 * sections spread it out evenly (all zero on the device). */
#define DTL_VEXT (SCR_H - HEADER_H - DSN_H)
#define DTL_Y1 (DTL_VEXT / 4)          /* route block */
#define DTL_Y2 (DTL_VEXT / 2)          /* progress block */
#define DTL_Y3 (DTL_VEXT * 3 / 4)      /* stats grid */

#include "theme.h"

static const char *TAG = "ui";

LV_IMG_DECLARE(img_plane);
LV_IMG_DECLARE(img_heli);
LV_IMG_DECLARE(img_small);
LV_IMG_DECLARE(img_mil);
LV_IMG_DECLARE(img_glider);
LV_IMG_DECLARE(img_balloon);
LV_IMG_DECLARE(img_drone);

/* map sprite per flight_sprite_t id */
static const lv_img_dsc_t *class_sprite(int spr)
{
    switch (spr) {
    case FSPR_SMALL:   return &img_small;
    case FSPR_HELI:    return &img_heli;
    case FSPR_MIL:     return &img_mil;
    case FSPR_GLIDER:  return &img_glider;
    case FSPR_BALLOON: return &img_balloon;
    case FSPR_DRONE:   return &img_drone;
    default:           return &img_plane;
    }
}

#define COL_BG        (app_theme()->bg)
#define COL_PANEL     (app_theme()->panel)
#define COL_ROW       (app_theme()->row)
#define COL_ROW_SEL   (app_theme()->row_sel)
#define COL_ACCENT    (app_theme()->accent)
#define COL_TEXT      (app_theme()->text)
#define COL_DIM       (app_theme()->dim)

typedef struct {
    aircraft_t   ac;
    route_info_t route;      /* route.callsign[0] == 0 -> no route snapshot */
    char         airline[NAME_LEN];
    char         iata[10];   /* commercial flight number, when known */
} shown_flight_t;

/* ~26 KB with routes and airline strings: no DMA/ISR involvement, so it
 * lives in PSRAM instead of the scarce internal .bss (allocated in
 * ui_init before anything renders). */
static shown_flight_t *s_shown;
static int  s_shown_count;
static int  s_selected = -1;
static char s_selected_hex[ICAO_HEX_LEN];

static lv_obj_t *s_status_label;
static lv_obj_t *s_weather_label;
static lv_obj_t *s_list_panel;
static lv_obj_t *s_list_rows[MAX_SHOWN];
static int s_list_plane_rows;    /* rows currently showing aircraft */
static int s_row_plane_idx[MAX_SHOWN];   /* row -> s_shown index */

/* Right-panel view modes */
/* 7B edition: the local map is the only panel. The view machinery is kept
 * as a single case rather than removed outright so the mode button, the
 * auto-cycle timer and ui_set_view() keep their existing shapes. */
#define VIEW_RADAR  0
#define VIEW_COUNT  1
#define CYCLE_MS    6000
static int        s_view_mode;
static lv_timer_t *s_cycle_timer;

/* Every aircraft the map may draw, flattened out of the poll result. */
typedef struct {
    char  callsign[9];
    float lat, lon, track;
    float dist_nm, dir_deg;
    int   alt_ft;
    bool  ground;
    uint8_t fcls;    /* flight_class_t for the marker sprite */
} map_target_t;
static map_target_t s_all[MAX_AIRCRAFT];
static int s_all_count;

static char s_weather_txt[96];
static bool s_bl_off;

/* Re-assert the configured brightness after a full-on: bl_on() drives the
 * panel to maximum on every board. No-op at 100%% or on on/off-only
 * backlights (CH422G). Exposed for the settings slider and input actions. */
void ui_apply_brightness(void)
{
    const settings_t *c = settings_get();
    /* Upstream applies this only below 100 %, so a board left dim could
     * never be driven back to full. With a schedule that is exactly the
     * morning transition, so the whole 5..100 range is applied here. */
    if (c->brightness_ctl && waveshare_rgb_lcd_bl_dimmable()) {
        waveshare_rgb_lcd_bl_pct(c->brightness);
    }
}

static lv_obj_t *s_stats_panel;
static lv_obj_t *s_sv_vals[4];
static lv_obj_t *s_sv_chart;
static lv_chart_series_t *s_sv_series;
static lv_obj_t *s_sv_top[8];
static lv_obj_t *s_sv_metar;
static lv_obj_t *s_sv_days;
static app_stats_t s_stats_snap;
static lv_obj_t *s_mb_logo, *s_mb_callsign, *s_mb_type, *s_mb_route, *s_mb_stats, *s_mb_bar;
static lv_obj_t *s_mode_btn_label;
static lv_obj_t *s_clock_label;
static lv_obj_t *s_gear_label;

/* Radar view */
static lv_obj_t *s_radar_panel;
static lv_obj_t *s_radar_dots[MAX_AIRCRAFT];
static lv_obj_t *s_radar_info;
static lv_obj_t *s_radar_bub, *s_radar_blogo;
static bool s_radar_bub_off;   /* tapped away by the user (#9) */
static lv_obj_t *s_radar_range;
static lv_obj_t *s_radar_img;
static lv_obj_t *s_radar_rings[3];
static lv_obj_t *s_radar_home;
/* breadcrumb trail of the selected aircraft, map mode only */
static lv_obj_t   *s_radar_trail;
static lv_point_t  s_radar_trail_pts[TRAIL_LEN];

#define RADAR_W  (SCR_W - LIST_W)
#define RADAR_H  (SCR_H - HEADER_H)
#define RADAR_RENDER_W (SCR_W > 800 ? 800 - 310 : RADAR_W)
#define RADAR_RENDER_H (SCR_W > 800 ? (RADAR_H * (800 - 310)) / RADAR_W : RADAR_H)
#define RADAR_K        ((float)RADAR_W / (float)RADAR_RENDER_W)
#define RADAR_CX (RADAR_W / 2)
#define RADAR_CY (RADAR_H / 2 - UISY(6))
#define RADAR_R  (LV_MIN(RADAR_CX, RADAR_CY) - UISY(25))

/* Radar map background (home-area tiles) */
static uint16_t     *s_radar_tiles;
static lv_img_dsc_t  s_radar_tiles_dsc;
static tile_view_t   s_radar_view;
static double s_radar_zoom = 1.0;   /* window multiplier, <1 = closer */
static float  s_radar_scale = 1.0f; /* post-render upscale to fill the panel */
static int    s_radar_spx, s_radar_spy;
static bool          s_radar_view_ok;
static volatile bool s_radar_busy;
static char          s_radar_key[48];
static int64_t       s_radar_partial_ms;   /* nonzero: rendered with missing tiles */
static int           s_radar_missing;
static char          s_radar_want[48];
static double        s_radar_bbox[4];
static double        s_home_lat, s_home_lon;
static bool          s_home_ok;

void ui_set_home(double lat, double lon)
{
    s_home_lat = lat;
    s_home_lon = lon;
    s_home_ok = true;
    ESP_LOGI("ui", "home set: %.4f, %.4f", lat, lon);
}

/* Detail widgets */
static lv_obj_t *s_logo_img;
static lv_obj_t *s_logo_fallback;
static lv_obj_t *s_callsign_label;
static lv_obj_t *s_airline_label;
static lv_obj_t *s_type_label;
/* Big-screen type tier for the detail view: on canvases much taller than
 * the 432px design (tablets) fonts and the logo scale up. The big fonts are
 * compiled only into the app build; the device always uses the small tier. */
static bool s_big;
static const lv_font_t *s_f_code;   /* callsign + airport codes */
static const lv_font_t *s_f_name;   /* airline, type, times */
static const lv_font_t *s_f_small;  /* cities, footers */
static const lv_font_t *s_f_val;    /* stat tile values */
static int s_city_y;                /* city row y (differs per tier) */
static int s_dcity_w;               /* dest city label width */

static lv_obj_t *s_orig_code, *s_orig_city;
static lv_obj_t *s_orig_flag, *s_dest_flag, *s_reg_flag;
static lv_obj_t *s_orig_time, *s_dest_time, *s_extra_label;
static lv_obj_t *s_dest_code, *s_dest_city;
static lv_obj_t *s_progress_bar;
static lv_obj_t *s_progress_label;
static lv_obj_t *s_look_label;
static lv_obj_t *s_stat_vals[6];
static lv_obj_t *s_detail_empty;
static lv_obj_t *s_detail_content;

static void render_list_selection(void);
static void fb_upscale(uint16_t *fb, int W, int H, int px, int py, float k);
static void render_radar_panel(void);
static void label_set_if_changed(lv_obj_t *l, const char *txt);

static void render_right(void)
{
    render_radar_panel();
}

/* Wall-clock time at the device's location: Open-Meteo home offset when
 * known, TZ env (Europe/Warsaw) otherwise */
static void home_localtime(time_t t, struct tm *tm)
{
    if (tz_home_known()) {
        time_t local = t + tz_home_offset();
        gmtime_r(&local, tm);
    } else {
        localtime_r(&t, tm);
    }
}



static void settings_click_cb(lv_event_t *e)
{
    ui_settings_open();
}

/* Airline ICAO: prefer the route's, else derive from the callsign prefix so
 * logos show up before the route lookup completes. */
static const char *airline_code(const aircraft_t *ac, const route_info_t *rt)
{
    if (rt != NULL && rt->callsign[0] && rt->valid && rt->airline_icao[0]) {
        return rt->airline_icao;
    }
    static char prefix[4];
    if (isalpha((unsigned char)ac->callsign[0]) &&
        isalpha((unsigned char)ac->callsign[1]) &&
        isalpha((unsigned char)ac->callsign[2])) {
        prefix[0] = toupper((unsigned char)ac->callsign[0]);
        prefix[1] = toupper((unsigned char)ac->callsign[1]);
        prefix[2] = toupper((unsigned char)ac->callsign[2]);
        prefix[3] = '\0';
        return prefix;
    }
    return NULL;
}



static void render_list_rows(void);

/* Logos load asynchronously in a worker; repaint the affected panels as
 * soon as a batch lands instead of waiting for the next data cycle. */
static void logo_tick_cb(lv_timer_t *t)
{
    static uint32_t s_last_gen;
    uint32_t gen = logos_generation();
    if (gen == s_last_gen) {
        return;
    }
    s_last_gen = gen;
    render_list_rows();
    render_right();
}

static void clock_timer_cb(lv_timer_t *t)
{
    time_t now = time(NULL);
    if (now < 1600000000) {
        return;
    }
    struct tm tm;
    home_localtime(now, &tm);
    char hm[12];
    clock_fmt(hm, sizeof(hm), tm.tm_hour, tm.tm_min);
    label_set_if_changed(s_clock_label, hm);
}

static void render_list_rows(void);
static bool radar_view_filter(double lat, double lon, float dist_km);

static int row_of_shown(int shown_idx)
{
    for (int r = 0; r < s_list_plane_rows; r++) {
        if (s_row_plane_idx[r] == shown_idx) {
            return r;
        }
    }
    return -1;
}
static lv_obj_t *make_label(lv_obj_t *parent, const lv_font_t *font, lv_color_t color);





static void row_click_cb(lv_event_t *e)
{
    intptr_t idx = (intptr_t)lv_event_get_user_data(e);
    if (idx >= 0 && idx < s_list_plane_rows) {
        s_selected = s_row_plane_idx[idx];
        strlcpy(s_selected_hex, s_shown[s_selected].ac.hex, sizeof(s_selected_hex));
        render_list_selection();
        lv_timer_reset(s_cycle_timer);
        render_right();
    }
}

static void cycle_timer_cb(lv_timer_t *t)
{
    if (s_shown_count == 0 || settings_get()->follow_mode) {
        return;
    }
    s_selected = (s_selected + 1) % s_shown_count;
    strlcpy(s_selected_hex, s_shown[s_selected].ac.hex, sizeof(s_selected_hex));
    render_list_selection();
    render_right();
    /* keep the cycled aircraft visible in the list */
    int row = row_of_shown(s_selected);
    if (row >= 0) {
        lv_obj_scroll_to_view(s_list_rows[row], LV_ANIM_ON);
    }
}


static void apply_view(int mode)
{
    s_view_mode = mode % VIEW_COUNT;
    lv_obj_clear_flag(s_radar_panel, LV_OBJ_FLAG_HIDDEN);
    lv_timer_resume(s_cycle_timer);
    lv_timer_reset(s_cycle_timer);
    render_right();
}

static void mode_click_cb(lv_event_t *e)
{
    apply_view(s_view_mode + 1);
}



void ui_set_view(int mode)
{
    apply_view(mode);
}

/* skip the realloc + invalidate when a cyclic update rewrites the same text */
static void label_set_if_changed(lv_obj_t *l, const char *txt)
{
    if (strcmp(lv_label_get_text(l), txt) != 0) {
        lv_label_set_text(l, txt);
    }
}

/* LVGL 8 style writes and lv_img_set_src have no old-value compare, so an
 * unconditional call invalidates (and repaints) even when nothing changed.
 * Reading the current value back is far cheaper than a needless redraw. */
static void text_color_if_changed(lv_obj_t *o, lv_color_t c)
{
    if (lv_obj_get_style_text_color(o, 0).full != c.full) {
        lv_obj_set_style_text_color(o, c, 0);
    }
}

static void img_recolor_if_changed(lv_obj_t *o, lv_color_t c)
{
    if (lv_obj_get_style_img_recolor(o, 0).full != c.full) {
        lv_obj_set_style_img_recolor(o, c, 0);
    }
}

static void img_src_if_changed(lv_obj_t *o, const void *src)
{
    if (lv_img_get_src(o) != src) {
        lv_img_set_src(o, src);
    }
}

static lv_obj_t *make_label(lv_obj_t *parent, const lv_font_t *font, lv_color_t color)
{
    lv_obj_t *l = lv_label_create(parent);
    lv_obj_set_style_text_font(l, font, 0);
    lv_obj_set_style_text_color(l, color, 0);
    lv_label_set_text(l, "");
    return l;
}

static lv_obj_t *make_panel(lv_obj_t *parent)
{
    lv_obj_t *p = lv_obj_create(parent);
    lv_obj_set_style_bg_color(p, COL_PANEL, 0);
    lv_obj_set_style_border_width(p, 0, 0);
    lv_obj_set_style_radius(p, 0, 0);
    lv_obj_set_style_pad_all(p, 0, 0);
    lv_obj_clear_flag(p, LV_OBJ_FLAG_SCROLLABLE);
    return p;
}

/* Basemap credit line; RainViewer joins it when the rain layer is on. */
static const char *map_attribution(void)
{
    static char buf[72];
    snprintf(buf, sizeof(buf), "%s%s", tilemap_source_credit(),
             settings_get()->rain_overlay ? " \xC2\xB7 \xC2\xA9 RainViewer" : "");
    return buf;
}

/* List marker color per aircraft class (liner keeps the accent). */
static lv_color_t class_color(flight_class_t fc)
{
    switch (fc) {
    case FCLS_MIL:   return lv_color_hex(0xff6b6b);
    case FCLS_HELI:  return lv_color_hex(0xffa94d);
    case FCLS_SMALL: return lv_color_hex(0x39d98a);
    case FCLS_OTHER: return COL_DIM;
    default:         return COL_ACCENT;
    }
}

static void build_header(lv_obj_t *scr)
{
    lv_obj_t *hdr = make_panel(scr);
    lv_obj_set_size(hdr, SCR_W, HEADER_H);
    lv_obj_set_pos(hdr, 0, 0);

    s_weather_label = make_label(hdr, UIFONT(&font_pl_20, &font_pl_12), COL_ACCENT);
    lv_obj_set_width(s_weather_label, UISX(340));
    lv_label_set_long_mode(s_weather_label, LV_LABEL_LONG_DOT);
    lv_label_set_text(s_weather_label, LV_SYMBOL_GPS " esp32flight");
    lv_obj_align(s_weather_label, LV_ALIGN_LEFT_MID, UISX(14), 0);

    s_clock_label = make_label(hdr, UIFONT(&font_pl_20, &font_pl_12), COL_TEXT);
    /* dead center: offset +60 collided with the status label on busy
     * headers (#33, clock painted over the city name) */
    lv_obj_align(s_clock_label, LV_ALIGN_CENTER, 0, 0);
    lv_label_set_text(s_clock_label, "");

    s_status_label = make_label(hdr, UIFONT(&font_pl_14, &font_pl_8), COL_DIM);
    lv_obj_set_width(s_status_label, UISX(230));
    lv_label_set_long_mode(s_status_label, LV_LABEL_LONG_DOT);
    lv_obj_set_style_text_align(s_status_label, LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_align(s_status_label, LV_ALIGN_RIGHT_MID, -UISX(122), 0);
    lv_label_set_text(s_status_label, "...");

    lv_obj_t *gear = lv_btn_create(hdr);
    lv_obj_set_size(gear, UISX(46), UISY(36));
    lv_obj_align(gear, LV_ALIGN_RIGHT_MID, -UISX(10), 0);
    lv_obj_set_style_bg_color(gear, COL_ROW, 0);
    lv_obj_add_event_cb(gear, settings_click_cb, LV_EVENT_CLICKED, NULL);
    s_gear_label = make_label(gear, UIFONT(&lv_font_montserrat_16, &lv_font_montserrat_10), COL_TEXT);
    lv_label_set_text(s_gear_label, LV_SYMBOL_SETTINGS);
    lv_obj_center(s_gear_label);

    lv_obj_t *mode = lv_btn_create(hdr);
    lv_obj_set_size(mode, UISX(46), UISY(36));
    lv_obj_align(mode, LV_ALIGN_RIGHT_MID, -UISX(62), 0);
    lv_obj_set_style_bg_color(mode, COL_ROW, 0);
    lv_obj_add_event_cb(mode, mode_click_cb, LV_EVENT_CLICKED, NULL);
    s_mode_btn_label = make_label(mode, UIFONT(&lv_font_montserrat_16, &lv_font_montserrat_10), COL_TEXT);
    lv_label_set_text(s_mode_btn_label, LV_SYMBOL_LOOP);
    lv_obj_center(s_mode_btn_label);
}

static void build_list(lv_obj_t *scr)
{
    s_list_panel = make_panel(scr);
    lv_obj_set_size(s_list_panel, LIST_W, SCR_H - HEADER_H);
    lv_obj_set_pos(s_list_panel, 0, HEADER_H);
    lv_obj_set_style_bg_color(s_list_panel, COL_BG, 0);
    lv_obj_set_style_pad_all(s_list_panel, UISY(8), 0);
    lv_obj_set_style_pad_row(s_list_panel, UISY(6), 0);
    lv_obj_set_flex_flow(s_list_panel, LV_FLEX_FLOW_COLUMN);
    lv_obj_add_flag(s_list_panel, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scroll_dir(s_list_panel, LV_DIR_VER);


    for (int i = 0; i < MAX_SHOWN; i++) {
        lv_obj_t *row = lv_obj_create(s_list_panel);
        lv_obj_set_size(row, LIST_W - UISX(16) - UISX(8), UISY(64));
        lv_obj_set_style_bg_color(row, COL_ROW, 0);
        lv_obj_set_style_border_width(row, 0, 0);
        lv_obj_set_style_radius(row, UISY(8), 0);
        lv_obj_set_style_pad_all(row, UISY(8), 0);
        lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_add_flag(row, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_event_cb(row, row_click_cb, LV_EVENT_CLICKED, (void *)(intptr_t)i);

        lv_obj_t *cs = make_label(row, UIFONT(&lv_font_montserrat_20, &lv_font_montserrat_12), COL_TEXT);
        lv_obj_align(cs, LV_ALIGN_TOP_LEFT, UISX(48), -2);
        lv_obj_t *type = make_label(row, UIFONT(&lv_font_montserrat_14, &lv_font_montserrat_8), COL_ACCENT);
        lv_obj_align(type, LV_ALIGN_TOP_RIGHT, 0, 0);
        lv_obj_t *info = make_label(row, UIFONT(&lv_font_montserrat_12, &lv_font_montserrat_8), COL_DIM);
        lv_obj_align(info, LV_ALIGN_BOTTOM_LEFT, UISX(48), 2);

        lv_obj_t *route = make_label(row, UIFONT(&lv_font_montserrat_12, &lv_font_montserrat_8), COL_DIM);
        lv_obj_align(route, LV_ALIGN_BOTTOM_RIGHT, 0, 2);

        /* chip border and rounded corners are baked into the PNG assets */
        lv_obj_t *logo = lv_img_create(row);
        lv_img_set_pivot(logo, 0, 0);
        lv_img_set_zoom(logo, 256 * UISY(40) / 90);
        lv_img_set_size_mode(logo, LV_IMG_SIZE_MODE_REAL);
        lv_obj_align(logo, LV_ALIGN_LEFT_MID, 0, 0);
        lv_obj_add_flag(logo, LV_OBJ_FLAG_HIDDEN);

        lv_obj_add_flag(row, LV_OBJ_FLAG_HIDDEN);
        s_list_rows[i] = row;
    }
}







/* Rotatable plane sprite, tinted by altitude at render time */
static lv_obj_t *plane_img(lv_obj_t *parent)
{
    lv_obj_t *im = lv_img_create(parent);
    lv_img_set_src(im, &img_plane);
    lv_img_set_zoom(im, UIZOOM(256));
    lv_obj_set_style_img_recolor_opa(im, LV_OPA_COVER, 0);
    lv_obj_clear_flag(im, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(im, LV_OBJ_FLAG_HIDDEN);
    return im;
}




static void radar_tiles_job(void)
{
    char key[48];
    double b[4];
    strlcpy(key, s_radar_want, sizeof(key));
    memcpy(b, s_radar_bbox, sizeof(b));

    /* Composed straight into the live canvas. Upstream borrowed the
     * screensaver's full-screen buffer here for a flicker-free swap; that
     * buffer is gone with the screensaver, and this is the fallback path
     * upstream already used whenever the canvas was not lendable. */
    uint16_t *scratch = s_radar_tiles;
    const bool in_place = true;
    tile_view_t view;
    bool ok = scratch != NULL &&
              tilemap_render(scratch, RADAR_RENDER_W, RADAR_RENDER_H,
                             b[0], b[1], b[2], b[3], &view);
    if (ok) {
        runways_draw(scratch, RADAR_RENDER_W, RADAR_RENDER_H, &view);
    }
    /* integer tile zooms leave dead margin around the range ring; upscale
       so the ring (divided by the user zoom) fills most of the height */
    float scale = 1.0f;
    int shx = 0, shy = 0;
    if (ok) {
        int ex, ey;
        tilemap_project(&view, s_home_lat, s_home_lon, &shx, &shy);
        tilemap_project(&view, s_home_lat, b[3], &ex, &ey);
        float r = (float)(ex - shx);
        float target = 0.92f * (RADAR_RENDER_H / 2) / (float)s_radar_zoom;
        if (r > 16.0f && target / r > 1.05f) {
            scale = target / r;
            if (scale > 2.6f) {
                scale = 2.6f;
            }
            fb_upscale(scratch, RADAR_RENDER_W, RADAR_RENDER_H, shx, shy, scale);
        }
    }

    if (lvgl_port_lock(-1)) {
        if (ok && strcmp(key, s_radar_want) == 0) {
            if (s_radar_view_ok && strcmp(key, s_radar_key) == 0 &&
                view.missing > s_radar_missing) {
                s_radar_partial_ms = esp_timer_get_time() / 1000;   /* retry later */
                ok = false;   /* keep the more complete frame on screen */
            }
            ok = ok && s_radar_tiles != NULL;   /* boot-allocated */
        } else {
            ok = false;
        }
        if (ok) {
            if (!in_place) {
                memcpy(s_radar_tiles, scratch, (size_t)RADAR_RENDER_W * RADAR_RENDER_H * 2);
            }
            s_radar_view = view;
            s_radar_view_ok = true;
            s_radar_scale = scale;
            s_radar_spx = shx;
            s_radar_spy = shy;
            strlcpy(s_radar_key, key, sizeof(s_radar_key));
            /* partial render (offline blip): keep it, retry in 20 s */
            s_radar_missing = view.missing;
            s_radar_partial_ms = view.missing > 0 ? esp_timer_get_time() / 1000 : 0;
            s_radar_tiles_dsc.header.always_zero = 0;
            s_radar_tiles_dsc.header.cf = LV_IMG_CF_TRUE_COLOR;
            s_radar_tiles_dsc.header.w = RADAR_RENDER_W;
            s_radar_tiles_dsc.header.h = RADAR_RENDER_H;
            s_radar_tiles_dsc.data = (const uint8_t *)s_radar_tiles;
            s_radar_tiles_dsc.data_size = (size_t)RADAR_RENDER_W * RADAR_RENDER_H * 2;
            lv_img_set_src(s_radar_img, &s_radar_tiles_dsc);
            if (RADAR_W > RADAR_RENDER_W) {
                lv_img_set_zoom(s_radar_img, (uint16_t)(RADAR_K * 256.0f + 0.5f));
                lv_obj_set_pos(s_radar_img, RADAR_W / 2 - RADAR_RENDER_W / 2,
                               RADAR_H / 2 - RADAR_RENDER_H / 2);
            }
            lv_obj_invalidate(s_radar_img);
            if (s_view_mode == VIEW_RADAR) {
                render_radar_panel();
                render_list_rows();
            }
        }
        lvgl_port_unlock();
    }
    s_radar_busy = false;
}

static void radar_tiles_want(void)
{
    if (!s_home_ok || s_radar_busy) {
        return;
    }
    int radius_nm = settings_get()->radius_nm;
    char key[48];
    snprintf(key, sizeof(key), "%.3f,%.3f,%d,%.2f", s_home_lat, s_home_lon,
             radius_nm, s_radar_zoom);
    bool radar_stale = s_radar_partial_ms != 0 &&
                       esp_timer_get_time() / 1000 - s_radar_partial_ms > 20000;
    if (strcmp(key, s_radar_key) == 0 && !radar_stale) {
        return;
    }
    double rkm = radius_nm * 1.852 * s_radar_zoom;
    if (rkm < 2.0) {
        rkm = 2.0;
    }
    double dlat = rkm / 111.0;
    double dlon = rkm / (111.0 * cos(s_home_lat * M_PI / 180.0));
    s_radar_bbox[0] = s_home_lat - dlat;
    s_radar_bbox[1] = s_home_lat + dlat;
    s_radar_bbox[2] = s_home_lon - dlon;
    s_radar_bbox[3] = s_home_lon + dlon;
    LV_LOG_USER("radar bbox %.3f..%.3f / %.3f..%.3f (home %.3f,%.3f r=%d)",
                s_radar_bbox[0], s_radar_bbox[1], s_radar_bbox[2], s_radar_bbox[3],
                s_home_lat, s_home_lon, radius_nm);
    strlcpy(s_radar_want, key, sizeof(s_radar_want));
    s_radar_busy = true;
    ESP_LOGI(TAG, "radar tiles: spawn for %s", key);
    if (!tilemap_worker_submit(radar_tiles_job)) {
        ESP_LOGE(TAG, "radar tiles: worker queue full");
        s_radar_busy = false;
    }
}

static void radar_zoom_cb(lv_event_t *e)
{
    int dir = (int)(intptr_t)lv_event_get_user_data(e);
    double z = s_radar_zoom * (dir > 0 ? 1.0 / 1.5 : 1.5);
    if (z < 0.18) {
        z = 0.18;
    }
    if (z > 3.4) {
        z = 3.4;
    }
    if (z > 0.95 && z < 1.06) {
        z = 1.0;   /* snap back to the exact radius fit */
    }
    s_radar_zoom = z;
    radar_tiles_want();
}

static void radar_dot_cb(lv_event_t *e)
{
    int i = (int)(intptr_t)lv_event_get_user_data(e);
    if (i < 0 || i >= s_all_count || s_all[i].callsign[0] == '\0') {
        return;
    }
    for (int j = 0; j < s_shown_count; j++) {
        if (strcmp(s_shown[j].ac.callsign, s_all[i].callsign) == 0) {
            if (j == s_selected) {
                /* second tap on the selected plane toggles the bubble away */
                s_radar_bub_off = !s_radar_bub_off;
                render_radar_panel();
                return;
            }
            s_radar_bub_off = false;
            s_selected = j;
            strlcpy(s_selected_hex, s_shown[j].ac.hex, sizeof(s_selected_hex));
            render_list_selection();
            int row = row_of_shown(j);
            if (row >= 0) {
                lv_obj_scroll_to_view(s_list_rows[row], LV_ANIM_ON);
            }
            render_radar_panel();
            lv_timer_reset(s_cycle_timer);
            return;
        }
    }
}

static void build_radar_panel(lv_obj_t *scr)
{
    s_radar_panel = make_panel(scr);
    lv_obj_set_size(s_radar_panel, RADAR_W, RADAR_H);
    lv_obj_set_pos(s_radar_panel, LIST_W, HEADER_H);
    lv_obj_set_style_bg_color(s_radar_panel, COL_BG, 0);
    lv_obj_add_flag(s_radar_panel, LV_OBJ_FLAG_HIDDEN);

    /* home-area tile map in the background (falls back to plain rings) */
    s_radar_img = lv_img_create(s_radar_panel);
    lv_obj_set_pos(s_radar_img, 0, 0);
    lv_obj_add_flag(s_radar_img, LV_OBJ_FLAG_HIDDEN);

    for (int i = 1; i <= 3; i++) {
        int r = RADAR_R * i / 3;
        lv_obj_t *ring = lv_obj_create(s_radar_panel);
        lv_obj_set_size(ring, r * 2, r * 2);
        lv_obj_set_pos(ring, RADAR_CX - r, RADAR_CY - r);
        lv_obj_set_style_radius(ring, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_bg_opa(ring, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(ring, 1, 0);
        lv_obj_set_style_border_color(ring, COL_ROW_SEL, 0);
        lv_obj_clear_flag(ring, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);
        s_radar_rings[i - 1] = ring;
    }

    lv_obj_t *home = lv_obj_create(s_radar_panel);
    lv_obj_set_size(home, UISX(10), UISY(10));
    lv_obj_set_pos(home, RADAR_CX - UISX(5), RADAR_CY - UISY(5));
    lv_obj_set_style_radius(home, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(home, COL_ACCENT, 0);
    lv_obj_set_style_border_width(home, 0, 0);
    lv_obj_clear_flag(home, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);
    s_radar_home = home;

    /* created before the sprites so the trail draws underneath them */
    s_radar_trail = lv_line_create(s_radar_panel);
    lv_obj_set_style_line_width(s_radar_trail, 2, 0);
    lv_obj_set_style_line_color(s_radar_trail, COL_ACCENT, 0);
    lv_obj_set_style_line_opa(s_radar_trail, LV_OPA_60, 0);
    lv_obj_set_style_line_rounded(s_radar_trail, true, 0);
    lv_obj_clear_flag(s_radar_trail, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(s_radar_trail, LV_OBJ_FLAG_HIDDEN);

    for (int i = 0; i < MAX_AIRCRAFT; i++) {
        s_radar_dots[i] = plane_img(s_radar_panel);
        lv_obj_add_flag(s_radar_dots[i], LV_OBJ_FLAG_CLICKABLE);
        lv_obj_set_ext_click_area(s_radar_dots[i], 8);
        lv_obj_add_event_cb(s_radar_dots[i], radar_dot_cb, LV_EVENT_CLICKED,
                            (void *)(intptr_t)i);
    }

    s_radar_bub = lv_obj_create(s_radar_panel);
    lv_obj_set_size(s_radar_bub, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_obj_set_style_bg_color(s_radar_bub, COL_PANEL, 0);
    lv_obj_set_style_bg_opa(s_radar_bub, LV_OPA_80, 0);
    lv_obj_set_style_border_width(s_radar_bub, 0, 0);
    lv_obj_set_style_pad_all(s_radar_bub, UISY(6), 0);
    lv_obj_set_style_pad_column(s_radar_bub, UISX(8), 0);
    lv_obj_set_style_radius(s_radar_bub, UISY(6), 0);
    lv_obj_set_flex_flow(s_radar_bub, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(s_radar_bub, LV_FLEX_ALIGN_START,
                          LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(s_radar_bub, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(s_radar_bub, LV_OBJ_FLAG_HIDDEN);
    s_radar_blogo = lv_img_create(s_radar_bub);
    lv_img_set_pivot(s_radar_blogo, 0, 0);
    lv_img_set_zoom(s_radar_blogo, 256 * UISY(36) / 90);
    lv_img_set_size_mode(s_radar_blogo, LV_IMG_SIZE_MODE_REAL);
    lv_obj_add_flag(s_radar_blogo, LV_OBJ_FLAG_HIDDEN);
    s_radar_info = make_label(s_radar_bub, UIFONT(&font_pl_14, &font_pl_8), COL_TEXT);
    lv_label_set_text(s_radar_info, "");

    lv_obj_t *rattr = make_label(s_radar_panel, UIFONT(&font_pl_14, &font_pl_8), COL_DIM);
    lv_label_set_text(rattr, map_attribution());
    lv_obj_align(rattr, LV_ALIGN_BOTTOM_LEFT, UISX(6), -2);

    static const char *rzsym[2] = { LV_SYMBOL_PLUS, LV_SYMBOL_MINUS };
    for (int zi = 0; zi < 2; zi++) {
        lv_obj_t *zb = lv_btn_create(s_radar_panel);
        lv_obj_set_size(zb, UISX(52), UISY(44));
        lv_obj_align(zb, LV_ALIGN_BOTTOM_RIGHT, -UISX(10), -UISY(78) + zi * UISY(52));
        lv_obj_set_style_bg_color(zb, COL_ACCENT, 0);
        lv_obj_set_style_bg_opa(zb, LV_OPA_90, 0);
        lv_obj_set_style_shadow_width(zb, 12, 0);
        lv_obj_set_style_shadow_opa(zb, LV_OPA_40, 0);
        lv_obj_add_event_cb(zb, radar_zoom_cb, LV_EVENT_CLICKED,
                            (void *)(intptr_t)(zi == 0 ? 1 : -1));
        lv_obj_t *zl = lv_label_create(zb);
        lv_obj_set_style_text_color(zl, COL_BG, 0);
        lv_label_set_text(zl, rzsym[zi]);
        lv_obj_center(zl);
    }

    s_radar_range = make_label(s_radar_panel, UIFONT(&font_pl_14, &font_pl_8), COL_DIM);
    lv_obj_align(s_radar_range, LV_ALIGN_BOTTOM_RIGHT, -UISX(10), -UISY(6));
    lv_label_set_text(s_radar_range, "");
}

/* ---------- optional extra objects: ISS, radiosondes, AIS ships ---------- */






/* tilemap projection plus the fill-the-panel upscale */
/* Overlay label on the map panel: airspace names ride on these. */
static lv_obj_t *extra_lbl(lv_color_t color)
{
    lv_obj_t *l = lv_label_create(s_radar_panel);
    lv_obj_set_style_text_font(l, UIFONT(&font_pl_14, &font_pl_8), 0);
    lv_obj_set_style_text_color(l, color, 0);
    lv_obj_add_flag(l, LV_OBJ_FLAG_HIDDEN);
    return l;
}

static void radar_project(double lat, double lon, int *x, int *y)
{
    tilemap_project(&s_radar_view, lat, lon, x, y);
    if (s_radar_scale != 1.0f) {
        *x = s_radar_spx + (int)((*x - s_radar_spx) * s_radar_scale);
        *y = s_radar_spy + (int)((*y - s_radar_spy) * s_radar_scale);
    }
    /* render space -> panel space (identity on 800x480) */
    *x = RADAR_W / 2 + (int)((*x - RADAR_RENDER_W / 2) * RADAR_K);
    *y = RADAR_H / 2 + (int)((*y - RADAR_RENDER_H / 2) * RADAR_K);
}

/* place a lat/lon at dist_km from home onto the radar; false = outside */
static bool radar_place(double lat, double lon, float dist_km,
                        bool map_mode, int radius_nm, int *x, int *y)
{
    if (map_mode) {
        radar_project(lat, lon, x, y);
        return *x >= -10 && *x <= RADAR_W + 10 && *y >= -10 && *y <= RADAR_H + 10;
    }
    float frac = dist_km / (radius_nm * 1.852f);
    if (frac > 1.0f) {
        return false;
    }
    float rad = (float)geo_bearing_deg(s_home_lat, s_home_lon, lat, lon) *
                (float)M_PI / 180.0f;
    *x = RADAR_CX + (int)(sinf(rad) * frac * RADAR_R);
    *y = RADAR_CY - (int)(cosf(rad) * frac * RADAR_R);
    return true;
}

static lv_obj_t *s_x_asp_line[MAX_AIRSPACES];
static lv_obj_t *s_x_asp_lbl[MAX_AIRSPACES];
static lv_point_t (*s_x_asp_pts)[MAX_ASP_POINTS + 1];

static lv_color_t airspace_color(int type)
{
    switch (type) {
    case 4: case 27:          return lv_color_hex(0xe06666);  /* CTR/MCTR */
    case 7: case 26: case 13: return lv_color_hex(0x6fa8dc);  /* TMA/MTMA/ATZ */
    default:                  return lv_color_hex(0xe6b34d);  /* R/D/P/TRA */
    }
}

static void render_airspaces(bool map_mode)
{
    int n = map_mode ? airspace_count() : 0;
    if (n > 0 && s_x_asp_pts == NULL) {
        s_x_asp_pts = heap_caps_calloc(MAX_AIRSPACES,
                                       sizeof(*s_x_asp_pts),
                                       MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (s_x_asp_pts == NULL) {
            return;
        }
    }
    for (int i = 0; i < MAX_AIRSPACES; i++) {
        bool show = i < n;
        const airspace_t *a = show ? airspace_get(i) : NULL;
        if (a == NULL) {
            if (s_x_asp_line[i] != NULL) {
                lv_obj_add_flag(s_x_asp_line[i], LV_OBJ_FLAG_HIDDEN);
                lv_obj_add_flag(s_x_asp_lbl[i], LV_OBJ_FLAG_HIDDEN);
            }
            continue;
        }
        if (s_x_asp_line[i] == NULL) {
            s_x_asp_line[i] = lv_line_create(s_radar_panel);
            lv_obj_set_style_line_width(s_x_asp_line[i], 1, 0);
            lv_obj_set_style_line_dash_width(s_x_asp_line[i], 4, 0);
            lv_obj_set_style_line_dash_gap(s_x_asp_line[i], 4, 0);
            lv_obj_set_style_line_opa(s_x_asp_line[i], LV_OPA_70, 0);
            lv_obj_clear_flag(s_x_asp_line[i], LV_OBJ_FLAG_CLICKABLE);
            s_x_asp_lbl[i] = extra_lbl(lv_color_white());
        }
        lv_color_t col = airspace_color(a->type);
        lv_obj_set_style_line_color(s_x_asp_line[i], col, 0);
        lv_obj_set_style_text_color(s_x_asp_lbl[i], col, 0);
        int m = 0, lx = 0, ly = 0;
        for (int k = 0; k < a->n_points; k++) {
            int x, y;
            radar_project(a->lat[k], a->lon[k], &x, &y);
            /* keep coordinates sane for lvgl even when far off screen */
            if (x < -2000) x = -2000;
            if (x > 2000) x = 2000;
            if (y < -2000) y = -2000;
            if (y > 2000) y = 2000;
            s_x_asp_pts[i][m].x = (lv_coord_t)x;
            s_x_asp_pts[i][m].y = (lv_coord_t)y;
            if (m == 0) {
                lx = x;
                ly = y;
            }
            m++;
        }
        /* close the ring */
        s_x_asp_pts[i][m] = s_x_asp_pts[i][0];
        m++;
        lv_line_set_points(s_x_asp_line[i], s_x_asp_pts[i], m);
        lv_obj_clear_flag(s_x_asp_line[i], LV_OBJ_FLAG_HIDDEN);
        if (lx >= 0 && lx <= RADAR_W - 40 && ly >= 8 && ly <= RADAR_H - 20) {
            lv_label_set_text(s_x_asp_lbl[i], airspace_type_str(a->type));
            lv_obj_set_pos(s_x_asp_lbl[i], lx + UISX(3), ly - UISY(16));
            lv_obj_clear_flag(s_x_asp_lbl[i], LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_add_flag(s_x_asp_lbl[i], LV_OBJ_FLAG_HIDDEN);
        }
    }
}





static void render_radar_panel(void)
{
    radar_tiles_want();

    int radius_nm = settings_get()->radius_nm;
    bool map_mode = s_radar_view_ok;
    int hx = RADAR_CX, hy = RADAR_CY;
    int rpx = RADAR_R;

    if (map_mode) {
        radar_project(s_home_lat, s_home_lon, &hx, &hy);
        int ex, ey;
        radar_project(s_home_lat, s_radar_bbox[3], &ex, &ey);
        rpx = ex - hx;
        if (rpx < 20) {
            rpx = 20;
        }
        lv_obj_clear_flag(s_radar_img, LV_OBJ_FLAG_HIDDEN);
        /* single range ring on the map */
        lv_obj_clear_flag(s_radar_rings[2], LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_size(s_radar_rings[2], rpx * 2, rpx * 2);
        lv_obj_set_pos(s_radar_rings[2], hx - rpx, hy - rpx);
        lv_obj_add_flag(s_radar_rings[0], LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_radar_rings[1], LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_pos(s_radar_home, hx - UISX(5), hy - UISY(5));
        lv_label_set_text_fmt(s_radar_range, "%d km", (int)(radius_nm * 1.852));
    } else {
        lv_obj_add_flag(s_radar_img, LV_OBJ_FLAG_HIDDEN);
        for (int i = 0; i < 3; i++) {
            int r = RADAR_R * (i + 1) / 3;
            lv_obj_clear_flag(s_radar_rings[i], LV_OBJ_FLAG_HIDDEN);
            lv_obj_set_size(s_radar_rings[i], r * 2, r * 2);
            lv_obj_set_pos(s_radar_rings[i], RADAR_CX - r, RADAR_CY - r);
        }
        lv_obj_set_pos(s_radar_home, RADAR_CX - UISX(5), RADAR_CY - UISY(5));
        lv_label_set_text_fmt(s_radar_range, L()->ring_fmt, (int)(radius_nm * 1.852 / 3));
    }

    const aircraft_t *selac = (s_selected >= 0 && s_selected < s_shown_count)
                                  ? &s_shown[s_selected].ac : NULL;

    /* Where the selected aircraft has already been. Only in map mode: the
     * range-ring projection is bearing/distance based, so a track drawn
     * through it would be a fan of straight radials rather than a path. */
    int tn = 0;
    if (selac != NULL && map_mode && s_radar_view_ok) {
        float tlat[TRAIL_LEN], tlon[TRAIL_LEN];
        int n = trails_get(selac->hex, tlat, tlon, TRAIL_LEN);
        for (int k = 0; k < n; k++) {
            int x, y;
            radar_project(tlat[k], tlon[k], &x, &y);
            /* keep the polyline inside the panel: a point far off-screen
             * drags the whole line across it */
            if (x < -RADAR_W || x > 2 * RADAR_W || y < -RADAR_H || y > 2 * RADAR_H) {
                tn = 0;      /* restart after the gap */
                continue;
            }
            s_radar_trail_pts[tn].x = (lv_coord_t)x;
            s_radar_trail_pts[tn].y = (lv_coord_t)y;
            tn++;
        }
    }
    if (tn >= 2) {
        lv_line_set_points(s_radar_trail, s_radar_trail_pts, (uint16_t)tn);
        lv_obj_clear_flag(s_radar_trail, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(s_radar_trail, LV_OBJ_FLAG_HIDDEN);
    }

    for (int i = 0; i < MAX_AIRCRAFT; i++) {
        if (i >= s_all_count || s_all[i].dist_nm < 0) {
            lv_obj_add_flag(s_radar_dots[i], LV_OBJ_FLAG_HIDDEN);
            continue;
        }
        const map_target_t *t = &s_all[i];
        int x, y;
        if (map_mode) {
            radar_project(t->lat, t->lon, &x, &y);
            if (x < -14 || x > RADAR_W + 14 || y < -14 || y > RADAR_H + 14) {
                lv_obj_add_flag(s_radar_dots[i], LV_OBJ_FLAG_HIDDEN);
                continue;
            }
        } else {
            float frac = t->dist_nm / (float)radius_nm;
            if (frac > 1.0f) {
                frac = 1.0f;
            }
            float rad = t->dir_deg * (float)M_PI / 180.0f;
            x = RADAR_CX + (int)(sinf(rad) * frac * RADAR_R);
            y = RADAR_CY - (int)(cosf(rad) * frac * RADAR_R);
        }
        bool sel = selac != NULL && selac->callsign[0] &&
                   strcmp(t->callsign, selac->callsign) == 0;
        lv_obj_set_pos(s_radar_dots[i], x - 14, y - 14);   /* UIZOOM keeps the 28px box */
        img_src_if_changed(s_radar_dots[i], class_sprite(t->fcls));
        lv_img_set_angle(s_radar_dots[i], (int)(t->track * 10));
        lv_img_set_zoom(s_radar_dots[i], sel ? UIZOOM(384) : UIZOOM(232));
        img_recolor_if_changed(s_radar_dots[i],
                               alt_color(t->alt_ft, t->ground));
        lv_obj_clear_flag(s_radar_dots[i], LV_OBJ_FLAG_HIDDEN);

        if (sel) {
            char info[96];
            if (selac->on_ground) {
                snprintf(info, sizeof(info), "%s\n%s  %.1f km",
                         selac->callsign[0] ? selac->callsign : selac->hex,
                         L()->ground, selac->dist_nm * 1.852);
            } else {
                char ua[20], us[20];
                snprintf(info, sizeof(info), "%s\n%s  %s  %.1f km",
                         selac->callsign[0] ? selac->callsign : selac->hex,
                         units_alt(selac->alt_baro_ft, ua, sizeof(ua)),
                         units_speed(selac->gs_kts, us, sizeof(us)),
                         selac->dist_nm * 1.852);
            }
            lv_label_set_text(s_radar_info, info);
            const char *lcode = airline_code(selac, &s_shown[s_selected].route);
            const lv_img_dsc_t *ldsc = lcode ? logos_get(lcode) : NULL;
            if (ldsc != NULL) {
                img_src_if_changed(s_radar_blogo, ldsc);
                lv_obj_clear_flag(s_radar_blogo, LV_OBJ_FLAG_HIDDEN);
            } else {
                lv_obj_add_flag(s_radar_blogo, LV_OBJ_FLAG_HIDDEN);
            }
            if (s_radar_bub_off) {
                lv_obj_add_flag(s_radar_bub, LV_OBJ_FLAG_HIDDEN);
            } else {
                lv_obj_clear_flag(s_radar_bub, LV_OBJ_FLAG_HIDDEN);
            }
            int lx = x + UISX(12), ly = y - UISY(10);
            if (lx > RADAR_W - UISX(190)) {
                lx = x - UISX(190);
            }
            if (ly < 0) {
                ly = 0;
            }
            if (ly > RADAR_H - UISY(60)) {
                ly = RADAR_H - UISY(60);
            }
            lv_obj_set_pos(s_radar_bub, lx, ly);
        }
    }
    if (s_selected < 0 || s_selected >= s_shown_count || selac == NULL) {
        lv_obj_add_flag(s_radar_bub, LV_OBJ_FLAG_HIDDEN);
    }
    render_airspaces(map_mode);
}

/* ---------- full-screen ambient screensaver ---------- */


/* Upscale the rendered map in place around (px,py) so the observation
 * circle spans the full screen height. One-time cost in the worker task;
 * the displayed image stays a plain untransformed bitmap. */
static void fb_upscale(uint16_t *fb, int W, int H, int px, int py, float k)
{
    /* Scratch-frame version: the buffer under our feet is usually the live
     * LVGL image source, and transforming it in place paints garbage on the
     * screen mid-pass. When the scratch allocation fails we simply skip the
     * upscale and the view stays at its native framing. */
    uint16_t *tmp = heap_caps_malloc((size_t)W * H * 2, MALLOC_CAP_SPIRAM);
    if (tmp == NULL) {
        return;
    }
    for (int y = 0; y < H; y++) {
        if ((y & 63) == 63) {
            vTaskDelay(1);      /* let IDLE0 feed the task watchdog */
        }
        float sy = py + (y - py) / k;
        int y0 = (int)sy;
        float fy = sy - y0;
        if (y0 < 0) { y0 = 0; fy = 0; }
        if (y0 > H - 2) { y0 = H - 2; fy = 1; }
        for (int x = 0; x < W; x++) {
            float sx = px + (x - px) / k;
            int x0 = (int)sx;
            float fx = sx - x0;
            if (x0 < 0) { x0 = 0; fx = 0; }
            if (x0 > W - 2) { x0 = W - 2; fx = 1; }
            uint16_t c00 = fb[y0 * W + x0], c01 = fb[y0 * W + x0 + 1];
            uint16_t c10 = fb[(y0 + 1) * W + x0], c11 = fb[(y0 + 1) * W + x0 + 1];
            float w00 = (1 - fx) * (1 - fy), w01 = fx * (1 - fy);
            float w10 = (1 - fx) * fy, w11 = fx * fy;
            int r = (int)((c00 >> 11) * w00 + (c01 >> 11) * w01 +
                          (c10 >> 11) * w10 + (c11 >> 11) * w11);
            int g = (int)(((c00 >> 5) & 0x3F) * w00 + ((c01 >> 5) & 0x3F) * w01 +
                          ((c10 >> 5) & 0x3F) * w10 + ((c11 >> 5) & 0x3F) * w11);
            int b = (int)((c00 & 0x1F) * w00 + (c01 & 0x1F) * w01 +
                          (c10 & 0x1F) * w10 + (c11 & 0x1F) * w11);
            tmp[y * W + x] = (uint16_t)((r << 11) | (g << 5) | b);
        }
    }
    memcpy(fb, tmp, (size_t)W * H * 2);
    free(tmp);
}

















/* idle watcher: screensaver + night backlight */
/* Which brightness slot covers local time now, or -1 for none. A range may
 * wrap midnight (22:00-07:00), and the first match wins so overlapping slots
 * resolve predictably instead of fighting each other. */
static int bsched_slot_now(void)
{
    const settings_t *cfg = settings_get();
    if (!cfg->bsched_on) {
        return -1;
    }
    time_t now = time(NULL);
    if (now < 1600000000) {
        return -1;              /* clock not set yet: leave brightness alone */
    }
    time_t l = now + (tz_home_known() ? tz_home_offset() : 0);
    struct tm tm;
    gmtime_r(&l, &tm);
    int m = tm.tm_hour * 60 + tm.tm_min;
    for (int s = 0; s < 2; s++) {
        int a = cfg->bsched_from[s], b = cfg->bsched_to[s];
        if (a == b) {
            continue;           /* empty range */
        }
        if (a < b ? (m >= a && m < b) : (m >= a || m < b)) {
            return s;
        }
    }
    return -1;
}

/* Night blackout used to wait for the screensaver to be up. With no
 * screensaver, plain inactivity is the trigger; this timer runs every 10 s,
 * so a touch relights the panel within one tick via the safety net below. */
#define NIGHT_IDLE_MS  60000U

/* idle watcher: night backlight + scheduled brightness */
static void idle_timer_cb(lv_timer_t *t)
{
    const settings_t *cfg = settings_get();
    uint32_t idle_ms = lv_disp_get_inactive_time(NULL);

    /* Scheduled brightness, applied when a slot is entered rather than on
     * every tick: a change from the slider or Home Assistant then holds
     * until the next boundary instead of being overwritten 10 s later. The
     * level is stored either way, but the panel is only re-lit if it is
     * already on - the schedule sets how bright, never whether. */
    {
        static int applied_slot = -2;
        int slot = bsched_slot_now();
        if (slot != applied_slot) {
            applied_slot = slot;
            if (slot >= 0) {
                settings_t *w = settings_get();
                if (w->brightness != w->bsched_pct[slot]) {
                    w->brightness = w->bsched_pct[slot];
                    ESP_LOGI(TAG, "brightness schedule: slot %d -> %d%%",
                             slot, w->brightness);
                }
                if (!s_bl_off) {
                    ui_apply_brightness();
                }
            }
        }
    }

    if (cfg->night_enabled) {
        time_t now = time(NULL);
        if (now > 1600000000) {
            time_t l = now + (tz_home_known() ? tz_home_offset() : 0);
            struct tm tm;
            gmtime_r(&l, &tm);
            int m = tm.tm_hour * 60 + tm.tm_min;
            int a = cfg->night_start_min, b = cfg->night_end_min;
            if (cfg->night_auto && s_home_ok) {
                int rise, set;
                if (geo_sun_times(s_home_lat, s_home_lon, (long long)now,
                                  tz_home_known() ? tz_home_offset() : 0,
                                  &rise, &set)) {
                    a = set;    /* dark from sunset... */
                    b = rise;   /* ...to sunrise */
                }
            }
            bool night = a <= b ? (m >= a && m < b) : (m >= a || m < b);
            if (night && !s_bl_off && idle_ms > NIGHT_IDLE_MS) {
                waveshare_rgb_lcd_bl_off();
                s_bl_off = true;
            } else if (!night && s_bl_off) {
                waveshare_rgb_lcd_bl_on();
                ui_apply_brightness();
                s_bl_off = false;
            }
        }
    }

    /* Any recent input while dark relights the panel (this timer runs every
     * 10 s), and so does disabling night mode from the web panel. */
    if (s_bl_off && (idle_ms < 11000U || !cfg->night_enabled)) {
        waveshare_rgb_lcd_bl_on();
        ui_apply_brightness();
        s_bl_off = false;
    }
}












void ui_init(void)
{
    s_shown = heap_caps_calloc(MAX_SHOWN, sizeof(*s_shown), MALLOC_CAP_SPIRAM);
    assert(s_shown != NULL);

    lv_obj_t *scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, COL_BG, 0);

    build_header(scr);
    build_list(scr);
    build_radar_panel(scr);

    /* All big tile framebuffers are session-persistent once used, so they
     * are allocated HERE, while PSRAM is one unfragmented block. Lazy
     * allocation used to fail on the 7B (1.2 MB each): after an hour of
     * caches and TLS churn no contiguous block that size was left, and
     * the ambient map never came up (issue #12, second act). Boot-time
     * allocation costs nothing extra in steady state - every buffer ends
     * up allocated anyway once each view has been visited. */
    s_radar_tiles = heap_caps_malloc((size_t)RADAR_RENDER_W * RADAR_RENDER_H * 2, MALLOC_CAP_SPIRAM);
    if (s_radar_tiles == NULL) {
        ESP_LOGE(TAG, "radar tile framebuffer alloc FAILED at boot");
    }
    s_cycle_timer = lv_timer_create(cycle_timer_cb, CYCLE_MS, NULL);
    lv_timer_pause(s_cycle_timer);
    lv_timer_create(clock_timer_cb, 5000, NULL);
    lv_timer_create(logo_tick_cb, 500, NULL);
    lv_timer_create(idle_timer_cb, 10000, NULL);

    apply_view(VIEW_RADAR);
}

static bool s_update_avail;
static char s_update_tag[16];
static lv_timer_t *s_gear_blink;

static void gear_blink_cb(lv_timer_t *t)
{
    static bool on;
    on = !on;
    if (s_gear_label != NULL) {
        lv_obj_set_style_text_color(s_gear_label,
                                    on ? lv_color_hex(0xffd166) : COL_TEXT, 0);
    }
}

void ui_set_update_available(bool available, const char *tag)
{
    s_update_avail = available;
    if (tag != NULL) {
        strlcpy(s_update_tag, tag, sizeof(s_update_tag));
    }
    if (available && s_gear_blink == NULL) {
        s_gear_blink = lv_timer_create(gear_blink_cb, 600, NULL);
    } else if (!available && s_gear_blink != NULL) {
        lv_timer_del(s_gear_blink);
        s_gear_blink = NULL;
        if (s_gear_label != NULL) {
            lv_obj_set_style_text_color(s_gear_label, COL_TEXT, 0);
        }
    }
}

bool ui_update_available(void)
{
    return s_update_avail;
}

const char *ui_update_tag(void)
{
    return s_update_tag;
}

static lv_obj_t *s_flyover_banner;




void ui_set_status_alert(bool alert)
{
    if (s_status_label != NULL) {
        lv_obj_set_style_text_color(s_status_label,
                                    alert ? lv_color_hex(0xff5252) : COL_DIM, 0);
        lv_obj_set_style_text_font(s_status_label,
                                   alert ? UIFONT(&font_pl_20, &font_pl_12) : UIFONT(&font_pl_14, &font_pl_8), 0);
    }
}

void ui_set_status(const char *text)
{
    if (s_status_label != NULL) {
        lv_label_set_text(s_status_label, text);
    }
}

void ui_set_weather(const char *text)
{
    strlcpy(s_weather_txt, text, sizeof(s_weather_txt));
    if (s_weather_label != NULL) {
        lv_label_set_text(s_weather_label, text);
    }
}

static void render_list_selection(void)
{
    /* LVGL style writes have no old-value compare: an unconditional write
     * invalidates the row even when nothing changed, which used to repaint
     * the whole list column every refresh. Shadow the last state per row
     * (3 = never written; theme changes bump s_theme_gen to force a pass). */
    static uint8_t s_row_sel_shadow[MAX_SHOWN];
    static uint8_t s_row_sel_theme = 0xFF;
    if (s_row_sel_theme != (uint8_t)settings_get()->theme) {
        s_row_sel_theme = (uint8_t)settings_get()->theme;
        memset(s_row_sel_shadow, 3, sizeof(s_row_sel_shadow));
    }
    for (int i = 0; i < MAX_SHOWN; i++) {
        bool sel = i < s_list_plane_rows && s_row_plane_idx[i] == s_selected;
        if (s_row_sel_shadow[i] == (uint8_t)sel) {
            continue;
        }
        s_row_sel_shadow[i] = (uint8_t)sel;
        lv_obj_set_style_bg_color(s_list_rows[i], sel ? COL_ROW_SEL : COL_ROW, 0);
        /* the dim sub-line drowns in the selection color on some themes
           (#11): brighten it towards the text color while selected */
        lv_obj_t *info = lv_obj_get_child(s_list_rows[i], 2);
        if (info != NULL) {
            lv_obj_set_style_text_color(info,
                sel ? lv_color_mix(COL_TEXT, COL_DIM, 170) : COL_DIM, 0);
        }
    }
}



void ui_update(const aircraft_list_t *list)
{
    /* Snapshot aircraft + routes so touch callbacks never race the fetcher. */
    flight_stats_get(&s_stats_snap);
    s_all_count = 0;
    for (int i = 0; i < list->count && i < MAX_AIRCRAFT; i++) {
        if (!list->ac[i].has_pos) {
            continue;
        }
        map_target_t *t = &s_all[s_all_count++];
        strlcpy(t->callsign, list->ac[i].callsign, sizeof(t->callsign));
        t->lat = (float)list->ac[i].lat;
        t->lon = (float)list->ac[i].lon;
        t->track = list->ac[i].track_deg;
        t->dist_nm = list->ac[i].dist_nm;
        t->dir_deg = list->ac[i].dir_deg;
        t->alt_ft = list->ac[i].alt_baro_ft;
        t->ground = list->ac[i].on_ground;
        t->fcls = (uint8_t)flight_sprite(&list->ac[i]);
    }
    s_shown_count = list->count < MAX_SHOWN ? list->count : MAX_SHOWN;
    for (int i = 0; i < s_shown_count; i++) {
        s_shown[i].ac = list->ac[i];
        const route_info_t *rt = routes_get_cached(list->ac[i].callsign);
        if (rt != NULL && rt->valid && list->ac[i].has_pos &&
            !geo_route_plausible_dir(rt->origin.lat, rt->origin.lon,
                                     rt->destination.lat, rt->destination.lon,
                                     list->ac[i].lat, list->ac[i].lon,
                                     list->ac[i].track_deg, list->ac[i].gs_kts,
                                     list->ac[i].baro_rate_fpm)) {
            /* Stale/reused callsign in the route DB - don't show nonsense. */
            rt = NULL;
        }
        if (rt != NULL) {
            s_shown[i].route = *rt;
        } else {
            memset(&s_shown[i].route, 0, sizeof(route_info_t));
        }

        s_shown[i].iata[0] = '\0';
        const char *fa = faflight_get_cached(list->ac[i].callsign);
        if (fa != NULL && fa[0]) {
            strlcpy(s_shown[i].iata, fa, sizeof(s_shown[i].iata));
        }

        /* Airline display name: route DB first, adsbdb airline lookup second */
        s_shown[i].airline[0] = '\0';
        if (rt != NULL && rt->valid && rt->airline_name[0]) {
            strlcpy(s_shown[i].airline, rt->airline_name, sizeof(s_shown[i].airline));
        } else {
            const char *code = airline_code(&s_shown[i].ac, rt);
            const char *name = code ? airlines_get_cached(code) : NULL;
            if (name != NULL) {
                strlcpy(s_shown[i].airline, name, sizeof(s_shown[i].airline));
            }
        }
    }

    /* Keep selection pinned to the same aircraft across refreshes. */
    s_selected = -1;
    for (int i = 0; i < s_shown_count; i++) {
        if (s_selected_hex[0] && strcmp(s_shown[i].ac.hex, s_selected_hex) == 0) {
            s_selected = i;
            break;
        }
    }
    if (s_selected < 0 && s_shown_count > 0) {
        s_selected = 0;
        strlcpy(s_selected_hex, s_shown[0].ac.hex, sizeof(s_selected_hex));
    }

    render_list_rows();
    render_list_selection();
    render_right();
}


/* on the radar map view the list mirrors the visible frame */
static bool radar_view_filter(double lat, double lon, float dist_km)
{
    if (s_view_mode != VIEW_RADAR || !s_radar_view_ok) {
        return true;
    }
    int x, y;
    return radar_place(lat, lon, dist_km, true,
                       settings_get()->radius_nm, &x, &y);
}

static void render_list_rows(void)
{
    int n_planes = 0;
    for (int i = 0; i < s_shown_count && n_planes < MAX_SHOWN; i++) {
        if (radar_view_filter(s_shown[i].ac.lat, s_shown[i].ac.lon,
                              (float)(s_shown[i].ac.dist_nm * 1.852))) {
            s_row_plane_idx[n_planes++] = i;
        }
    }
    s_list_plane_rows = n_planes;

    for (int i = 0; i < MAX_SHOWN; i++) {
        if (i < n_planes) {
            const aircraft_t *ac = &s_shown[s_row_plane_idx[i]].ac;
            lv_obj_t *row = s_list_rows[i];
            lv_obj_clear_flag(row, LV_OBJ_FLAG_HIDDEN);

            lv_obj_t *cs_label = lv_obj_get_child(row, 0);
            label_set_if_changed(cs_label, ac->callsign[0] ? ac->callsign : ac->hex);
            /* gold for military / heavies / watchlist hits */
            bool interesting = flight_is_interesting(ac, settings_get()->watch_regs);
            text_color_if_changed(cs_label,
                                  interesting ? lv_color_hex(0xffd166) : COL_TEXT);
            lv_obj_t *typechip = lv_obj_get_child(row, 1);
            label_set_if_changed(typechip, ac->type_icao[0] ? ac->type_icao : "?");
            text_color_if_changed(typechip, class_color(flight_class(ac)));

            char info[64];
            if (ac->on_ground) {
                snprintf(info, sizeof(info), "%s  -  %.1f km", L()->ground, ac->dist_nm * 1.852);
            } else {
                const char *trend = "";
                if (ac->baro_rate_fpm > 300) {
                    trend = LV_SYMBOL_UP " ";
                } else if (ac->baro_rate_fpm < -300) {
                    trend = LV_SYMBOL_DOWN " ";
                }
                char ua[20], us[20];
                snprintf(info, sizeof(info), "%s%s  %s  %.1f km",
                         trend, units_alt(ac->alt_baro_ft, ua, sizeof(ua)),
                         units_speed(ac->gs_kts, us, sizeof(us)), ac->dist_nm * 1.852);
            }
            label_set_if_changed(lv_obj_get_child(row, 2), info);

            /* origin/destination codes on the right of the info line (#28) */
            lv_obj_t *route_lbl = lv_obj_get_child(row, 3);
            const route_info_t *rrt = &s_shown[s_row_plane_idx[i]].route;
            if (settings_get()->show_route && rrt->callsign[0] && rrt->valid) {
                char rtxt[16];
                snprintf(rtxt, sizeof(rtxt), "%s " LV_SYMBOL_RIGHT " %s",
                         rrt->origin.iata[0] ? rrt->origin.iata : rrt->origin.icao,
                         rrt->destination.iata[0] ? rrt->destination.iata
                                                  : rrt->destination.icao);
                label_set_if_changed(route_lbl, rtxt);
                lv_obj_clear_flag(route_lbl, LV_OBJ_FLAG_HIDDEN);
            } else {
                lv_obj_add_flag(route_lbl, LV_OBJ_FLAG_HIDDEN);
            }

            lv_obj_t *logo_img = lv_obj_get_child(row, 4);
            const char *code = airline_code(ac, &s_shown[s_row_plane_idx[i]].route);
            const lv_img_dsc_t *logo = code ? logos_get(code) : NULL;
            if (logo != NULL) {
                lv_img_set_src(logo_img, logo);
                lv_obj_clear_flag(logo_img, LV_OBJ_FLAG_HIDDEN);
            } else {
                lv_obj_add_flag(logo_img, LV_OBJ_FLAG_HIDDEN);
            }
        } else {
            lv_obj_add_flag(s_list_rows[i], LV_OBJ_FLAG_HIDDEN);
        }
    }
}


/* ---------- toast + physical input actions (#13) ---------- */

static lv_obj_t *s_toast;

static void toast_close(lv_timer_t *t)
{
    (void)t;
    if (lvgl_port_lock(200 / portTICK_PERIOD_MS)) {
        if (s_toast != NULL) {
            lv_obj_del(s_toast);
            s_toast = NULL;
        }
        lvgl_port_unlock();
    }
}

void ui_toast(const char *text)
{
    if (!lvgl_port_lock(200 / portTICK_PERIOD_MS)) {
        return;
    }
    if (s_toast != NULL) {
        lv_obj_del(s_toast);
        s_toast = NULL;
    }
    s_toast = lv_obj_create(lv_layer_top());
    lv_obj_set_size(s_toast, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_obj_align(s_toast, LV_ALIGN_BOTTOM_MID, 0, -UISY(24));
    lv_obj_set_style_bg_color(s_toast, COL_PANEL, 0);
    lv_obj_set_style_border_width(s_toast, 1, 0);
    lv_obj_set_style_border_color(s_toast, COL_ACCENT, 0);
    lv_obj_set_style_radius(s_toast, UISY(10), 0);
    lv_obj_set_style_pad_all(s_toast, UISY(10), 0);
    lv_obj_clear_flag(s_toast, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_t *l = make_label(s_toast, UIFONT(&font_pl_16, &font_pl_10), COL_TEXT);
    lv_label_set_text(l, text);
    lv_timer_t *t = lv_timer_create(toast_close, 2200, NULL);
    lv_timer_set_repeat_count(t, 1);
    lvgl_port_unlock();
}

/* UI-side actions for input_ctl. MUST be called while holding the LVGL
 * lock. Returns false for names it does not own. */
bool ui_input_action(const char *a)
{
    if (strcmp(a, "next_view") == 0 || strcmp(a, "prev_view") == 0) {
        int v = (s_view_mode + (a[0] == 'n' ? 1 : 4)) % 5;
        apply_view(v);
        return true;
    }
    if (strcmp(a, "next_ac") == 0 || strcmp(a, "prev_ac") == 0) {
        if (s_shown_count == 0) {
            return true;
        }
        int d = a[0] == 'n' ? 1 : s_shown_count - 1;
        s_selected = s_selected < 0 ? 0 : (s_selected + d) % s_shown_count;
        strlcpy(s_selected_hex, s_shown[s_selected].ac.hex, sizeof(s_selected_hex));
        render_list_selection();
        render_right();
        int row = row_of_shown(s_selected);
        if (row >= 0) {
            lv_obj_scroll_to_view(s_list_rows[row], LV_ANIM_ON);
        }
        if (s_cycle_timer != NULL) {
            lv_timer_reset(s_cycle_timer);
        }
        return true;
    }
    if (strcmp(a, "zoom_in") == 0 || strcmp(a, "zoom_out") == 0) {
        double z = s_radar_zoom * (strcmp(a, "zoom_in") == 0 ? 1.0 / 1.5 : 1.5);
        if (z < 0.18) z = 0.18;
        if (z > 3.4) z = 3.4;
        if (z > 0.95 && z < 1.06) z = 1.0;
        s_radar_zoom = z;
        radar_tiles_want();
        return true;
    }
    if (strcmp(a, "wake") == 0) {
        if (s_bl_off) {
            waveshare_rgb_lcd_bl_on();
            ui_apply_brightness();
            s_bl_off = false;
        }
        return true;
    }
    return false;
}
