#include "crt_radar.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "airspace.h"
#include "fonts.h"
#include "rainviewer.h"
#include "routes.h"
#include "runways.h"
#include "settings.h"
#include "tilemap.h"
#include "tz.h"

static const char *TAG = "crt";

/* ---- geometry (scope-local pixels; the scope is a square object) ---- */
#define SCOPE_D     576
#define SCOPE_C     288          /* tube centre */
#define TUBE_R      280          /* edge of the phosphor */
#define RANGE_R     270          /* radius that equals the selected range */
#define SCOPE_X     12
#define SCOPE_Y     12
#define QW          (TUBE_R + 1) /* quadrant angle table width */
#define RAIN_D      (SCOPE_D / 2)
#define RAIN_TMP    192          /* RainViewer serves z7 at most: 192 px holds it */

#define CON_X       604          /* console column */
#define CON_W       408

/* ---- timing ---- */
#define REV_MS      4000         /* one antenna revolution */
#define TICK_MS     66           /* ~15 frames per second */
#define TRAIL_DEG   45
#define ANG_FULL    65536u
#define DEG_U(d)    ((uint32_t)((d) * 65536.0 / 360.0))
#define RAIN_EVERY_MS   (5 * 60 * 1000)
#define RAIN_RETRY_MS   (60 * 1000)

/* ---- phosphor ---- */
#define ECHO_FLOOR  92           /* echo persistence outside the trail, /256 */
#define TRAIL_WASH  58           /* glow of the swept area itself */
#define CLUTTER_R   22
#define BLIP_FLOOR  34
#define HIST_N      5
#define TAG_W       64           /* 8 characters of unscii 16 */
#define TAG_H       32
#define TAG_BYTES   (TAG_W / 8 * TAG_H)
#define K_MAIN      13
#define K_SMALL     5
#define K_RING      21

typedef struct {
    bool     used, alive, ground;
    char     hex[ICAO_HEX_LEN];
    char     cs[CALLSIGN_LEN];
    char     type[TYPE_ICAO_LEN];
    char     reg[REG_LEN];
    char     sq[5];
    double   lat, lon;
    float    gs, trk, dist_nm, brg;
    int      alt, vs;
    long long data_ms;
    /* display state */
    int16_t  x, y;               /* painted position, -1 = not on the scope */
    int16_t  hx[HIST_N], hy[HIST_N];
    uint8_t  hn;
    uint8_t  lvl;                /* drawn intensity, quantized */
    int64_t  paint_ms;
    int16_t  tx, ty;             /* data tag, top left */
    uint8_t  tw;                 /* tag width in px, 0 = no tag */
    char     tag_txt[24];
    uint8_t  tag[TAG_BYTES];
} blip_t;

static lv_obj_t *s_scr;
static crt_radar_exit_cb_t s_on_exit;
static bool s_active;
static bool s_mem_ok;

static uint16_t *s_qang;     /* QW x QW bearing of the NE quadrant, 65536 = 360 deg */
static uint8_t  *s_map;      /* video map, 2 bits per pixel */
static uint8_t  *s_rain;     /* echo intensity, half resolution */
static uint16_t *s_rain_tmp; /* RainViewer render, RGB565 */
static uint8_t  *s_row;      /* one row of intensities */
static uint16_t *s_pal;      /* [0..255] even rows, [256..511] scanline rows */
static uint8_t  *s_glow;     /* 1024 steps of beam distance -> glow */
static uint8_t  *s_vig;      /* vignette by r^2 >> 7 */
static int16_t  *s_hw;       /* tube half width per |dy| */
static uint8_t  *s_kmain, *s_ksmall, *s_kring;
static uint8_t  *s_decay;    /* blip intensity by age / 40 ms */
static blip_t   *s_blips;
static const uint8_t s_map_i[4] = { 0, 44, 74, 120 };
static const uint8_t s_hist_i[HIST_N] = { 84, 62, 44, 30, 18 };

static lv_obj_t *s_root, *s_scope;
static lv_obj_t *s_l_title, *s_l_clock, *s_l_info, *s_l_sel, *s_l_list;
static lv_obj_t *s_l_chip, *s_l_apt;
static lv_timer_t *s_timer;
static lv_font_t s_mono;

static uint32_t s_beam_acc;  /* cumulative beam angle, wraps every 65536 revs */
static bool     s_beam_init;
static double   s_home_lat, s_home_lon;
static bool     s_home_ok;
static float    s_kmpp = 0.2f;   /* km per scope pixel */
static int      s_map_radius = -1;
static double   s_map_lat, s_map_lon;
static int      s_map_asp = -1;
static char     s_sel_hex[ICAO_HEX_LEN];
static int64_t  s_console_ms;

static volatile bool s_rain_busy;
static int64_t  s_rain_ms = -RAIN_EVERY_MS;
static float    s_rain_km;       /* range the layer was built for */
static float    s_rain_req_km;
static double   s_rain_req_lat, s_rain_req_lon;
static int      s_rain_gen = -1;
static char     s_rain_frame[16] = "--:--";

static char s_weather[96];
static int  s_apt_level = 3;
static char s_apt_chip[40], s_apt_text[400];

#define COL_TXT    lv_color_hex(0x6cf06a)
#define COL_DIM    lv_color_hex(0x2f9a3e)
#define COL_AMBER  lv_color_hex(0xffb020)
#define COL_RED    lv_color_hex(0xff4a3a)
#define COL_BG     lv_color_hex(0x0a0d0b)

static inline int64_t now_ms(void)
{
    return esp_timer_get_time() / 1000;
}

static inline uint32_t hash3(uint32_t x, uint32_t y, uint32_t z)
{
    uint32_t h = x * 0x8da6b343u ^ y * 0xd8163841u ^ z * 0xcb1ab31fu;
    h ^= h >> 13;
    h *= 0x5bd1e995u;
    h ^= h >> 15;
    return h;
}

/* Scanline rows: 75 %, with the same bit trick everywhere so the scope and
 * the console darken identically and no seam shows between them. */
static inline uint16_t dark75(uint16_t p)
{
    return (uint16_t)(((p >> 1) & 0x7BEF) + ((p >> 2) & 0x39E7));
}

static inline uint16_t rgb(int r, int g, int b)
{
    return lv_color_make(r < 0 ? 0 : r > 255 ? 255 : r,
                         g < 0 ? 0 : g > 255 ? 255 : g,
                         b < 0 ? 0 : b > 255 ? 255 : b).full;
}

/* ------------------------------------------------------------------ */
/* tables                                                              */
/* ------------------------------------------------------------------ */

/* Intensity to colour: green-yellow afterglow, blue-white where the beam
 * is exciting the phosphor right now, like a P7 tube. */
static void build_palette(void)
{
    static const struct { int i, r, g, b; } cp[] = {
        { 0, 4, 10, 6 },     { 24, 6, 30, 12 },   { 64, 16, 72, 24 },
        { 120, 48, 150, 46 }, { 180, 128, 215, 88 }, { 225, 190, 245, 150 },
        { 255, 225, 250, 255 },
    };
    for (int i = 0; i < 256; i++) {
        int k = 0;
        while (k < 5 && i > cp[k + 1].i) {
            k++;
        }
        int span = cp[k + 1].i - cp[k].i;
        int t = span > 0 ? (i - cp[k].i) * 256 / span : 0;
        int r = cp[k].r + ((cp[k + 1].r - cp[k].r) * t >> 8);
        int g = cp[k].g + ((cp[k + 1].g - cp[k].g) * t >> 8);
        int b = cp[k].b + ((cp[k + 1].b - cp[k].b) * t >> 8);
        s_pal[i] = rgb(r, g, b);
        s_pal[256 + i] = dark75(s_pal[i]);
    }
}

static void build_tables(void)
{
    for (int ay = 0; ay < QW; ay++) {
        for (int ax = 0; ax < QW; ax++) {
            float a = atan2f((float)ax, (float)ay);
            s_qang[ay * QW + ax] = (uint16_t)(a * 65536.0f / (2.0f * (float)M_PI) + 0.5f);
        }
    }
    for (int i = 0; i < 1024; i++) {
        float deg = i * 360.0f / 1024.0f;
        int v = 0;
        if (deg < 0.8f) {
            v = 255;
        } else if (deg < TRAIL_DEG) {
            float t = (deg - 0.8f) / (TRAIL_DEG - 0.8f);
            v = (int)(235.0f * (1.0f - t) * (1.0f - t));
        }
        s_glow[i] = (uint8_t)(v > 254 && deg >= 0.8f ? 254 : v);
    }
    int vn = (TUBE_R * TUBE_R >> 7) + 1;
    for (int i = 0; i < vn; i++) {
        float r = sqrtf((float)(i << 7)) / TUBE_R;
        float f = 1.0f - 0.34f * powf(r, 2.5f);
        if (r > 0.985f) {
            f *= 0.6f;      /* soft tube edge */
        }
        s_vig[i] = (uint8_t)(f * 255.0f);
    }
    for (int dy = 0; dy <= TUBE_R; dy++) {
        s_hw[dy] = (int16_t)sqrtf((float)(TUBE_R * TUBE_R - dy * dy));
    }
    for (int y = 0; y < K_MAIN; y++) {
        for (int x = 0; x < K_MAIN; x++) {
            float d2 = (float)((x - K_MAIN / 2) * (x - K_MAIN / 2) + (y - K_MAIN / 2) * (y - K_MAIN / 2));
            float v = 255.0f * expf(-d2 / (2.0f * 1.4f * 1.4f)) + 70.0f * expf(-d2 / (2.0f * 3.6f * 3.6f));
            s_kmain[y * K_MAIN + x] = (uint8_t)(v > 255.0f ? 255.0f : v);
        }
    }
    for (int y = 0; y < K_SMALL; y++) {
        for (int x = 0; x < K_SMALL; x++) {
            float d2 = (float)((x - 2) * (x - 2) + (y - 2) * (y - 2));
            s_ksmall[y * K_SMALL + x] = (uint8_t)(255.0f * expf(-d2 / (2.0f * 0.9f * 0.9f)));
        }
    }
    for (int y = 0; y < K_RING; y++) {
        for (int x = 0; x < K_RING; x++) {
            float d = sqrtf((float)((x - 10) * (x - 10) + (y - 10) * (y - 10)));
            float v = 255.0f * expf(-(d - 8.5f) * (d - 8.5f) / (2.0f * 0.6f * 0.6f));
            s_kring[y * K_RING + x] = (uint8_t)v;
        }
    }
    for (int i = 0; i < 256; i++) {
        float age = i * 40.0f;
        s_decay[i] = (uint8_t)(BLIP_FLOOR + (255 - BLIP_FLOOR) * expf(-age / 1500.0f));
    }
}

static void *ps_alloc(size_t n)
{
    void *p = heap_caps_calloc(1, n, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (p == NULL) {
        ESP_LOGE(TAG, "PSRAM alloc of %u bytes failed", (unsigned)n);
    }
    return p;
}

static bool ensure_mem(void)
{
    if (s_mem_ok) {
        return true;
    }
    s_qang = ps_alloc((size_t)QW * QW * 2);
    s_map = ps_alloc((size_t)SCOPE_D * SCOPE_D / 4);
    s_rain = ps_alloc((size_t)RAIN_D * RAIN_D);
    s_rain_tmp = ps_alloc((size_t)RAIN_TMP * RAIN_TMP * 2);
    s_blips = ps_alloc(sizeof(blip_t) * MAX_AIRCRAFT);
    uint8_t *small = ps_alloc(SCOPE_D + 512 * 2 + 1024 + ((TUBE_R * TUBE_R >> 7) + 1) +
                              (TUBE_R + 1) * 2 + K_MAIN * K_MAIN + K_SMALL * K_SMALL +
                              K_RING * K_RING + 256 + 16);
    if (!s_qang || !s_map || !s_rain || !s_rain_tmp || !s_blips || !small) {
        free(s_qang);
        free(s_map);
        free(s_rain);
        free(s_rain_tmp);
        free(s_blips);
        free(small);
        s_qang = NULL;
        s_map = s_rain = NULL;
        s_rain_tmp = NULL;
        s_blips = NULL;
        return false;
    }
    uint8_t *p = small;
    s_pal = (uint16_t *)p;           p += 512 * 2;
    s_hw = (int16_t *)p;             p += (TUBE_R + 1) * 2;
    s_row = p;                       p += SCOPE_D;
    s_glow = p;                      p += 1024;
    s_vig = p;                       p += (TUBE_R * TUBE_R >> 7) + 1;
    s_kmain = p;                     p += K_MAIN * K_MAIN;
    s_ksmall = p;                    p += K_SMALL * K_SMALL;
    s_kring = p;                     p += K_RING * K_RING;
    s_decay = p;
    build_palette();
    build_tables();
    for (int i = 0; i < MAX_AIRCRAFT; i++) {
        s_blips[i].x = -1;
    }
    s_mem_ok = true;
    ESP_LOGI(TAG, "scope buffers ready, PSRAM free %u",
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    return true;
}

/* ------------------------------------------------------------------ */
/* text rasterizer (data tags and the video map share the LVGL fonts)  */
/* ------------------------------------------------------------------ */

typedef void (*plot_fn)(void *ctx, int x, int y);

static int raster_text(const lv_font_t *f, const char *s, int x, int y, plot_fn fn, void *ctx)
{
    uint32_t i = 0;
    while (s[i] != '\0') {
        uint32_t letter = _lv_txt_encoded_next(s, &i);
        uint32_t j = i;
        uint32_t next = s[j] != '\0' ? _lv_txt_encoded_next(s, &j) : 0;
        lv_font_glyph_dsc_t g;
        if (!lv_font_get_glyph_dsc(f, &g, letter, next)) {
            x += 8;
            continue;
        }
        const lv_font_t *rf = g.resolved_font != NULL ? g.resolved_font : f;
        const uint8_t *bmp = g.box_w > 0 ? lv_font_get_glyph_bitmap(rf, letter) : NULL;
        int bpp = g.bpp;
        if (bmp != NULL && (bpp == 1 || bpp == 2 || bpp == 4 || bpp == 8)) {
            int bx = x + g.ofs_x;
            int by = y + (f->line_height - f->base_line) - g.box_h - g.ofs_y;
            uint32_t maxv = (1u << bpp) - 1;
            uint32_t bit = 0;
            for (int gy = 0; gy < g.box_h; gy++) {
                for (int gx = 0; gx < g.box_w; gx++, bit += bpp) {
                    uint32_t v = (bmp[bit >> 3] >> (8 - bpp - (bit & 7))) & maxv;
                    if (v * 2 > maxv) {
                        fn(ctx, bx + gx, by + gy);
                    }
                }
            }
        }
        x += g.adv_w;
    }
    return x;
}

/* ------------------------------------------------------------------ */
/* video map                                                           */
/* ------------------------------------------------------------------ */

static inline void map_set(int x, int y, int v)
{
    if ((unsigned)x >= SCOPE_D || (unsigned)y >= SCOPE_D) {
        return;
    }
    int dx = x - SCOPE_C, dy = y - SCOPE_C;
    if (dx * dx + dy * dy > (TUBE_R - 1) * (TUBE_R - 1)) {
        return;
    }
    uint8_t *p = &s_map[(y * SCOPE_D + x) >> 2];
    int sh = (x & 3) * 2;
    if (((*p >> sh) & 3) < v) {
        *p = (uint8_t)((*p & ~(3 << sh)) | (v << sh));
    }
}

static void map_line(int x0, int y0, int x1, int y1, int v, int dash)
{
    int dx = abs(x1 - x0), dy = -abs(y1 - y0);
    int sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1;
    int err = dx + dy, n = 0;
    for (;;) {
        if (dash == 0 || (n / dash) % 2 == 0) {
            map_set(x0, y0, v);
        }
        n++;
        if (x0 == x1 && y0 == y1) {
            break;
        }
        int e2 = 2 * err;
        if (e2 >= dy) {
            err += dy;
            x0 += sx;
        }
        if (e2 <= dx) {
            err += dx;
            y0 += sy;
        }
    }
}

static void map_circle(int r, int v)
{
    int x = r, y = 0, err = 1 - r;
    while (x >= y) {
        map_set(SCOPE_C + x, SCOPE_C + y, v);
        map_set(SCOPE_C + y, SCOPE_C + x, v);
        map_set(SCOPE_C - y, SCOPE_C + x, v);
        map_set(SCOPE_C - x, SCOPE_C + y, v);
        map_set(SCOPE_C - x, SCOPE_C - y, v);
        map_set(SCOPE_C - y, SCOPE_C - x, v);
        map_set(SCOPE_C + y, SCOPE_C - x, v);
        map_set(SCOPE_C + x, SCOPE_C - y, v);
        y++;
        if (err < 0) {
            err += 2 * y + 1;
        } else {
            x--;
            err += 2 * (y - x) + 1;
        }
    }
}

static void plot_map2(void *ctx, int x, int y)
{
    map_set(x, y, (int)(intptr_t)ctx);
}

static void geo_to_scope(double lat, double lon, float *x, float *y)
{
    double e = (lon - s_home_lon) * 111.32 * cos(s_home_lat * M_PI / 180.0);
    double n = (lat - s_home_lat) * 110.57;
    *x = SCOPE_C + (float)(e / s_kmpp);
    *y = SCOPE_C - (float)(n / s_kmpp);
}

static bool seg_visible(float x0, float y0, float x1, float y1)
{
    float lim = TUBE_R + 40;
    return !((x0 < SCOPE_C - lim && x1 < SCOPE_C - lim) || (x0 > SCOPE_C + lim && x1 > SCOPE_C + lim) ||
             (y0 < SCOPE_C - lim && y1 < SCOPE_C - lim) || (y0 > SCOPE_C + lim && y1 > SCOPE_C + lim));
}

static void build_map(void)
{
    memset(s_map, 0, (size_t)SCOPE_D * SCOPE_D / 4);
    int range = settings_get()->radius_nm;
    s_kmpp = (float)(range * 1.852 / RANGE_R);

    int step = range <= 15 ? 5 : range <= 40 ? 10 : range <= 80 ? 20 : 50;
    for (int nm = step; nm <= range; nm += step) {
        int r = (int)(nm * RANGE_R / (float)range + 0.5f);
        map_circle(r, 1);
        char t[8];
        snprintf(t, sizeof(t), "%d", nm);
        float a = 0.32f;     /* label a little east of north */
        raster_text(&s_mono, t, SCOPE_C + (int)(r * sinf(a)) + 3,
                    SCOPE_C - (int)(r * cosf(a)) - 15, plot_map2, (void *)2);
    }
    for (int deg = 0; deg < 360; deg += 5) {
        int len = deg % 30 == 0 ? 14 : deg % 10 == 0 ? 8 : 4;
        float a = deg * (float)M_PI / 180.0f;
        float s = sinf(a), c = cosf(a);
        map_line(SCOPE_C + (int)((TUBE_R - 2) * s), SCOPE_C - (int)((TUBE_R - 2) * c),
                 SCOPE_C + (int)((TUBE_R - 2 - len) * s), SCOPE_C - (int)((TUBE_R - 2 - len) * c),
                 2, 0);
        if (deg % 30 == 0) {
            char t[8];
            snprintf(t, sizeof(t), "%03d", deg);
            int lx = SCOPE_C + (int)((TUBE_R - 30) * s) - 12;
            int ly = SCOPE_C - (int)((TUBE_R - 30) * c) - 8;
            raster_text(&s_mono, t, lx, ly, plot_map2, (void *)2);
        }
    }
    /* home: a small cross */
    map_line(SCOPE_C - 5, SCOPE_C, SCOPE_C + 5, SCOPE_C, 3, 0);
    map_line(SCOPE_C, SCOPE_C - 5, SCOPE_C, SCOPE_C + 5, 3, 0);

    /* runways, with dashed extended centrelines on the long ones */
    int nrw = runways_count();
    for (int i = 0; i < nrw; i++) {
        float la0, lo0, la1, lo1, x0, y0, x1, y1;
        runways_get(i, &la0, &lo0, &la1, &lo1);
        geo_to_scope(la0, lo0, &x0, &y0);
        geo_to_scope(la1, lo1, &x1, &y1);
        if (!seg_visible(x0, y0, x1, y1)) {
            continue;
        }
        map_line((int)x0, (int)y0, (int)x1, (int)y1, 3, 0);
        map_line((int)x0 + 1, (int)y0, (int)x1 + 1, (int)y1, 3, 0);
        float len_px = hypotf(x1 - x0, y1 - y0);
        if (len_px * s_kmpp > 1.8f) {
            float ux = (x1 - x0) / len_px, uy = (y1 - y0) / len_px;
            float ext = 10.0f * 1.852f / s_kmpp;    /* 10 NM final */
            map_line((int)x0, (int)y0, (int)(x0 - ux * ext), (int)(y0 - uy * ext), 1, 4);
            map_line((int)x1, (int)y1, (int)(x1 + ux * ext), (int)(y1 + uy * ext), 1, 4);
        }
    }

    int nasp = airspace_count();
    for (int i = 0; i < nasp; i++) {
        const airspace_t *a = airspace_get(i);
        if (a == NULL || a->n_points < 2) {
            continue;
        }
        int v = a->type == 4 ? 2 : 1;   /* openAIP 4 = CTR */
        for (int k = 0; k < a->n_points; k++) {
            int k2 = (k + 1) % a->n_points;
            float x0, y0, x1, y1;
            geo_to_scope(a->lat[k], a->lon[k], &x0, &y0);
            geo_to_scope(a->lat[k2], a->lon[k2], &x1, &y1);
            if (seg_visible(x0, y0, x1, y1)) {
                map_line((int)x0, (int)y0, (int)x1, (int)y1, v, 6);
            }
        }
    }
    s_map_radius = range;
    s_map_lat = s_home_lat;
    s_map_lon = s_home_lon;
    s_map_asp = nasp;
    ESP_LOGI(TAG, "video map: %d NM, %d runways scanned, %d airspaces", range, nrw, nasp);
}

/* ------------------------------------------------------------------ */
/* precipitation                                                       */
/* ------------------------------------------------------------------ */

static void rain_job(void)
{
    double hlat = s_rain_req_lat, hlon = s_rain_req_lon;
    float km = s_rain_req_km;
    float kmpp = km / RANGE_R;
    double dlat = km * 1.06 / 110.57;
    double dlon = km * 1.06 / (111.32 * cos(hlat * M_PI / 180.0));
    tile_view_t v;
    bool ok = tilemap_render_rain(s_rain_tmp, RAIN_TMP, RAIN_TMP,
                                  hlat - dlat, hlat + dlat, hlon - dlon, hlon + dlon, &v);
    if (ok) {
        double coslat = cos(hlat * M_PI / 180.0);
        int lit = 0;
        for (int cy = 0; cy < RAIN_D; cy++) {
            double n_km = (SCOPE_C - (2 * cy + 1)) * kmpp;
            double lat = hlat + n_km / 110.57;
            double lon_a = hlon + (1 - SCOPE_C) * kmpp / (111.32 * coslat);
            double lon_b = hlon + (SCOPE_D - 1 - SCOPE_C) * kmpp / (111.32 * coslat);
            int xa, ya, xb, yb;
            tilemap_project(&v, lat, lon_a, &xa, &ya);
            tilemap_project(&v, lat, lon_b, &xb, &yb);
            uint8_t *row = s_rain + (size_t)cy * RAIN_D;
            for (int cx = 0; cx < RAIN_D; cx++) {
                int sx = 2 * cx + 1;
                int px = xa + (xb - xa) * (sx - 1) / (SCOPE_D - 2);
                int py = ya;
                uint8_t e = 0;
                if (px >= 0 && px < RAIN_TMP && py >= 0 && py < RAIN_TMP) {
                    uint16_t c = s_rain_tmp[py * RAIN_TMP + px];
                    int r = (c >> 11) << 3, g = ((c >> 5) & 0x3F) << 2, b = (c & 0x1F) << 3;
                    int mx = r > g ? (r > b ? r : b) : (g > b ? g : b);
                    if (mx >= 24) {
                        /* RainViewer's scheme runs blue -> yellow -> red with
                         * intensity, so red is the best single measure */
                        int iv = 64 + (r * 150 >> 8) + (mx >> 3);
                        e = (uint8_t)(iv > 255 ? 255 : iv);
                        lit++;
                    }
                }
                row[cx] = e;
            }
        }
        s_rain_km = km;
        s_rain_gen = rainviewer_generation();
        const char *fp = rainviewer_frame_path();
        const char *slash = strrchr(fp, '/');
        if (slash != NULL) {
            time_t t = (time_t)atoll(slash + 1);
            if (t > 1600000000) {
                time_t l = t + (tz_home_known() ? tz_home_offset() : 0);
                struct tm tm;
                gmtime_r(&l, &tm);
                snprintf(s_rain_frame, sizeof(s_rain_frame), "%02d:%02d", tm.tm_hour, tm.tm_min);
            }
        }
        s_rain_ms = now_ms();
        ESP_LOGI(TAG, "rain layer z%d, %d echo cells, frame %s", v.z, lit, s_rain_frame);
    } else {
        s_rain_ms = now_ms() - RAIN_EVERY_MS + RAIN_RETRY_MS;
        ESP_LOGW(TAG, "rain render failed, retry in 60 s");
    }
    s_rain_busy = false;
}

static void rain_maybe_request(void)
{
    if (!s_home_ok || s_rain_busy) {
        return;
    }
    float km = settings_get()->radius_nm * 1.852f;
    bool range_moved = fabsf(km - s_rain_km) > 0.5f;
    if (!range_moved && now_ms() - s_rain_ms < RAIN_EVERY_MS) {
        return;
    }
    if (range_moved) {
        memset(s_rain, 0, (size_t)RAIN_D * RAIN_D);   /* old echoes at the wrong scale */
    }
    s_rain_req_km = km;
    s_rain_req_lat = s_home_lat;
    s_rain_req_lon = s_home_lon;
    s_rain_busy = true;
    s_rain_ms = now_ms();
    if (!tilemap_worker_submit(rain_job)) {
        s_rain_busy = false;
    }
}

/* ------------------------------------------------------------------ */
/* blips                                                               */
/* ------------------------------------------------------------------ */

static void blip_est(const blip_t *b, int64_t now, float *x, float *y)
{
    float dt = (float)(now - b->data_ms) / 1000.0f;
    if (dt < 0) {
        dt = 0;
    } else if (dt > 60) {
        dt = 60;
    }
    geo_to_scope(b->lat, b->lon, x, y);
    float d_km = b->gs * 1.852f * dt / 3600.0f;
    float a = b->trk * (float)M_PI / 180.0f;
    *x += sinf(a) * d_km / s_kmpp;
    *y -= cosf(a) * d_km / s_kmpp;
}

static inline uint32_t bearing_u(float x, float y)
{
    float a = atan2f(x - SCOPE_C, SCOPE_C - y);
    return (uint32_t)((int32_t)(a * 65536.0f / (2.0f * (float)M_PI))) & 0xFFFF;
}

static inline uint8_t blip_level(const blip_t *b, int64_t now)
{
    int64_t age = now - b->paint_ms;
    if (age < 0) {
        age = 0;
    }
    int i = (int)(age / 40);
    return i < 256 ? s_decay[i] : BLIP_FLOOR;
}

/* Bounding box of everything a blip draws, scope coordinates. */
static void blip_box(const blip_t *b, lv_area_t *a, bool with_hist)
{
    a->x1 = b->x - K_RING / 2;
    a->x2 = b->x + K_RING / 2;
    a->y1 = b->y - K_RING / 2;
    a->y2 = b->y + K_RING / 2;
    if (b->tw > 0) {
        a->x1 = LV_MIN(a->x1, b->tx);
        a->x2 = LV_MAX(a->x2, b->tx + b->tw - 1);
        a->y1 = LV_MIN(a->y1, b->ty);
        a->y2 = LV_MAX(a->y2, b->ty + TAG_H - 1);
    }
    if (with_hist) {
        for (int k = 0; k < b->hn; k++) {
            a->x1 = LV_MIN(a->x1, b->hx[k] - 2);
            a->x2 = LV_MAX(a->x2, b->hx[k] + 2);
            a->y1 = LV_MIN(a->y1, b->hy[k] - 2);
            a->y2 = LV_MAX(a->y2, b->hy[k] + 2);
        }
    }
}

static void inval_scope(const lv_area_t *sa)
{
    lv_area_t a = {
        .x1 = s_scope->coords.x1 + sa->x1, .y1 = s_scope->coords.y1 + sa->y1,
        .x2 = s_scope->coords.x1 + sa->x2, .y2 = s_scope->coords.y1 + sa->y2,
    };
    lv_obj_invalidate_area(s_scope, &a);
}

static void inval_blip(const blip_t *b, bool with_hist)
{
    if (b->x < 0) {
        return;
    }
    lv_area_t a;
    blip_box(b, &a, with_hist);
    inval_scope(&a);
}

static void plot_tag(void *ctx, int x, int y)
{
    uint8_t *bits = ctx;
    if ((unsigned)x < TAG_W && (unsigned)y < TAG_H) {
        bits[y * (TAG_W / 8) + (x >> 3)] |= (uint8_t)(0x80 >> (x & 7));
    }
}

static void blip_render_tag(blip_t *b)
{
    char l2[12];
    int fl = b->alt / 100;
    if (fl < 0) {
        fl = 0;
    } else if (fl > 999) {
        fl = 999;
    }
    char trend = b->vs > 400 ? '^' : b->vs < -400 ? 'v' : ' ';
    snprintf(l2, sizeof(l2), "%03d%c%d", fl, trend, (int)(b->gs + 0.5f));
    char txt[24];
    snprintf(txt, sizeof(txt), "%s|%s", b->cs[0] ? b->cs : b->hex, l2);
    if (strcmp(txt, b->tag_txt) == 0) {
        return;
    }
    strlcpy(b->tag_txt, txt, sizeof(b->tag_txt));
    memset(b->tag, 0, TAG_BYTES);
    const char *l1 = b->cs[0] ? b->cs : b->hex;
    raster_text(&s_mono, l1, 0, 0, plot_tag, b->tag);
    raster_text(&s_mono, l2, 0, 16, plot_tag, b->tag);
}

static bool tag_overlaps(const blip_t *self, int x, int y, int w)
{
    for (int i = 0; i < MAX_AIRCRAFT; i++) {
        const blip_t *o = &s_blips[i];
        if (o == self || !o->used || o->x < 0 || o->tw == 0) {
            continue;
        }
        if (x < o->tx + o->tw && o->tx < x + w && y < o->ty + TAG_H && o->ty < y + TAG_H) {
            return true;
        }
    }
    return false;
}

static void blip_place_tag(blip_t *b)
{
    int l1 = (int)strlen(b->cs[0] ? b->cs : b->hex);
    const char *bar = strchr(b->tag_txt, '|');
    int l2 = bar != NULL ? (int)strlen(bar + 1) : 0;
    int w = (l1 > l2 ? l1 : l2) * 8;
    if (w > TAG_W) {
        w = TAG_W;
    }
    const int cand[4][2] = { { 9, -30 }, { 9, 2 }, { -9 - w, -30 }, { -9 - w, 2 } };
    bool sel = s_sel_hex[0] && strcmp(b->hex, s_sel_hex) == 0;
    for (int k = 0; k < 4; k++) {
        int x = b->x + cand[k][0], y = b->y + cand[k][1];
        if (x < 0 || y < 0 || x + w > SCOPE_D || y + TAG_H > SCOPE_D) {
            continue;
        }
        if (!sel && tag_overlaps(b, x, y, w)) {
            continue;
        }
        b->tx = (int16_t)x;
        b->ty = (int16_t)y;
        b->tw = (uint8_t)w;
        return;
    }
    b->tw = 0;
}

static void blip_remove(blip_t *b)
{
    inval_blip(b, true);
    b->used = false;
    b->x = -1;
    b->hn = 0;
    b->tw = 0;
}

/* The beam just crossed this target: move the echo to where it is now. */
static void blip_paint(blip_t *b, float fx, float fy, int64_t now)
{
    int x = (int)lroundf(fx), y = (int)lroundf(fy);
    int dx = x - SCOPE_C, dy = y - SCOPE_C;
    if (dx * dx + dy * dy > (TUBE_R - 4) * (TUBE_R - 4)) {
        if (b->x >= 0) {
            inval_blip(b, true);
            b->x = -1;
            b->hn = 0;
            b->tw = 0;
        }
        return;
    }
    if (b->x >= 0) {
        inval_blip(b, true);
        if (abs(x - b->x) + abs(y - b->y) >= 2) {
            if (b->hn == HIST_N) {
                b->hn--;
            }
            memmove(&b->hx[1], &b->hx[0], sizeof(b->hx[0]) * b->hn);
            memmove(&b->hy[1], &b->hy[0], sizeof(b->hy[0]) * b->hn);
            b->hx[0] = b->x;
            b->hy[0] = b->y;
            b->hn++;
        }
    } else {
        b->hn = 0;
    }
    b->x = (int16_t)x;
    b->y = (int16_t)y;
    b->paint_ms = now;
    b->lvl = 255 >> 4;
    blip_render_tag(b);
    blip_place_tag(b);
    inval_blip(b, true);
}

/* ------------------------------------------------------------------ */
/* drawing                                                             */
/* ------------------------------------------------------------------ */

static inline void add_sat(uint8_t *p, uint32_t a)
{
    uint32_t v = *p + a;
    *p = v > 255 ? 255 : (uint8_t)v;
}

static inline void stamp(const uint8_t *k, int ks, int cx, int cy, uint32_t pw,
                         int sy, int x0, int x1, uint8_t *I)
{
    int kh = ks / 2;
    int ky = sy - cy + kh;
    if (ky < 0 || ky >= ks) {
        return;
    }
    int xa = cx - kh, xb = cx + kh;
    if (xa < x0) {
        xa = x0;
    }
    if (xb > x1) {
        xb = x1;
    }
    const uint8_t *kr = k + ky * ks;
    for (int x = xa; x <= xb; x++) {
        add_sat(&I[x - x0], (kr[x - cx + kh] * pw) >> 8);
    }
}

static void stamp_blip(const blip_t *b, int64_t now, int sy, int x0, int x1, uint8_t *I)
{
    for (int k = 0; k < b->hn; k++) {
        stamp(s_ksmall, K_SMALL, b->hx[k], b->hy[k], s_hist_i[k], sy, x0, x1, I);
    }
    uint32_t pw = blip_level(b, now);
    stamp(s_kmain, K_MAIN, b->x, b->y, pw, sy, x0, x1, I);
    bool sel = s_sel_hex[0] && strcmp(b->hex, s_sel_hex) == 0;
    if (sel) {
        stamp(s_kring, K_RING, b->x, b->y, 190, sy, x0, x1, I);
    }
    if (b->tw > 0) {
        int ty = sy - b->ty;
        if (ty >= 0 && ty < TAG_H) {
            uint8_t ti = sel ? 255 : (uint8_t)LV_MAX(118, pw * 220 / 256);
            const uint8_t *bits = b->tag + ty * (TAG_W / 8);
            int xa = LV_MAX(x0, b->tx), xb = LV_MIN(x1, b->tx + b->tw - 1);
            for (int x = xa; x <= xb; x++) {
                int bx = x - b->tx;
                if (bits[bx >> 3] & (0x80 >> (bx & 7))) {
                    if (I[x - x0] < ti) {
                        I[x - x0] = ti;
                    }
                }
            }
        }
    }
}

static void compose_span(int sy, int x0, int x1, uint16_t *dst,
                         const uint8_t *bl, int nbl, int64_t now)
{
    const int dy = sy - SCOPE_C;
    const int ady = dy < 0 ? -dy : dy;
    const bool odd = (sy + SCOPE_Y) & 1;
    const uint16_t *pal = s_pal + (odd ? 256 : 0);
    const int hw = ady <= TUBE_R ? s_hw[ady] : -1;
    const int tx0 = SCOPE_C - hw, tx1 = SCOPE_C + hw;
    uint8_t *I = s_row;
    const uint32_t acc = s_beam_acc;
    const uint32_t beam = acc & 0xFFFF;
    const uint16_t *qrow = s_qang + (size_t)(ady <= TUBE_R ? ady : 0) * QW;
    const uint8_t *mrow = s_map + (size_t)sy * (SCOPE_D / 4);
    const uint8_t *rrow = s_rain + (size_t)(sy >> 1) * RAIN_D;
    const int ix0 = x0 > tx0 ? x0 : tx0;
    const int ix1 = x1 < tx1 ? x1 : tx1;

    for (int x = ix0; x <= ix1; x++) {
        int dx = x - SCOPE_C;
        uint32_t b;
        if (dx >= 0) {
            uint32_t a = qrow[dx];
            b = dy <= 0 ? a : 32768u - a;
        } else {
            uint32_t a = qrow[-dx];
            b = dy > 0 ? 32768u + a : 65536u - a;
        }
        b &= 0xFFFF;
        uint32_t g = s_glow[((beam - b) & 0xFFFF) >> 6];
        uint32_t v;
        if (g == 255) {
            v = 255;
        } else {
            uint32_t h = hash3((uint32_t)x, (uint32_t)sy, (acc - b) >> 16);
            uint32_t persist = ECHO_FLOOR + ((g * (256 - ECHO_FLOOR)) >> 8);
            v = (g * TRAIL_WASH) >> 8;
            uint32_t e = rrow[x >> 1];
            if (e) {
                e = (e * (150 + (h & 105))) >> 8;
                v += (e * persist) >> 8;
            }
            if (((h >> 8) & 1023) < 5) {
                v += 24 + (g >> 2);          /* receiver noise */
            }
            int r2 = dx * dx + dy * dy;
            if (r2 < CLUTTER_R * CLUTTER_R) {
                uint32_t c = ((h >> 20) & 127) * (uint32_t)(CLUTTER_R * CLUTTER_R - r2) /
                             (CLUTTER_R * CLUTTER_R);
                v += (c * persist) >> 8;
            }
            uint32_t mv = s_map_i[(mrow[x >> 2] >> ((x & 3) * 2)) & 3];
            if (mv > v) {
                v = mv;
            }
            if (v > 254) {
                v = 254;
            }
        }
        I[x - x0] = (uint8_t)v;
    }

    for (int k = 0; k < nbl; k++) {
        stamp_blip(&s_blips[bl[k]], now, sy, x0, x1, I);
    }

    /* bezel colours for this row: dark gap, metal ring lit from above, desk */
    int shade = 30 - dy * 12 / TUBE_R;
    uint16_t c_gap = rgb(2, 4, 3);
    uint16_t c_ring = rgb(shade, shade + 3, shade);
    uint16_t c_edge = rgb(shade + 18, shade + 21, shade + 18);
    uint16_t c_bg = COL_BG.full;
    if (odd) {
        c_gap = dark75(c_gap);
        c_ring = dark75(c_ring);
        c_edge = dark75(c_edge);
        c_bg = dark75(c_bg);
    }
    const int r_gap = (TUBE_R + 2) * (TUBE_R + 2);
    const int r_ring = (TUBE_R + 8) * (TUBE_R + 8);
    const int r_edge = (TUBE_R + 9) * (TUBE_R + 9);

    for (int x = x0; x <= x1; x++) {
        int dx = x - SCOPE_C;
        int r2 = dx * dx + dy * dy;
        uint16_t c;
        if (x >= tx0 && x <= tx1) {
            c = pal[(I[x - x0] * s_vig[r2 >> 7]) >> 8];
        } else if (r2 <= r_gap) {
            c = c_gap;
        } else if (r2 <= r_ring) {
            c = c_ring;
        } else if (r2 <= r_edge) {
            c = c_edge;
        } else {
            c = c_bg;
        }
        dst[x - x0] = c;
    }
}

static void scope_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t *obj = lv_event_get_target(e);
    if (code == LV_EVENT_COVER_CHECK) {
        lv_cover_check_info_t *info = lv_event_get_param(e);
        if (info->res != LV_COVER_RES_MASKED) {
            info->res = _lv_area_is_in(info->area, &obj->coords, 0) ? LV_COVER_RES_COVER
                                                                     : LV_COVER_RES_NOT_COVER;
        }
    } else if (code == LV_EVENT_DRAW_MAIN) {
        lv_draw_ctx_t *dc = lv_event_get_draw_ctx(e);
        lv_area_t a;
        if (!_lv_area_intersect(&a, dc->clip_area, &obj->coords)) {
            return;
        }
        int ox = obj->coords.x1, oy = obj->coords.y1;
        lv_area_t sa = { a.x1 - ox, a.y1 - oy, a.x2 - ox, a.y2 - oy };
        uint8_t bl[MAX_AIRCRAFT];
        int nbl = 0;
        for (int i = 0; i < MAX_AIRCRAFT; i++) {
            const blip_t *b = &s_blips[i];
            if (!b->used || b->x < 0) {
                continue;
            }
            lv_area_t bb;
            blip_box(b, &bb, true);
            if (bb.x1 <= sa.x2 && bb.x2 >= sa.x1 && bb.y1 <= sa.y2 && bb.y2 >= sa.y1) {
                bl[nbl++] = (uint8_t)i;
            }
        }
        int stride = lv_area_get_width(dc->buf_area);
        int64_t now = now_ms();
        for (int y = a.y1; y <= a.y2; y++) {
            lv_color_t *row = (lv_color_t *)dc->buf +
                              (size_t)(y - dc->buf_area->y1) * stride + (a.x1 - dc->buf_area->x1);
            compose_span(y - oy, sa.x1, sa.x2, (uint16_t *)row, bl, nbl, now);
        }
    } else if (code == LV_EVENT_CLICKED) {
        lv_point_t p;
        lv_indev_get_point(lv_indev_get_act(), &p);
        int px = p.x - obj->coords.x1, py = p.y - obj->coords.y1;
        int best = -1, best_d = 28 * 28;
        for (int i = 0; i < MAX_AIRCRAFT; i++) {
            const blip_t *b = &s_blips[i];
            if (!b->used || b->x < 0) {
                continue;
            }
            int d = (b->x - px) * (b->x - px) + (b->y - py) * (b->y - py);
            if (d < best_d) {
                best_d = d;
                best = i;
            }
        }
        for (int i = 0; i < MAX_AIRCRAFT; i++) {
            if (s_blips[i].used && s_sel_hex[0] && strcmp(s_blips[i].hex, s_sel_hex) == 0) {
                inval_blip(&s_blips[i], false);
            }
        }
        if (best >= 0 && strcmp(s_blips[best].hex, s_sel_hex) != 0) {
            strlcpy(s_sel_hex, s_blips[best].hex, sizeof(s_sel_hex));
            inval_blip(&s_blips[best], false);
            blip_place_tag(&s_blips[best]);
            inval_blip(&s_blips[best], false);
        } else {
            s_sel_hex[0] = '\0';
        }
        s_console_ms = 0;   /* refresh the console on the next tick */
    }
}

/* Scanlines over everything outside the scope (the scope does its own). */
static void root_event_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_DRAW_POST) {
        return;
    }
    lv_draw_ctx_t *dc = lv_event_get_draw_ctx(e);
    const lv_area_t *c = dc->clip_area;
    const lv_area_t *s = &s_scope->coords;
    int stride = lv_area_get_width(dc->buf_area);
    for (int y = c->y1 | 1; y <= c->y2; y += 2) {
        uint16_t *row = (uint16_t *)dc->buf + (size_t)(y - dc->buf_area->y1) * stride - dc->buf_area->x1;
        bool in_scope_rows = y >= s->y1 && y <= s->y2;
        for (int x = c->x1; x <= c->x2; x++) {
            if (in_scope_rows && x >= s->x1 && x <= s->x2) {
                x = s->x2;
                continue;
            }
            row[x] = dark75(row[x]);
        }
    }
}

/* ------------------------------------------------------------------ */
/* sector invalidation                                                 */
/* ------------------------------------------------------------------ */

static void edge_point(uint32_t a, int *x, int *y)
{
    float r = a * (2.0f * (float)M_PI / 65536.0f);
    *x = SCOPE_C + (int)lroundf((TUBE_R + 1) * sinf(r));
    *y = SCOPE_C - (int)lroundf((TUBE_R + 1) * cosf(r));
}

/* Invalidate the wedge from angle `from` clockwise through `span` units, as
 * one rectangle per quadrant it touches: inside a quadrant the arc is
 * monotonic, so its ends and the centre bound it. */
static void invalidate_wedge(uint32_t from, uint32_t span)
{
    uint32_t a = from & 0xFFFF;
    while (span > 0) {
        uint32_t q_end = (a / 16384u + 1u) * 16384u;
        uint32_t seg = q_end - a;
        if (seg > span) {
            seg = span;
        }
        int xa, ya, xb, yb;
        edge_point(a, &xa, &ya);
        edge_point(a + seg, &xb, &yb);
        lv_area_t r = {
            .x1 = LV_MIN(SCOPE_C, LV_MIN(xa, xb)) - 2, .y1 = LV_MIN(SCOPE_C, LV_MIN(ya, yb)) - 2,
            .x2 = LV_MAX(SCOPE_C, LV_MAX(xa, xb)) + 2, .y2 = LV_MAX(SCOPE_C, LV_MAX(ya, yb)) + 2,
        };
        inval_scope(&r);
        a = (a + seg) & 0xFFFF;
        span -= seg;
    }
}

/* ------------------------------------------------------------------ */
/* console                                                             */
/* ------------------------------------------------------------------ */

static void set_text(lv_obj_t *l, const char *t)
{
    if (strcmp(lv_label_get_text(l), t) != 0) {
        lv_label_set_text(l, t);
    }
}

static int cmp_dist(const void *a, const void *b)
{
    float da = s_blips[*(const uint8_t *)a].dist_nm, db = s_blips[*(const uint8_t *)b].dist_nm;
    return da < db ? -1 : da > db;
}

static void console_update(int64_t now)
{
    time_t t = time(NULL);
    if (t > 1600000000) {
        time_t l = t + (tz_home_known() ? tz_home_offset() : 0);
        struct tm tm;
        gmtime_r(&l, &tm);
        char clk[16];
        snprintf(clk, sizeof(clk), "%02d:%02d:%02d", tm.tm_hour, tm.tm_min, tm.tm_sec);
        set_text(s_l_clock, clk);
    }

    uint8_t idx[MAX_AIRCRAFT];
    int n = 0;
    for (int i = 0; i < MAX_AIRCRAFT; i++) {
        if (s_blips[i].used && s_blips[i].alive && !s_blips[i].ground) {
            idx[n++] = (uint8_t)i;
        }
    }
    qsort(idx, n, 1, cmp_dist);

    char buf[720];
    snprintf(buf, sizeof(buf), "RNG %d NM   ROT %.1f S   TGT %d\nWX %s  %.60s",
             settings_get()->radius_nm, REV_MS / 1000.0, n, s_rain_frame, s_weather);
    set_text(s_l_info, buf);

    const blip_t *sel = NULL;
    for (int i = 0; i < MAX_AIRCRAFT && s_sel_hex[0]; i++) {
        if (s_blips[i].used && strcmp(s_blips[i].hex, s_sel_hex) == 0) {
            sel = &s_blips[i];
        }
    }
    if (sel != NULL) {
        char route[96] = "";
        const route_info_t *rt = sel->cs[0] ? routes_get_cached(sel->cs) : NULL;
        if (rt != NULL && rt->valid) {
            snprintf(route, sizeof(route), "\n%s > %s  %.28s",
                     rt->origin.iata[0] ? rt->origin.iata : rt->origin.icao,
                     rt->destination.iata[0] ? rt->destination.iata : rt->destination.icao,
                     rt->destination.city);
        }
        snprintf(buf, sizeof(buf), "SEL %-8s %-5s %s\nALT %5d  GS %3d  TRK %03d  VS %+d%s\nSQK %-4s  DST %.1f NM  BRG %03d",
                 sel->cs[0] ? sel->cs : sel->hex, sel->type, sel->reg,
                 sel->alt, (int)(sel->gs + 0.5f), (int)(sel->trk + 0.5f) % 360, sel->vs, route,
                 sel->sq[0] ? sel->sq : "----", sel->dist_nm, (int)(sel->brg + 0.5f) % 360);
    } else {
        snprintf(buf, sizeof(buf), "SEL ---\nTAP A TARGET ON THE SCOPE");
    }
    set_text(s_l_sel, buf);

    int len = snprintf(buf, sizeof(buf), "CALLSIGN  FL   DST  BRG  GS\n");
    for (int k = 0; k < n && k < 11 && len < (int)sizeof(buf) - 48; k++) {
        const blip_t *b = &s_blips[idx[k]];
        len += snprintf(buf + len, sizeof(buf) - len, "%-8s %03d %5.1f  %03d %3d\n",
                        b->cs[0] ? b->cs : b->hex, LV_MAX(0, b->alt / 100), b->dist_nm,
                        (int)(b->brg + 0.5f) % 360, (int)(b->gs + 0.5f));
    }
    if (len > 0 && buf[len - 1] == '\n') {
        buf[len - 1] = '\0';
    }
    set_text(s_l_list, buf);

    set_text(s_l_chip, s_apt_chip[0] ? s_apt_chip : "KRK NO DATA");
    lv_color_t cc = s_apt_level == 0 ? COL_TXT : s_apt_level == 1 ? COL_AMBER
                  : s_apt_level == 2 ? COL_RED : COL_DIM;
    if (lv_obj_get_style_text_color(s_l_chip, 0).full != cc.full) {
        lv_obj_set_style_text_color(s_l_chip, cc, 0);
    }
    set_text(s_l_apt, s_apt_text);
    (void)now;
}

/* ------------------------------------------------------------------ */
/* frame tick                                                          */
/* ------------------------------------------------------------------ */

static void tick_cb(lv_timer_t *t)
{
    (void)t;
    int64_t now = now_ms();
    uint32_t acc = (uint32_t)((now * 65536LL) / REV_MS);
    uint32_t prev = s_beam_init ? s_beam_acc : acc - 1;
    uint32_t step = acc - prev;
    if (step > 16384) {
        prev = acc - 16384;     /* after a stall, sweep at most a quadrant */
        step = 16384;
    }
    s_beam_init = true;
    s_beam_acc = acc;

    /* video map follows the range, the location and late airspace data */
    if (s_home_ok && (settings_get()->radius_nm != s_map_radius || s_map_lat != s_home_lat ||
                      s_map_lon != s_home_lon || airspace_count() != s_map_asp)) {
        build_map();
        for (int i = 0; i < MAX_AIRCRAFT; i++) {
            if (s_blips[i].used) {
                s_blips[i].x = -1;
                s_blips[i].hn = 0;
                s_blips[i].tw = 0;
            }
        }
        lv_obj_invalidate(s_scope);
    }

    for (int i = 0; i < MAX_AIRCRAFT; i++) {
        blip_t *b = &s_blips[i];
        if (!b->used) {
            continue;
        }
        if (!b->alive || b->ground) {
            if (b->x < 0 || now - b->paint_ms > REV_MS + REV_MS / 4) {
                blip_remove(b);
            }
            continue;
        }
        float x, y;
        blip_est(b, now, &x, &y);
        uint32_t rel = (bearing_u(x, y) - (prev & 0xFFFF)) & 0xFFFF;
        if (rel > 0 && rel <= step) {
            blip_paint(b, x, y, now);
        }
    }

    invalidate_wedge(acc - DEG_U(TRAIL_DEG) - step - DEG_U(1), DEG_U(TRAIL_DEG) + step + DEG_U(2));

    /* fading echoes outside the trail, within LVGL's invalid-area budget */
    lv_disp_t *disp = lv_disp_get_default();
    int budget = 10;
    for (int i = 0; i < MAX_AIRCRAFT && budget > 0; i++) {
        blip_t *b = &s_blips[i];
        if (!b->used || b->x < 0) {
            continue;
        }
        uint8_t q = blip_level(b, now) >> 4;
        if (q != b->lvl) {
            if (disp->inv_p >= LV_INV_BUF_SIZE - 6) {
                break;
            }
            b->lvl = q;
            inval_blip(b, false);
            budget--;
        }
    }

    if (now - s_console_ms >= 1000) {
        s_console_ms = now;
        console_update(now);
        rain_maybe_request();
    }
}

/* ------------------------------------------------------------------ */
/* objects                                                             */
/* ------------------------------------------------------------------ */

static void exit_cb(lv_event_t *e)
{
    (void)e;
    if (s_on_exit != NULL) {
        s_on_exit();
    }
}

static lv_obj_t *con_label(int y, lv_color_t col)
{
    lv_obj_t *l = lv_label_create(s_root);
    lv_obj_set_style_text_font(l, &s_mono, 0);
    lv_obj_set_style_text_color(l, col, 0);
    lv_obj_set_style_text_line_space(l, 1, 0);
    lv_obj_set_width(l, CON_W);
    lv_label_set_long_mode(l, LV_LABEL_LONG_CLIP);
    lv_obj_set_pos(l, CON_X, y);
    lv_label_set_text(l, "");
    return l;
}

static void build_objects(void)
{
    s_mono = lv_font_unscii_16;
    s_mono.fallback = &font_pl_14;

    s_root = lv_obj_create(s_scr);
    lv_obj_remove_style_all(s_root);
    lv_obj_set_size(s_root, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_pos(s_root, 0, 0);
    lv_obj_set_style_bg_color(s_root, COL_BG, 0);
    lv_obj_set_style_bg_opa(s_root, LV_OPA_COVER, 0);
    lv_obj_clear_flag(s_root, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(s_root, root_event_cb, LV_EVENT_DRAW_POST, NULL);
    lv_obj_add_flag(s_root, LV_OBJ_FLAG_HIDDEN);

    s_scope = lv_obj_create(s_root);
    lv_obj_remove_style_all(s_scope);
    lv_obj_set_size(s_scope, SCOPE_D, SCOPE_D);
    lv_obj_set_pos(s_scope, SCOPE_X, SCOPE_Y);
    lv_obj_add_flag(s_scope, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(s_scope, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(s_scope, scope_event_cb, LV_EVENT_ALL, NULL);

    s_l_title = con_label(14, COL_TXT);
    lv_label_set_text(s_l_title, "PPI SURVEILLANCE  EPKK");
    s_l_clock = con_label(14, COL_TXT);
    lv_obj_set_style_text_align(s_l_clock, LV_TEXT_ALIGN_RIGHT, 0);
    s_l_info = con_label(40, COL_DIM);
    s_l_sel = con_label(90, COL_TXT);
    s_l_list = con_label(178, COL_TXT);
    s_l_chip = con_label(392, COL_TXT);
    s_l_apt = con_label(412, COL_DIM);
    lv_label_set_long_mode(s_l_apt, LV_LABEL_LONG_WRAP);
    lv_obj_set_height(s_l_apt, 112);

    lv_obj_t *btn = lv_btn_create(s_root);
    lv_obj_remove_style_all(btn);
    lv_obj_set_size(btn, 120, 40);
    lv_obj_set_pos(btn, CON_X + CON_W - 120, 548);
    lv_obj_set_style_border_width(btn, 2, 0);
    lv_obj_set_style_border_color(btn, COL_DIM, 0);
    lv_obj_set_style_bg_color(btn, lv_color_hex(0x0f2a14), LV_STATE_PRESSED);
    lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, LV_STATE_PRESSED);
    lv_obj_set_ext_click_area(btn, 10);
    lv_obj_add_event_cb(btn, exit_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *bl = lv_label_create(btn);
    lv_obj_set_style_text_font(bl, &s_mono, 0);
    lv_obj_set_style_text_color(bl, COL_TXT, 0);
    lv_label_set_text(bl, "[ MAP ]");
    lv_obj_center(bl);

    lv_obj_t *hint = con_label(560, COL_DIM);
    lv_obj_set_width(hint, CON_W - 130);
    lv_label_set_text(hint, "RAIN: RAINVIEWER");

    s_timer = lv_timer_create(tick_cb, TICK_MS, NULL);
    lv_timer_pause(s_timer);
}

/* ------------------------------------------------------------------ */
/* public                                                              */
/* ------------------------------------------------------------------ */

void crt_radar_init(lv_obj_t *scr, crt_radar_exit_cb_t on_exit)
{
    s_scr = scr;
    s_on_exit = on_exit;
}

bool crt_radar_active(void)
{
    return s_active;
}

bool crt_radar_show(bool on)
{
    if (!on) {
        if (s_active) {
            s_active = false;
            lv_timer_pause(s_timer);
            lv_obj_add_flag(s_root, LV_OBJ_FLAG_HIDDEN);
        }
        return true;
    }
    if (s_active) {
        return true;
    }
    if (!ensure_mem()) {
        return false;
    }
    if (s_root == NULL) {
        build_objects();
    }
    s_active = true;
    s_beam_init = false;
    s_map_radius = -1;      /* rebuild the video map on the first tick */
    s_console_ms = 0;
    lv_obj_clear_flag(s_root, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(s_root);
    lv_obj_invalidate(s_root);
    lv_timer_resume(s_timer);
    return true;
}

void crt_radar_set_home(double lat, double lon)
{
    s_home_lat = lat;
    s_home_lon = lon;
    s_home_ok = true;
}

void crt_radar_set_weather(const char *text)
{
    strlcpy(s_weather, text, sizeof(s_weather));
}

void crt_radar_set_airport(int level, const char *chip, const char *wx,
                           const char *arr, const char *dep)
{
    s_apt_level = level;
    strlcpy(s_apt_chip, chip, sizeof(s_apt_chip));
    snprintf(s_apt_text, sizeof(s_apt_text), "%s\n%s\n%s", wx, arr, dep);
}

void crt_radar_set_targets(const aircraft_list_t *list)
{
    if (!s_mem_ok) {
        return;
    }
    for (int i = 0; i < MAX_AIRCRAFT; i++) {
        s_blips[i].alive = false;
    }
    for (int k = 0; k < list->count && k < MAX_AIRCRAFT; k++) {
        const aircraft_t *ac = &list->ac[k];
        if (!ac->has_pos) {
            continue;
        }
        blip_t *b = NULL, *free_slot = NULL;
        for (int i = 0; i < MAX_AIRCRAFT; i++) {
            if (s_blips[i].used && strcmp(s_blips[i].hex, ac->hex) == 0) {
                b = &s_blips[i];
                break;
            }
            if (!s_blips[i].used && free_slot == NULL) {
                free_slot = &s_blips[i];
            }
        }
        if (b == NULL) {
            if (free_slot == NULL) {
                continue;
            }
            b = free_slot;
            memset(b, 0, sizeof(*b));
            b->used = true;
            b->x = -1;
            strlcpy(b->hex, ac->hex, sizeof(b->hex));
        }
        b->alive = true;
        strlcpy(b->cs, ac->callsign, sizeof(b->cs));
        strlcpy(b->type, ac->type_icao, sizeof(b->type));
        strlcpy(b->reg, ac->reg, sizeof(b->reg));
        strlcpy(b->sq, ac->squawk, sizeof(b->sq));
        b->lat = ac->lat;
        b->lon = ac->lon;
        b->gs = ac->gs_kts;
        b->trk = ac->track_deg;
        b->alt = ac->alt_baro_ft;
        b->vs = ac->baro_rate_fpm;
        b->ground = ac->on_ground;
        b->dist_nm = ac->dist_nm;
        b->brg = ac->dir_deg;
        b->data_ms = list->fetched_at_ms;
    }
}
