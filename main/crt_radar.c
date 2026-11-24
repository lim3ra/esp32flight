#include "crt_radar.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "airspace.h"
#include "rainviewer.h"
#include "raingrid.h"
#include "routes.h"
#include "runways.h"
#include "settings.h"
#include "sysmon.h"
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
#define RAIN_TMP    288          /* RainViewer render; z7 fits up to ~70 NM */

/* ---- terminal (console column): unscii 8x8 glyphs drawn 8x16 ---- */
#define TERM_X      600
#define TERM_Y      12
#define TERM_COLS   52
#define TERM_ROWS   32
#define CELL_W      8
#define CELL_H      18
#define KEY_ROW     (TERM_ROWS - 1)
#define KEY_COL     40           /* "[   MAP   ]" occupies cols 40..51 */
#define LIST_ROW0   10
#define LIST_N      12

/* ---- timing ---- */
#define REV_MS      5000         /* one antenna revolution, ASR-like */
#define TICK_MS     90           /* ~11 frames per second */
#define TRAIL_DEG   30
#define FORCE_N     40           /* forced-repaint rectangles in flight */
#define FORCE_TICKS 4            /* frames a forced rectangle stays forced */
#define DEG_U(d)    ((uint32_t)((d) * 65536.0 / 360.0))
#define RAIN_EVERY_MS   (5 * 60 * 1000)
#define RAIN_RETRY_MS   (60 * 1000)

/* ---- phosphor ---- */
#define ECHO_FLOOR  64           /* echo persistence outside the trail, /256 */
#define TRAIL_WASH  52           /* glow of the swept area itself */
#define CLUTTER_R   22
#define BLIP_FLOOR  34
#define HIST_N      5
#define TAG_W       64           /* 8 characters */
#define TAG_H       32           /* 2 lines of 16 */
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
    char     tag_txt[32];
    uint8_t  tag[TAG_BYTES];
} blip_t;

typedef struct {
    char    txt[TERM_COLS + 1];
    uint8_t col;                 /* colour index */
    uint8_t inv0, inv1;          /* inverse video span [inv0, inv1), key caps */
} term_row_t;

static lv_obj_t *s_scr;
static crt_radar_exit_cb_t s_on_exit;
static bool s_active, s_frozen;
static bool s_mem_ok;

static uint16_t *s_qang;     /* QW x QW bearing of the NE quadrant, 65536 = 360 deg */
static uint8_t  *s_map;      /* video map, 2 bits per pixel */
static uint8_t  *s_rain;     /* echo intensity, half resolution */
static uint16_t *s_rain_tmp; /* RainViewer render, RGB565 */
static uint8_t  *s_row;      /* one row of intensities */
static uint16_t *s_pal;      /* [0..255] even rows, [256..511] scanline rows */
static uint8_t  *s_glow;     /* 512 steps of beam distance -> glow */
static uint8_t  *s_vig;      /* vignette by r^2 >> 8 */
static int16_t  *s_hw;       /* tube half width per |dy| */
static uint8_t  *s_kmain, *s_ksmall, *s_kring;
static uint8_t  *s_decay;    /* blip intensity by age / 40 ms */
static uint8_t  *s_glyph;    /* 95 x 8 rows, ASCII 32..126 from unscii 8 */
static uint8_t  *s_noise;    /* 32 x 32 white noise, shifted per revolution */
static uint8_t  *s_map_i;    /* video map level -> intensity */
static blip_t   *s_blips;
static term_row_t *s_term;
static const uint8_t s_hist_i[HIST_N] = { 84, 62, 44, 30, 18 };

static lv_obj_t *s_root, *s_scope, *s_termobj;
static lv_timer_t *s_timer;

static uint32_t s_beam_acc;  /* cumulative beam angle, wraps every 65536 revs */
static bool     s_beam_init;
static double   s_home_lat, s_home_lon;
static bool     s_home_ok;
static float    s_kmpp = 0.2f;   /* km per scope pixel */
static int      s_map_radius = -1;
static double   s_map_lat, s_map_lon;
static int      s_map_asp = -1;
static char     s_sel_hex[ICAO_HEX_LEN];
static char     s_list_hex[LIST_N][ICAO_HEX_LEN];
static int64_t  s_console_ms;
static uint32_t s_draw_us, s_draw_px, s_ticks;
static char     s_frame_stat[40] = "";
static uint32_t s_tickno;        /* monotonic frame counter */
static uint32_t s_active_u;      /* beam distance still being repainted */
static int      s_full_ticks;
static int      s_nforce;
static struct { lv_area_t a; uint32_t tick; } s_force[FORCE_N];
static uint8_t *s_must;          /* per-row repaint mask */

static volatile bool s_rain_busy;
static int64_t  s_rain_ms = -RAIN_EVERY_MS;
static float    s_rain_km;
static float    s_rain_req_km;
static double   s_rain_req_lat, s_rain_req_lon;
static char     s_rain_when[8] = "--:--";
static int      s_rain_cells = -1;

static char s_weather[96];
static char s_warning[160];
static int  s_apt_level = 3;
static char s_apt_chip[40], s_apt_wx[200], s_apt_arr[100], s_apt_dep[100];

#define COL_BG_HEX  0x0a0d0b
enum { TC_TXT, TC_DIM, TC_AMBER, TC_RED, TC_COUNT };
static uint16_t s_tc_fg[TC_COUNT], s_tc_glow[TC_COUNT], s_term_bg;

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
    s_term_bg = lv_color_hex(COL_BG_HEX).full;
    s_tc_fg[TC_TXT] = rgb(108, 240, 106);
    s_tc_fg[TC_DIM] = rgb(52, 160, 64);
    s_tc_fg[TC_AMBER] = rgb(255, 176, 32);
    s_tc_fg[TC_RED] = rgb(255, 74, 58);
    s_tc_glow[TC_TXT] = rgb(22, 60, 26);
    s_tc_glow[TC_DIM] = rgb(14, 38, 18);
    s_tc_glow[TC_AMBER] = rgb(70, 46, 10);
    s_tc_glow[TC_RED] = rgb(70, 22, 16);
}

/* ASCII 32..126 of unscii 8 into a plain 8x8 table, so every text path
 * (terminal, data tags, map labels) is a byte lookup. */
static void build_glyphs(void)
{
    const lv_font_t *f = &lv_font_unscii_8;
    for (int c = 32; c < 127; c++) {
        uint8_t *cell = s_glyph + (c - 32) * 8;
        lv_font_glyph_dsc_t g;
        if (!lv_font_get_glyph_dsc(f, &g, (uint32_t)c, 0) || g.box_w == 0) {
            continue;
        }
        const uint8_t *bmp = lv_font_get_glyph_bitmap(f, (uint32_t)c);
        if (bmp == NULL || g.bpp != 1) {
            continue;
        }
        int bx = g.ofs_x;
        int by = (f->line_height - f->base_line) - g.box_h - g.ofs_y;
        uint32_t bit = 0;
        for (int gy = 0; gy < g.box_h; gy++) {
            for (int gx = 0; gx < g.box_w; gx++, bit++) {
                if ((bmp[bit >> 3] >> (7 - (bit & 7))) & 1) {
                    int x = bx + gx, y = by + gy;
                    if ((unsigned)x < 8 && (unsigned)y < 8) {
                        cell[y] |= (uint8_t)(0x80 >> x);
                    }
                }
            }
        }
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
    for (int i = 0; i < 512; i++) {
        float deg = i * 360.0f / 512.0f;
        int v = 0;
        if (deg < 0.5f) {
            v = 255;
        } else if (deg < TRAIL_DEG) {
            float t = (deg - 0.7f) / (TRAIL_DEG - 0.7f);
            v = (int)(235.0f * (1.0f - t) * (1.0f - t));
        }
        s_glow[i] = (uint8_t)v;
    }
    int vn = (TUBE_R * TUBE_R >> 8) + 1;
    for (int i = 0; i < vn; i++) {
        float r = sqrtf((float)(i << 8)) / TUBE_R;
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
    /* PSRAM for the bulky and the rarely touched; the per-pixel lookup
     * tables live in internal RAM. On the 7B the bounce-buffer ISR streams
     * the whole frame through the 32 KB data cache, so PSRAM-resident tables
     * miss on nearly every random lookup. */
    const size_t small_n = (TUBE_R + 1) * 2 + K_MAIN * K_MAIN + K_SMALL * K_SMALL +
                           K_RING * K_RING + 256 + 95 * 8 + 16;
    const size_t hot_n = 512 * 2 + 512 + ((TUBE_R * TUBE_R >> 8) + 1) + 1024 + SCOPE_D * 2 + 4 + 8;
    s_qang = ps_alloc((size_t)QW * QW * 2);
    s_map = ps_alloc((size_t)SCOPE_D * SCOPE_D / 4);
    s_rain = ps_alloc((size_t)RAIN_D * RAIN_D);
    s_rain_tmp = ps_alloc((size_t)RAIN_TMP * RAIN_TMP * 2);
    s_blips = ps_alloc(sizeof(blip_t) * MAX_AIRCRAFT);
    s_term = ps_alloc(sizeof(term_row_t) * TERM_ROWS);
    uint8_t *small = ps_alloc(small_n);
    uint8_t *hot = heap_caps_calloc(1, hot_n, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!s_qang || !s_map || !s_rain || !s_rain_tmp || !s_blips || !s_term || !small || !hot) {
        free(hot);
        free(s_qang);
        free(s_map);
        free(s_rain);
        free(s_rain_tmp);
        free(s_blips);
        free(s_term);
        free(small);
        s_qang = NULL;
        s_map = s_rain = NULL;
        s_rain_tmp = NULL;
        s_blips = NULL;
        s_term = NULL;
        return false;
    }
    uint8_t *h = hot;
    s_pal = (uint16_t *)h;           h += 512 * 2;
    s_glow = h;                      h += 512;
    s_vig = h;                       h += (TUBE_R * TUBE_R >> 8) + 1;
    s_noise = h;                     h += 1024;
    s_row = h;                       h += SCOPE_D;
    s_must = h;                      h += SCOPE_D;
    s_map_i = h;
    s_map_i[1] = 40;
    s_map_i[2] = 70;
    s_map_i[3] = 118;
    uint8_t *p = small;
    s_hw = (int16_t *)p;             p += (TUBE_R + 1) * 2;
    s_kmain = p;                     p += K_MAIN * K_MAIN;
    s_ksmall = p;                    p += K_SMALL * K_SMALL;
    s_kring = p;                     p += K_RING * K_RING;
    s_decay = p;                     p += 256;
    s_glyph = p;
    for (int i = 0; i < 1024; i++) {
        s_noise[i] = (uint8_t)(hash3((uint32_t)i, 0x9e37u, 0x79b9u) >> 24);
    }
    build_palette();
    build_glyphs();
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
/* text                                                                */
/* ------------------------------------------------------------------ */

/* The glyph table is ASCII: fold Polish letters to their base, anything
 * else unknown becomes '?'. */
static void to_ascii(char *dst, size_t n, const char *src)
{
    static const struct { uint16_t cp; char c; } fold[] = {
        { 0x104, 'A' }, { 0x105, 'a' }, { 0x106, 'C' }, { 0x107, 'c' }, { 0x118, 'E' },
        { 0x119, 'e' }, { 0x141, 'L' }, { 0x142, 'l' }, { 0x143, 'N' }, { 0x144, 'n' },
        { 0xD3, 'O' },  { 0xF3, 'o' },  { 0x15A, 'S' }, { 0x15B, 's' }, { 0x179, 'Z' },
        { 0x17A, 'z' }, { 0x17B, 'Z' }, { 0x17C, 'z' }, { 0xB7, '.' },
        { 0xE9, 'e' },  { 0xE1, 'a' },  { 0xED, 'i' },  { 0xFC, 'u' },  { 0xF6, 'o' },
        { 0xE4, 'a' },  { 0xDF, 's' },
    };
    size_t o = 0;
    uint32_t i = 0;
    while (src[i] != '\0' && o + 1 < n) {
        uint32_t c = _lv_txt_encoded_next(src, &i);
        if (c >= 32 && c < 127) {
            dst[o++] = (char)c;
            continue;
        }
        char r = 0;          /* symbols and icons are dropped */
        for (size_t k = 0; k < sizeof(fold) / sizeof(fold[0]); k++) {
            if (fold[k].cp == c) {
                r = fold[k].c;
                break;
            }
        }
        if (c == 0x2192) {      /* arrow */
            r = '>';
        }
        if (r != 0) {
            dst[o++] = r;
        }
    }
    dst[o] = '\0';
}

typedef void (*plot_fn)(void *ctx, int x, int y);

/* ASCII text at 8x16 (each glyph row doubled), top-left at x, y. */
static void draw_text(const char *s, int x, int y, plot_fn fn, void *ctx)
{
    for (; *s; s++, x += 8) {
        unsigned c = (unsigned char)*s;
        if (c < 32 || c > 126) {
            c = '?';
        }
        const uint8_t *g = s_glyph + (c - 32) * 8;
        for (int gy = 0; gy < 8; gy++) {
            uint8_t bits = g[gy];
            for (int gx = 0; bits; gx++, bits <<= 1) {
                if (bits & 0x80) {
                    fn(ctx, x + gx, y + gy * 2);
                    fn(ctx, x + gx, y + gy * 2 + 1);
                }
            }
        }
    }
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

static void plot_map(void *ctx, int x, int y)
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
        char t[12];
        snprintf(t, sizeof(t), "%d", nm);
        float a = 0.30f;     /* label a little east of north */
        draw_text(t, SCOPE_C + (int)(r * sinf(a)) + 3, SCOPE_C - (int)(r * cosf(a)) - 17,
                  plot_map, (void *)2);
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
            draw_text(t, SCOPE_C + (int)((TUBE_R - 30) * s) - 12,
                      SCOPE_C - (int)((TUBE_R - 30) * c) - 8, plot_map, (void *)2);
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
        if (len_px > 0.5f && len_px * s_kmpp > 1.8f) {
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
        /* openAIP types: control zones bright and solid, terminal areas and
         * ATZs dashed, restricted/prohibited/danger/TRA faint and dotted */
        int v, dash;
        switch (a->type) {
        case 4: case 27:  v = 3; dash = 0; break;    /* CTR, MCTR */
        case 7: case 26:  v = 2; dash = 8; break;    /* TMA, MTMA */
        case 13:          v = 2; dash = 4; break;    /* ATZ */
        default:          v = 1; dash = 2; break;    /* R, P, D, TRA */
        }
        float cx = 0, cy = 0;
        for (int k = 0; k < a->n_points; k++) {
            int k2 = (k + 1) % a->n_points;
            float x0, y0, x1, y1;
            geo_to_scope(a->lat[k], a->lon[k], &x0, &y0);
            geo_to_scope(a->lat[k2], a->lon[k2], &x1, &y1);
            cx += x0;
            cy += y0;
            if (seg_visible(x0, y0, x1, y1)) {
                map_line((int)x0, (int)y0, (int)x1, (int)y1, v, dash);
            }
        }
        /* type label at the vertex centroid, when that lies on the scope */
        cx /= a->n_points;
        cy /= a->n_points;
        const char *lbl = airspace_type_str(a->type);
        float ddx = cx - SCOPE_C, ddy = cy - SCOPE_C;
        if (lbl[0] && ddx * ddx + ddy * ddy < (TUBE_R - 30) * (TUBE_R - 30)) {
            draw_text(lbl, (int)cx - (int)strlen(lbl) * 4, (int)cy - 8, plot_map,
                      (void *)(intptr_t)(v > 1 ? 2 : 1));
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

/* RainViewer's scheme runs blue -> yellow -> red with intensity, so the red
 * channel is the best single measure. Light rain stays faint on purpose. */
static inline int rain_level(uint16_t c)
{
    int r = (c >> 11) << 3, g = ((c >> 5) & 0x3F) << 2, b = (c & 0x1F) << 3;
    int mx = r > g ? (r > b ? r : b) : (g > b ? g : b);
    if (mx < 24) {
        return 0;
    }
    int iv = 34 + (r * 180 >> 8) + (mx >> 4);
    return iv > 255 ? 255 : iv;
}

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
        /* The view is web-mercator and the scope is a flat km grid; across
         * this small an area both axes are linear enough, which gives the
         * fractional coordinates the bilinear sampling needs. */
        double lon_a = hlon + (1 - SCOPE_C) * kmpp / (111.32 * coslat);
        double lon_b = hlon + (SCOPE_D - 1 - SCOPE_C) * kmpp / (111.32 * coslat);
        double lat_t = hlat + (SCOPE_C - 1) * kmpp / 110.57;
        double lat_b = hlat + (SCOPE_C - (SCOPE_D - 1)) * kmpp / 110.57;
        int xa, ya, xb, yb, xd, yd;
        tilemap_project(&v, lat_t, lon_a, &xa, &ya);
        tilemap_project(&v, lat_t, lon_b, &xb, &yb);
        tilemap_project(&v, lat_b, lon_a, &xd, &yd);
        for (int cy = 0; cy < RAIN_D; cy++) {
            float fy = ya + (float)(yd - ya) * (2 * cy) / (float)(SCOPE_D - 2);
            uint8_t *row = s_rain + (size_t)cy * RAIN_D;
            for (int cx = 0; cx < RAIN_D; cx++) {
                float fx = xa + (float)(xb - xa) * (2 * cx) / (float)(SCOPE_D - 2);
                int x0 = (int)floorf(fx), y0 = (int)floorf(fy);
                float tx = fx - x0, ty = fy - y0;
                int e = 0;
                if (x0 >= 0 && x0 + 1 < RAIN_TMP && y0 >= 0 && y0 + 1 < RAIN_TMP) {
                    const uint16_t *p = s_rain_tmp + y0 * RAIN_TMP + x0;
                    float a = rain_level(p[0]), b = rain_level(p[1]);
                    float c = rain_level(p[RAIN_TMP]), d = rain_level(p[RAIN_TMP + 1]);
                    e = (int)((a * (1 - tx) + b * tx) * (1 - ty) + (c * (1 - tx) + d * tx) * ty);
                    if (e > 0) {
                        lit++;
                    }
                }
                row[cx] = (uint8_t)e;
            }
        }
        s_rain_km = km;
        time_t t = time(NULL);
        if (t > 1600000000) {
            time_t l = t + (tz_home_known() ? tz_home_offset() : 0);
            struct tm tm;
            gmtime_r(&l, &tm);
            snprintf(s_rain_when, sizeof(s_rain_when), "%02d:%02d", tm.tm_hour, tm.tm_min);
        }
        s_rain_cells = lit;
        s_rain_ms = now_ms();
        ESP_LOGI(TAG, "rain layer z%d, %d echo cells", v.z, lit);
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
    if (raingrid_fresh()) {
        /* collector grid: straight lookups, no tile fetch, no worker */
        static int s_grid_gen = -1;
        if (!range_moved && raingrid_generation() == s_grid_gen) {
            return;
        }
        s_grid_gen = raingrid_generation();
        float kmpp = km / RANGE_R;
        double coslat = cos(s_home_lat * M_PI / 180.0);
        int lit = 0;
        for (int cy = 0; cy < RAIN_D; cy++) {
            double lat = s_home_lat + (SCOPE_C - (2 * cy + 1)) * kmpp / 110.574;
            uint8_t *row = s_rain + (size_t)cy * RAIN_D;
            for (int cx = 0; cx < RAIN_D; cx++) {
                double lon = s_home_lon + ((2 * cx + 1) - SCOPE_C) * kmpp / (111.320 * coslat);
                uint8_t idx = raingrid_sample(lat, lon), r, g, b, a, in = 0;
                if (idx) {
                    raingrid_palette(idx, &r, &g, &b, &a, &in);
                    lit++;
                }
                row[cx] = in;
            }
        }
        s_rain_km = km;
        s_rain_cells = lit;
        time_t ft = (time_t)raingrid_frame_time();
        if (ft > 1600000000) {
            time_t l = ft + (tz_home_known() ? tz_home_offset() : 0);
            struct tm tm;
            gmtime_r(&l, &tm);
            snprintf(s_rain_when, sizeof(s_rain_when), "%02d:%02d", tm.tm_hour, tm.tm_min);
        }
        return;
    }
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

static inline bool blip_selected(const blip_t *b)
{
    return s_sel_hex[0] && strcmp(b->hex, s_sel_hex) == 0;
}

/* Bounding box of everything a blip draws, scope coordinates. */
static void blip_box(const blip_t *b, lv_area_t *a, bool with_hist)
{
    a->x1 = b->x - K_RING / 2;
    a->x2 = b->x + K_RING / 2;
    a->y1 = b->y - K_RING / 2;
    a->y2 = b->y + K_RING / 2;
    if (b->tw > 0) {
        a->x1 = LV_MIN(a->x1, b->tx - 1);
        a->x2 = LV_MAX(a->x2, b->tx + b->tw);
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

static void inval_area(const lv_area_t *sa)
{
    lv_area_t a = {
        .x1 = s_scope->coords.x1 + sa->x1, .y1 = s_scope->coords.y1 + sa->y1,
        .x2 = s_scope->coords.x1 + sa->x2, .y2 = s_scope->coords.y1 + sa->y2,
    };
    lv_obj_invalidate_area(s_scope, &a);
}

/* Redraw everything for a few frames (both frame buffers get it). */
static void force_full(void)
{
    s_full_ticks = FORCE_TICKS;
    s_nforce = 0;
    lv_obj_invalidate(s_scope);
}

/* Invalidate a rectangle whose every pixel must be repainted. The sector
 * sweep instead repaints only pixels near the beam (see compose_span), so
 * anything else that changes - echoes, tags, a fading dot - goes through
 * here and is remembered long enough to reach both frame buffers. */
static void inval_scope(const lv_area_t *sa)
{
    if (s_full_ticks == 0) {
        if (s_nforce == FORCE_N) {
            force_full();
            return;
        }
        s_force[s_nforce].a = *sa;
        s_force[s_nforce].tick = s_tickno;
        s_nforce++;
    }
    inval_area(sa);
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
    char l1[CALLSIGN_LEN + 4], l2[12];
    int fl = b->alt / 100;
    if (fl < 0) {
        fl = 0;
    } else if (fl > 999) {
        fl = 999;
    }
    char trend = b->vs > 400 ? '^' : b->vs < -400 ? 'v' : ' ';
    to_ascii(l1, sizeof(l1), b->cs[0] ? b->cs : b->hex);
    snprintf(l2, sizeof(l2), "%03d%c%d", fl, trend, (int)(b->gs + 0.5f));
    char txt[sizeof(l1) + sizeof(l2) + 2];
    snprintf(txt, sizeof(txt), "%s|%s", l1, l2);
    if (strcmp(txt, b->tag_txt) == 0) {
        return;
    }
    strlcpy(b->tag_txt, txt, sizeof(b->tag_txt));
    memset(b->tag, 0, TAG_BYTES);
    draw_text(l1, 0, 0, plot_tag, b->tag);
    draw_text(l2, 0, 16, plot_tag, b->tag);
}

static bool tag_overlaps(const blip_t *self, int x, int y, int w)
{
    for (int i = 0; i < MAX_AIRCRAFT; i++) {
        const blip_t *o = &s_blips[i];
        if (o == self || !o->used || o->x < 0 || o->tw == 0) {
            continue;
        }
        if (x < o->tx + o->tw + 4 && o->tx < x + w + 4 && y < o->ty + TAG_H && o->ty < y + TAG_H) {
            return true;
        }
    }
    return false;
}

static void blip_place_tag(blip_t *b)
{
    const char *bar = strchr(b->tag_txt, '|');
    int l1 = bar != NULL ? (int)(bar - b->tag_txt) : 0;
    int l2 = bar != NULL ? (int)strlen(bar + 1) : 0;
    int w = (l1 > l2 ? l1 : l2) * 8;
    if (w > TAG_W) {
        w = TAG_W;
    }
    const int cand[4][2] = { { 9, -30 }, { 9, 2 }, { -9 - w, -30 }, { -9 - w, 2 } };
    bool sel = blip_selected(b);
    for (int k = 0; k < 4; k++) {
        int x = b->x + cand[k][0], y = b->y + cand[k][1];
        /* all four corners on the phosphor, not over the bezel */
        bool inside = true;
        for (int c = 0; c < 4 && inside; c++) {
            int cx = (c & 1 ? x + w : x) - SCOPE_C, cy = (c & 2 ? y + TAG_H : y) - SCOPE_C;
            inside = cx * cx + cy * cy < (TUBE_R - 6) * (TUBE_R - 6);
        }
        if (!inside) {
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
    b->tw = 0;               /* re-placed below, without colliding with itself */
    blip_render_tag(b);
    blip_place_tag(b);
    inval_blip(b, true);
}

/* ------------------------------------------------------------------ */
/* scope drawing                                                       */
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
    bool sel = blip_selected(b);
    if (sel) {
        stamp(s_kring, K_RING, b->x, b->y, 190, sy, x0, x1, I);
    }
    if (b->tw > 0) {
        int ty = sy - b->ty;
        if (ty >= 0 && ty < TAG_H) {
            /* constant, so a fading echo only repaints its own 13x13 dot */
            uint32_t ti = sel ? 255 : 150;
            const uint8_t *bits = b->tag + ty * (TAG_W / 8);
            int xa = LV_MAX(x0, b->tx - 1), xb = LV_MIN(x1, b->tx + b->tw);
            for (int x = xa; x <= xb; x++) {
                int bx = x - b->tx;
                bool on = bx >= 0 && bx < TAG_W && (bits[bx >> 3] & (0x80 >> (bx & 7)));
                bool nb = (bx - 1 >= 0 && bx - 1 < TAG_W && (bits[(bx - 1) >> 3] & (0x80 >> ((bx - 1) & 7)))) ||
                          (bx + 1 >= 0 && bx + 1 < TAG_W && (bits[(bx + 1) >> 3] & (0x80 >> ((bx + 1) & 7))));
                uint8_t *p = &I[x - x0];
                if (on) {
                    if (*p < ti) {
                        *p = (uint8_t)ti;
                    }
                } else if (nb) {
                    add_sat(p, ti / 4);   /* phosphor bloom around the strokes */
                }
            }
        }
    }
}

static inline uint32_t bear_at(const uint16_t *qrow, int dx, int dy)
{
    uint32_t b;
    if (dx >= 0) {
        uint32_t a = qrow[dx];
        b = dy <= 0 ? a : 32768u - a;
    } else {
        uint32_t a = qrow[-dx];
        b = dy > 0 ? 32768u + a : 65536u - a;
    }
    return b & 0xFFFF;
}

static void mark_must(uint8_t *must, int sy, int x0, int x1, const lv_area_t *r)
{
    if (sy < r->y1 || sy > r->y2) {
        return;
    }
    int a = LV_MAX(x0, r->x1), b = LV_MIN(x1, r->x2);
    if (a <= b) {
        memset(must + (a - x0), 1, (size_t)(b - a + 1));
    }
}

/* One row of the scope. In LVGL's double-buffered direct mode the buffer
 * being drawn still holds the frame from two refreshes ago, so a pixel whose
 * value has not changed since then can be left alone: away from the beam the
 * phosphor sits at its floor and only the bearing test is paid, not the
 * composition or the PSRAM write. Pixels near the beam, inside echo or tag
 * boxes, inside forced rectangles, or all of them after a full invalidation,
 * are painted. */
static void compose_span(int sy, int x0, int x1, uint16_t *dst,
                         const uint8_t *bl, int nbl, const uint8_t *fr, int nfr,
                         bool full, int64_t now)
{
    uint8_t *must = s_must;
    if (full) {
        memset(must, 1, (size_t)(x1 - x0 + 1));
    } else {
        memset(must, 0, (size_t)(x1 - x0 + 1));
        for (int k = 0; k < nbl; k++) {
            lv_area_t bb;
            blip_box(&s_blips[bl[k]], &bb, true);
            mark_must(must, sy, x0, x1, &bb);
        }
        for (int k = 0; k < nfr; k++) {
            mark_must(must, sy, x0, x1, &s_force[fr[k]].a);
        }
    }
    const uint32_t active_u = s_active_u;
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

    /* Bearing along a row is monotonic - clockwise going right above the
     * centre, anticlockwise below - so a 16-pixel chunk whose end bearings
     * bracket no part of the active arc has no pixel to paint, and is
     * skipped for two table reads instead of sixteen. */
    const uint32_t arc_s = (beam - active_u - DEG_U(1)) & 0xFFFF;
    const uint32_t arc_l = active_u + DEG_U(4);
    for (int cx0 = ix0; cx0 <= ix1; cx0 += 16) {
    const int cx1 = cx0 + 15 < ix1 ? cx0 + 15 : ix1;
    if (!full && memchr(must + (cx0 - x0), 1, (size_t)(cx1 - cx0 + 1)) == NULL) {
        uint32_t fa = bear_at(qrow, cx0 - SCOPE_C, dy), fb = bear_at(qrow, cx1 - SCOPE_C, dy);
        uint32_t lo = dy <= 0 ? fa : fb, hi = dy <= 0 ? fb : fa;
        uint32_t span = (hi - lo) & 0xFFFF;
        if (((arc_s - lo) & 0xFFFF) > span && ((lo - arc_s) & 0xFFFF) >= arc_l) {
            continue;
        }
    }
    for (int x = cx0; x <= cx1; x++) {
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
        uint32_t d = (beam - b) & 0xFFFF;
        if (!must[x - x0]) {
            if (d >= active_u && d < 0xFFFF - DEG_U(2)) {
                continue;
            }
            must[x - x0] = 1;
        }
        uint32_t g = s_glow[d >> 7];
        uint32_t v;
        if (g == 255) {
            v = 255;
        } else {
            /* noise is fixed for a pixel until the beam passes it again */
            uint32_t rev = (acc - b) >> 16;
            uint32_t n = s_noise[(((uint32_t)sy + rev * 23u) & 31u) * 32u +
                                 (((uint32_t)x + rev * 41u) & 31u)];
            uint32_t persist = ECHO_FLOOR + ((g * (256 - ECHO_FLOOR)) >> 8);
            v = (g * TRAIL_WASH) >> 8;
            uint32_t e = rrow[x >> 1];
            if (e) {
                e = (e * (130 + (n >> 1))) >> 8;      /* grain */
                v += (e * persist) >> 8;
            }
            if (g && n == 255) {
                v += 24 + (g >> 2);          /* receiver noise, in the trail */
            }
            int r2 = dx * dx + dy * dy;
            if (r2 < CLUTTER_R * CLUTTER_R) {
                uint32_t c = (n >> 1) * (uint32_t)(CLUTTER_R * CLUTTER_R - r2) /
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
    }

    for (int k = 0; k < nbl; k++) {
        stamp_blip(&s_blips[bl[k]], now, sy, x0, x1, I);
    }

    /* bezel colours for this row: dark gap, metal ring lit from above, desk */
    int shade = 30 - dy * 12 / TUBE_R;
    uint16_t c_gap = rgb(2, 4, 3);
    uint16_t c_ring = rgb(shade, shade + 3, shade);
    uint16_t c_edge = rgb(shade + 18, shade + 21, shade + 18);
    uint16_t c_bg = s_term_bg;
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
        if (!must[x - x0]) {
            continue;
        }
        int dx = x - SCOPE_C;
        int r2 = dx * dx + dy * dy;
        uint16_t c;
        if (x >= tx0 && x <= tx1) {
            c = pal[(I[x - x0] * s_vig[r2 >> 8]) >> 8];
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

static void select_hex(const char *hex)
{
    for (int i = 0; i < MAX_AIRCRAFT; i++) {
        if (s_blips[i].used && blip_selected(&s_blips[i])) {
            inval_blip(&s_blips[i], false);
        }
    }
    if (hex == NULL || strcmp(hex, s_sel_hex) == 0) {
        s_sel_hex[0] = '\0';
    } else {
        strlcpy(s_sel_hex, hex, sizeof(s_sel_hex));
        for (int i = 0; i < MAX_AIRCRAFT; i++) {
            blip_t *b = &s_blips[i];
            if (b->used && b->x >= 0 && blip_selected(b)) {
                inval_blip(b, false);
                blip_place_tag(b);
                inval_blip(b, false);
            }
        }
    }
    s_console_ms = 0;   /* refresh the console on the next tick */
}

static void cover_check(lv_event_t *e, lv_obj_t *obj)
{
    lv_cover_check_info_t *info = lv_event_get_param(e);
    if (info->res != LV_COVER_RES_MASKED) {
        info->res = _lv_area_is_in(info->area, &obj->coords, 0) ? LV_COVER_RES_COVER
                                                                 : LV_COVER_RES_NOT_COVER;
    }
}

static void scope_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t *obj = lv_event_get_target(e);
    if (code == LV_EVENT_COVER_CHECK) {
        cover_check(e, obj);
    } else if (code == LV_EVENT_DRAW_MAIN) {
        int64_t t0 = esp_timer_get_time();
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
        uint8_t fr[FORCE_N];
        int nfr = 0;
        for (int i = 0; i < s_nforce; i++) {
            const lv_area_t *f = &s_force[i].a;
            if (f->x1 <= sa.x2 && f->x2 >= sa.x1 && f->y1 <= sa.y2 && f->y2 >= sa.y1) {
                fr[nfr++] = (uint8_t)i;
            }
        }
        bool full = s_full_ticks > 0 || !s_active;
        int stride = lv_area_get_width(dc->buf_area);
        int64_t now = now_ms();
        for (int y = a.y1; y <= a.y2; y++) {
            lv_color_t *row = (lv_color_t *)dc->buf +
                              (size_t)(y - dc->buf_area->y1) * stride + (a.x1 - dc->buf_area->x1);
            compose_span(y - oy, sa.x1, sa.x2, (uint16_t *)row, bl, nbl, fr, nfr, full, now);
        }
        s_draw_us += (uint32_t)(esp_timer_get_time() - t0);
        s_draw_px += (uint32_t)lv_area_get_size(&a);
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
        select_hex(best >= 0 ? s_blips[best].hex : NULL);
    }
}

/* ------------------------------------------------------------------ */
/* terminal                                                            */
/* ------------------------------------------------------------------ */

static void term_inval_row(int r)
{
    lv_area_t a = {
        .x1 = s_termobj->coords.x1, .x2 = s_termobj->coords.x2,
        .y1 = s_termobj->coords.y1 + r * CELL_H,
        .y2 = s_termobj->coords.y1 + (r + 1) * CELL_H - 1,
    };
    lv_obj_invalidate_area(s_termobj, &a);
}

static void term_put(int r, int col, const char *fmt, ...)
{
    if (r < 0 || r >= TERM_ROWS) {
        return;
    }
    char raw[160], txt[TERM_COLS + 1];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(raw, sizeof(raw), fmt, ap);
    va_end(ap);
    to_ascii(txt, sizeof(txt), raw);
    term_row_t *t = &s_term[r];
    if (t->col != col || strcmp(t->txt, txt) != 0) {
        strlcpy(t->txt, txt, sizeof(t->txt));
        t->col = (uint8_t)col;
        term_inval_row(r);
    }
}

/* Word-wrap text over rows r0..r0+n-1; returns rows used. */
static int term_wrap(int r0, int n, int col, const char *text)
{
    char a[400];
    to_ascii(a, sizeof(a), text);
    const char *p = a;
    int used = 0;
    while (*p && used < n) {
        while (*p == ' ') {
            p++;
        }
        int len = (int)strlen(p);
        int take = len;
        if (len > TERM_COLS) {
            take = TERM_COLS;
            while (take > 0 && p[take] != ' ') {
                take--;
            }
            if (take == 0) {
                take = TERM_COLS;
            }
        }
        term_put(r0 + used, col, "%.*s", take, p);
        p += take;
        used++;
    }
    return used;
}

static void term_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t *obj = lv_event_get_target(e);
    if (code == LV_EVENT_COVER_CHECK) {
        cover_check(e, obj);
    } else if (code == LV_EVENT_DRAW_MAIN) {
        lv_draw_ctx_t *dc = lv_event_get_draw_ctx(e);
        lv_area_t a;
        if (!_lv_area_intersect(&a, dc->clip_area, &obj->coords)) {
            return;
        }
        int stride = lv_area_get_width(dc->buf_area);
        for (int y = a.y1; y <= a.y2; y++) {
            uint16_t *row = (uint16_t *)dc->buf + (size_t)(y - dc->buf_area->y1) * stride -
                            dc->buf_area->x1;
            int ly = y - obj->coords.y1;
            int r = ly / CELL_H, cy = ly % CELL_H;
            const term_row_t *t = r < TERM_ROWS ? &s_term[r] : NULL;
            int tlen = t != NULL ? (int)strlen(t->txt) : 0;
            int gy = cy / 2;      /* glyph row, 8 rows doubled, then 2 px gap */
            for (int x = a.x1; x <= a.x2; x++) {
                int lx = x - obj->coords.x1;
                int c = lx / CELL_W, gx = lx % CELL_W;
                uint16_t px = s_term_bg;
                if (t != NULL && c < TERM_COLS) {
                    bool inv = c >= t->inv0 && c < t->inv1;
                    uint16_t fg = s_tc_fg[t->col], glow = s_tc_glow[t->col];
                    if (inv) {
                        px = fg;
                        fg = s_term_bg;
                    }
                    unsigned ch = c < tlen ? (unsigned char)t->txt[c] : ' ';
                    if (gy < 8 && ch > 32 && ch < 127) {
                        uint8_t bits = s_glyph[(ch - 32) * 8 + gy];
                        if (bits & (0x80 >> gx)) {
                            px = fg;
                        } else if (!inv && (((bits << 1) | (bits >> 1)) & (0x80 >> gx))) {
                            px = glow;    /* phosphor bloom beside the strokes */
                        }
                    }
                }
                row[x] = px;
            }
        }
    } else if (code == LV_EVENT_CLICKED) {
        lv_point_t p;
        lv_indev_get_point(lv_indev_get_act(), &p);
        int r = (p.y - obj->coords.y1) / CELL_H;
        int c = (p.x - obj->coords.x1) / CELL_W;
        if (r >= KEY_ROW - 1 && c >= KEY_COL - 2) {
            if (s_on_exit != NULL) {
                s_on_exit();
            }
        } else if (r >= KEY_ROW - 1 && c < 18) {
            crt_radar_cycle_rotation();
        } else if (r >= LIST_ROW0 && r < LIST_ROW0 + LIST_N && s_list_hex[r - LIST_ROW0][0]) {
            select_hex(s_list_hex[r - LIST_ROW0]);
        }
    }
}

/* Scanlines over everything outside the scope (the scope does its own). */
static void root_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_draw_ctx_t *dc = lv_event_get_draw_ctx(e);
    const lv_area_t *c = dc->clip_area;
    const lv_area_t *s = &s_scope->coords;
    const lv_area_t *tm = &s_termobj->coords;
    int stride = lv_area_get_width(dc->buf_area);
    if (code == LV_EVENT_DRAW_MAIN) {
        /* The root has no LVGL background: in direct mode LVGL tests cover
         * against the whole frame buffer, so an opaque root would be painted
         * under every refreshed area and wipe what the scope leaves alone.
         * It paints only the gaps around the scope and the terminal. */
        for (int y = c->y1; y <= c->y2; y++) {
            uint16_t *row = (uint16_t *)dc->buf + (size_t)(y - dc->buf_area->y1) * stride - dc->buf_area->x1;
            bool rs = y >= s->y1 && y <= s->y2;
            bool rt = y >= tm->y1 && y <= tm->y2;
            for (int x = c->x1; x <= c->x2; x++) {
                if (rs && x >= s->x1 && x <= s->x2) {
                    x = s->x2;
                    continue;
                }
                if (rt && x >= tm->x1 && x <= tm->x2) {
                    x = tm->x2;
                    continue;
                }
                row[x] = s_term_bg;
            }
        }
        return;
    }
    if (code != LV_EVENT_DRAW_POST) {
        return;
    }
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

static void polar_point(uint32_t a, int radius, int *x, int *y)
{
    float r = a * (2.0f * (float)M_PI / 65536.0f);
    *x = SCOPE_C + (int)lroundf(radius * sinf(r));
    *y = SCOPE_C - (int)lroundf(radius * cosf(r));
}

/* Invalidate the wedge from angle `from` clockwise through `span` units.
 * Split per quadrant (inside one the arc is monotonic, so the corner points
 * bound it) and into radial bands: one box from the centre to the rim would
 * be close to twice the sector's real area. */
static void invalidate_wedge(uint32_t from, uint32_t span, bool fine)
{
    /* coarse bands for the narrow wedges keep the count of invalid areas
     * well inside LVGL's 32 (beyond that it redraws the whole screen) */
    static const int band_fine[] = { 0, 90, 160, 225, TUBE_R + 2 };
    static const int band_coarse[] = { 0, 150, TUBE_R + 2, TUBE_R + 2, TUBE_R + 2 };
    const int *band = fine ? band_fine : band_coarse;
    const int nb = fine ? 4 : 2;
    uint32_t a = from & 0xFFFF;
    while (span > 0) {
        uint32_t q_end = (a / 16384u + 1u) * 16384u;
        uint32_t seg = q_end - a;
        if (seg > span) {
            seg = span;
        }
        for (int k = 0; k < nb; k++) {
            int px[4], py[4];
            polar_point(a, band[k], &px[0], &py[0]);
            polar_point(a, band[k + 1], &px[1], &py[1]);
            polar_point(a + seg, band[k], &px[2], &py[2]);
            polar_point(a + seg, band[k + 1], &px[3], &py[3]);
            lv_area_t r = { px[0], py[0], px[0], py[0] };
            for (int i = 1; i < 4; i++) {
                r.x1 = LV_MIN(r.x1, px[i]);
                r.x2 = LV_MAX(r.x2, px[i]);
                r.y1 = LV_MIN(r.y1, py[i]);
                r.y2 = LV_MAX(r.y2, py[i]);
            }
            r.x1 -= 2;
            r.y1 -= 2;
            r.x2 += 2;
            r.y2 += 2;
            inval_area(&r);
        }
        a = (a + seg) & 0xFFFF;
        span -= seg;
    }
}

/* ------------------------------------------------------------------ */
/* console content                                                     */
/* ------------------------------------------------------------------ */

static int cmp_dist(const void *a, const void *b)
{
    float da = s_blips[*(const uint8_t *)a].dist_nm, db = s_blips[*(const uint8_t *)b].dist_nm;
    return da < db ? -1 : da > db;
}

static void console_update(void)
{
    char clk[12] = "--:--:--";
    time_t t = time(NULL);
    if (t > 1600000000) {
        time_t l = t + (tz_home_known() ? tz_home_offset() : 0);
        struct tm tm;
        gmtime_r(&l, &tm);
        snprintf(clk, sizeof(clk), "%02d:%02d:%02d", tm.tm_hour, tm.tm_min, tm.tm_sec);
    }

    uint8_t idx[MAX_AIRCRAFT];
    int n = 0;
    for (int i = 0; i < MAX_AIRCRAFT; i++) {
        if (s_blips[i].used && s_blips[i].alive && !s_blips[i].ground) {
            idx[n++] = (uint8_t)i;
        }
    }
    qsort(idx, n, 1, cmp_dist);

    term_put(0, TC_TXT, "PPI SURVEILLANCE  EPKK%*s", TERM_COLS - 22, clk);
    term_put(1, TC_DIM, "RNG %d NM  ROT %.1f S  TGT %d  %s",
             settings_get()->radius_nm, REV_MS / 1000.0, n, s_frame_stat);
    if (s_warning[0]) {
        term_put(2, TC_RED, "! %s", s_warning);
    } else if (s_rain_cells < 0) {
        term_put(2, TC_DIM, "WX ----  %s", s_weather);
    } else {
        term_put(2, TC_DIM, "WX %s %s  %s", s_rain_when, s_rain_cells ? "ECHO" : "CLEAR", s_weather);
    }
    sysmon_t sm;
    sysmon_get(&sm);
    if (sm.valid) {
        term_put(3, sm.chip_temp_c >= 70 ? TC_AMBER : TC_DIM,
                 "SYS %.0fC (MAX %.0fC)  CPU0 %d%%  CPU1 %d%%  RSSI %d",
                 sm.chip_temp_c, sm.chip_temp_max_c, sm.cpu_load_pct[0], sm.cpu_load_pct[1],
                 sm.wifi_rssi_dbm);
    } else {
        term_put(3, TC_DIM, "SYS ---");
    }

    const blip_t *sel = NULL;
    for (int i = 0; i < MAX_AIRCRAFT && s_sel_hex[0]; i++) {
        if (s_blips[i].used && blip_selected(&s_blips[i])) {
            sel = &s_blips[i];
        }
    }
    if (sel != NULL) {
        term_put(4, TC_TXT, "SEL %-8s %-5s %s", sel->cs[0] ? sel->cs : sel->hex, sel->type, sel->reg);
        term_put(5, TC_TXT, "ALT %5d  GS %3d  TRK %03d  VS %+d", sel->alt, (int)(sel->gs + 0.5f),
                 (int)(sel->trk + 0.5f) % 360, sel->vs);
        const route_info_t *rt = sel->cs[0] ? routes_get_cached(sel->cs) : NULL;
        if (rt != NULL && rt->valid) {
            term_put(6, TC_TXT, "%s > %s  %s",
                     rt->origin.iata[0] ? rt->origin.iata : rt->origin.icao,
                     rt->destination.iata[0] ? rt->destination.iata : rt->destination.icao,
                     rt->destination.city);
        } else {
            term_put(6, TC_DIM, "ROUTE UNKNOWN");
        }
        term_put(7, TC_TXT, "SQK %-4s  DST %.1f NM  BRG %03d", sel->sq[0] ? sel->sq : "----",
                 sel->dist_nm, (int)(sel->brg + 0.5f) % 360);
    } else {
        term_put(4, TC_DIM, "SEL ---");
        term_put(5, TC_DIM, "TAP A TARGET ON THE SCOPE OR IN THE LIST");
        term_put(6, TC_DIM, "");
        term_put(7, TC_DIM, "");
    }
    term_put(8, TC_DIM, "%.*s", TERM_COLS, "----------------------------------------------------");
    term_put(9, TC_DIM, "CALLSIGN  TYPE   FL   DST   BRG   GS");
    for (int k = 0; k < LIST_N; k++) {
        if (k < n) {
            const blip_t *b = &s_blips[idx[k]];
            strlcpy(s_list_hex[k], b->hex, sizeof(s_list_hex[k]));
            term_put(LIST_ROW0 + k, blip_selected(b) ? TC_AMBER : TC_TXT,
                     "%-8s  %-5s  %03d %5.1f  %03d  %3d", b->cs[0] ? b->cs : b->hex, b->type,
                     LV_MAX(0, b->alt / 100), b->dist_nm, (int)(b->brg + 0.5f) % 360,
                     (int)(b->gs + 0.5f));
        } else {
            s_list_hex[k][0] = '\0';
            term_put(LIST_ROW0 + k, TC_DIM, "");
        }
    }
    term_put(22, TC_DIM, "%.*s", TERM_COLS, "----------------------------------------------------");
    int lc = s_apt_level == 0 ? TC_TXT : s_apt_level == 1 ? TC_AMBER : s_apt_level == 2 ? TC_RED : TC_DIM;
    term_put(23, lc, "%s", s_apt_chip[0] ? s_apt_chip : "KRK NO DATA");
    int r = 24;
    r += term_wrap(r, 2, TC_DIM, s_apt_wx);
    term_put(r++, TC_DIM, "%s", s_apt_arr);
    term_put(r++, TC_DIM, "%s", s_apt_dep);
    while (r < KEY_ROW - 1) {
        term_put(r++, TC_DIM, "");
    }
    term_put(KEY_ROW - 1, TC_DIM, "%.*s", TERM_COLS, "----------------------------------------------------");
    uint8_t rot = settings_get()->screen_rotate_min;
    char autok[24];
    if (rot) {
        snprintf(autok, sizeof(autok), "[ AUTO %2d MIN ]", rot);
    } else {
        snprintf(autok, sizeof(autok), "[ AUTO   OFF  ]");
    }
    term_put(KEY_ROW, rot ? TC_AMBER : TC_TXT, "%-*s[    MAP    ]", KEY_COL - 1, autok);
    s_term[KEY_ROW].inv0 = KEY_COL - 1;
    s_term[KEY_ROW].inv1 = TERM_COLS;
}

void crt_radar_cycle_rotation(void)
{
    static const uint8_t steps[] = { 0, 1, 2, 5, 10, 15, 30 };
    settings_t *cfg = settings_get();
    size_t i = 0;
    while (i < sizeof(steps) && steps[i] != cfg->screen_rotate_min) {
        i++;
    }
    cfg->screen_rotate_min = steps[(i + 1) % sizeof(steps)];
    settings_save();
    s_console_ms = 0;
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
    s_ticks++;
    s_tickno++;
    if (s_full_ticks > 0) {
        s_full_ticks--;
    }
    int keep = 0;
    for (int i = 0; i < s_nforce; i++) {
        if (s_tickno - s_force[i].tick <= FORCE_TICKS) {
            s_force[keep++] = s_force[i];
        }
    }
    s_nforce = keep;
    /* everything the last frames painted near the beam is still settling in
     * one of the two buffers: trail, plus three steps, plus margin */
    s_active_u = DEG_U(TRAIL_DEG + 2) + 3 * step;

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
        force_full();
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

    /* loose boxes are fine: pixels away from the beam cost a bearing test */
    invalidate_wedge(acc - s_active_u, s_active_u + DEG_U(2), true);

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
            lv_area_t dot = { b->x - K_MAIN / 2, b->y - K_MAIN / 2,
                              b->x + K_MAIN / 2, b->y + K_MAIN / 2 };
            inval_scope(&dot);
            budget--;
        }
    }

    if (now - s_console_ms >= 1000) {
        if (s_ticks > 0 && s_console_ms > 0) {
            snprintf(s_frame_stat, sizeof(s_frame_stat), "DRAW %u MS  %u FPS",
                     (unsigned)(s_draw_us / s_ticks / 1000),
                     (unsigned)((s_ticks * 1000 + 500) / (uint32_t)(now - s_console_ms)));
        }
        s_draw_us = s_draw_px = s_ticks = 0;
        s_console_ms = now;
        console_update();
        rain_maybe_request();
    }
}

/* ------------------------------------------------------------------ */
/* objects                                                             */
/* ------------------------------------------------------------------ */

static lv_obj_t *custom_obj(lv_obj_t *parent, int x, int y, int w, int h, lv_event_cb_t cb)
{
    lv_obj_t *o = lv_obj_create(parent);
    lv_obj_remove_style_all(o);
    lv_obj_set_size(o, w, h);
    lv_obj_set_pos(o, x, y);
    lv_obj_add_flag(o, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(o, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(o, cb, LV_EVENT_ALL, NULL);
    return o;
}

static void build_objects(void)
{
    s_root = lv_obj_create(s_scr);
    lv_obj_remove_style_all(s_root);
    lv_obj_set_size(s_root, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_pos(s_root, 0, 0);
    lv_obj_add_event_cb(s_root, root_event_cb, LV_EVENT_DRAW_MAIN, NULL);
    lv_obj_clear_flag(s_root, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(s_root, root_event_cb, LV_EVENT_DRAW_POST, NULL);
    lv_obj_add_flag(s_root, LV_OBJ_FLAG_HIDDEN);

    s_scope = custom_obj(s_root, SCOPE_X, SCOPE_Y, SCOPE_D, SCOPE_D, scope_event_cb);
    s_termobj = custom_obj(s_root, TERM_X, TERM_Y, TERM_COLS * CELL_W, TERM_ROWS * CELL_H,
                           term_event_cb);

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
            s_frozen = false;
            lv_timer_pause(s_timer);
            lv_obj_add_flag(s_root, LV_OBJ_FLAG_HIDDEN);
            lv_obj_set_style_bg_opa(s_scr, LV_OPA_COVER, 0);
            lv_disp_set_bg_opa(NULL, LV_OPA_COVER);
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
    /* nothing may paint under the scope (see root_event_cb) */
    lv_obj_set_style_bg_opa(s_scr, LV_OPA_TRANSP, 0);
    lv_disp_set_bg_opa(NULL, LV_OPA_TRANSP);
    lv_obj_clear_flag(s_root, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(s_root);
    lv_obj_invalidate(s_root);
    s_full_ticks = FORCE_TICKS;
    s_nforce = 0;
    lv_timer_resume(s_timer);
    return true;
}

void crt_radar_freeze(bool frozen)
{
    if (!s_active || frozen == s_frozen) {
        return;
    }
    s_frozen = frozen;
    if (frozen) {
        lv_timer_pause(s_timer);
    } else {
        s_beam_init = false;
        lv_timer_resume(s_timer);
    }
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

void crt_radar_set_warning(const char *text)
{
    strlcpy(s_warning, text, sizeof(s_warning));
}

void crt_radar_set_airport(int level, const char *chip, const char *wx,
                           const char *arr, const char *dep)
{
    s_apt_level = level;
    strlcpy(s_apt_chip, chip, sizeof(s_apt_chip));
    strlcpy(s_apt_wx, wx, sizeof(s_apt_wx));
    strlcpy(s_apt_arr, arr, sizeof(s_apt_arr));
    strlcpy(s_apt_dep, dep, sizeof(s_apt_dep));
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
