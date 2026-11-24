#include "raingrid.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "srcstat.h"

static const char *TAG = "raingrid";

#define MAX_CELLS   400
#define FRESH_US    (30LL * 60 * 1000 * 1000)

/* Format: "RAIN1 <frame> <lat> <lon> <half_km> <cells> <npal>\n", npal x
 * (r, g, b, a, intensity), then (count, index) run-length pairs, row 0 at
 * the north edge. */
static uint8_t *s_grid;           /* PSRAM, cells x cells */
static uint8_t *s_next;           /* PSRAM, decode target, swapped in */
static uint8_t  s_pal[256][5];
static volatile int s_cells;      /* 0 = no grid */
static double   s_lat, s_lon, s_half_km, s_km_per_cell, s_km_per_deg_lon;
static long long s_frame;
static int64_t  s_got_us;
static volatile int s_gen;

void raingrid_feed(const uint8_t *data, int len)
{
    const uint8_t *nl = memchr(data, '\n', len > 128 ? 128 : (size_t)len);
    if (nl == NULL || len < 6 || memcmp(data, "RAIN1", 5) != 0) {
        ESP_LOGW(TAG, "feed ignored (%d bytes)", len);
        return;
    }
    char head[128];
    size_t hl = (size_t)(nl - data);
    memcpy(head, data, hl);
    head[hl] = '\0';
    long long frame;
    double lat, lon, half;
    int cells, npal;
    if (sscanf(head, "RAIN1 %lld %lf %lf %lf %d %d", &frame, &lat, &lon, &half, &cells, &npal) != 6 ||
        cells <= 0 || cells > MAX_CELLS || npal <= 0 || npal > 256 ||
        (int)(hl + 1 + (size_t)npal * 5) > len) {
        ESP_LOGW(TAG, "bad header: %s", head);
        return;
    }
    size_t n = (size_t)cells * cells;
    if (s_next == NULL) {
        s_next = heap_caps_malloc((size_t)MAX_CELLS * MAX_CELLS, MALLOC_CAP_SPIRAM);
    }
    if (s_grid == NULL) {
        s_grid = heap_caps_calloc(1, (size_t)MAX_CELLS * MAX_CELLS, MALLOC_CAP_SPIRAM);
    }
    if (s_next == NULL || s_grid == NULL) {
        return;
    }
    const uint8_t *p = nl + 1 + npal * 5, *end = data + len;
    size_t o = 0;
    while (p + 1 < end && o < n) {
        size_t k = p[0];
        uint8_t v = p[1] < npal ? p[1] : 0;
        if (k > n - o) {
            k = n - o;
        }
        memset(s_next + o, v, k);
        o += k;
        p += 2;
    }
    if (o != n) {
        ESP_LOGW(TAG, "grid short: %u of %u cells", (unsigned)o, (unsigned)n);
        return;
    }
    /* readers sample from other tasks: hide, swap, show */
    s_cells = 0;
    uint8_t *t = s_grid;
    s_grid = s_next;
    s_next = t;
    memcpy(s_pal, nl + 1, (size_t)npal * 5);
    s_lat = lat;
    s_lon = lon;
    s_half_km = half;
    s_km_per_cell = 2.0 * half / cells;
    s_km_per_deg_lon = 111.320 * cos(lat * M_PI / 180.0);
    s_frame = frame;
    s_got_us = esp_timer_get_time();
    s_cells = cells;
    s_gen++;
    srcstat_ok(SRC_RAIN);
    ESP_LOGI(TAG, "grid %dx%d, %d colours, frame %lld", cells, cells, npal, frame);
}

bool raingrid_fresh(void)
{
    return s_cells > 0 && esp_timer_get_time() - s_got_us < FRESH_US;
}

int raingrid_generation(void)
{
    return s_gen;
}

long long raingrid_frame_time(void)
{
    return s_cells > 0 ? s_frame : 0;
}

uint8_t raingrid_sample(double lat, double lon)
{
    int cells = s_cells;
    if (cells == 0) {
        return 0;
    }
    double north = (lat - s_lat) * 110.574;
    double east = (lon - s_lon) * s_km_per_deg_lon;
    int i = (int)floor((east + s_half_km) / s_km_per_cell);
    int j = (int)floor((s_half_km - north) / s_km_per_cell);
    if (i < 0 || j < 0 || i >= cells || j >= cells) {
        return 0;
    }
    return s_grid[j * cells + i];
}

void raingrid_palette(uint8_t idx, uint8_t *r, uint8_t *g, uint8_t *b, uint8_t *a,
                      uint8_t *intensity)
{
    *r = s_pal[idx][0];
    *g = s_pal[idx][1];
    *b = s_pal[idx][2];
    *a = s_pal[idx][3];
    *intensity = s_pal[idx][4];
}
