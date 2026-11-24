#include "airspace.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "settings.h"
#include "srcstat.h"

static const char *TAG = "airspace";

/* Since 2026-11-05 openAIP refuses geospatial queries (pos/dist, bbox) from
 * external clients, and paging a whole country of JSON through TLS is more
 * than this board can spare. krk-airport-collector does the fetching,
 * filtering and simplifying and publishes a retained text message:
 *   ASP1 <unix> <count>
 *   <type>\t<name>\t<lat>,<lon>;<lat>,<lon>;...
 * The last feed is kept in flash so outlines show before MQTT connects. */

#define CACHE_PATH   "/assets/airspace.bin"
#define CACHE_MAGIC  0x32505341u        /* "ASP2": collector feed */

typedef struct {
    uint32_t magic;
    int64_t  feed_unix;
    int32_t  count;
} asp_cache_hdr_t;

static airspace_t *s_asp;     /* PSRAM, MAX_AIRSPACES entries */
static airspace_t *s_tmp;     /* PSRAM, parse target swapped in at the end */
static int s_count;
static bool s_loaded;         /* cache file consulted once this boot */
static int64_t s_feed_unix;   /* collector timestamp of the outlines shown */

static bool ensure_tables(void)
{
    if (s_asp == NULL) {
        s_asp = heap_caps_calloc(MAX_AIRSPACES, sizeof(airspace_t),
                                 MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    }
    if (s_tmp == NULL) {
        s_tmp = heap_caps_calloc(MAX_AIRSPACES, sizeof(airspace_t),
                                 MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    }
    return s_asp != NULL && s_tmp != NULL;
}

static void cache_save(void)
{
    if (s_count <= 0) {
        return;
    }
    FILE *f = fopen(CACHE_PATH, "wb");
    if (f == NULL) {
        return;
    }
    asp_cache_hdr_t h = { .magic = CACHE_MAGIC, .feed_unix = s_feed_unix, .count = s_count };
    if (fwrite(&h, sizeof(h), 1, f) != 1 ||
        fwrite(s_asp, sizeof(airspace_t), (size_t)s_count, f) != (size_t)s_count) {
        fclose(f);
        unlink(CACHE_PATH);     /* a half-written cache is worse than none */
        return;
    }
    fclose(f);
}

static void cache_load(void)
{
    FILE *f = fopen(CACHE_PATH, "rb");
    if (f == NULL) {
        return;
    }
    asp_cache_hdr_t h;
    if (fread(&h, sizeof(h), 1, f) == 1 && h.magic == CACHE_MAGIC &&
        h.count > 0 && h.count <= MAX_AIRSPACES && ensure_tables() &&
        fread(s_asp, sizeof(airspace_t), (size_t)h.count, f) == (size_t)h.count) {
        s_feed_unix = h.feed_unix;
        s_count = h.count;
        ESP_LOGI(TAG, "%d outlines restored from cache", s_count);
    }
    fclose(f);
}

/* openAIP numeric airspace types, the handful worth naming on screen */
const char *airspace_type_str(int type)
{
    switch (type) {
    case 1:  return "R";     /* restricted */
    case 2:  return "D";     /* danger */
    case 3:  return "P";     /* prohibited */
    case 4:  return "CTR";
    case 7:  return "TMA";
    case 13: return "ATZ";
    case 21: return "TRA";
    case 26: return "MTMA";
    case 27: return "MCTR";
    default: return "";
    }
}

void airspace_feed(const char *data, int len)
{
    if (len < 5 || strncmp(data, "ASP1", 4) != 0) {
        ESP_LOGW(TAG, "feed ignored (%d bytes, no ASP1 header)", len);
        return;
    }
    if (!ensure_tables()) {
        return;
    }
    const char *end = data + len;
    const char *p = memchr(data, '\n', (size_t)len);
    int64_t unix_ts = atoll(data + 4);
    p = p != NULL ? p + 1 : end;
    int n = 0;
    while (p < end && n < MAX_AIRSPACES) {
        const char *eol = memchr(p, '\n', (size_t)(end - p));
        if (eol == NULL) {
            eol = end;
        }
        const char *t1 = memchr(p, '\t', (size_t)(eol - p));
        const char *t2 = t1 != NULL ? memchr(t1 + 1, '\t', (size_t)(eol - t1 - 1)) : NULL;
        if (t2 != NULL) {
            airspace_t *a = &s_tmp[n];
            memset(a, 0, sizeof(*a));
            a->type = atoi(p);
            size_t nlen = (size_t)(t2 - t1 - 1);
            if (nlen >= sizeof(a->name)) {
                nlen = sizeof(a->name) - 1;
            }
            memcpy(a->name, t1 + 1, nlen);
            const char *q = t2 + 1;
            while (q < eol && a->n_points < MAX_ASP_POINTS) {
                char *e1 = NULL, *e2 = NULL;
                float la = strtof(q, &e1);
                if (e1 == q || e1 >= eol || *e1 != ',') {
                    break;
                }
                float lo = strtof(e1 + 1, &e2);
                if (e2 == e1 + 1) {
                    break;
                }
                a->lat[a->n_points] = la;
                a->lon[a->n_points] = lo;
                a->n_points++;
                q = (e2 < eol && *e2 == ';') ? e2 + 1 : eol;
            }
            if (a->n_points >= 3 && airspace_type_str(a->type)[0] != '\0') {
                n++;
            }
        }
        p = eol + 1;
    }
    /* renderers read from other tasks: hide, copy, show */
    s_count = 0;
    memcpy(s_asp, s_tmp, sizeof(airspace_t) * (size_t)n);
    s_feed_unix = unix_ts;
    s_count = n;
    srcstat_ok(SRC_COLLECTOR);
    ESP_LOGI(TAG, "%d airspace outlines from the collector", n);
    cache_save();
}

void airspace_poll(double home_lat, double home_lon)
{
    (void)home_lat;
    (void)home_lon;
    if (!s_loaded) {
        s_loaded = true;
        if (s_count == 0) {
            cache_load();
        }
    }
}

int airspace_count(void)
{
    return settings_get()->airspace_enabled ? s_count : 0;
}

const airspace_t *airspace_get(int idx)
{
    return (idx >= 0 && idx < airspace_count()) ? &s_asp[idx] : NULL;
}

long long airspace_feed_time(void)
{
    return s_feed_unix;
}
