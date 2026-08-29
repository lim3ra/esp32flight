#include "airspace.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "cJSON.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "geo_math.h"
#include "http_util.h"
#include "settings.h"

static const char *TAG = "airspace";

/* Outlines survive a reboot here. Header first, then s_count entries. */
#define CACHE_PATH   "/assets/airspace.bin"
#define CACHE_MAGIC  0x31505341u        /* "ASP1" */

typedef struct {
    uint32_t magic;
    uint32_t key_fp;    /* so a new openAIP key invalidates the cache */
    float    lat, lon;  /* where it was fetched, for the 25 km check */
    int32_t  count;
} asp_cache_hdr_t;

#define FETCH_DIST_M   50000   /* openAIP caps dist at 50 km */
#define RETRY_US       (30LL * 60 * 1000 * 1000)

/* Ask only for the types we draw. Unfiltered, a 50 km query answers with
 * ~340 KB, and cJSON needs several times the text size in nodes to parse it
 * - more than this board has free, so the parse failed and the layer stayed
 * empty. Filtered, the same area is ~38 KB. Keep in step with
 * airspace_type_str. */
#define TYPE_QUERY \
    "&type=1&type=2&type=3&type=4&type=7&type=13&type=21&type=26&type=27"

#define BUF_SIZE       (128 * 1024)

static airspace_t *s_asp;     /* PSRAM, MAX_AIRSPACES entries */
static int s_count;
static double s_try_lat = 999, s_try_lon = 999;
static int64_t s_last_try = -RETRY_US;
static char s_try_key[64];
static bool s_ok;             /* last fetch for s_try location and key succeeded */
static bool s_loaded;         /* cache file consulted once this boot */

static uint32_t key_fp(const char *s)
{
    uint32_t h = 2166136261u;           /* FNV-1a */
    for (; *s != '\0'; s++) {
        h = (h ^ (uint8_t)*s) * 16777619u;
    }
    return h;
}

static void cache_save(double home_lat, double home_lon, const char *key)
{
    if (s_count <= 0 || s_asp == NULL) {
        return;
    }
    FILE *f = fopen(CACHE_PATH, "wb");
    if (f == NULL) {
        return;
    }
    asp_cache_hdr_t h = {
        .magic = CACHE_MAGIC,
        .key_fp = key_fp(key),
        .lat = (float)home_lat,
        .lon = (float)home_lon,
        .count = s_count,
    };
    if (fwrite(&h, sizeof(h), 1, f) != 1 ||
        fwrite(s_asp, sizeof(airspace_t), (size_t)s_count, f) != (size_t)s_count) {
        fclose(f);
        unlink(CACHE_PATH);     /* a half-written cache is worse than none */
        return;
    }
    fclose(f);
}

/* Draw something at boot instead of waiting out a fetch - and, when the
 * fetch fails, instead of waiting out the 30 minute retry with a bare map. */
static void cache_load(double home_lat, double home_lon, const char *key)
{
    FILE *f = fopen(CACHE_PATH, "rb");
    if (f == NULL) {
        return;
    }
    asp_cache_hdr_t h;
    if (fread(&h, sizeof(h), 1, f) != 1 || h.magic != CACHE_MAGIC ||
        h.key_fp != key_fp(key) || h.count <= 0 || h.count > MAX_AIRSPACES ||
        geo_haversine_km(h.lat, h.lon, home_lat, home_lon) > 25.0) {
        fclose(f);
        return;
    }
    if (s_asp == NULL) {
        s_asp = heap_caps_calloc(MAX_AIRSPACES, sizeof(airspace_t),
                                 MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    }
    if (s_asp != NULL &&
        fread(s_asp, sizeof(airspace_t), (size_t)h.count, f) == (size_t)h.count) {
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

static bool type_wanted(int type)
{
    return airspace_type_str(type)[0] != '\0';
}

static void parse_airspaces(const char *json, double home_lat, double home_lon)
{
    cJSON *root = cJSON_Parse(json);
    if (root == NULL) {
        /* Silently tolerating this is what made an out-of-memory parse look
         * like "there are no airspaces here". */
        /* keep whatever is already on screen rather than blanking it */
        ESP_LOGW(TAG, "response did not parse (%u bytes, PSRAM free %u)",
                 (unsigned)strlen(json),
                 (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
        return;
    }
    s_count = 0;   /* renderer reads from another task; hide during rewrite */
    const cJSON *items = cJSON_GetObjectItem(root, "items");
    int n = 0;
    for (const cJSON *it = items != NULL ? items->child : NULL;
         it != NULL && n < MAX_AIRSPACES; it = it->next) {
        const cJSON *ty = cJSON_GetObjectItem(it, "type");
        if (!cJSON_IsNumber(ty) || !type_wanted(ty->valueint)) {
            continue;
        }
        const cJSON *geom = cJSON_GetObjectItem(it, "geometry");
        const cJSON *coords = geom != NULL ? cJSON_GetObjectItem(geom, "coordinates") : NULL;
        /* Polygon: coordinates[0] = outer ring of [lon, lat] pairs.
           MultiPolygon nests one level deeper; take its first polygon. */
        const cJSON *ring = coords != NULL ? coords->child : NULL;
        if (ring != NULL && ring->child != NULL && ring->child->child != NULL &&
            cJSON_IsArray(ring->child->child)) {
            ring = ring->child;
        }
        if (ring == NULL) {
            continue;
        }
        int total = cJSON_GetArraySize((cJSON *)ring);
        if (total < 4) {
            continue;
        }
        airspace_t *a = &s_asp[n];
        memset(a, 0, sizeof(*a));
        a->type = ty->valueint;
        const cJSON *nm = cJSON_GetObjectItem(it, "name");
        if (cJSON_IsString(nm)) {
            strlcpy(a->name, nm->valuestring, sizeof(a->name));
        }
        /* decimate long rings down to MAX_ASP_POINTS */
        int step = (total + MAX_ASP_POINTS - 1) / MAX_ASP_POINTS;
        if (step < 1) {
            step = 1;
        }
        int idx = 0;
        for (const cJSON *pt = ring->child; pt != NULL && a->n_points < MAX_ASP_POINTS;
             pt = pt->next, idx++) {
            if (idx % step != 0) {
                continue;
            }
            const cJSON *plon = pt->child;
            const cJSON *plat = plon != NULL ? plon->next : NULL;
            if (!cJSON_IsNumber(plon) || !cJSON_IsNumber(plat)) {
                break;
            }
            a->lat[a->n_points] = (float)plat->valuedouble;
            a->lon[a->n_points] = (float)plon->valuedouble;
            a->n_points++;
        }
        if (a->n_points >= 4) {
            n++;
        }
    }
    if (root != NULL) {
        cJSON_Delete(root);
    }
    s_count = n;
    (void)home_lat;
    (void)home_lon;
    ESP_LOGI(TAG, "%d airspace outlines cached", n);
}

void airspace_poll(double home_lat, double home_lon)
{
    const settings_t *cfg = settings_get();
    if (!cfg->airspace_enabled || cfg->openaip_key[0] == '\0') {
        s_count = 0;
        return;
    }
    if (!s_loaded) {
        s_loaded = true;
        cache_load(home_lat, home_lon, cfg->openaip_key);
    }
    bool moved = geo_haversine_km(s_try_lat, s_try_lon, home_lat, home_lon) > 25.0;
    bool rekeyed = strcmp(s_try_key, cfg->openaip_key) != 0;
    if (s_ok && !moved && !rekeyed) {
        return;
    }
    /* after a failed attempt wait out the retry window, even for a new key:
       hammering the API every poll cycle trips its rate limit */
    int64_t now = esp_timer_get_time();
    if (now - s_last_try < RETRY_US) {
        return;
    }
    s_last_try = now;
    s_try_lat = home_lat;
    s_try_lon = home_lon;
    strlcpy(s_try_key, cfg->openaip_key, sizeof(s_try_key));

    if (s_asp == NULL) {
        s_asp = heap_caps_calloc(MAX_AIRSPACES, sizeof(airspace_t),
                                 MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (s_asp == NULL) {
            ESP_LOGW(TAG, "no PSRAM for the %u byte outline table",
                     (unsigned)(MAX_AIRSPACES * sizeof(airspace_t)));
            return;
        }
    }
    char *buf = heap_caps_malloc(BUF_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (buf == NULL) {
        ESP_LOGW(TAG, "no PSRAM for the %d KB fetch buffer (free %u, largest %u)",
                 BUF_SIZE / 1024,
                 (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
                 (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM));
        return;
    }
    char url[320];
    snprintf(url, sizeof(url),
             "https://api.core.openaip.net/api/airspaces?pos=%.4f,%.4f&dist=%d&limit=60"
             TYPE_QUERY,
             home_lat, home_lon, FETCH_DIST_M);
    if (http_get_to_buffer_hdr(url, buf, BUF_SIZE, NULL,
                               "x-openaip-api-key", cfg->openaip_key) == ESP_OK) {
        parse_airspaces(buf, home_lat, home_lon);
        s_ok = s_count > 0;
        ESP_LOGI(TAG, "airspace fetch ok, %d outlines kept", s_count);
        cache_save(home_lat, home_lon, cfg->openaip_key);
    } else {
        s_ok = false;
        ESP_LOGW(TAG, "fetch failed (rate limit, bad key or offline), retry in 30 min");
    }
    heap_caps_free(buf);
}

int airspace_count(void)
{
    return s_count;
}

const airspace_t *airspace_get(int idx)
{
    return (idx >= 0 && idx < s_count) ? &s_asp[idx] : NULL;
}
