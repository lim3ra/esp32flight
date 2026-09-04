#include "flight_task.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "cJSON.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_netif_sntp.h"
#include "esp_wifi.h"
#include "esp_timer.h"
#include "lvgl.h"
#include "sdkconfig.h"

#include "flight_data.h"
#include "geolocate.h"
#include "geocode.h"
#include "airlines.h"
#include "lvgl_port.h"
#include "routes.h"
#include "settings.h"
#include "clock_fmt.h"
#include <math.h>
#include "geo_math.h"
#include "http_util.h"
#include "faflight.h"
#include "lang.h"
#include "mqtt_pub.h"
#include "airports.h"
#include "airspace.h"
#include "regcountry.h"
#include "trails.h"
#include "tz.h"
#include "waveshare_rgb_lcd_port.h"
#include "ui.h"
#include "weather.h"
#include "web_server.h"
#include "wifi_mgr.h"
#include "esp_app_desc.h"

static const char *TAG = "flight_task";

/* New adsbdb lookups per poll cycle - keep the free service happy. */
#define MAX_ROUTE_LOOKUPS_PER_CYCLE 8
/* Only resolve routes for the closest N aircraft. */
#define ROUTE_LOOKUP_TOP_N MAX_AIRCRAFT

static void set_status(const char *text)
{
    if (lvgl_port_lock(1000)) {
        ui_set_status(text);
        lvgl_port_unlock();
    }
}

/* Session stats (also exposed on the web panel and the stats view) */
static struct {
    uint32_t hexes[1024];
    int      unique;
    int      max_alt_ft;
    float    max_gs_kt;
    float    max_dist_km;
    char     max_dist_cs[CALLSIGN_LEN];
    uint16_t hours[24];
    struct {
        char     code[4];
        uint16_t n;
    } airlines[32];
    int      airlines_n;
} s_stats;

void flight_stats_get(app_stats_t *out)
{
    memset(out, 0, sizeof(*out));
    out->unique = s_stats.unique;
    out->max_alt_ft = s_stats.max_alt_ft;
    out->max_gs_kt = s_stats.max_gs_kt;
    out->max_dist_km = s_stats.max_dist_km;
    strlcpy(out->max_dist_cs, s_stats.max_dist_cs, sizeof(out->max_dist_cs));
    memcpy(out->hours, s_stats.hours, sizeof(out->hours));

    /* top 8 airlines by count */
    bool used[32] = { 0 };
    for (int k = 0; k < 8; k++) {
        int best = -1;
        for (int i = 0; i < s_stats.airlines_n; i++) {
            if (!used[i] && (best < 0 || s_stats.airlines[i].n > s_stats.airlines[best].n)) {
                best = i;
            }
        }
        if (best < 0) {
            break;
        }
        used[best] = true;
        strlcpy(out->top[k].code, s_stats.airlines[best].code, sizeof(out->top[k].code));
        out->top[k].n = s_stats.airlines[best].n;
        out->top_n = k + 1;
    }
}

static void count_airline(const aircraft_t *ac)
{
    if (!flight_is_airline(ac)) {
        return;
    }
    char code[4] = { ac->callsign[0], ac->callsign[1], ac->callsign[2], '\0' };
    for (int i = 0; i < s_stats.airlines_n; i++) {
        if (strcmp(s_stats.airlines[i].code, code) == 0) {
            s_stats.airlines[i].n++;
            return;
        }
    }
    if (s_stats.airlines_n < 32) {
        strlcpy(s_stats.airlines[s_stats.airlines_n].code, code, 4);
        s_stats.airlines[s_stats.airlines_n].n = 1;
        s_stats.airlines_n++;
    }
}

static void stats_update(const aircraft_list_t *list)
{
    for (int i = 0; i < list->count; i++) {
        const aircraft_t *ac = &list->ac[i];
        uint32_t h = (uint32_t)strtoul(ac->hex[0] == '~' ? ac->hex + 1 : ac->hex, NULL, 16);
        bool known = false;
        for (int k = 0; k < s_stats.unique; k++) {
            if (s_stats.hexes[k] == h) {
                known = true;
                break;
            }
        }
        if (!known && s_stats.unique < 1024) {
            s_stats.hexes[s_stats.unique++] = h;

            /* first sighting this session */
            time_t now = time(NULL);
            if (now > 1600000000) {
                time_t local = now + (tz_home_known() ? tz_home_offset() : 0);
                struct tm tm;
                gmtime_r(&local, &tm);
                s_stats.hours[tm.tm_hour]++;
            }
            count_airline(ac);
        }
        if (ac->alt_baro_ft > s_stats.max_alt_ft) {
            s_stats.max_alt_ft = ac->alt_baro_ft;
        }
        if (ac->gs_kts > s_stats.max_gs_kt) {
            s_stats.max_gs_kt = ac->gs_kts;
        }
        float dkm = ac->dist_nm >= 0 ? ac->dist_nm * 1.852f : 0;
        if (dkm > s_stats.max_dist_km) {
            s_stats.max_dist_km = dkm;
            strlcpy(s_stats.max_dist_cs, ac->callsign[0] ? ac->callsign : ac->hex,
                    sizeof(s_stats.max_dist_cs));
        }
    }
}



/* Once a day: compare the newest GitHub release against the running build */
static void check_updates(void)
{
    char *buf = heap_caps_malloc(16 * 1024, MALLOC_CAP_SPIRAM);
    if (buf == NULL) {
        return;
    }
    if (http_get_to_buffer_hdr(
            "https://api.github.com/repos/theqkash/esp32flight/releases/latest",
            buf, 16 * 1024, NULL,
            "Accept", "application/vnd.github+json") == ESP_OK) {
        cJSON *root = cJSON_Parse(buf);
        const cJSON *tag = root ? cJSON_GetObjectItem(root, "tag_name") : NULL;
        if (cJSON_IsString(tag)) {
            int lmaj = 0, lmin = 0, lpat = 0, cmaj = 0, cmin = 0, cpat = 0;
            sscanf(tag->valuestring, "v%d.%d.%d", &lmaj, &lmin, &lpat);
            sscanf(esp_app_get_description()->version, "%d.%d.%d", &cmaj, &cmin, &cpat);
            long latest = (long)lmaj * 1000000 + lmin * 1000 + lpat;
            long current = (long)cmaj * 1000000 + cmin * 1000 + cpat;
            if (latest > current) {
                ESP_LOGI(TAG, "update available: %s (running %s)",
                         tag->valuestring, esp_app_get_description()->version);
                if (lvgl_port_lock(1000)) {
                    ui_set_update_available(true, tag->valuestring);
                    lvgl_port_unlock();
                }
            }
        }
        if (root != NULL) {
            cJSON_Delete(root);
        }
    }
    free(buf);
}

/* NULL if no emergency squawk on the list */
static const aircraft_t *find_emergency(const aircraft_list_t *list)
{
    static const char *codes[] = { "7700", "7500", "7600" };
    for (size_t c = 0; c < 3; c++) {
        for (int i = 0; i < list->count; i++) {
            if (strcmp(list->ac[i].squawk, codes[c]) == 0) {
                return &list->ac[i];
            }
        }
    }
    return NULL;
}

static void publish_web_state(const aircraft_list_t *list, const weather_t *wx,
                              double lat, double lon, const char *city, int radius_nm)
{
    /* scalars for /metrics first: Prometheus scrapes must stay fresh even
     * when no browser keeps the full JSON build alive */
    web_metrics_publish(list->count, s_stats.unique, s_stats.max_alt_ft,
                        list->count > 0 && list->ac[0].dist_nm >= 0
                            ? list->ac[0].dist_nm * 1.852 : 0.0);
    if (!web_state_wanted()) {
        return;   /* nobody is looking at /api/state right now */
    }
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "city", city);
    cJSON_AddNumberToObject(root, "lat", lat);
    cJSON_AddNumberToObject(root, "lon", lon);
    cJSON_AddNumberToObject(root, "radius_km", (int)(radius_nm * 1.852));
    if (wx->valid) {
        cJSON *jw = cJSON_AddObjectToObject(root, "weather");
        cJSON_AddNumberToObject(jw, "temp_c", (int)(wx->temp_c + 0.5f));
        cJSON_AddStringToObject(jw, "desc", weather_code_str(wx->code));
        cJSON_AddNumberToObject(jw, "wind_kmh", (int)wx->wind_kmh);
    }
    /* Network info for the panel */
    cJSON *jn = cJSON_AddObjectToObject(root, "net");
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    esp_netif_ip_info_t ip_info;
    if (netif != NULL && esp_netif_get_ip_info(netif, &ip_info) == ESP_OK) {
        char ip[16];
        snprintf(ip, sizeof(ip), IPSTR, IP2STR(&ip_info.ip));
        cJSON_AddStringToObject(jn, "ip", ip);
#ifndef APKFLIGHT   /* a desktop-class heap number is just noise */
        cJSON_AddNumberToObject(jn, "heap_int",
                                (double)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
#endif
        snprintf(ip, sizeof(ip), IPSTR, IP2STR(&ip_info.gw));
        cJSON_AddStringToObject(jn, "gateway", ip);
    }
    wifi_ap_record_t ap;
    if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
        cJSON_AddStringToObject(jn, "ssid", (const char *)ap.ssid);
        cJSON_AddNumberToObject(jn, "rssi", ap.rssi);
        cJSON_AddNumberToObject(jn, "channel", ap.primary);
    }
#ifdef APKFLIGHT
    /* no mDNS in the app - the panel is reached by ip:8080 */
#else
    cJSON_AddStringToObject(jn, "mdns", "esp32flight.local");
#endif
    /* ota_enabled is injected live by the /api/state handler, not cached here */

    cJSON_AddBoolToObject(root, "metric", settings_get()->metric_units);
    cJSON *js = cJSON_AddObjectToObject(root, "stats");
    cJSON_AddNumberToObject(js, "unique_aircraft", s_stats.unique);
    cJSON_AddNumberToObject(js, "max_alt_ft", s_stats.max_alt_ft);
    cJSON_AddNumberToObject(js, "max_gs_kt", (int)s_stats.max_gs_kt);
    cJSON_AddNumberToObject(js, "max_dist_km", (int)s_stats.max_dist_km);
    cJSON_AddStringToObject(js, "max_dist_callsign", s_stats.max_dist_cs);
    cJSON_AddNumberToObject(js, "uptime_min", (int)(esp_timer_get_time() / 60000000LL));
    cJSON_AddStringToObject(js, "version", esp_app_get_description()->version);
    cJSON_AddStringToObject(js, "board", waveshare_lcd_board_name());
    cJSON *jh = cJSON_AddArrayToObject(js, "hours");
    for (int i = 0; i < 24; i++) {
        cJSON_AddItemToArray(jh, cJSON_CreateNumber(s_stats.hours[i]));
    }
    app_stats_t snap;
    flight_stats_get(&snap);
    cJSON *jt = cJSON_AddArrayToObject(js, "top_airlines");
    for (int i = 0; i < snap.top_n; i++) {
        cJSON *e = cJSON_CreateObject();
        cJSON_AddStringToObject(e, "code", snap.top[i].code);
        cJSON_AddNumberToObject(e, "n", snap.top[i].n);
        cJSON_AddItemToArray(jt, e);
    }

    cJSON *arr = cJSON_AddArrayToObject(root, "flights");
    for (int i = 0; i < list->count; i++) {
        const aircraft_t *ac = &list->ac[i];
        cJSON *jf = cJSON_CreateObject();
        cJSON_AddStringToObject(jf, "callsign", ac->callsign[0] ? ac->callsign : ac->hex);
        cJSON_AddStringToObject(jf, "hex", ac->hex);
        cJSON_AddStringToObject(jf, "type", ac->type_desc[0] ? ac->type_desc : ac->type_icao);
        cJSON_AddNumberToObject(jf, "alt_ft", ac->on_ground ? 0 : ac->alt_baro_ft);
        cJSON_AddNumberToObject(jf, "gs_kt", (int)ac->gs_kts);
        cJSON_AddNumberToObject(jf, "dist_km", (double)(int)(ac->dist_nm * 1.852f * 10) / 10);
        cJSON_AddStringToObject(jf, "squawk", ac->squawk);
        cJSON_AddNumberToObject(jf, "cls", flight_class(ac));
        cJSON_AddNumberToObject(jf, "spr", flight_sprite(ac));
        if (ac->has_pos) {
            cJSON_AddNumberToObject(jf, "lat", round(ac->lat * 1e5) / 1e5);
            cJSON_AddNumberToObject(jf, "lon", round(ac->lon * 1e5) / 1e5);
            cJSON_AddNumberToObject(jf, "track", (int)ac->track_deg);
            float tlat[12], tlon[12];
            int tn = trails_get(ac->hex, tlat, tlon, 12);
            if (tn >= 2) {
                cJSON *jt2 = cJSON_AddArrayToObject(jf, "trail");
                for (int k = 0; k < tn; k++) {
                    cJSON *pt = cJSON_CreateArray();
                    cJSON_AddItemToArray(pt, cJSON_CreateNumber(round(tlat[k] * 1e4) / 1e4));
                    cJSON_AddItemToArray(pt, cJSON_CreateNumber(round(tlon[k] * 1e4) / 1e4));
                    cJSON_AddItemToArray(jt2, pt);
                }
            }
        }
        if (ac->reg[0]) {
            cJSON_AddStringToObject(jf, "reg", ac->reg);
            const char *cc = reg_country(ac->reg);
            if (cc != NULL) {
                cJSON_AddStringToObject(jf, "cc", cc);
            }
        }

        const route_info_t *rt = routes_get_cached(ac->callsign);
        bool rt_ok = rt != NULL && rt->valid &&
                     (!ac->has_pos ||
                      geo_route_plausible(rt->origin.lat, rt->origin.lon,
                                          rt->destination.lat, rt->destination.lon,
                                          ac->lat, ac->lon));
        if (rt_ok) {
            cJSON *jr = cJSON_AddObjectToObject(jf, "route");
            cJSON_AddStringToObject(jr, "from", rt->origin.iata[0] ? rt->origin.iata : rt->origin.icao);
            cJSON_AddStringToObject(jr, "to", rt->destination.iata[0] ? rt->destination.iata : rt->destination.icao);
            cJSON_AddStringToObject(jr, "from_city", rt->origin.city);
            cJSON_AddStringToObject(jr, "to_city", rt->destination.city);
            cJSON_AddStringToObject(jr, "from_cc", rt->origin.country);
            cJSON_AddStringToObject(jr, "to_cc", rt->destination.country);
            time_t now = time(NULL);
            if (now > 1600000000) {
                char lt[12];
                struct tm tm;
                if (rt->origin.tz_known &&
                    !(tz_home_known() && rt->origin.tz_offset_s == tz_home_offset())) {
                    time_t l = now + rt->origin.tz_offset_s;
                    gmtime_r(&l, &tm);
                    clock_fmt(lt, sizeof(lt), tm.tm_hour, tm.tm_min);
                    cJSON_AddStringToObject(jr, "from_time", lt);
                }
                if (rt->destination.tz_known &&
                    !(tz_home_known() && rt->destination.tz_offset_s == tz_home_offset())) {
                    time_t l = now + rt->destination.tz_offset_s;
                    gmtime_r(&l, &tm);
                    clock_fmt(lt, sizeof(lt), tm.tm_hour, tm.tm_min);
                    cJSON_AddStringToObject(jr, "to_time", lt);
                }
            }
            cJSON_AddNumberToObject(jr, "from_lat", rt->origin.lat);
            cJSON_AddNumberToObject(jr, "from_lon", rt->origin.lon);
            cJSON_AddNumberToObject(jr, "to_lat", rt->destination.lat);
            cJSON_AddNumberToObject(jr, "to_lon", rt->destination.lon);
            cJSON_AddNumberToObject(jr, "progress",
                                    (int)(geo_progress(rt->origin.lat, rt->origin.lon,
                                                       rt->destination.lat, rt->destination.lon,
                                                       ac->lat, ac->lon) * 100));
            if (rt->airline_name[0]) {
                cJSON_AddStringToObject(jf, "airline", rt->airline_name);
            }
        }
        if (!cJSON_GetObjectItem(jf, "airline") && flight_is_airline(ac)) {
            char code[4] = { ac->callsign[0], ac->callsign[1], ac->callsign[2], '\0' };
            const char *name = airlines_get_cached(code);
            if (name != NULL && name[0]) {
                cJSON_AddStringToObject(jf, "airline", name);
            }
        }
        const char *iata = faflight_get_cached(ac->callsign);
        if (iata != NULL && iata[0]) {
            cJSON_AddStringToObject(jf, "flight_iata", iata);
        }
        if (flight_is_interesting(ac, settings_get()->watch_regs)) {
            cJSON_AddBoolToObject(jf, "interesting", true);
        }
        cJSON_AddItemToArray(arr, jf);
    }

    char *json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (json != NULL) {
        web_state_publish(json);
        free(json);
    }
}

static void flight_task(void *arg)
{
    if (settings_get()->wifi_ssid[0]) {
        set_status(L()->connecting);
    } else {
        char sbuf[64];
        snprintf(sbuf, sizeof(sbuf), L()->setup_wifi, LV_SYMBOL_SETTINGS);
        set_status(sbuf);
    }
    while (!wifi_mgr_wait_connected(10000)) {
        if (settings_get()->wifi_ssid[0]) {
            int r = wifi_mgr_last_reason();
            const char *hint = NULL;
            if (r == 201) {
                hint = L()->wifi_no_ap;
            } else if (r == 2 || r == 3 || r == 15 || r == 204 || r == 205) {
                hint = L()->wifi_badpass;
            }
            char sbuf[96];
            if (hint != NULL) {
                snprintf(sbuf, sizeof(sbuf), "%s (%s)", L()->connecting, hint);
            } else {
                strlcpy(sbuf, L()->connecting, sizeof(sbuf));
            }
            set_status(sbuf);
        }
    }

    esp_sntp_config_t sntp_cfg = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
    esp_netif_sntp_init(&sntp_cfg);

    set_status(L()->locating);
    double lat = 0, lon = 0;
    char city[32] = "";
    while (geolocate_get(&lat, &lon, city, sizeof(city)) != ESP_OK) {
        set_status(L()->geo_retry);
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
    if (city[0] == '\0') {
        /* fixed coordinates carry no name - resolve one from the map */
        geocode_reverse(lat, lon, city, sizeof(city));
    }

    /* Clock + ETA follow the device's location, wherever it is */
    int home_off;
    if (tz_offset_for("HOME", lat, lon, &home_off)) {
        tz_set_home_offset(home_off);
    }
    if (lvgl_port_lock(1000)) {
        ui_set_home(lat, lon);
        lvgl_port_unlock();
    }

    mqtt_pub_start();

    char status[96];
    aircraft_list_t *list = calloc(1, sizeof(aircraft_list_t));
    if (list == NULL) {
        set_status("out of memory");
        vTaskDelete(NULL);
        return;
    }

    int radius_nm = settings_get()->radius_nm;
    int consecutive_failures = 0;
    /* First cycle goes straight to the aircraft fetch: the update check,
     * weather and METAR are each a 1-3 s TLS handshake, and running them
     * first kept the screen on "waiting for aircraft" for a minute. They
     * start on the next cycle, once planes are up (or after 3 dry runs). */
    bool primed = false;
    bool first_shown = false;
    int fetch_attempts = 0;
    int64_t last_weather_ms = -1;
    int64_t last_update_check_ms = -1;
    static weather_t wx;
#ifdef APKFLIGHT
    /* App build applies settings live (no restart): re-resolve location and
     * radius whenever the settings screen saves. */
    extern unsigned apk_settings_gen(void);
    unsigned cfg_gen = apk_settings_gen();
#endif
    while (true) {
        int64_t tick_ms = esp_timer_get_time() / 1000;
#ifdef APKFLIGHT
        if (apk_settings_gen() != cfg_gen) {
            cfg_gen = apk_settings_gen();
            double nlat = lat, nlon = lon;
            char ncity[32] = "";
            if (geolocate_get(&nlat, &nlon, ncity, sizeof(ncity)) == ESP_OK) {
                lat = nlat;
                lon = nlon;
                if (ncity[0] == '\0') {
                    geocode_reverse(nlat, nlon, ncity, sizeof(ncity));
                }
                if (ncity[0]) {
                    strlcpy(city, ncity, sizeof(city));
                }
                int off;
                char tzkey[12];   /* fresh key per change: skip the tz cache */
                snprintf(tzkey, sizeof(tzkey), "HOME%u", cfg_gen);
                if (tz_offset_for(tzkey, lat, lon, &off)) {
                    tz_set_home_offset(off);
                }
                if (lvgl_port_lock(1000)) {
                    ui_set_home(lat, lon);
                    lvgl_port_unlock();
                }
                last_weather_ms = -1;   /* refresh weather for the new spot */
            }
            radius_nm = settings_get()->radius_nm;
            last_weather_ms = -1;
        }
#endif
        /* Aircraft first: everything else (weather, layers, update check)
         * runs after the fetch + render, so it can never delay positions. */
        esp_err_t err = flight_fetch_nearby(lat, lon, radius_nm, list);
        fetch_attempts++;
        if (!primed && (err == ESP_OK || fetch_attempts >= 3)) {
            primed = true;   /* don't starve weather forever on a bad link */
        }
        if (err == ESP_OK) {
            consecutive_failures = 0;

            const settings_t *cfg = settings_get();
            if (cfg->hide_ground ||
                (cfg->show_classes & FCLS_ALL_MASK) != FCLS_ALL_MASK) {
                int w = 0;
                for (int i = 0; i < list->count; i++) {
                    const aircraft_t *ac = &list->ac[i];
                    if (cfg->hide_ground && ac->on_ground) {
                        continue;
                    }
                    if (!(cfg->show_classes & (1u << flight_class(ac)))) {
                        continue;
                    }
                    list->ac[w++] = *ac;
                }
                list->count = w;
            }

            /* The airport filter must run before the early render below or
             * to-be-hidden flights would flash on screen every cycle. It
             * judges from cached routes only, so it costs nothing; routes
             * fetched later this cycle take effect next cycle. */
            const char *fapt0 = settings_get()->filter_airport;
            if (fapt0[0] != '\0') {
                int w = 0;
                bool excl = settings_get()->filter_apt_exclude;
                for (int i = 0; i < list->count; i++) {
                    const route_info_t *rt = routes_get_cached(list->ac[i].callsign);
                    bool match = rt != NULL && rt->valid &&
                        (strcasecmp(rt->origin.icao, fapt0) == 0 ||
                         strcasecmp(rt->origin.iata, fapt0) == 0 ||
                         strcasecmp(rt->destination.icao, fapt0) == 0 ||
                         strcasecmp(rt->destination.iata, fapt0) == 0);
                    /* "show only": unknown routes hidden; "hide": kept */
                    if (excl ? !match : match) {
                        list->ac[w++] = list->ac[i];
                    }
                }
                list->count = w;
            }

            /* How many enrichment lookups this cycle will actually hit the
             * network decides whether an early render pays off: with a warm
             * cache the cycle finishes in milliseconds anyway. */
            int top = list->count < ROUTE_LOOKUP_TOP_N ? list->count : ROUTE_LOOKUP_TOP_N;
            int pending = 0;
            for (int i = 0; i < top; i++) {
                const aircraft_t *ac = &list->ac[i];
                if (ac->callsign[0] == '\0') {
                    continue;
                }
                /* evict cached routes the current geometry refutes */
                routes_revalidate(ac->callsign, ac->lat, ac->lon, ac->has_pos,
                                  ac->track_deg, ac->gs_kts, ac->baro_rate_fpm);
                if (routes_get_cached(ac->callsign) == NULL) {
                    pending++;
                }
            }
            bool early_render = (pending > 0 || !first_shown) && list->count > 0;
            if (early_render) {
                /* Fresh positions hit the screen before route enrichment:
                 * one slow route DB otherwise holds the whole frame back. */
                first_shown = true;
                if (lvgl_port_lock(2000)) {
                    ui_update(list);
                    lvgl_port_unlock();
                }
                publish_web_state(list, &wx, lat, lon, city, radius_nm);
            }

            int lookups = 0;
            for (int i = 0; i < top && lookups < MAX_ROUTE_LOOKUPS_PER_CYCLE; i++) {
                const char *cs = list->ac[i].callsign;
                if (cs[0] != '\0' && routes_get_cached(cs) == NULL) {
                    routes_fetch(cs, list->ac[i].lat, list->ac[i].lon, list->ac[i].has_pos,
                                 list->ac[i].track_deg, list->ac[i].gs_kts,
                                 list->ac[i].baro_rate_fpm);
                    lookups++;
                    if (lookups < MAX_ROUTE_LOOKUPS_PER_CYCLE && lookups < pending) {
                        vTaskDelay(pdMS_TO_TICKS(250));   /* pace the APIs, skip after the last */
                    }
                }
            }

            /* Airline full names for prefixes the route DBs didn't name */
            int alookups = 0;
            for (int i = 0; i < top; i++) {
                const char *cs = list->ac[i].callsign;
                if (!flight_is_airline(&list->ac[i])) {
                    continue;
                }
                const route_info_t *rt = routes_get_cached(cs);
                if (rt != NULL && rt->valid && rt->airline_name[0]) {
                    continue;
                }
                char code[4] = { cs[0], cs[1], cs[2], '\0' };
                if (airlines_get_cached(code) == NULL) {
                    airlines_fetch(code);
                    if (++alookups >= 2) {
                        break;
                    }
                }
            }

            /* Commercial flight numbers, when a FlightAware key is set */
            if (settings_get()->fa_key[0] != '\0') {
                int fa_done = 0;
                for (int i = 0; i < top; i++) {
                    if (flight_is_airline(&list->ac[i]) &&
                        faflight_get_cached(list->ac[i].callsign) == NULL) {
                        faflight_fetch(list->ac[i].callsign);
                        if (++fa_done >= 2) {
                            break;  /* two lookups per cycle */
                        }
                    }
                }
            }

            /* airport filter runs after the route lookups above, because it
             * can only judge flights whose route is already known */
            const char *fapt = settings_get()->filter_airport;
            if (fapt[0] != '\0') {
                int w = 0;
                bool excl = settings_get()->filter_apt_exclude;
                for (int i = 0; i < list->count; i++) {
                    const route_info_t *rt = routes_get_cached(list->ac[i].callsign);
                    bool match = rt != NULL && rt->valid &&
                        (strcasecmp(rt->origin.icao, fapt) == 0 ||
                         strcasecmp(rt->origin.iata, fapt) == 0 ||
                         strcasecmp(rt->destination.icao, fapt) == 0 ||
                         strcasecmp(rt->destination.iata, fapt) == 0);
                    /* "show only": unknown routes hidden; "hide": kept */
                    if (excl ? !match : match) {
                        list->ac[w++] = list->ac[i];
                    }
                }
                list->count = w;
            }

            stats_update(list);
            trails_update(list);

            const aircraft_t *emergency = find_emergency(list);
            if (emergency != NULL) {
                snprintf(status, sizeof(status), LV_SYMBOL_WARNING " %s squawk %s!",
                         emergency->callsign[0] ? emergency->callsign : emergency->hex,
                         emergency->squawk);
            } else {
                snprintf(status, sizeof(status), L()->status_fmt,
                         city[0] ? city : "home", list->count,
                         (int)(radius_nm * 1.852));
            }
            if (lvgl_port_lock(2000)) {
                ui_set_status_alert(emergency != NULL);
                ui_set_status(status);
                ui_update(list);
                lvgl_port_unlock();
            }
            /* the early publish already carried this cycle's positions;
             * repeat only when enrichment could have changed anything */
            if (!early_render || lookups > 0) {
                publish_web_state(list, &wx, lat, lon, city, radius_nm);
            }

            /* Home Assistant state */
            char mq[192];
            if (list->count > 0) {
                const aircraft_t *n0 = &list->ac[0];
                snprintf(mq, sizeof(mq),
                         "{\"nearest\":\"%s\",\"nearest_km\":%.1f,"
                         "\"count\":%d,\"unique\":%d}",
                         n0->callsign[0] ? n0->callsign : n0->hex,
                         n0->dist_nm * 1.852, list->count, s_stats.unique);
            } else {
                snprintf(mq, sizeof(mq),
                         "{\"nearest\":\"none\",\"nearest_km\":0,"
                         "\"count\":0,\"unique\":%d}", s_stats.unique);
            }
            mqtt_pub_state(mq);
        } else {
            consecutive_failures++;
            ESP_LOGW(TAG, "fetch failed (%s), %d in a row",
                     esp_err_to_name(err), consecutive_failures);
            if (consecutive_failures >= 3) {
                set_status(L()->no_data);
            }
        }

        /* Housekeeping after the aircraft are on screen (still gated by
         * primed so a dead link does not starve the first fetch retries). */
        if (primed && (last_update_check_ms < 0 || tick_ms - last_update_check_ms > 24LL * 3600 * 1000)) {
            check_updates();
            last_update_check_ms = tick_ms;
        }
        /* Weather for the header, refreshed every 15 min */
        int64_t now_ms = esp_timer_get_time() / 1000;
        if (primed && (last_weather_ms < 0 || now_ms - last_weather_ms > 15 * 60 * 1000)) {
            if (weather_fetch(lat, lon, &wx) == ESP_OK) {
                char wbuf[96];
                bool fahr = settings_get()->temp_f;
                float temp = fahr ? wx.temp_c * 9.0f / 5.0f + 32.0f : wx.temp_c;
                /* wind before the description so truncation trims words, not data */
                snprintf(wbuf, sizeof(wbuf), "%s %.0f\xC2\xB0%s  \xEF\x9C\xAE %s %.0f  %s",
                         weather_icon_str(wx.code), (double)temp, fahr ? "F" : "C",
                         wx.wind_dir_deg >= 0 ? lang_compass(wx.wind_dir_deg) : "",
                         (double)wx.wind_kmh, lang_weather_desc(wx.code));
                if (lvgl_port_lock(1000)) {
                    ui_set_weather(wbuf);
                    lvgl_port_unlock();
                }
            }
            last_weather_ms = now_ms;
        }
        if (primed) {
            airspace_poll(lat, lon);
        }

        vTaskDelay(pdMS_TO_TICKS(CONFIG_CANFLIGHT_POLL_SECONDS * 1000));
    }
}

void flight_task_start(void)
{
    BaseType_t ok = xTaskCreatePinnedToCore(flight_task, "flight_task", 8192, NULL, 4, NULL, 0);
    if (ok != pdPASS) {
        /* Without this task the radar silently shows "waiting for aircraft"
         * forever - make the failure loud (seen on 480x272 bring-up: large
         * bounce buffers fragmented internal RAM below the 8 KB stack). */
        ESP_LOGE("flight", "flight_task create FAILED (largest free block %u B)",
                 (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
        set_status("out of memory - flight task dead");
    }
}
