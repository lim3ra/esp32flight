#include "srcstat.h"

#include <stdio.h>
#include <string.h>
#include "esp_timer.h"
#include "settings.h"

typedef struct {
    const char *name;
    int         window_s;    /* down when no success for this long */
    int64_t     ok_us;       /* last success, 0 = never */
    bool        link_up;
    bool        link_based;
} src_t;

/* Windows sit well above each source's own cadence: aircraft every ~10 s,
 * the collector every 2 min, weather every 10-15 min, RainViewer 10 min. */
static src_t s_src[SRC_COUNT] = {
    [SRC_ADSB]      = { "ADS-B API", 3 * 60 },
    [SRC_BROKER]    = { "MQTT BROKER", 2 * 60, .link_based = true },
    [SRC_COLLECTOR] = { "COLLECTOR", 15 * 60 },
    [SRC_WEATHER]   = { "WEATHER API", 60 * 60 },
    [SRC_RAIN]      = { "RAINVIEWER", 40 * 60 },
};
static int64_t s_link_down_us[SRC_COUNT];

static int64_t now_us(void)
{
    return esp_timer_get_time();
}

void srcstat_ok(src_id_t s)
{
    if (s < SRC_COUNT) {
        s_src[s].ok_us = now_us();
    }
}

void srcstat_fail(src_id_t s)
{
    (void)s;    /* the window decides; a single failure is not an outage */
}

void srcstat_link(src_id_t s, bool up)
{
    if (s >= SRC_COUNT) {
        return;
    }
    if (up) {
        s_src[s].ok_us = now_us();
    } else if (s_src[s].link_up) {
        s_link_down_us[s] = now_us();
    }
    s_src[s].link_up = up;
}

static bool configured(src_id_t s)
{
    /* without a broker URI there is no broker and no collector to miss */
    if (s == SRC_BROKER || s == SRC_COLLECTOR) {
        return settings_get()->mqtt_uri[0] != '\0';
    }
    return true;
}

int srcstat_age_min(src_id_t s)
{
    if (s >= SRC_COUNT || s_src[s].ok_us == 0) {
        return -1;
    }
    return (int)((now_us() - s_src[s].ok_us) / 60000000LL);
}

int srcstat_problems(char *buf, size_t n)
{
    int64_t now = now_us();
    int down = 0;
    size_t len = 0;
    buf[0] = '\0';
    for (int i = 0; i < SRC_COUNT; i++) {
        src_t *s = &s_src[i];
        if (!configured((src_id_t)i)) {
            continue;
        }
        int64_t since;     /* when it went quiet: last success, or boot */
        if (s->link_based) {
            if (s->link_up) {
                continue;
            }
            since = s->ok_us ? s_link_down_us[i] : 0;
        } else {
            since = s->ok_us;
        }
        int64_t quiet_s = (now - since) / 1000000LL;
        if (quiet_s < s->window_s) {
            continue;
        }
        down++;
        if (len < n) {
            len += (size_t)snprintf(buf + len, n - len, "%s%s %s %d MIN",
                                    len ? " | " : "", s->name,
                                    s->ok_us ? "OFFLINE" : "NO CONTACT",
                                    (int)(quiet_s / 60));
        }
    }
    return down;
}
