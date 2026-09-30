#include "eventlog.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "nvs.h"
#include "tz.h"

static const char *TAG = "evlog";

#define EVLOG_NS      "evlog"
#define EVLOG_KEY     "ring"
#define EVLOG_MAGIC   0x314c5645u      /* "EVL1" */
#define EVLOG_SLOTS   16
#define EVLOG_LEN     96

typedef struct {
    uint32_t magic;
    uint16_t next;                     /* slot the next record goes into */
    uint16_t count;                    /* how many slots are filled */
    char     rec[EVLOG_SLOTS][EVLOG_LEN];
} evlog_t;

static evlog_t *s_log;                 /* NULL until init succeeds */
static SemaphoreHandle_t s_mux;

void eventlog_init(void)
{
    if (s_log != NULL) {
        return;
    }
    s_log = calloc(1, sizeof(*s_log));
    if (s_log == NULL) {
        ESP_LOGE(TAG, "no memory for the %u byte ring", (unsigned)sizeof(*s_log));
        return;
    }
    s_mux = xSemaphoreCreateMutex();

    nvs_handle_t h;
    if (nvs_open(EVLOG_NS, NVS_READONLY, &h) == ESP_OK) {
        size_t len = sizeof(*s_log);
        if (nvs_get_blob(h, EVLOG_KEY, s_log, &len) != ESP_OK ||
            len != sizeof(*s_log) || s_log->magic != EVLOG_MAGIC) {
            memset(s_log, 0, sizeof(*s_log));
        }
        nvs_close(h);
    }
    s_log->magic = EVLOG_MAGIC;
    if (s_log->next >= EVLOG_SLOTS || s_log->count > EVLOG_SLOTS) {
        s_log->next = 0;
        s_log->count = 0;
    }
}

static void evlog_store(void)
{
    nvs_handle_t h;
    if (nvs_open(EVLOG_NS, NVS_READWRITE, &h) != ESP_OK) {
        return;
    }
    if (nvs_set_blob(h, EVLOG_KEY, s_log, sizeof(*s_log)) == ESP_OK) {
        nvs_commit(h);
    }
    nvs_close(h);
}

void eventlog_add(const char *fmt, ...)
{
    if (s_log == NULL) {
        return;
    }
    char body[EVLOG_LEN];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(body, sizeof(body), fmt, ap);
    va_end(ap);

    /* Wall clock when SNTP has been round, uptime otherwise - a record from
     * before the first sync is still worth keeping, just less precisely. */
    char stamp[24];
    time_t now = time(NULL);
    long up_s = (long)(esp_timer_get_time() / 1000000);
    if (now > 1600000000) {
        time_t local = now + (tz_home_known() ? tz_home_offset() : 0);
        struct tm tm;
        gmtime_r(&local, &tm);
        snprintf(stamp, sizeof(stamp), "%02d-%02d %02d:%02d:%02d",
                 tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
    } else {
        snprintf(stamp, sizeof(stamp), "up%lds", up_s);
    }

    if (s_mux != NULL && xSemaphoreTake(s_mux, pdMS_TO_TICKS(200)) != pdTRUE) {
        return;
    }
    /* Composed in a roomy local and copied with an explicit truncation:
       formatting straight into the slot is a -Wformat-truncation error. */
    char line[EVLOG_LEN + sizeof(stamp) + 24];
    snprintf(line, sizeof(line), "%s (up%lds) %s", stamp, up_s, body);
    strlcpy(s_log->rec[s_log->next], line, EVLOG_LEN);
    s_log->next = (uint16_t)((s_log->next + 1) % EVLOG_SLOTS);
    if (s_log->count < EVLOG_SLOTS) {
        s_log->count++;
    }
    evlog_store();
    if (s_mux != NULL) {
        xSemaphoreGive(s_mux);
    }
    ESP_LOGI(TAG, "%s", body);
}

size_t eventlog_dump(char *dst, size_t n)
{
    if (dst == NULL || n == 0) {
        return 0;
    }
    dst[0] = '\0';
    if (s_log == NULL || s_log->count == 0) {
        return 0;
    }
    size_t used = 0;
    int first = s_log->count < EVLOG_SLOTS
                    ? 0
                    : s_log->next;                 /* oldest slot */
    for (int i = 0; i < s_log->count; i++) {
        const char *r = s_log->rec[(first + i) % EVLOG_SLOTS];
        if (r[0] == '\0') {
            continue;
        }
        int w = snprintf(dst + used, n - used, "%s\n", r);
        if (w < 0 || (size_t)w >= n - used) {
            break;
        }
        used += (size_t)w;
    }
    return used;
}
