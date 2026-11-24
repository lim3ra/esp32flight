#include "sysmon.h"

#include <stdio.h>
#include <string.h>
#include "driver/temperature_sensor.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mqtt_pub.h"

static const char *TAG = "sysmon";

#define SAMPLE_US       (10LL * 1000 * 1000)
#define PUBLISH_EVERY   6               /* samples: once a minute */

static temperature_sensor_handle_t s_ts;
static esp_timer_handle_t s_timer;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static sysmon_t s_now;
static uint32_t s_idle_prev[2];
static int64_t  s_wall_prev;
static int      s_samples;

/* Idle-task run time per core. The run-time counter is esp_timer based
 * (microseconds), so its delta against wall time is the idle fraction. */
static uint32_t idle_runtime(int core)
{
    TaskHandle_t h = xTaskGetIdleTaskHandleForCore(core);
    if (h == NULL) {
        return 0;
    }
    TaskStatus_t st;
    vTaskGetInfo(h, &st, pdFALSE, eRunning);
    return (uint32_t)st.ulRunTimeCounter;
}

static void sample_cb(void *arg)
{
    (void)arg;
    sysmon_t s;
    portENTER_CRITICAL(&s_lock);
    s = s_now;
    portEXIT_CRITICAL(&s_lock);

    float t = 0;
    if (s_ts != NULL && temperature_sensor_get_celsius(s_ts, &t) == ESP_OK) {
        s.chip_temp_c = t;
        if (!s.valid || t > s.chip_temp_max_c) {
            s.chip_temp_max_c = t;
        }
    }

    int64_t wall = esp_timer_get_time();
    uint32_t dwall = (uint32_t)(wall - s_wall_prev);
    for (int c = 0; c < 2; c++) {
        uint32_t idle = idle_runtime(c);
        uint32_t didle = idle - s_idle_prev[c];
        s_idle_prev[c] = idle;
        if (s_wall_prev != 0 && dwall > 0) {
            int load = 100 - (int)((uint64_t)didle * 100 / dwall);
            load = load < 0 ? 0 : load > 100 ? 100 : load;
            s.cpu_load_pct[c] = load;
            if (load > s.cpu_load_max_pct[c]) {
                s.cpu_load_max_pct[c] = load;
            }
        }
    }
    bool first = s_wall_prev == 0;
    s_wall_prev = wall;

    wifi_ap_record_t ap;
    s.wifi_rssi_dbm = esp_wifi_sta_get_ap_info(&ap) == ESP_OK ? ap.rssi : 0;
    s.valid = !first;

    portENTER_CRITICAL(&s_lock);
    s_now = s;
    portEXIT_CRITICAL(&s_lock);

    if (s.valid && ++s_samples % PUBLISH_EVERY == 1) {
        char json[320];
        snprintf(json, sizeof(json),
                 "{\"chip_temp\":%.1f,\"chip_temp_max\":%.1f,"
                 "\"cpu0\":%d,\"cpu1\":%d,\"rssi\":%d,"
                 "\"heap_int_free\":%u,\"heap_int_min\":%u,\"psram_free\":%u,"
                 "\"uptime_h\":%.1f}",
                 s.chip_temp_c, s.chip_temp_max_c, s.cpu_load_pct[0], s.cpu_load_pct[1],
                 s.wifi_rssi_dbm,
                 (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                 (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL),
                 (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
                 esp_timer_get_time() / 3.6e9);
        mqtt_pub_health(json);
    }
}

void sysmon_start(void)
{
    temperature_sensor_config_t cfg = TEMPERATURE_SENSOR_CONFIG_DEFAULT(10, 80);
    if (temperature_sensor_install(&cfg, &s_ts) == ESP_OK) {
        temperature_sensor_enable(s_ts);
    } else {
        s_ts = NULL;
        ESP_LOGW(TAG, "temperature sensor unavailable");
    }
    const esp_timer_create_args_t ta = { .callback = sample_cb, .name = "sysmon" };
    if (esp_timer_create(&ta, &s_timer) == ESP_OK) {
        esp_timer_start_periodic(s_timer, SAMPLE_US);
    }
}

void sysmon_get(sysmon_t *out)
{
    portENTER_CRITICAL(&s_lock);
    *out = s_now;
    portEXIT_CRITICAL(&s_lock);
}
