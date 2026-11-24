#include "mqtt_pub.h"

#include <ctype.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "mqtt_client.h"
#include "settings.h"
#ifndef APKFLIGHT
#include "waveshare_rgb_lcd_port.h"
#include "ui.h"
#include "airspace.h"
#include "raingrid.h"
#include "srcstat.h"
#include "weather.h"
#include "esp_heap_caps.h"
#endif

static const char *TAG = "mqtt";

#define STATE_TOPIC     "esp32flight/state"
#define HEALTH_TOPIC    "esp32flight/health"
#define AIRSPACE_TOPIC  "krk-airport/airspaces"
#define RAIN_TOPIC      "krk-airport/rain"
#define WEATHER_TOPIC   "krk-airport/weather"
#define BL_CONFIG_TOPIC "homeassistant/light/esp32flight_backlight/config"
#define BL_STATE_TOPIC  "esp32flight/backlight/state"
#define BL_BRI_TOPIC    "esp32flight/backlight/brightness"
#define BL_SET_TOPIC    "esp32flight/backlight/set"
#define BL_BRI_SET      "esp32flight/backlight/brightness/set"
/* Retained status from krk-airport-collector, shown in the airport bar */
#define AIRPORT_TOPIC   "krk-airport/panel"

static esp_mqtt_client_handle_t s_client;
static bool s_connected;
static bool s_light_on = true;

#ifndef APKFLIGHT
/* HA brightness sliders stream messages while dragged; batch the NVS write */
static esp_timer_handle_t s_bl_save;

static void bl_save_cb(void *arg)
{
    (void)arg;
    settings_save();
}

static void bl_save_later(void)
{
    if (s_bl_save == NULL) {
        const esp_timer_create_args_t a = { .callback = bl_save_cb, .name = "mqttbl" };
        if (esp_timer_create(&a, &s_bl_save) != ESP_OK) {
            return;
        }
    }
    esp_timer_stop(s_bl_save);
    esp_timer_start_once(s_bl_save, 3 * 1000 * 1000);
}

static bool bl_entity_active(void)
{
    return settings_get()->brightness_ctl && waveshare_rgb_lcd_bl_dimmable();
}

static void publish_bl_state(void)
{
    if (!bl_entity_active()) {
        return;
    }
    char bri[8];
    snprintf(bri, sizeof(bri), "%u", settings_get()->brightness);
    esp_mqtt_client_publish(s_client, BL_STATE_TOPIC, s_light_on ? "ON" : "OFF", 0, 1, 1);
    esp_mqtt_client_publish(s_client, BL_BRI_TOPIC, bri, 0, 1, 1);
}

/* Drive the panel to match the light's on/off state. Guarded on the current
 * state: bl_on() re-asserts full brightness, so calling it on an already-lit
 * panel would flash it before ui_apply_brightness() pulls it back down. */
static void bl_set_on(bool on)
{
    if (on == s_light_on) {
        return;
    }
    s_light_on = on;
    if (on) {
        waveshare_rgb_lcd_bl_on();
        ui_apply_brightness();
    } else {
        waveshare_rgb_lcd_bl_off();
    }
}
#endif

static void publish_discovery(void)
{
    static const struct {
        const char *object_id, *name, *tpl, *unit;
    } sensors[] = {
        { "nearest",  "Nearest aircraft", "{{ value_json.nearest }}", NULL },
        { "count",    "Aircraft in range", "{{ value_json.count }}", "aircraft" },
        { "unique",   "Unique aircraft (session)", "{{ value_json.unique }}", "aircraft" },
        { "nearest_dist", "Nearest distance", "{{ value_json.nearest_km }}", "km" },
    };
    const char *model =
#ifndef APKFLIGHT
        waveshare_lcd_board_name();
#else
        "Android";
#endif

    /* Discovery configs are retained, so an entity outlives the firmware
     * that announced it: dropping the publish code leaves it on the broker
     * and in Home Assistant forever. An empty retained payload is how MQTT
     * discovery deletes one. These two were the 7B edition's own additions
     * (next-panel button, screensaver switch) and went away with the views
     * they drove; the sweep is cheap and idempotent, so it just runs on
     * every connect rather than carrying a "have I cleaned up yet" flag. */
    static const char *k_retired[] = {
        "homeassistant/button/esp32flight_next_view/config",
        "homeassistant/switch/esp32flight_screensaver/config",
        /* the state topics outlive the entity the same way the configs do */
        "esp32flight/switch/screensaver/state",
        "esp32flight/button/next_view/press",
        "esp32flight/light/backlight/state",
    };
    for (size_t i = 0; i < sizeof(k_retired) / sizeof(k_retired[0]); i++) {
        esp_mqtt_client_publish(s_client, k_retired[i], "", 0, 1, 1);
    }
    /* board health from sysmon.c, diagnostic entities on their own topic */
    static const struct {
        const char *object_id, *name, *key, *unit, *dev_class;
    } health[] = {
        { "chip_temp", "Chip temperature", "chip_temp", "°C", "temperature" },
        { "chip_temp_max", "Chip temperature max", "chip_temp_max", "°C", "temperature" },
        { "cpu0_load", "CPU core 0 load", "cpu0", "%", NULL },
        { "cpu1_load", "CPU core 1 load", "cpu1", "%", NULL },
        { "wifi_rssi", "Wi-Fi signal", "rssi", "dBm", "signal_strength" },
        { "heap_int_free", "Internal heap free", "heap_int_free", "B", "data_size" },
        { "heap_int_min", "Internal heap minimum", "heap_int_min", "B", "data_size" },
        { "psram_free", "PSRAM free", "psram_free", "B", "data_size" },
        { "uptime", "Uptime", "uptime_h", "h", "duration" },
    };
    for (size_t i = 0; i < sizeof(health) / sizeof(health[0]); i++) {
        char topic[96], payload[640];
        snprintf(topic, sizeof(topic),
                 "homeassistant/sensor/esp32flight_%s/config", health[i].object_id);
        snprintf(payload, sizeof(payload),
                 "{\"name\":\"%s\",\"state_topic\":\"" HEALTH_TOPIC "\","
                 "\"value_template\":\"{{ value_json.%s }}\","
                 "\"unit_of_measurement\":\"%s\",%s%s%s"
                 "\"state_class\":\"measurement\",\"entity_category\":\"diagnostic\","
                 "\"expire_after\":300,"
                 "\"unique_id\":\"esp32flight_%s\","
                 "\"device\":{\"identifiers\":[\"esp32flight\"],"
                 "\"name\":\"esp32flight\",\"manufacturer\":\"theqkash\","
                 "\"model\":\"%s\"}}",
                 health[i].name, health[i].key, health[i].unit,
                 health[i].dev_class ? "\"device_class\":\"" : "",
                 health[i].dev_class ? health[i].dev_class : "",
                 health[i].dev_class ? "\"," : "",
                 health[i].object_id, model);
        esp_mqtt_client_publish(s_client, topic, payload, 0, 1, 1);
    }
    for (size_t i = 0; i < sizeof(sensors) / sizeof(sensors[0]); i++) {
        char topic[96], payload[512];
        snprintf(topic, sizeof(topic),
                 "homeassistant/sensor/esp32flight_%s/config", sensors[i].object_id);
        snprintf(payload, sizeof(payload),
                 "{\"name\":\"%s\",\"state_topic\":\"" STATE_TOPIC "\","
                 "\"value_template\":\"%s\","
                 "%s%s%s"
                 "\"unique_id\":\"esp32flight_%s\","
                 "\"device\":{\"identifiers\":[\"esp32flight\"],"
                 "\"name\":\"esp32flight\",\"manufacturer\":\"theqkash\","
                 "\"model\":\"%s\"}}",
                 sensors[i].name, sensors[i].tpl,
                 sensors[i].unit ? "\"unit_of_measurement\":\"" : "",
                 sensors[i].unit ? sensors[i].unit : "",
                 sensors[i].unit ? "\"," : "",
                 sensors[i].object_id, model);
        esp_mqtt_client_publish(s_client, topic, payload, 0, 1, 1);
    }
#ifndef APKFLIGHT
    esp_mqtt_client_subscribe(s_client, AIRPORT_TOPIC, 0);
    esp_mqtt_client_subscribe(s_client, AIRSPACE_TOPIC, 0);
    esp_mqtt_client_subscribe(s_client, RAIN_TOPIC, 0);
    esp_mqtt_client_subscribe(s_client, WEATHER_TOPIC, 0);
    /* Backlight as an HA light with brightness (lim3ra's request). Only
     * when the master brightness switch is on and the panel can dim; an
     * empty retained payload removes a previously discovered entity. */
    if (bl_entity_active()) {
        char payload[560];
        snprintf(payload, sizeof(payload),
                 "{\"name\":\"Screen backlight\","
                 "\"command_topic\":\"" BL_SET_TOPIC "\","
                 "\"state_topic\":\"" BL_STATE_TOPIC "\","
                 "\"brightness_command_topic\":\"" BL_BRI_SET "\","
                 "\"brightness_state_topic\":\"" BL_BRI_TOPIC "\","
                 "\"brightness_scale\":100,"
                 "\"unique_id\":\"esp32flight_backlight\","
                 "\"device\":{\"identifiers\":[\"esp32flight\"],"
                 "\"name\":\"esp32flight\",\"manufacturer\":\"theqkash\","
                 "\"model\":\"%s\"}}", model);
        esp_mqtt_client_publish(s_client, BL_CONFIG_TOPIC, payload, 0, 1, 1);
        esp_mqtt_client_subscribe(s_client, BL_SET_TOPIC, 0);
        esp_mqtt_client_subscribe(s_client, BL_BRI_SET, 0);
        publish_bl_state();
    } else {
        esp_mqtt_client_publish(s_client, BL_CONFIG_TOPIC, "", 0, 1, 1);
    }
#endif
}

#ifndef APKFLIGHT
static void handle_bl_command(const char *topic, size_t tlen,
                              const char *data, size_t dlen)
{
    if (!bl_entity_active()) {
        return;
    }
    char val[16];
    if (dlen >= sizeof(val)) {
        dlen = sizeof(val) - 1;
    }
    memcpy(val, data, dlen);
    val[dlen] = '\0';
    if (tlen == strlen(BL_SET_TOPIC) && strncmp(topic, BL_SET_TOPIC, tlen) == 0) {
        if (strcmp(val, "ON") == 0) {
            bl_set_on(true);
        } else if (strcmp(val, "OFF") == 0) {
            bl_set_on(false);
        }
        publish_bl_state();
        return;
    }
    if (tlen == strlen(BL_BRI_SET) && strncmp(topic, BL_BRI_SET, tlen) == 0) {
        int b = atoi(val);
        if (b < 5) b = 5;
        if (b > 100) b = 100;
        settings_get()->brightness = (uint8_t)b;
        if (s_light_on) {
            waveshare_rgb_lcd_bl_pct(b);
        }
        bl_save_later();
        publish_bl_state();
    }
}
#endif

static void mqtt_event(void *arg, esp_event_base_t base, int32_t event_id, void *data)
{
    if (event_id == MQTT_EVENT_CONNECTED) {
        s_connected = true;
        srcstat_link(SRC_BROKER, true);
        ESP_LOGI(TAG, "connected");
        publish_discovery();
    } else if (event_id == MQTT_EVENT_DISCONNECTED) {
        s_connected = false;
        srcstat_link(SRC_BROKER, false);
#ifndef APKFLIGHT
    } else if (event_id == MQTT_EVENT_DATA) {
        esp_mqtt_event_handle_t ev = data;
        /* The airspace (~12 KB) and rain (~35 KB) messages are larger than
         * the client buffer and arrive in pieces; only the first piece
         * carries the topic, so it decides where the rest goes. */
        static char *s_big_buf;
        static int s_big_total = -1;
        static int s_big_kind;            /* 1 airspace, 2 rain */
        if (ev->current_data_offset == 0 && ev->topic_len > 0) {
            int kind = 0;
            if (ev->topic_len == (int)strlen(AIRSPACE_TOPIC) &&
                strncmp(ev->topic, AIRSPACE_TOPIC, (size_t)ev->topic_len) == 0) {
                kind = 1;
            } else if (ev->topic_len == (int)strlen(RAIN_TOPIC) &&
                       strncmp(ev->topic, RAIN_TOPIC, (size_t)ev->topic_len) == 0) {
                kind = 2;
            }
            if (kind != 0) {
                free(s_big_buf);
                s_big_buf = NULL;
                s_big_total = -1;
                if (ev->total_data_len > 0 && ev->total_data_len <= 128 * 1024) {
                    s_big_buf = heap_caps_malloc((size_t)ev->total_data_len, MALLOC_CAP_SPIRAM);
                    s_big_total = s_big_buf != NULL ? ev->total_data_len : -1;
                    s_big_kind = kind;
                }
            }
        }
        if (s_big_total > 0 && ev->total_data_len == s_big_total &&
            ev->current_data_offset + ev->data_len <= s_big_total &&
            (ev->current_data_offset > 0 || ev->topic_len > 0)) {
            memcpy(s_big_buf + ev->current_data_offset, ev->data, (size_t)ev->data_len);
            if (ev->current_data_offset + ev->data_len == s_big_total) {
                if (s_big_kind == 1) {
                    airspace_feed(s_big_buf, s_big_total);
                } else {
                    raingrid_feed((const uint8_t *)s_big_buf, s_big_total);
                }
                free(s_big_buf);
                s_big_buf = NULL;
                s_big_total = -1;
            }
            return;
        }
        if (ev->topic_len == (int)strlen(WEATHER_TOPIC) &&
            strncmp(ev->topic, WEATHER_TOPIC, (size_t)ev->topic_len) == 0) {
            weather_feed(ev->data, ev->data_len);
            return;
        }
        if (ev->topic_len == (int)strlen(AIRPORT_TOPIC) &&
            strncmp(ev->topic, AIRPORT_TOPIC, (size_t)ev->topic_len) == 0) {
            /* The message is far below the client's buffer size, so a
             * fragmented one is not ours to reassemble. */
            if (ev->current_data_offset == 0 && ev->data_len == ev->total_data_len) {
                ui_airport_feed(ev->data, ev->data_len);
            } else {
                ESP_LOGW(TAG, "airport message fragmented (%d bytes), ignored",
                         ev->total_data_len);
            }
        } else {
            handle_bl_command(ev->topic, (size_t)ev->topic_len,
                              ev->data, (size_t)ev->data_len);
        }
#endif
    }
}


/* In-place %XX decoding of a URI component. */
static void pct_decode(char *s)
{
    char *o = s;
    for (; *s; s++) {
        if (s[0] == '%' && isxdigit((unsigned char)s[1]) && isxdigit((unsigned char)s[2])) {
            char hex[3] = { s[1], s[2], 0 };
            *o++ = (char)strtol(hex, NULL, 16);
            s += 2;
        } else {
            *o++ = *s;
        }
    }
    *o = '\0';
}

void mqtt_pub_start(void)
{
    const char *uri = settings_get()->mqtt_uri;
    if (uri[0] == '\0') {
        return;
    }
    /* esp-mqtt takes user:pass@ from the URI verbatim, and a raw '#', '@'
     * or '/' in a password breaks the URL parse - so the credentials are
     * split off here, %XX-decoded, and passed as separate fields. A
     * password with '#' is written as %23 in the settings. */
    static char s_uri[96], s_user[64], s_pass[64];
    strlcpy(s_uri, uri, sizeof(s_uri));
    s_user[0] = s_pass[0] = '\0';
    char *auth = strstr(s_uri, "://");
    char *at = auth != NULL ? strrchr(auth + 3, '@') : NULL;
    if (at != NULL) {
        char *info = auth + 3;
        *at = '\0';
        char *colon = strchr(info, ':');
        if (colon != NULL) {
            *colon = '\0';
            strlcpy(s_pass, colon + 1, sizeof(s_pass));
            pct_decode(s_pass);
        }
        strlcpy(s_user, info, sizeof(s_user));
        pct_decode(s_user);
        memmove(info, at + 1, strlen(at + 1) + 1);   /* drop "user:pass@" */
    }
    esp_mqtt_client_config_t cfg = {
        .broker.address.uri = s_uri,
        .credentials.username = s_user[0] ? s_user : NULL,
        .credentials.authentication.password = s_pass[0] ? s_pass : NULL,
        .task.priority = 4,
    };
    s_client = esp_mqtt_client_init(&cfg);
    if (s_client == NULL) {
        ESP_LOGE(TAG, "bad broker uri");
        return;
    }
    esp_mqtt_client_register_event(s_client, ESP_EVENT_ANY_ID, mqtt_event, NULL);
    esp_mqtt_client_start(s_client);
    /* credentials were split off above: s_uri holds the host only */
    ESP_LOGI(TAG, "connecting to %s", s_uri);
}

void mqtt_pub_backlight_changed(void)
{
#ifndef APKFLIGHT
    if (s_client != NULL && s_connected) {
        publish_bl_state();
    }
#endif
}

void mqtt_pub_backlight_on(bool on)
{
#ifndef APKFLIGHT
    bl_set_on(on);
    mqtt_pub_backlight_changed();
#else
    (void)on;
#endif
}

/* Called from the esp_timer task: enqueue, never block on the network. */
void mqtt_pub_health(const char *json)
{
    if (s_client == NULL || !s_connected) {
        return;
    }
    esp_mqtt_client_enqueue(s_client, HEALTH_TOPIC, json, 0, 0, 0, true);
}

void mqtt_pub_state(const char *json)
{
    if (s_client == NULL || !s_connected) {
        return;
    }
    esp_mqtt_client_publish(s_client, STATE_TOPIC, json, 0, 0, 0);
}
