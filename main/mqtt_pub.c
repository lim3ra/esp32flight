#include "mqtt_pub.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <strings.h>
#include "cJSON.h"
#include "esp_log.h"
#include "mqtt_client.h"
#include "settings.h"
#include "ui.h"

static const char *TAG = "mqtt";

#define STATE_TOPIC       "esp32flight/state"
#define LIGHT_STATE_TOPIC "esp32flight/light/backlight/state"
#define LIGHT_CMD_TOPIC   "esp32flight/light/backlight/set"
#define NEXT_VIEW_TOPIC   "esp32flight/button/next_view/press"
#define SAVER_STATE_TOPIC "esp32flight/switch/screensaver/state"
#define SAVER_CMD_TOPIC   "esp32flight/switch/screensaver/set"

#define DEVICE_JSON \
    "\"device\":{\"identifiers\":[\"esp32flight\"]," \
    "\"name\":\"esp32flight\",\"manufacturer\":\"theqkash\"," \
    "\"model\":\"ESP32-S3-Touch-LCD-7\"}"

static esp_mqtt_client_handle_t s_client;
static bool s_connected;

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
    for (size_t i = 0; i < sizeof(sensors) / sizeof(sensors[0]); i++) {
        char topic[96], payload[512];
        snprintf(topic, sizeof(topic),
                 "homeassistant/sensor/esp32flight_%s/config", sensors[i].object_id);
        snprintf(payload, sizeof(payload),
                 "{\"name\":\"%s\",\"state_topic\":\"" STATE_TOPIC "\","
                 "\"value_template\":\"%s\","
                 "%s%s%s"
                 "\"unique_id\":\"esp32flight_%s\","
                 DEVICE_JSON "}",
                 sensors[i].name, sensors[i].tpl,
                 sensors[i].unit ? "\"unit_of_measurement\":\"" : "",
                 sensors[i].unit ? sensors[i].unit : "",
                 sensors[i].unit ? "\"," : "",
                 sensors[i].object_id);
        esp_mqtt_client_publish(s_client, topic, payload, 0, 1, 1);
    }

    /* The backlight as a dimmable light: Home Assistant renders an on/off
     * card with a brightness slider, and the device pushes its own state
     * back, so dimming on the touchscreen or the night blackout is
     * reflected there instead of drifting apart. */
    esp_mqtt_client_publish(s_client,
                            "homeassistant/light/esp32flight_backlight/config",
                            "{\"name\":\"Backlight\",\"schema\":\"json\","
                            "\"brightness\":true,\"brightness_scale\":255,"
                            "\"state_topic\":\"" LIGHT_STATE_TOPIC "\","
                            "\"command_topic\":\"" LIGHT_CMD_TOPIC "\","
                            "\"unique_id\":\"esp32flight_backlight\","
                            DEVICE_JSON "}", 0, 1, 1);

    /* Stepping the panel is an action with no state to report, so it is a
     * button rather than a switch - a switch would need an on/off that
     * means nothing here and would stick in whichever position was used
     * last. */
    esp_mqtt_client_publish(s_client,
                            "homeassistant/button/esp32flight_next_view/config",
                            "{\"name\":\"Next panel\","
                            "\"command_topic\":\"" NEXT_VIEW_TOPIC "\","
                            "\"unique_id\":\"esp32flight_next_view\","
                            "\"icon\":\"mdi:page-next-outline\","
                            DEVICE_JSON "}", 0, 1, 1);

    esp_mqtt_client_publish(s_client,
                            "homeassistant/switch/esp32flight_screensaver/config",
                            "{\"name\":\"Screensaver\","
                            "\"command_topic\":\"" SAVER_CMD_TOPIC "\","
                            "\"state_topic\":\"" SAVER_STATE_TOPIC "\","
                            "\"unique_id\":\"esp32flight_screensaver\","
                            "\"icon\":\"mdi:monitor-screenshot\","
                            DEVICE_JSON "}", 0, 1, 1);
}

static bool topic_is(esp_mqtt_event_handle_t ev, const char *topic)
{
    return ev->topic_len == (int)strlen(topic) &&
           strncmp(ev->topic, topic, (size_t)ev->topic_len) == 0;
}

/* {"state":"ON","brightness":128} - brightness is optional, and the plain
 * on/off toggle in HA sends a bare {"state":"OFF"}. */
static void handle_light_cmd(const char *data, int len)
{
    char buf[128];
    if (len <= 0 || len >= (int)sizeof(buf)) {
        return;
    }
    memcpy(buf, data, (size_t)len);
    buf[len] = '\0';

    cJSON *root = cJSON_Parse(buf);
    if (root == NULL) {
        ESP_LOGW(TAG, "light command is not JSON: %s", buf);
        return;
    }
    const cJSON *st = cJSON_GetObjectItem(root, "state");
    const cJSON *br = cJSON_GetObjectItem(root, "brightness");
    bool on = !(cJSON_IsString(st) && strcasecmp(st->valuestring, "OFF") == 0);
    int pct = 0;   /* 0 = keep the brightness we already have */
    if (cJSON_IsNumber(br)) {
        pct = ((int)br->valuedouble * 100 + 127) / 255;
        if (pct < 1) {
            pct = 1;
        }
        if (pct > 100) {
            pct = 100;
        }
    }
    cJSON_Delete(root);

    ESP_LOGI(TAG, "light command: %s %d%%", on ? "on" : "off", pct);
    ui_set_backlight(on, pct);
}

static void mqtt_event(void *arg, esp_event_base_t base, int32_t event_id, void *data)
{
    esp_mqtt_event_handle_t ev = (esp_mqtt_event_handle_t)data;

    if (event_id == MQTT_EVENT_CONNECTED) {
        s_connected = true;
        ESP_LOGI(TAG, "connected");
        publish_discovery();
        esp_mqtt_client_subscribe(s_client, LIGHT_CMD_TOPIC, 1);
        esp_mqtt_client_subscribe(s_client, NEXT_VIEW_TOPIC, 1);
        esp_mqtt_client_subscribe(s_client, SAVER_CMD_TOPIC, 1);
        mqtt_pub_light_state(ui_backlight_on(), settings_get()->brightness);
        mqtt_pub_screensaver_state(ui_screensaver_active());
    } else if (event_id == MQTT_EVENT_DISCONNECTED) {
        s_connected = false;
    } else if (event_id == MQTT_EVENT_DATA && ev != NULL) {
        if (topic_is(ev, LIGHT_CMD_TOPIC)) {
            handle_light_cmd(ev->data, ev->data_len);
        } else if (topic_is(ev, NEXT_VIEW_TOPIC)) {
            /* a button carries no meaningful payload - arriving is the event */
            ESP_LOGI(TAG, "next panel");
            ui_next_view();
        } else if (topic_is(ev, SAVER_CMD_TOPIC)) {
            bool on = ev->data_len >= 2 &&
                      strncasecmp(ev->data, "ON", 2) == 0;
            ESP_LOGI(TAG, "screensaver %s", on ? "on" : "off");
            ui_set_screensaver(on);
        }
    }
}

void mqtt_pub_start(void)
{
    const char *uri = settings_get()->mqtt_uri;
    if (uri[0] == '\0') {
        return;
    }
    esp_mqtt_client_config_t cfg = {
        .broker.address.uri = uri,
        .task.priority = 4,
    };
    s_client = esp_mqtt_client_init(&cfg);
    if (s_client == NULL) {
        ESP_LOGE(TAG, "bad broker uri");
        return;
    }
    esp_mqtt_client_register_event(s_client, ESP_EVENT_ANY_ID, mqtt_event, NULL);
    esp_mqtt_client_start(s_client);
    ESP_LOGI(TAG, "connecting to %s", uri);
}

void mqtt_pub_state(const char *json)
{
    if (s_client == NULL || !s_connected) {
        return;
    }
    esp_mqtt_client_publish(s_client, STATE_TOPIC, json, 0, 0, 0);
}

void mqtt_pub_light_state(bool on, int pct)
{
    if (s_client == NULL || !s_connected) {
        return;
    }
    if (pct < 1) {
        pct = 1;
    }
    if (pct > 100) {
        pct = 100;
    }
    char payload[64];
    snprintf(payload, sizeof(payload), "{\"state\":\"%s\",\"brightness\":%d}",
             on ? "ON" : "OFF", pct * 255 / 100);
    /* Retained, so HA shows the right state after a restart of either end.
     * QoS 0 on purpose: a command echoes its own state back from inside
     * the MQTT event handler, and a queued QoS 1 publish can deadlock
     * there. */
    esp_mqtt_client_publish(s_client, LIGHT_STATE_TOPIC, payload, 0, 0, 1);
}

void mqtt_pub_screensaver_state(bool on)
{
    if (s_client == NULL || !s_connected) {
        return;
    }
    /* QoS 0 for the same reason as the light state: this can run from
     * inside the MQTT event handler when a command echoes back. */
    esp_mqtt_client_publish(s_client, SAVER_STATE_TOPIC,
                            on ? "ON" : "OFF", 0, 0, 1);
}
