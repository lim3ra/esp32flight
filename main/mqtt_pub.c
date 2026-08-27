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
        mqtt_pub_light_state(ui_backlight_on(), settings_get()->brightness);
    } else if (event_id == MQTT_EVENT_DISCONNECTED) {
        s_connected = false;
    } else if (event_id == MQTT_EVENT_DATA && ev != NULL &&
               ev->topic_len == (int)strlen(LIGHT_CMD_TOPIC) &&
               strncmp(ev->topic, LIGHT_CMD_TOPIC, (size_t)ev->topic_len) == 0) {
        handle_light_cmd(ev->data, ev->data_len);
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
