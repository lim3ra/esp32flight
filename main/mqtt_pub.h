#pragma once

#include <stdbool.h>

/* MQTT publishing with Home Assistant discovery. No-op when no broker URI
 * is configured. */
void mqtt_pub_start(void);
void mqtt_pub_state(const char *json);

/* Push the current backlight state to HA after a local change. */
void mqtt_pub_backlight_changed(void);
