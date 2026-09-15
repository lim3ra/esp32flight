#pragma once

#include <stdbool.h>

/* MQTT publishing with Home Assistant discovery. No-op when no broker URI
 * is configured. */
void mqtt_pub_start(void);
void mqtt_pub_state(const char *json);

/* Push the current backlight state to HA after a local change. */
void mqtt_pub_backlight_changed(void);

/* The firmware switched the panel on or off by itself (night/day schedule,
 * a wake action). Without this the HA light keeps reporting whatever was
 * last commanded over MQTT, which is how it ended up showing OFF over a
 * screen the morning step had already relit. */
void mqtt_pub_backlight_on(bool on);
