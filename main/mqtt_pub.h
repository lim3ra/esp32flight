#pragma once

#include <stdbool.h>

/* MQTT publishing with Home Assistant discovery. No-op when no broker URI
 * is configured. */
void mqtt_pub_start(void);
void mqtt_pub_state(const char *json);

/* Push the current backlight state to Home Assistant. pct is 1-100; the
 * topic carries HA's 0-255 brightness scale. Called from the UI whenever
 * the panel light changes, so a touch on the screen or the night blackout
 * shows up in HA too. */
void mqtt_pub_light_state(bool on, int pct);

/* Same idea for the screensaver switch: the screensaver also appears on its
 * own after the idle timeout and goes away on a tap. */
void mqtt_pub_screensaver_state(bool on);
