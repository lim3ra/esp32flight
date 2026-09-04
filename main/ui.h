#pragma once

#include "flight_model.h"

/* All ui_* functions must be called while holding the LVGL lock
 * (lvgl_port_lock/unlock), except from LVGL's own callbacks. */

void ui_init(void);

/* The 7B edition has a single panel; kept so the desktop screenshot path
 * and the web /view endpoint still have something to call. */
void ui_set_view(int mode);
void ui_apply_brightness(void); /* re-apply settings brightness after bl_on */
void ui_toast(const char *text);          /* 2s bottom-center notice, any task */
bool ui_input_action(const char *action); /* #13 actions; call under LVGL lock */

/* One-line status in the header (Wi-Fi / location / errors). */
void ui_set_status(const char *text);

/* Weather summary shown in place of the app title. */
void ui_set_weather(const char *text);

/* Emergency styling for the status line (red, larger). */
void ui_set_status_alert(bool alert);

/* Highlight the gear icon when a newer release is on GitHub. */
void ui_set_update_available(bool available, const char *tag);
bool ui_update_available(void);
const char *ui_update_tag(void);

/* Home coordinates (after geolocation) - used by the radar map view. */
void ui_set_home(double lat, double lon);

/* Replace displayed flight data. Routes are looked up via routes_get_cached
 * and snapshotted, so later touch interactions don't race the network task. */
void ui_update(const aircraft_list_t *list);
