#pragma once

#include "flight_model.h"

/* All ui_* functions must be called while holding the LVGL lock
 * (lvgl_port_lock/unlock), except from LVGL's own callbacks. */

void ui_init(void);

/* Force a right-panel view (0 detail, 1 map, 2 radar, 3 stats, 4 retro);
 * used by the desktop build for screenshot testing. */
void ui_set_view(int mode);
void ui_prewarm_ambient(void);   /* boot-time screensaver map prefetch */
void ui_apply_brightness(void); /* re-apply settings brightness after bl_on */
void ui_toast(const char *text);          /* 2s bottom-center notice, any task */
bool ui_input_action(const char *action); /* #13 actions; call under LVGL lock */
void ui_set_list_mode(int mode);   /* 0 planes, 1 ships, 2 all */

/* Step the right-hand panel to the next view, exactly like the on-screen
 * mode button. Safe to call from any task - it takes the LVGL lock itself
 * and gives up rather than block if the UI is busy. */
void ui_next_view(void);

/* Show or dismiss the map screensaver on demand, same conditions. */
void ui_set_screensaver(bool on);
bool ui_screensaver_active(void);

/* One-line status in the header (Wi-Fi / location / errors). */
void ui_set_status(const char *text);

/* Weather summary shown in place of the app title. */
void ui_set_weather(const char *text);

/* Emergency styling for the status line (red, larger). */
void ui_set_status_alert(bool alert);

/* Amber on-screen banner for an incoming flyover (auto-hides, tap to close).
 * Safe to call from any task. */
void ui_flyover_banner(const char *callsign, int eta_min, double cpa_km);

/* Highlight the gear icon when a newer release is on GitHub. */
void ui_set_update_available(bool available, const char *tag);
bool ui_update_available(void);
const char *ui_update_tag(void);

/* Home coordinates (after geolocation) - used by the radar map view. */
void ui_set_home(double lat, double lon);

/* Replace displayed flight data. Routes are looked up via routes_get_cached
 * and snapshotted, so later touch interactions don't race the network task. */
void ui_update(const aircraft_list_t *list);
