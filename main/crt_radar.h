#pragma once

/* Full-screen PPI scope in the style of a long-persistence radar CRT: a
 * rotating beam with a P7-like afterglow, echoes painted only when the beam
 * passes them, precipitation from RainViewer, a video map of range rings,
 * runways and airspaces, and ATC-style data tags.
 *
 * Drawn straight into LVGL's frame buffer from a custom draw callback, and
 * only the sector under the beam plus a few small decay rectangles are
 * invalidated per frame. On the 7B every redraw becomes a frame-buffer copy
 * (AVOID_TEAR mode 3), so a full-screen animation would load PSRAM enough to
 * bring the panel jitter back.
 *
 * All functions must be called with the LVGL lock held. */

#include <stdbool.h>
#include "lvgl.h"
#include "flight_model.h"

typedef void (*crt_radar_exit_cb_t)(void);

/* Remember the parent screen and the callback of the on-screen MAP key.
 * Allocates nothing: buffers and objects are created on the first show. */
void crt_radar_init(lv_obj_t *scr, crt_radar_exit_cb_t on_exit);

/* Show or hide the scope. Returns false when it cannot be shown (PSRAM
 * allocation failed), so the caller can stay on the map. */
bool crt_radar_show(bool on);
bool crt_radar_active(void);

void crt_radar_set_home(double lat, double lon);
void crt_radar_set_targets(const aircraft_list_t *list);
void crt_radar_set_weather(const char *text);
/* KRK airport status from the airport bar (level 0 ok .. 3 unknown). */
void crt_radar_set_airport(int level, const char *chip, const char *wx,
                           const char *arr, const char *dep);
