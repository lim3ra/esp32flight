#pragma once

#include <stdbool.h>
#include <stdint.h>

/* Precipitation grid from krk-airport-collector (topic krk-airport/rain):
 * the newest RainViewer frame around home as palette indexes, so neither
 * the map overlay nor the radar scope fetches or decodes PNG tiles here.
 * When no fresh grid has arrived the callers fall back to RainViewer. */

/* A complete message, from the MQTT task. */
void raingrid_feed(const uint8_t *data, int len);

/* True while the last grid is younger than 30 minutes. */
bool raingrid_fresh(void);

/* Bumped on every new grid: goes into the map render key. */
int raingrid_generation(void);

/* Frame time (unix) of the grid, 0 = none. */
long long raingrid_frame_time(void);

/* Palette index at a point, 0 = dry or outside the grid. */
uint8_t raingrid_sample(double lat, double lon);

/* Colour (RainViewer's own, with alpha) and radar intensity of an index. */
void raingrid_palette(uint8_t idx, uint8_t *r, uint8_t *g, uint8_t *b, uint8_t *a,
                      uint8_t *intensity);
