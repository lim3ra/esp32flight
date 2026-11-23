#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "tilemap.h"

/* Draw runway strips of scheduled-service airports into an RGB565 map
 * buffer rendered with the given view. Cheap enough to call after every
 * tilemap_render; does nothing when the runway database is absent. */
void runways_draw(uint16_t *fb, int w, int h, const tile_view_t *view);

/* Raw runway database for other renderers (the CRT scope's video map):
 * count, and the two ends of one strip. Loads the database on first use. */
int  runways_count(void);
void runways_get(int idx, float *le_lat, float *le_lon, float *he_lat, float *he_lon);

/* True when the aircraft looks like it is on approach: low, descending and
 * aligned with a runway it is flying toward. Returns the airport ICAO and
 * a rough ETA in minutes. */
bool runways_approach(double lat, double lon, float track_deg, int alt_ft,
                      int vs_fpm, float gs_kts, char *icao_out, size_t icao_len,
                      int *eta_min);
