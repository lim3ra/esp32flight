#pragma once

#include "esp_err.h"
#include "flight_model.h"

/* Fetch aircraft within radius_nm of lat/lon. Tries airplanes.live first,
 * falls back to adsb.lol (same JSON schema). Result sorted by distance. */
esp_err_t flight_fetch_nearby(double lat, double lon, int radius_nm, aircraft_list_t *out);

/* krk-airport/aircraft message from the collector (any task). While it is
 * younger than 20 s, flight_fetch_nearby parses it instead of going online. */
void flight_feed(const char *data, int len);

/* Airline-shaped traffic heuristic (AAA123 callsign + non-light category). */
bool flight_is_airline(const aircraft_t *ac);

/* Coarse class for the user filter / list markers. */
flight_class_t flight_class(const aircraft_t *ac);

/* Map-marker sprite: the class, refined by the ADS-B emitter category so
 * balloons and UAVs do not masquerade as gliders. */
typedef enum {
    FSPR_PLANE = 0, FSPR_SMALL, FSPR_HELI, FSPR_MIL,
    FSPR_GLIDER, FSPR_BALLOON, FSPR_DRONE
} flight_sprite_t;
flight_sprite_t flight_sprite(const aircraft_t *ac);
