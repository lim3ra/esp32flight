#pragma once

#include <stdbool.h>

/* Great-circle distance in km between two WGS84 points. */
double geo_haversine_km(double lat1, double lon1, double lat2, double lon2);

/* Initial great-circle bearing in degrees (0..360) from point 1 to point 2. */
double geo_bearing_deg(double lat1, double lon1, double lat2, double lon2);

/* Elevation angle (deg above the horizon) of an aircraft seen from home. */
double geo_elevation_deg(double dist_km, int alt_ft);

/* Closest point of approach: when and how close the aircraft will pass by
 * home, assuming constant track/speed. Returns false when it is moving
 * away. t in seconds, distance in km. */
bool geo_cpa(double home_lat, double home_lon,
             double ac_lat, double ac_lon, double track_deg, double gs_kts,
             double *t_s, double *cpa_km);

/* Point at fraction f (0..1) along the great circle from point 1 to point 2. */
void geo_gc_point(double lat1, double lon1, double lat2, double lon2,
                  double f, double *lat, double *lon);

/* Sanity check for callsign-based route lookups: is the aircraft's position
 * anywhere near the origin->destination great circle? Catches stale database
 * entries (e.g. a callsign reused on a different continent). */
/* Corridor + direction-of-flight test; track_deg < 0 or low ground speed
 * skips the direction part. */
/* Tri-state route-vs-position verdict: 0 = refuted, 1 = merely plausible
 * (the discriminating checks stayed quiet: near an airport, low speed, no
 * track), 2 = confirmed (en-route track evidence actually passed). */
#define GEO_FIT_NO        0
#define GEO_FIT_WEAK      1
#define GEO_FIT_CONFIRMED 2
int geo_route_fit_dir(double orig_lat, double orig_lon,
                      double dest_lat, double dest_lon,
                      double cur_lat, double cur_lon,
                      float track_deg, float gs_kts, int vrate_fpm);

bool geo_route_plausible_dir(double orig_lat, double orig_lon,
                             double dest_lat, double dest_lon,
                             double cur_lat, double cur_lon,
                             float track_deg, float gs_kts, int vrate_fpm);

bool geo_route_plausible(double orig_lat, double orig_lon,
                         double dest_lat, double dest_lon,
                         double cur_lat, double cur_lon);

/* Flight progress 0.0..1.0 based on distance already flown vs. remaining.
 * Computed as flown/(flown+remaining) so off-route detours still behave. */
double geo_progress(double orig_lat, double orig_lon,
                    double dest_lat, double dest_lon,
                    double cur_lat, double cur_lon);

/* lon shifted by a multiple of 360 into the hemisphere-frame nearest ref;
 * lets Pacific-crossing routes use a continuous longitude axis. */
double geo_lon_unwrap(double ref, double lon);

/* Civil sunrise/sunset (NOAA approximation) as minutes from local midnight
 * for the given day. Returns false in polar day/night conditions. */
bool geo_sun_times(double lat, double lon, long long epoch_utc, int tz_off_s,
                   int *rise_min, int *set_min);

/* Sun elevation above the horizon in degrees (same approximation, no
 * refraction), negative below it. */
double geo_sun_elevation(double lat, double lon, long long epoch_utc);
