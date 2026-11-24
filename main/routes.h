#pragma once

#include "flight_model.h"

/* Cached route lookup. Returns entry (valid or negative-cached) or NULL if
 * this callsign has not been fetched yet. */
const route_info_t *routes_get_cached(const char *callsign);

/* Fetch route for callsign and store in cache (also stores negative
 * results). Sources: adsbdb.com, then
 * adsb.lol routeset, then hexdb.io. When the aircraft position
 * is known, each source's answer is sanity-checked against it - a route the
 * aircraft can't plausibly be on triggers the next source instead of being
 * shown. Call from a network task, never from the UI. */
/* Re-check a cached route against the aircraft's current geometry: a
 * wrong-but-plausible entry gets refuted (and refetched) the moment the
 * flight's track starts contradicting it. Cheap local math, rate-limited
 * per callsign. */
void routes_revalidate(const char *callsign,
                       double ac_lat, double ac_lon, bool has_pos,
                       float track_deg, float gs_kts, int vrate_fpm);

const route_info_t *routes_fetch(const char *callsign,
                                 double ac_lat, double ac_lon, bool has_pos,
                                 float track_deg, float gs_kts, int vrate_fpm);

/* Inject an authoritative route (e.g. from FlightAware live data) for a
 * callsign: airports resolved from the bundled database. Overwrites a
 * negative cache entry; keeps an existing valid one. */
void routes_inject(const char *callsign, const char *orig_icao, const char *dest_icao);

/* The collector looked this callsign up and found no route: keep (or make)
 * a negative entry so the panel does not query the route DBs itself. */
void routes_mark_unknown(const char *callsign);
