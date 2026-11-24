#pragma once

/* Commercial (IATA) flight numbers via FlightAware AeroAPI. Only active
 * when an API key is configured. Cached per callsign; empty string means
 * a negative result. */
const char *faflight_get_cached(const char *callsign);   /* NULL = not fetched */
const char *faflight_fetch(const char *callsign);        /* blocking */
/* Flight number from krk-airport-collector ("" = looked up, none). */
void faflight_put(const char *callsign, const char *iata);
