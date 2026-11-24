#pragma once

#include <stdbool.h>

/* Optional airspace outlines (CTR/TMA/danger/restricted) from openAIP.
 * Needs the user's free openAIP API key. Fetched once per location. */

#define MAX_AIRSPACES     28
#define MAX_ASP_POINTS    48

typedef struct {
    char  name[40];
    int   type;         /* openAIP numeric type, see airspace_type_str */
    int   n_points;
    float lat[MAX_ASP_POINTS];
    float lon[MAX_ASP_POINTS];
} airspace_t;

/* Restores the last collector feed from flash once; the outlines themselves
 * come from airspace_feed (krk-airport-collector over MQTT). */
void airspace_poll(double home_lat, double home_lon);

/* A complete krk-airport/airspaces message, from the MQTT task. */
void airspace_feed(const char *data, int len);

/* Collector timestamp (unix) of the outlines shown, 0 = none. */
long long airspace_feed_time(void);

int  airspace_count(void);
const airspace_t *airspace_get(int idx);
const char *airspace_type_str(int type);
