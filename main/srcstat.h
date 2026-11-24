#pragma once

#include <stdbool.h>
#include <stddef.h>

/* Which data sources are answering. Each source reports successes (and,
 * where it knows, failures); a source counts as down once it has not
 * succeeded for longer than its own window. The UI turns that into a
 * visible warning instead of a map that just quietly stops changing. */

typedef enum {
    SRC_ADSB,        /* aircraft positions API */
    SRC_BROKER,      /* MQTT broker (Home Assistant) */
    SRC_COLLECTOR,   /* krk-airport-collector, via the broker */
    SRC_WEATHER,     /* Open-Meteo */
    SRC_RAIN,        /* RainViewer frame list */
    SRC_COUNT
} src_id_t;

void srcstat_ok(src_id_t s);
void srcstat_fail(src_id_t s);
/* Broker: up while connected, regardless of message traffic. */
void srcstat_link(src_id_t s, bool up);

/* Short English text naming every source that is down and for how long,
 * e.g. "COLLECTOR OFFLINE 12 MIN | ADS-B API DOWN 3 MIN". Returns the number
 * of sources down; buf gets "" when all are fine. */
int srcstat_problems(char *buf, size_t n);

/* Minutes since the source last succeeded, -1 if never. */
int srcstat_age_min(src_id_t s);
