#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

typedef struct {
    char   wifi_ssid[33];
    char   wifi_pass[65];
    bool   use_fixed_loc;
    double lat;
    double lon;
    int    radius_nm;
    bool   hide_ground;
    bool   hide_private;    /* hide non-airline traffic (callsign not AAA123-style) */
    int    theme;           /* index into theme.c palettes */
    int    lang;            /* 0 = English, 1 = Polski */
    bool   ota_enabled;     /* volatile: always false after boot, armed from
                               the on-device settings screen only */
    /* integrations, edited via the web panel (/api/config) */
    char   mqtt_uri[96];    /* e.g. mqtt://user:pass@192.168.1.5:1883 */
    char   fa_key[48];      /* FlightAware AeroAPI key (IATA flight numbers) */
    char   watch_regs[96];  /* comma-separated watchlist (regs/callsign prefixes) */
    char   local_adsb[96];  /* dump1090/readsb aircraft.json URL (LAN receiver) */
    bool   local_adsb_use;  /* off = internet sources despite a saved URL (#17) */
    bool   temp_f;          /* header weather in Fahrenheit (#25) */
    char   web_pass[33];    /* HTTP Basic Auth for the web panel, empty = open */
    /* One night window, two brightness levels. Replaces the old blackout:
     * the panel dims instead of switching off, so the map stays readable. */
    int    night_start_min; /* minutes from midnight, local */
    int    night_end_min;
    uint8_t bright_day;     /* 5..100, outside the window */
    uint8_t bright_night;   /* 5..100, inside it */
    char   filter_airport[8]; /* airport filter (ICAO/IATA), empty=off */
    bool   filter_apt_exclude; /* false: show only that airport, true: hide it */
    int    alt_min_ft;        /* altitude band filter, 0 = no bound */
    int    alt_max_ft;
    /* keep new fields at the end: the app build persists this struct as a
     * versioned blob and reads old files as a prefix */
    uint8_t show_classes;     /* bitmask of flight_class_t, FCLS_ALL_MASK = all */
    bool    rain_overlay;     /* RainViewer precipitation layer on maps/radar */
    bool    show_route;       /* route codes on list rows + cities in the map bubble (#28) */
    bool    brightness_ctl;  /* master enable for backlight dimming, off = stock behavior */
    uint8_t brightness;       /* 5..100, applied where the backlight can dim (#13/HA) */
    bool    clock_12h;        /* AM/PM clock everywhere a time is shown */
    char    input_map[176];   /* physical input mapping, see input_ctl.c (#13) */
    bool    airspace_enabled; /* openAIP airspace outlines (needs openaip_key) */
    char    openaip_key[64];  /* openAIP client id */
    char    carto_key[48];    /* CARTO basemaps key; empty = OSM tiles + on-device dark style */
    char    tile_url[112];    /* custom tile URL template with {z}/{x}/{y}; overrides both */
    bool    metric_units;     /* m + km/h instead of ft + kt */
    bool    metar_decoded;    /* human-readable METAR instead of raw */
    bool    follow_mode;      /* stick to the selected aircraft, no auto-cycle */
    char    fav_name[3][24];  /* favorite locations, empty name = free slot */
    double  fav_lat[3], fav_lon[3];
    bool    map_light;        /* lift dark map tiles for readability (#10) */
} settings_t;

/* Load from NVS (menuconfig values as first-boot defaults). Call once at
 * startup, after nvs_flash_init. */
void settings_load(void);

/* Persist current contents of settings_get() to NVS. */
esp_err_t settings_save(void);

settings_t *settings_get(void);
