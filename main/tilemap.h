#pragma once

#include <stdbool.h>
#include <stdint.h>

/* Web-mercator tile view composed from CARTO dark tiles (online).
 * Blocking (downloads tiles); call from a worker task, never from the UI. */

typedef struct {
    int    z;           /* tile zoom */
    double px0, py0;    /* view origin in global pixels at zoom z */
    int    w, h;
    int    missing;     /* base tiles that failed to fetch (0 = complete);
                           callers keep the partial view but should retry */
} tile_view_t;

/* Call once at startup (creates the render serialization mutex and the
 * shared tile worker task). */
void tilemap_init(void);

/* Drop every cached base tile, in PSRAM and on flash. Call after the CARTO
 * API key changes: keyless requests answer 200 with "API KEY REQUIRED"
 * stamped into the PNG, so the caches happily hold unusable images that no
 * status check would ever have rejected. Safe to call from any task - the
 * purge itself runs at the start of the next render, under the render
 * mutex. */
void tilemap_flush_cache(void);

/* One persistent worker replaces the per-view spawn-and-die render tasks:
 * five task types used to allocate 10 KB internal stacks at arbitrary
 * times and fragment the internal heap (observed: spawn failures under
 * pressure). Jobs run serialized, which they effectively already were
 * through the render mutex. Returns false when the queue is full - the
 * caller resets its busy flag exactly like a failed task spawn. */
typedef void (*tile_job_fn)(void);
bool tilemap_worker_submit(tile_job_fn job);

/* Compose a view around the bbox into dst (RGB565, dst_w x dst_h).
 * Returns false when tiles could not be fetched (offline etc.). */
bool tilemap_render(uint16_t *dst, int dst_w, int dst_h,
                    double lat_min, double lat_max,
                    double lon_min, double lon_max,
                    tile_view_t *out_view);

/* Precipitation only, over black: the retro radar's weather echo. */
bool tilemap_render_rain(uint16_t *dst, int dst_w, int dst_h,
                         double lat_min, double lat_max,
                         double lon_min, double lon_max,
                         tile_view_t *out_view);

/* Project WGS84 to view pixel coordinates. */
void tilemap_project(const tile_view_t *v, double lat, double lon, int *x, int *y);
