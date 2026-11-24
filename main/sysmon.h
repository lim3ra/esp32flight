#pragma once

#include <stdbool.h>

/* Board health, sampled every 10 s from an esp_timer callback (no task of
 * its own): chip temperature from the ESP32-S3's internal sensor, per-core
 * load from the FreeRTOS idle-task run time, Wi-Fi RSSI. Published to Home
 * Assistant once a minute, exposed in /metrics and on the radar console.
 *
 * Supply voltage is not measurable here: nothing on the 7B routes VBUS or
 * 3V3 to an ADC pin. A sagging supply shows up as reset reason 9 (brownout)
 * in the event log instead. */

typedef struct {
    bool  valid;
    float chip_temp_c;
    float chip_temp_max_c;     /* since boot */
    int   cpu_load_pct[2];     /* last 10 s window */
    int   cpu_load_max_pct[2]; /* since boot */
    int   wifi_rssi_dbm;       /* 0 when not associated */
} sysmon_t;

void sysmon_start(void);
void sysmon_get(sysmon_t *out);
