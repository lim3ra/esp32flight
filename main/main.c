#include <stdlib.h>
#include <time.h>
#include "esp_log.h"
#include "cJSON.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "waveshare_rgb_lcd_port.h"

#include "airports.h"
#include "flight_task.h"
#include "logos.h"
#include "settings.h"
#include "tilemap.h"
#include "ui.h"
#include "ui_settings.h"
#include "ui_map.h"
#include "web_server.h"
#include "wifi_mgr.h"

static const char *TAG = "canflight";

static void *psram_prefer_malloc(size_t n)
{
    return heap_caps_malloc_prefer(n, 2,
                                   MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT,
                                   MALLOC_CAP_8BIT);
}

void app_main(void)
{
    /* first thing in the log after any spontaneous restart: why it happened */
    ESP_LOGW("boot", "reset reason: %d (4=panic 5=int_wdt 6=task_wdt 7=other_wdt 9=brownout)",
             (int)esp_reset_reason());
    /* Local time for the clock and ETAs (Europe/Warsaw with DST) */
    setenv("TZ", "CET-1CEST,M3.5.0,M10.5.0/3", 1);
    tzset();

    /* cJSON trees (state JSON, API parses) are large and transient:
     * keep them out of the scarce internal heap. */
    cJSON_Hooks jh = { .malloc_fn = psram_prefer_malloc, .free_fn = free };
    cJSON_InitHooks(&jh);

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    settings_load();

    ESP_ERROR_CHECK(waveshare_esp32_s3_rgb_lcd_init());
    /* settings_load ran above, so the panel comes up at the saved
     * brightness instead of flashing full bright first. */
    waveshare_rgb_lcd_bl_set(settings_get()->brightness);

    logos_init();
    airports_init();
    tilemap_init();
    /* pre-decode both world maps so map opens never decode PNGs at draw time */
    ui_map_get_image();
    ui_map_get_image_small();

    if (lvgl_port_lock(-1)) {
        ui_init();
        if (settings_get()->wifi_ssid[0] == '\0') {
            ui_settings_open();   /* first boot: land straight in setup */
        }
        lvgl_port_unlock();
    }

    err = wifi_mgr_start();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Wi-Fi init failed: %s", esp_err_to_name(err));
    }

    web_server_start();
    flight_task_start();
}
