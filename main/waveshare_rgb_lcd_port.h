/*
 * Display bring-up for Waveshare ESP32-S3-Touch-LCD-7 (800x480 RGB, GT911 touch).
 * Adapted from Waveshare's 08_lvgl_Porting demo (CC0-1.0).
 */
#pragma once

#include "sdkconfig.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_lcd_panel_ops.h"
#if !CONFIG_IDF_TARGET_ESP32P4
#include "esp_lcd_panel_rgb.h"   /* RGB peripheral exists only on the S3 boards */
#endif
#include "esp_lcd_touch_gt911.h"
#include "lvgl_port.h"

#define I2C_MASTER_SCL_IO           9
#define I2C_MASTER_SDA_IO           8
#define I2C_MASTER_NUM              0
#define I2C_MASTER_FREQ_HZ          400000
#define I2C_MASTER_TIMEOUT_MS       1000

/* GT911 INT pin, driven low during reset to select I2C addr 0x5D */
#define GPIO_TOUCH_INT              4

#define EXAMPLE_LCD_H_RES               (LVGL_PORT_H_RES)
#define EXAMPLE_LCD_V_RES               (LVGL_PORT_V_RES)
#if CONFIG_CANFLIGHT_BOARD_WAVESHARE_7B
/* 30.85M is at the edge of some panels' tolerance: units shifted the whole
 * image on PSRAM traffic spikes (touch, refresh) until the clock came down.
 * 28M still left this unit shifting the frame and flickering horizontal
 * lines after hours of uptime: the frame is 1386 x 661 pixel times, so this
 * is 27.3 Hz against 30.6 Hz at 28M, for ~11 % less bandwidth demand on the
 * PSRAM the app composes tiles in. */
#define EXAMPLE_LCD_PIXEL_CLOCK_HZ      (25000000)
#else
#define EXAMPLE_LCD_PIXEL_CLOCK_HZ      (16 * 1000 * 1000)
#endif
#define EXAMPLE_RGB_BIT_PER_PIXEL       (16)
#define EXAMPLE_RGB_DATA_WIDTH          (16)
#define EXAMPLE_RGB_BOUNCE_BUFFER_SIZE  (EXAMPLE_LCD_H_RES * CONFIG_EXAMPLE_LCD_RGB_BOUNCE_BUFFER_HEIGHT)

#define EXAMPLE_LCD_IO_RGB_DISP         (-1)
#define EXAMPLE_LCD_IO_RGB_VSYNC        (GPIO_NUM_3)
#define EXAMPLE_LCD_IO_RGB_HSYNC        (GPIO_NUM_46)
#define EXAMPLE_LCD_IO_RGB_DE           (GPIO_NUM_5)
#define EXAMPLE_LCD_IO_RGB_PCLK         (GPIO_NUM_7)
#define EXAMPLE_LCD_IO_RGB_DATA0        (GPIO_NUM_14)
#define EXAMPLE_LCD_IO_RGB_DATA1        (GPIO_NUM_38)
#define EXAMPLE_LCD_IO_RGB_DATA2        (GPIO_NUM_18)
#define EXAMPLE_LCD_IO_RGB_DATA3        (GPIO_NUM_17)
#define EXAMPLE_LCD_IO_RGB_DATA4        (GPIO_NUM_10)
#define EXAMPLE_LCD_IO_RGB_DATA5        (GPIO_NUM_39)
#define EXAMPLE_LCD_IO_RGB_DATA6        (GPIO_NUM_0)
#define EXAMPLE_LCD_IO_RGB_DATA7        (GPIO_NUM_45)
#define EXAMPLE_LCD_IO_RGB_DATA8        (GPIO_NUM_48)
#define EXAMPLE_LCD_IO_RGB_DATA9        (GPIO_NUM_47)
#define EXAMPLE_LCD_IO_RGB_DATA10       (GPIO_NUM_21)
#define EXAMPLE_LCD_IO_RGB_DATA11       (GPIO_NUM_1)
#define EXAMPLE_LCD_IO_RGB_DATA12       (GPIO_NUM_2)
#define EXAMPLE_LCD_IO_RGB_DATA13       (GPIO_NUM_42)
#define EXAMPLE_LCD_IO_RGB_DATA14       (GPIO_NUM_41)
#define EXAMPLE_LCD_IO_RGB_DATA15       (GPIO_NUM_40)

esp_err_t waveshare_esp32_s3_rgb_lcd_init(void);

/* Name of the board picked at boot (autodetect or forced). */
const char *waveshare_lcd_board_name(void);
esp_err_t waveshare_rgb_lcd_bl_on(void);
esp_err_t waveshare_rgb_lcd_bl_off(void);

/* Brightness: 5..100 percent. bl_dimmable() says whether this board's
 * backlight hardware can dim at all (CH422G boards are on/off only);
 * bl_pct() is a no-op returning ESP_ERR_NOT_SUPPORTED there. Every port
 * (S3 RGB family, Tab5, JC10) implements both. */
bool waveshare_rgb_lcd_bl_dimmable(void);
esp_err_t waveshare_rgb_lcd_bl_pct(int pct);

/* First RGB frame buffer (800x480 RGB565 in PSRAM), NULL before init. */
void *waveshare_lcd_get_fb(void);

/* Framebuffer dimensions (fixed 800x480 on the panel; the app build may
 * render a larger canvas). */
void waveshare_lcd_get_res(int *w, int *h);

#if CONFIG_IDF_TARGET_ESP32P4
/* Tab5 (MIPI-DSI): block until the last draw_bitmap DMA transfer landed. */
void tab5_lcd_wait_trans_done(void);
#endif
