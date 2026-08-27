/*
 * Display bring-up for M5Stack Tab5 (ESP32-P4, 720x1280 MIPI-DSI, touch).
 *
 * Implements the same API surface as waveshare_rgb_lcd_port.c so the shared
 * code does not care which board it runs on. Two panel generations exist and
 * are probed at boot exactly like the vendor firmware does it:
 *   - ILI9881C panel + GT911 touch (units sold before ~2025-10)
 *   - ST7123 integrated panel+touch (newer units)
 * Hardware facts (pins, timings, expander bits, vendor init tables) come
 * from m5stack/M5Tab5-UserDemo (MIT) and espressif/esp-bsp (Apache-2.0).
 */
#include "waveshare_rgb_lcd_port.h"

#include <string.h>
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/ledc.h"
#include "esp_ldo_regulator.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_lcd_ili9881c.h"
#include "esp_lcd_touch_gt911.h"
#include "esp_lcd_touch_st7123.h"
#include "freertos/semphr.h"
#include "tab5/esp_lcd_st7123.h"
#include "tab5/esp_lcd_st7121.h"

static const char *TAG = "tab5";

/* SYS I2C bus: touch, the two PI4IOE expanders, codecs, IMU, RTC */
#define TAB5_I2C_SDA        31
#define TAB5_I2C_SCL        32
#define TAB5_I2C_FREQ_HZ    100000

#define TAB5_TOUCH_INT      23
#define TAB5_LCD_BACKLIGHT  22
#define TAB5_LEDC_CH        LEDC_CHANNEL_1

/* MIPI DSI PHY is powered from the on-chip LDO (VO3 = VDD_MIPI_DPHY) */
#define TAB5_DSI_LDO_CHAN   3
#define TAB5_DSI_LDO_MV     2500

#define TAB5_LCD_H_RES      720     /* panel is native portrait */
#define TAB5_LCD_V_RES      1280

/* PI4IOE5V6408 expanders: #1 has LCD/TP reset lines, #2 the WLAN power */
#define PI4IOE1_ADDR        0x43
#define PI4IOE2_ADDR        0x44
#define PI4IO_REG_CHIP_RESET 0x01
#define PI4IO_REG_IO_DIR     0x03
#define PI4IO_REG_OUT_SET    0x05
#define PI4IO_REG_OUT_H_IM   0x07
#define PI4IO_REG_IN_DEF_STA 0x09
#define PI4IO_REG_PULL_EN    0x0B
#define PI4IO_REG_PULL_SEL   0x0D
#define PI4IO_REG_INT_MASK   0x11

static i2c_master_bus_handle_t s_i2c;
static i2c_master_dev_handle_t s_exp1, s_exp2;
static esp_lcd_panel_handle_t s_panel;
static SemaphoreHandle_t s_trans_done;
static bool s_st7123;
static bool s_st7121;   /* older Sitronix generation, same I2C addr 0x55 */
static const char *s_board_name = "M5Stack Tab5";

#include "tab5/tab5_ili9881_init.inc"

/* ST7123 panel vendor init (from M5Tab5-UserDemo) */
static const st7123_lcd_init_cmd_t k_st7123_init[] = {
    {0x60, (uint8_t[]){0x71, 0x23, 0xa2}, 3, 0},
    {0x60, (uint8_t[]){0x71, 0x23, 0xa3}, 3, 0},
    {0x60, (uint8_t[]){0x71, 0x23, 0xa4}, 3, 0},
    {0xA4, (uint8_t[]){0x31}, 1, 0},
    {0xD7, (uint8_t[]){0x10, 0x0A, 0x10, 0x2A, 0x80, 0x80}, 6, 0},
    {0x90, (uint8_t[]){0x71, 0x23, 0x5A, 0x20, 0x24, 0x09, 0x09}, 7, 0},
    {0xA3, (uint8_t[]){0x80, 0x01, 0x88, 0x30, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x46, 0x00, 0x00,
                       0x1E, 0x5C, 0x1E, 0x80, 0x00, 0x4F, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x46,
                       0x00, 0x00, 0x1E, 0x5C, 0x1E, 0x80, 0x00, 0x6F, 0x58, 0x00, 0x00, 0x00, 0xFF},
     40, 0},
    {0xA6, (uint8_t[]){0x03, 0x00, 0x24, 0x55, 0x36, 0x00, 0x39, 0x00, 0x6E, 0x6E, 0x91, 0xFF, 0x00, 0x24,
                       0x55, 0x38, 0x00, 0x37, 0x00, 0x6E, 0x6E, 0x91, 0xFF, 0x00, 0x24, 0x11, 0x00, 0x00,
                       0x00, 0x00, 0x6E, 0x6E, 0x91, 0xFF, 0x00, 0xEC, 0x11, 0x00, 0x03, 0x00, 0x03, 0x6E,
                       0x6E, 0xFF, 0xFF, 0x00, 0x08, 0x80, 0x08, 0x80, 0x06, 0x00, 0x00, 0x00, 0x00},
     55, 0},
    {0xA7, (uint8_t[]){0x19, 0x19, 0x80, 0x64, 0x40, 0x07, 0x16, 0x40, 0x00, 0x44, 0x03, 0x6E, 0x6E, 0x91, 0xFF,
                       0x08, 0x80, 0x64, 0x40, 0x25, 0x34, 0x40, 0x00, 0x02, 0x01, 0x6E, 0x6E, 0x91, 0xFF, 0x08,
                       0x80, 0x64, 0x40, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x6E, 0x6E, 0x91, 0xFF, 0x08, 0x80,
                       0x64, 0x40, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00, 0x6E, 0x6E, 0x84, 0xFF, 0x08, 0x80, 0x44},
     60, 0},
    {0xAC, (uint8_t[]){0x03, 0x19, 0x19, 0x18, 0x18, 0x06, 0x13, 0x13, 0x11, 0x11, 0x08, 0x08, 0x0A, 0x0A, 0x1C,
                       0x1C, 0x07, 0x07, 0x00, 0x00, 0x02, 0x02, 0x01, 0x19, 0x19, 0x18, 0x18, 0x06, 0x12, 0x12,
                       0x10, 0x10, 0x09, 0x09, 0x0B, 0x0B, 0x1C, 0x1C, 0x07, 0x07, 0x03, 0x03, 0x01, 0x01},
     44, 0},
    {0xAD, (uint8_t[]){0xF0, 0x00, 0x46, 0x00, 0x03, 0x50, 0x50, 0xFF, 0xFF, 0xF0, 0x40, 0x06, 0x01,
                       0x07, 0x42, 0x42, 0xFF, 0xFF, 0x01, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF},
     25, 0},
    {0xAE, (uint8_t[]){0xFE, 0x3F, 0x3F, 0xFE, 0x3F, 0x3F, 0x00}, 7, 0},
    {0xB2,
     (uint8_t[]){0x15, 0x19, 0x05, 0x23, 0x49, 0xAF, 0x03, 0x2E, 0x5C, 0xD2, 0xFF, 0x10, 0x20, 0xFD, 0x20, 0xC0, 0x00},
     17, 0},
    {0xE8, (uint8_t[]){0x20, 0x6F, 0x04, 0x97, 0x97, 0x3E, 0x04, 0xDC, 0xDC, 0x3E, 0x06, 0xFA, 0x26, 0x3E}, 15, 0},
    {0x75, (uint8_t[]){0x03, 0x04}, 2, 0},
    {0xE7, (uint8_t[]){0x3B, 0x00, 0x00, 0x7C, 0xA1, 0x8C, 0x20, 0x1A, 0xF0, 0xB1, 0x50, 0x00,
                       0x50, 0xB1, 0x50, 0xB1, 0x50, 0xD8, 0x00, 0x55, 0x00, 0xB1, 0x00, 0x45,
                       0xC9, 0x6A, 0xFF, 0x5A, 0xD8, 0x18, 0x88, 0x15, 0xB1, 0x01, 0x01, 0x77},
     36, 0},
    {0xEA, (uint8_t[]){0x13, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x2C}, 8, 0},
    {0xB0, (uint8_t[]){0x22, 0x43, 0x11, 0x61, 0x25, 0x43, 0x43}, 7, 0},
    {0xB7, (uint8_t[]){0x00, 0x00, 0x73, 0x73}, 4, 0},
    {0xBF, (uint8_t[]){0xA6, 0xAA}, 2, 0},
    {0xA9, (uint8_t[]){0x00, 0x00, 0x73, 0xFF, 0x00, 0x00, 0x03, 0x00, 0x00, 0x03}, 10, 0},
    {0xC8, (uint8_t[]){0x00, 0x00, 0x10, 0x1F, 0x36, 0x00, 0x5D, 0x04, 0x9D, 0x05, 0x10, 0xF2, 0x06,
                       0x60, 0x03, 0x11, 0xAD, 0x00, 0xEF, 0x01, 0x22, 0x2E, 0x0E, 0x74, 0x08, 0x32,
                       0xDC, 0x09, 0x33, 0x0F, 0xF3, 0x77, 0x0D, 0xB0, 0xDC, 0x03, 0xFF},
     37, 0},
    {0xC9, (uint8_t[]){0x00, 0x00, 0x10, 0x1F, 0x36, 0x00, 0x5D, 0x04, 0x9D, 0x05, 0x10, 0xF2, 0x06,
                       0x60, 0x03, 0x11, 0xAD, 0x00, 0xEF, 0x01, 0x22, 0x2E, 0x0E, 0x74, 0x08, 0x32,
                       0xDC, 0x09, 0x33, 0x0F, 0xF3, 0x77, 0x0D, 0xB0, 0xDC, 0x03, 0xFF},
     37, 0},
    {0x36, (uint8_t[]){0x00}, 1, 0},
    {0x11, (uint8_t[]){0x00}, 1, 100},   /* sleep out */
    {0x29, (uint8_t[]){0x00}, 1, 0},     /* display on */
    {0x35, (uint8_t[]){0x00}, 1, 100},   /* tearing effect on */
};

/* ---------- IO expanders ---------- */

static void exp_write(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    i2c_master_transmit(dev, buf, 2, 50);
}

static uint8_t exp_read(i2c_master_dev_handle_t dev, uint8_t reg)
{
    uint8_t val = 0;
    i2c_master_transmit_receive(dev, &reg, 1, &val, 1, 50);
    return val;
}

static void expanders_init(void)
{
    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = PI4IOE1_ADDR,
        .scl_speed_hz = 400000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(s_i2c, &cfg, &s_exp1));
    cfg.device_address = PI4IOE2_ADDR;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(s_i2c, &cfg, &s_exp2));

    /* #1: P1 SPK_EN, P2 EXT5V_EN, P4 LCD_RST, P5 TP_RST, P6 CAM_RST out-high */
    exp_write(s_exp1, PI4IO_REG_CHIP_RESET, 0xFF);
    exp_read(s_exp1, PI4IO_REG_CHIP_RESET);
    exp_write(s_exp1, PI4IO_REG_IO_DIR, 0x7F);
    exp_write(s_exp1, PI4IO_REG_OUT_H_IM, 0x00);
    exp_write(s_exp1, PI4IO_REG_PULL_SEL, 0x7F);
    exp_write(s_exp1, PI4IO_REG_PULL_EN, 0x7F);
    exp_write(s_exp1, PI4IO_REG_OUT_SET, 0x76);

    /* #2: P0 WLAN_PWR_EN, P3 USB5V_EN out-high (C6 gets power here) */
    exp_write(s_exp2, PI4IO_REG_CHIP_RESET, 0xFF);
    exp_read(s_exp2, PI4IO_REG_CHIP_RESET);
    exp_write(s_exp2, PI4IO_REG_IO_DIR, 0xB9);
    exp_write(s_exp2, PI4IO_REG_OUT_H_IM, 0x06);
    exp_write(s_exp2, PI4IO_REG_PULL_SEL, 0xB9);
    exp_write(s_exp2, PI4IO_REG_PULL_EN, 0xF9);
    exp_write(s_exp2, PI4IO_REG_IN_DEF_STA, 0x40);
    exp_write(s_exp2, PI4IO_REG_INT_MASK, 0xBF);
    exp_write(s_exp2, PI4IO_REG_OUT_SET, 0x09);

    /* The factory firmware leaves the C6 in an arbitrary state; give it a
       clean power cycle so its SDIO slave enumerates for esp-hosted. */
    exp_write(s_exp2, PI4IO_REG_OUT_SET, 0x08);   /* WLAN_PWR off, USB5V on */
    vTaskDelay(pdMS_TO_TICKS(120));
    exp_write(s_exp2, PI4IO_REG_OUT_SET, 0x09);
    vTaskDelay(pdMS_TO_TICKS(700));               /* C6 slave boot time */
}

/* LCD_RST (P4) + TP_RST (P5) low pulse through expander #1 */
static void panel_touch_reset(void)
{
    gpio_reset_pin(TAB5_TOUCH_INT);
    uint8_t out = exp_read(s_exp1, PI4IO_REG_OUT_SET);
    exp_write(s_exp1, PI4IO_REG_OUT_SET, out & ~((1 << 4) | (1 << 5)));
    vTaskDelay(pdMS_TO_TICKS(100));
    exp_write(s_exp1, PI4IO_REG_OUT_SET, out | (1 << 4) | (1 << 5));
    vTaskDelay(pdMS_TO_TICKS(200));
}

/* ---------- backlight (LEDC PWM on GPIO 22) ---------- */

static void backlight_init(void)
{
    const ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_12_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 5000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));
    const ledc_channel_config_t ch = {
        .gpio_num = TAB5_LCD_BACKLIGHT,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = TAB5_LEDC_CH,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch));
}

esp_err_t waveshare_rgb_lcd_bl_on(void)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, TAB5_LEDC_CH, 4095);
    return ledc_update_duty(LEDC_LOW_SPEED_MODE, TAB5_LEDC_CH);
}

esp_err_t waveshare_rgb_lcd_bl_off(void)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, TAB5_LEDC_CH, 0);
    return ledc_update_duty(LEDC_LOW_SPEED_MODE, TAB5_LEDC_CH);
}

esp_err_t waveshare_rgb_lcd_bl_set(int pct)
{
    if (pct < 0) {
        pct = 0;
    }
    if (pct > 100) {
        pct = 100;
    }
    ledc_set_duty(LEDC_LOW_SPEED_MODE, TAB5_LEDC_CH, (uint32_t)pct * 4095 / 100);
    return ledc_update_duty(LEDC_LOW_SPEED_MODE, TAB5_LEDC_CH);
}

/* ---------- DSI panel ---------- */

static IRAM_ATTR bool trans_done_cb(esp_lcd_panel_handle_t panel,
                                    esp_lcd_dpi_panel_event_data_t *edata, void *ctx)
{
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(s_trans_done, &woken);
    return woken == pdTRUE;
}

void tab5_lcd_wait_trans_done(void)
{
    if (s_trans_done != NULL) {
        xSemaphoreTake(s_trans_done, pdMS_TO_TICKS(100));
    }
}

static esp_err_t panel_init(esp_lcd_dsi_bus_handle_t *out_bus)
{
    static esp_ldo_channel_handle_t phy_ldo;
    esp_ldo_channel_config_t ldo_cfg = {
        .chan_id = TAB5_DSI_LDO_CHAN,
        .voltage_mv = TAB5_DSI_LDO_MV,
    };
    ESP_ERROR_CHECK(esp_ldo_acquire_channel(&ldo_cfg, &phy_ldo));

    esp_lcd_dsi_bus_config_t bus_cfg = {
        .bus_id = 0,
        .num_data_lanes = 2,
        .phy_clk_src = MIPI_DSI_PHY_CLK_SRC_DEFAULT,
        .lane_bit_rate_mbps = s_st7123 ? 965 : 730,
    };
    esp_lcd_dsi_bus_handle_t bus = NULL;
    ESP_ERROR_CHECK(esp_lcd_new_dsi_bus(&bus_cfg, &bus));

    esp_lcd_dbi_io_config_t dbi_cfg = {
        .virtual_channel = 0,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
    };
    esp_lcd_panel_io_handle_t io = NULL;
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_dbi(bus, &dbi_cfg, &io));

    esp_lcd_dpi_panel_config_t dpi_cfg = {
        .virtual_channel = 0,
        .dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT,
        .dpi_clock_freq_mhz = s_st7123 ? 70 : 60,
        .pixel_format = LCD_COLOR_PIXEL_FORMAT_RGB565,
        .num_fbs = 1,
        .video_timing = {
            .h_size = TAB5_LCD_H_RES,
            .v_size = TAB5_LCD_V_RES,
            .hsync_pulse_width = s_st7123 ? 2 : 40,
            .hsync_back_porch = s_st7123 ? 40 : 140,
            .hsync_front_porch = 40,
            .vsync_pulse_width = s_st7121 ? 20 : (s_st7123 ? 2 : 4),
            .vsync_back_porch = s_st7121 ? 24 : (s_st7123 ? 8 : 20),
            .vsync_front_porch = s_st7121 ? 200 : (s_st7123 ? 220 : 20),
        },
        .flags.use_dma2d = true,
    };

    if (s_st7121) {
        /* Vendored driver from M5Tab5-UserDemo, used exactly like their
         * BSP does: init_cmds = NULL selects the driver's built-in table
         * (with its own inter-command settle delays). Hand-feeding that
         * table through the st7123 driver looked close but strobed on
         * real glass - the reference path is the one that works. */
        st7121_vendor_config_t vendor = {
            .init_cmds = NULL,
            .init_cmds_size = 0,
            .mipi_config = {
                .dsi_bus = bus,
                .dpi_config = &dpi_cfg,
            },
        };
        esp_lcd_panel_dev_config_t dev_cfg = {
            .reset_gpio_num = -1,
            .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
            .data_endian = LCD_RGB_DATA_ENDIAN_LITTLE,
            .bits_per_pixel = 24,
            .vendor_config = &vendor,
        };
        ESP_ERROR_CHECK(esp_lcd_new_panel_st7121(io, &dev_cfg, &s_panel));
    } else if (s_st7123) {
        st7123_vendor_config_t vendor = {
            .init_cmds = k_st7123_init,
            .init_cmds_size = sizeof(k_st7123_init) / sizeof(k_st7123_init[0]),
            .mipi_config = {
                .dsi_bus = bus,
                .dpi_config = &dpi_cfg,
                .lane_num = 2,
            },
        };
        esp_lcd_panel_dev_config_t dev_cfg = {
            .reset_gpio_num = -1,
            .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
            .data_endian = LCD_RGB_DATA_ENDIAN_LITTLE,
            .bits_per_pixel = 24,
            .vendor_config = &vendor,
        };
        ESP_ERROR_CHECK(esp_lcd_new_panel_st7123(io, &dev_cfg, &s_panel));
    } else {
        ili9881c_vendor_config_t vendor = {
            .init_cmds = tab5_lcd_ili9881c_specific_init_code_default,
            .init_cmds_size = sizeof(tab5_lcd_ili9881c_specific_init_code_default) /
                              sizeof(tab5_lcd_ili9881c_specific_init_code_default[0]),
            .mipi_config = {
                .dsi_bus = bus,
                .dpi_config = &dpi_cfg,
                .lane_num = 2,
            },
        };
        esp_lcd_panel_dev_config_t dev_cfg = {
            .bits_per_pixel = 16,
            .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
            .reset_gpio_num = -1,
            .vendor_config = &vendor,
        };
        ESP_ERROR_CHECK(esp_lcd_new_panel_ili9881c(io, &dev_cfg, &s_panel));
    }

    ESP_ERROR_CHECK(esp_lcd_panel_reset(s_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(s_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(s_panel, true));

    s_trans_done = xSemaphoreCreateBinary();
    esp_lcd_dpi_panel_event_callbacks_t cbs = {
        .on_color_trans_done = trans_done_cb,
    };
    ESP_ERROR_CHECK(esp_lcd_dpi_panel_register_event_callbacks(s_panel, &cbs, NULL));

    *out_bus = bus;
    return ESP_OK;
}

/* ---------- touch ---------- */

static esp_err_t touch_init(esp_lcd_touch_handle_t *out_tp)
{
    esp_lcd_touch_config_t tp_cfg = {
        .x_max = TAB5_LCD_H_RES,
        .y_max = TAB5_LCD_V_RES,
        .rst_gpio_num = -1,          /* reset line lives on the expander */
        .int_gpio_num = TAB5_TOUCH_INT,
        .levels = {
            .reset = 0,
            .interrupt = 0,
        },
        .flags = {
            .swap_xy = 0,
            .mirror_x = 0,
            .mirror_y = 0,
        },
    };
    esp_lcd_panel_io_handle_t tp_io = NULL;

    if (s_st7123) {
        esp_lcd_panel_io_i2c_config_t io_cfg = ESP_LCD_TOUCH_IO_I2C_ST7123_CONFIG();
        io_cfg.scl_speed_hz = TAB5_I2C_FREQ_HZ;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(s_i2c, &io_cfg, &tp_io));
        return esp_lcd_touch_new_i2c_st7123(tp_io, &tp_cfg, out_tp);
    }

    esp_lcd_panel_io_i2c_config_t io_cfg = ESP_LCD_TOUCH_IO_I2C_GT911_CONFIG();
    io_cfg.dev_addr = ESP_LCD_TOUCH_IO_I2C_GT911_ADDRESS_BACKUP;   /* 0x14 on the Tab5 */
    io_cfg.scl_speed_hz = TAB5_I2C_FREQ_HZ;
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(s_i2c, &io_cfg, &tp_io));
    esp_err_t err = esp_lcd_touch_new_i2c_gt911(tp_io, &tp_cfg, out_tp);
    if (err == ESP_OK) {
        esp_lcd_touch_exit_sleep(*out_tp);   /* the Tab5 GT911 boots asleep */
    }
    return err;
}

/* ---------- public API (same names as the S3 port) ---------- */

const char *waveshare_lcd_board_name(void)
{
    return s_board_name;
}

void *waveshare_lcd_get_fb(void)
{
    if (s_panel == NULL) {
        return NULL;
    }
    void *fb = NULL;
    if (esp_lcd_dpi_panel_get_frame_buffer(s_panel, 1, &fb) != ESP_OK) {
        return NULL;
    }
    return fb;
}

void waveshare_lcd_get_res(int *w, int *h)
{
    /* the framebuffer is the native portrait panel; LVGL rotates in software */
    *w = TAB5_LCD_H_RES;
    *h = TAB5_LCD_V_RES;
}

esp_err_t waveshare_esp32_s3_rgb_lcd_init(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = 0,
        .sda_io_num = TAB5_I2C_SDA,
        .scl_io_num = TAB5_I2C_SCL,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &s_i2c));

    expanders_init();       /* also powers the C6 WLAN rail (exp2 P0) */
    panel_touch_reset();

    /* Panel generation probe: GT911 at 0x14 = ILI9881C units, 0x55 = the
     * Sitronix generations. Both ST7121 and ST7123 answer at 0x55; the
     * touch firmware version register (0x0000) tells them apart the way
     * the vendor BSP does it: 1 = ST7121, 3 = ST7123. They need different
     * vsync timings and completely different init tables. */
    if (i2c_master_probe(s_i2c, ESP_LCD_TOUCH_IO_I2C_GT911_ADDRESS_BACKUP, 50) == ESP_OK) {
        s_st7123 = false;
        s_board_name = "M5Stack Tab5 (ILI9881C + GT911)";
    } else if (i2c_master_probe(s_i2c, 0x55, 50) == ESP_OK) {
        s_st7123 = true;
        i2c_device_config_t vcfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = 0x55,
            .scl_speed_hz = 400000,
        };
        i2c_master_dev_handle_t vdev = NULL;
        uint8_t vreg[2] = { 0x00, 0x00 };
        uint8_t fw_ver = 0;
        if (i2c_master_bus_add_device(s_i2c, &vcfg, &vdev) == ESP_OK) {
            esp_err_t verr = i2c_master_transmit_receive(vdev, vreg, 2, &fw_ver, 1, 100);
            ESP_LOGI(TAG, "sitronix touch fw version: %u (read %s)",
                     fw_ver, esp_err_to_name(verr));
#if !CONFIG_CANFLIGHT_TAB5_FORCE_ST7123
            if (verr == ESP_OK && fw_ver == 1) {
                s_st7121 = true;
            }
#else
            ESP_LOGW(TAG, "ST7121 detection disabled by config, using ST7123 path");
#endif
            i2c_master_bus_rm_device(vdev);
        }
        s_board_name = s_st7121 ? "M5Stack Tab5 (ST7121)"
                                : "M5Stack Tab5 (ST7123)";
    } else {
        s_st7123 = false;
        s_board_name = "M5Stack Tab5 (no touch detected)";
        ESP_LOGW(TAG, "no touch controller found, assuming ILI9881C panel");
    }
    ESP_LOGI(TAG, "board: %s", s_board_name);

    backlight_init();

    esp_lcd_dsi_bus_handle_t bus = NULL;
    ESP_ERROR_CHECK(panel_init(&bus));

    esp_lcd_touch_handle_t tp = NULL;
    if (touch_init(&tp) != ESP_OK) {
        ESP_LOGE(TAG, "touch init failed - running without touch");
        tp = NULL;
    }

    ESP_ERROR_CHECK(lvgl_port_init(s_panel, tp));
    return ESP_OK;
}
