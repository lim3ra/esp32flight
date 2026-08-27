/*
 * Display bring-up for the Guition JC8012P4A1 (ESP32-P4, 10.1" 800x1280
 * MIPI-DSI, JD9365 panel, Silead GSL3680 touch, ESP32-C6 for Wi-Fi).
 *
 * Same API surface as the Tab5 and S3 ports. The panel is native portrait;
 * LVGL renders the 1280x800 landscape UI and software-rotates it (see
 * lvgl_port.c). Two hardware revisions exist with the same JD9365 driver
 * chip but different glass: V1 (rear-case batch <= 2627) and V2 (>= 2628)
 * need different init tables, pixel clock and vsync porch, and there is no
 * known way to tell them apart at runtime, so it is a build-time choice.
 *
 * Hardware facts cross-checked between Guition's demo code, ESPHome's
 * mipi_dsi presets and the espcontrol/BETTA-HA-PANEL working configs:
 *   LCD_RST 27, backlight LEDC on 23, touch I2C SDA 7 / SCL 8 (no internal
 *   pullups), TP_RST 22, TP_INT 21, GSL3680 at 0x40; DSI 2 lanes,
 *   V1: 1 Gbps/lane, 60 MHz DPI, vsync 4/8/20; V2: 1.5 Gbps/lane, 70 MHz,
 *   vsync 4/10/20; hsync 20/20/40 on both. C6 over SDIO on the P4 EV-board
 *   pins (CLK 18, CMD 19, D0-3 14..17, reset 54) - handled by esp_hosted.
 */
#include "waveshare_rgb_lcd_port.h"

#include <string.h>
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/ledc.h"
#include "esp_ldo_regulator.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_lcd_jd9365.h"
#include "freertos/semphr.h"
#include "jc10/esp_lcd_touch_gsl3680.h"

static const char *TAG = "jc10";

#define JC10_I2C_SDA        7
#define JC10_I2C_SCL        8
#define JC10_I2C_FREQ_HZ    400000
#define JC10_TOUCH_RST      22
#define JC10_TOUCH_INT      21
#define JC10_LCD_RST        27
#define JC10_LCD_BACKLIGHT  23
#define JC10_LEDC_CH        LEDC_CHANNEL_1

#define JC10_DSI_LDO_CHAN   3
#define JC10_DSI_LDO_MV     2500

#define JC10_LCD_H_RES      800     /* native portrait */
#define JC10_LCD_V_RES      1280

#if CONFIG_CANFLIGHT_JC10_PANEL_V2
#define JC10_LANE_MBPS      1500
#define JC10_DPI_MHZ        70
#define JC10_VS_BP          10
#define JC10_BOARD_NAME     "Guition JC8012P4A1 (10.1in, panel V2)"
#else
#define JC10_LANE_MBPS      1000
#define JC10_DPI_MHZ        60
#define JC10_VS_BP          8
#define JC10_BOARD_NAME     "Guition JC8012P4A1 (10.1in, panel V1)"
#endif

static i2c_master_bus_handle_t s_i2c;
static esp_lcd_panel_handle_t s_panel;
static SemaphoreHandle_t s_trans_done;

#include "jc10/jc10_jd9365_init.inc"

/* ---------- backlight ---------- */

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
        .gpio_num = JC10_LCD_BACKLIGHT,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = JC10_LEDC_CH,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch));
}

esp_err_t waveshare_rgb_lcd_bl_on(void)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, JC10_LEDC_CH, 4095);
    return ledc_update_duty(LEDC_LOW_SPEED_MODE, JC10_LEDC_CH);
}

esp_err_t waveshare_rgb_lcd_bl_off(void)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, JC10_LEDC_CH, 0);
    return ledc_update_duty(LEDC_LOW_SPEED_MODE, JC10_LEDC_CH);
}

esp_err_t waveshare_rgb_lcd_bl_set(int pct)
{
    if (pct < 0) {
        pct = 0;
    }
    if (pct > 100) {
        pct = 100;
    }
    ledc_set_duty(LEDC_LOW_SPEED_MODE, JC10_LEDC_CH, (uint32_t)pct * 4095 / 100);
    return ledc_update_duty(LEDC_LOW_SPEED_MODE, JC10_LEDC_CH);
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

static esp_err_t panel_init(void)
{
    static esp_ldo_channel_handle_t phy_ldo;
    esp_ldo_channel_config_t ldo_cfg = {
        .chan_id = JC10_DSI_LDO_CHAN,
        .voltage_mv = JC10_DSI_LDO_MV,
    };
    ESP_ERROR_CHECK(esp_ldo_acquire_channel(&ldo_cfg, &phy_ldo));

    esp_lcd_dsi_bus_config_t bus_cfg = {
        .bus_id = 0,
        .num_data_lanes = 2,
        .phy_clk_src = MIPI_DSI_PHY_CLK_SRC_DEFAULT,
        .lane_bit_rate_mbps = JC10_LANE_MBPS,
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
        .dpi_clock_freq_mhz = JC10_DPI_MHZ,
        .pixel_format = LCD_COLOR_PIXEL_FORMAT_RGB565,
        .num_fbs = 1,
        .video_timing = {
            .h_size = JC10_LCD_H_RES,
            .v_size = JC10_LCD_V_RES,
            .hsync_pulse_width = 20,
            .hsync_back_porch = 20,
            .hsync_front_porch = 40,
            .vsync_pulse_width = 4,
            .vsync_back_porch = JC10_VS_BP,
            .vsync_front_porch = 20,
        },
        .flags.use_dma2d = true,
    };

    jd9365_vendor_config_t vendor = {
#if CONFIG_CANFLIGHT_JC10_PANEL_V2
        .init_cmds = k_jd9365_init_v2,
        .init_cmds_size = sizeof(k_jd9365_init_v2) / sizeof(k_jd9365_init_v2[0]),
#else
        .init_cmds = k_jd9365_init_v1,
        .init_cmds_size = sizeof(k_jd9365_init_v1) / sizeof(k_jd9365_init_v1[0]),
#endif
        .mipi_config = {
            .dsi_bus = bus,
            .dpi_config = &dpi_cfg,
            .lane_num = 2,
        },
    };
    esp_lcd_panel_dev_config_t dev_cfg = {
        .reset_gpio_num = JC10_LCD_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
        .vendor_config = &vendor,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_jd9365(io, &dev_cfg, &s_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(s_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(s_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(s_panel, true));

    s_trans_done = xSemaphoreCreateBinary();
    esp_lcd_dpi_panel_event_callbacks_t cbs = {
        .on_color_trans_done = trans_done_cb,
    };
    ESP_ERROR_CHECK(esp_lcd_dpi_panel_register_event_callbacks(s_panel, &cbs, NULL));
    return ESP_OK;
}

/* ---------- touch ---------- */

static esp_err_t touch_init(esp_lcd_touch_handle_t *out_tp)
{
    esp_lcd_touch_config_t tp_cfg = {
        .x_max = JC10_LCD_H_RES,
        .y_max = JC10_LCD_V_RES,
        .rst_gpio_num = JC10_TOUCH_RST,
        .int_gpio_num = JC10_TOUCH_INT,
        .levels = {
            .reset = 0,
            .interrupt = 0,
        },
        .flags = {
            .swap_xy = CONFIG_CANFLIGHT_JC10_TP_SWAP_XY,
            .mirror_x = CONFIG_CANFLIGHT_JC10_TP_MIRROR_X,
            .mirror_y = CONFIG_CANFLIGHT_JC10_TP_MIRROR_Y,
        },
    };
    esp_lcd_panel_io_i2c_config_t io_cfg = ESP_LCD_TOUCH_IO_I2C_GSL3680_CONFIG();
    io_cfg.scl_speed_hz = JC10_I2C_FREQ_HZ;
    esp_lcd_panel_io_handle_t tp_io = NULL;
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(s_i2c, &io_cfg, &tp_io));
    return esp_lcd_touch_new_i2c_gsl3680(tp_io, &tp_cfg, out_tp);
}

/* ---------- public API ---------- */

const char *waveshare_lcd_board_name(void)
{
    return JC10_BOARD_NAME;
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
    *w = JC10_LCD_H_RES;
    *h = JC10_LCD_V_RES;
}

esp_err_t waveshare_esp32_s3_rgb_lcd_init(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = 0,
        .sda_io_num = JC10_I2C_SDA,
        .scl_io_num = JC10_I2C_SCL,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = false,   /* board has its own pullups */
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &s_i2c));
    ESP_LOGI(TAG, "board: %s", JC10_BOARD_NAME);

    backlight_init();
    ESP_ERROR_CHECK(panel_init());

    esp_lcd_touch_handle_t tp = NULL;
    if (touch_init(&tp) != ESP_OK) {
        ESP_LOGE(TAG, "touch init failed - running without touch");
        tp = NULL;
    }
    ESP_ERROR_CHECK(lvgl_port_init(s_panel, tp));
    return ESP_OK;
}
