#include "lvgl.h"
#include "esp_timer.h"
#include "driver/spi_master.h"
#include "esp_log.h" 

// Tag for LVGL log for checking display_flush
static const char *TAG = "LVGL";

/* atanisoft ILI9488 driver */
#include "esp_lcd_ili9488.h"
#include "esp_lcd_panel_ops.h"     // for esp_lcd_panel_draw_bitmap

/* ---------- LCD IO ---------- */
    static esp_lcd_panel_io_handle_t io_handle;
    static esp_lcd_panel_handle_t panel_handle;
/* ================= USER CONFIG ================= */
#define LCD_H_RES 320
#define LCD_V_RES 480

#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   5
#define PIN_NUM_DC   21
#define PIN_NUM_RST  4
/* ============================================== */

static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf1[LCD_H_RES * 40];

/* LVGL calls this to draw */
static void disp_flush_cb(lv_disp_drv_t *drv,
                          const lv_area_t *area,
                          lv_color_t *color_p)
{
    ESP_LOGI(TAG, "flush called");
    esp_lcd_panel_draw_bitmap(
        panel_handle,
        area->x1,
        area->y1,
        area->x2 + 1,
        area->y2 + 1,
        color_p
    );

    lv_disp_flush_ready(drv);
}

/* 1 ms tick */
static void lv_tick_cb(void *arg)
{
    lv_tick_inc(1);
}

void lvgl_port_init(void)
{
    lv_init();

    // ---------- SPI BUS (driver manages internally) ----------
spi_bus_config_t buscfg = {
    .mosi_io_num = PIN_NUM_MOSI,
    .miso_io_num = -1,
    .sclk_io_num = PIN_NUM_CLK,
    .max_transfer_sz = LCD_H_RES * 40 * 2
};
spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);

// ---------- ILI9488 PANEL ----------
esp_lcd_panel_dev_config_t panel_config = {
    .reset_gpio_num = PIN_NUM_RST,
    .color_space = ESP_LCD_COLOR_SPACE_RGB,
    .bits_per_pixel = 16,
};

esp_lcd_panel_handle_t panel_handle;

esp_lcd_new_panel_ili9488(
    NULL,                  // IO handle (atanisoft driver manages internally)
    &panel_config,         // panel config
    LCD_H_RES * LCD_V_RES, // buffer size
    &panel_handle          // panel handle output
);

// Reset and initialize
esp_lcd_panel_reset(panel_handle);
esp_lcd_panel_init(panel_handle);
esp_lcd_panel_disp_on_off(panel_handle, true);

    /* ---------- LVGL BUFFER ---------- */
    static lv_disp_draw_buf_t draw_buf;
    static lv_color_t buf1[LCD_H_RES * 40];
    lv_disp_draw_buf_init(&draw_buf, buf1, NULL, LCD_H_RES * 40);

    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = LCD_H_RES;
    disp_drv.ver_res = LCD_V_RES;
    disp_drv.flush_cb = disp_flush_cb;
    disp_drv.draw_buf = &draw_buf;

    lv_disp_drv_register(&disp_drv);

    /* ---------- LVGL TICK ---------- */
    const esp_timer_create_args_t tick_args = {
        .callback = lv_tick_cb,
        .name = "lv_tick"
    };

    esp_timer_handle_t tick_timer;
    esp_timer_create(&tick_args, &tick_timer);
    esp_timer_start_periodic(tick_timer, 1000);
}