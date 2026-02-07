#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <string.h>
#include "esp_lcd_ili9488.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "lvgl.h"
#include "lvgl_port.h"
#include "ui.h"

#define TAG "ILI9488"

/* ===== PIN CONFIG (ESP32 DevKit V1) ===== */
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   5
#define PIN_NUM_DC   21
#define PIN_NUM_RST  4

#define LCD_SPI_HOST HSPI_HOST

spi_device_handle_t tft = NULL;

#define LCD_W 480
#define LCD_H 320

#define FONT_W 5
#define FONT_H 7

#define COLOR_WHITE 0xFFFF

//void ili9488_draw_string(int x, int y, const char *text, uint16_t color, int scale);

void draw_text_centered(const char *text, int scale) {
    int len = strlen(text);

    int text_w = len * FONT_W * scale;
    int text_h = FONT_H * scale;

    int x = (LCD_W - text_w) / 2;
    int y = (LCD_H - text_h) / 2;

    //ili9488_draw_string(x, y, text, COLOR_WHITE, scale);
}

/* ===== Low-level helpers ===== */
static void lcd_cmd(uint8_t cmd)
{
    gpio_set_level(PIN_NUM_DC, 0);
    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &cmd
    };
    spi_device_transmit(tft, &t);
}

static void lcd_data(const uint8_t *data, int len)
{
    gpio_set_level(PIN_NUM_DC, 1);
    spi_transaction_t t = {
        .length = len * 8,
        .tx_buffer = data
    };
    spi_device_transmit(tft, &t);
}

/* ===== ILI9488 Init ===== */
static void ili9488_init(void)
{
    gpio_set_level(PIN_NUM_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(PIN_NUM_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(120));

    lcd_cmd(0xE0); // Positive Gamma
    uint8_t pgamma[] = {0x00,0x03,0x09,0x08,0x16,0x0A,0x3F,0x78,0x4C,0x09,0x0A,0x08,0x16,0x1A,0x0F};
    lcd_data(pgamma, sizeof(pgamma));

    lcd_cmd(0xE1); // Negative Gamma
    uint8_t ngamma[] = {0x00,0x16,0x19,0x03,0x0F,0x05,0x32,0x45,0x46,0x04,0x0E,0x0D,0x35,0x37,0x0F};
    lcd_data(ngamma, sizeof(ngamma));

    lcd_cmd(0xC0); uint8_t pwr1[] = {0x17,0x15}; lcd_data(pwr1,2);
    lcd_cmd(0xC1); uint8_t pwr2[] = {0x41}; lcd_data(pwr2,1);
    lcd_cmd(0xC5); uint8_t vcom[] = {0x00,0x12,0x80}; lcd_data(vcom,3);

    lcd_cmd(0x36); uint8_t mem[] = {0x28}; lcd_data(mem,1); // Landscape
    lcd_cmd(0x3A); uint8_t pix[] = {0x55}; lcd_data(pix,1); // RGB565

    lcd_cmd(0x11); // Sleep out
    vTaskDelay(pdMS_TO_TICKS(120));

    lcd_cmd(0x29); // Display ON
}

/* ===== Fill screen ===== */
static void ili9488_fill(uint16_t color)
{
    uint8_t data[4];

    lcd_cmd(0x2A); // Column
    data[0]=0; data[1]=0; data[2]=0x01; data[3]=0xDF;
    lcd_data(data,4);

    lcd_cmd(0x2B); // Row
    data[0]=0; data[1]=0; data[2]=0x01; data[3]=0x3F;
    lcd_data(data,4);

    lcd_cmd(0x2C);

    uint8_t hi = color >> 8;
    uint8_t lo = color & 0xFF;

    gpio_set_level(PIN_NUM_DC, 1);
    for(int i=0;i<480*320;i++){
        uint8_t px[2] = {hi, lo};
        spi_transaction_t t = {
            .length = 16,
            .tx_buffer = px
        };
        spi_device_transmit(tft, &t);
        if ((i & 0x3FF) == 0) {   // every ~1024 ops
        vTaskDelay(1);       // yield to RTOS
    }
    }
}

/* ===== Main ===== */
void app_main(void)
{
   /* gpio_set_direction(PIN_NUM_DC, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_NUM_RST, GPIO_MODE_OUTPUT);

    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = -1,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 480 * 2
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 20 * 1000 * 1000, // safe MAX
        .mode = 0,
        .spics_io_num = PIN_NUM_CS,
        .queue_size = 7,
        .flags = SPI_DEVICE_HALFDUPLEX  // optional for full speed
    };
    //---Error Check---
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(LCD_SPI_HOST, &devcfg, &tft));

    ESP_LOGI("LCD", "SPI handle = %p", tft); 
    // ---end---
    spi_bus_initialize(SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    spi_bus_add_device(SPI_HOST, &devcfg, &tft);

    ESP_LOGI(TAG, "ILI9488 Init");
    ili9488_init();

    ESP_LOGI(TAG, "Fill screen black");
    ili9488_fill(0x0000); // Black

    ESP_LOGI(TAG, "Hello World displayed (graphics OK)");
    draw_text_centered("HELLO", 2);*/
    lvgl_port_init();
    ui_init();
     
    while (1) {
        lv_timer_handler();
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
