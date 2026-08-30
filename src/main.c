#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_ieee802154.h"
#include "esp_lcd_st7735.h"
#include "esp_heap_caps.h"

/* ============================================================
   PIN MAPPING — XIAO ESP32-C6 + ST7735S 128x160 + Touch
   ============================================================ */
#define TFT_RST    (1)
#define TFT_DC     (2)
#define TFT_CS     (22)   /* GPIO fixat pentru TFT CS — nu se schimba */
#define TFT_SCLK   (19)
#define TFT_MOSI   (18)

/*
 * Touch controller SPI — pinii sunt GENERICI; ajusteaza-i conform
 * modulului tau TFT+touch. Touch-ul foloseste SPI separat (CS separat)
 * dar poate partaja SCLK/MOSI cu TFT daca controllerul permite.
 *
 * Decomenteaza si seteaza pinii corecti dupa cablaj:
 */
// #define T_IRQ      (5)   /* IRQ atingere — activ LOW optional */
// #define T_DO       (6)   /* MISO touch */
// #define T_DIN      (7)   /* MOSI touch (poate fi comun cu TFT_MOSI) */
// #define T_CS       (4)   /* CS touch — OBLIGATORIU separat de TFT_CS */
// #define T_CLK      (19)  /* SCLK touch (poate fi comun cu TFT_SCLK) */

#define LCD_WIDTH  128
#define LCD_HEIGHT 160

static const char *TAG = "ZBTEST";
static esp_lcd_panel_handle_t panel_handle = NULL;
static esp_lcd_panel_io_handle_t io_handle = NULL;

/* ============================================================
   FRAME IEEE 802.15.4
   ============================================================ */
#define FRAME_MAX_LEN  127

typedef struct {
    uint8_t  data[FRAME_MAX_LEN + 1]; /* primul byte = lungime PHR */
    uint8_t  len;
    int8_t   rssi;
    uint8_t  lqi;
    uint8_t  channel;
    uint32_t timestamp_ms;
} radio_frame_t;

static radio_frame_t s_last_frame = {0};
static volatile bool s_frame_ready = false;

/* ============================================================
   RADIO ENGINE STATS
   ============================================================ */
typedef struct {
    uint32_t rx_total;
    uint32_t tx_total;
    uint32_t tx_errors;
    uint8_t  channel;
    int8_t   last_rssi;
    uint8_t  last_lqi;
} radio_stats_t;

static radio_stats_t s_radio_stats = {0};
static uint32_t s_scan_packets[16] = {0};  /* canalele 11..26 */
static int8_t   s_scan_rssi[16]    = {0};

/* ============================================================
   SETTINGS
   ============================================================ */
typedef struct {
    uint16_t pan_id;
    uint16_t dst_addr;
    uint16_t src_addr;
    uint8_t  channel;
    uint16_t tx_interval_ms;
    uint16_t dwell_time_ms;
    uint8_t  tx_payload[16];
    uint8_t  tx_payload_len;
} settings_t;

static settings_t s_settings = {
    .pan_id = 0x1234,
    .dst_addr = 0xFFFF,
    .src_addr = 0xABCD,
    .channel = 15,
    .tx_interval_ms = 500,
    .dwell_time_ms = 500,
    .tx_payload = {'Z', 'B', 'T', 'E', 'S', 'T'},
    .tx_payload_len = 6,
};

/* ============================================================
   UI STATE MACHINE
   ============================================================ */
typedef enum {
    SCREEN_MAIN,
    SCREEN_GENERATOR_LOCKED,
    SCREEN_GENERATOR_SPREAD,
    SCREEN_SNIFFER_LOCKED,
    SCREEN_SNIFFER_SCAN,
    SCREEN_LAST_FRAME,
    SCREEN_SETUP,
    SCREEN_INFO,
} screen_t;

static screen_t s_screen = SCREEN_MAIN;
static screen_t s_prev_screen = SCREEN_MAIN;

/* Sub-moduri pentru generator si sniffer */
static bool s_gen_running = false;
static bool s_sniffer_running = false;

/* Generator spread: bitmap canale selectate (11..26 -> bit 0..15) */
static uint16_t s_spread_channels = 0xFFFF; /* implicit toate */

/* Sniffer scan: canal selectat manual dupa scan */
static uint8_t s_scan_selected_ch = 11;

/* ============================================================
   FONT 8x8 — ASCII 32..122
   ============================================================ */
const uint8_t font8x8[91][8] = {
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, // sp
    {0x18,0x3C,0x3C,0x18,0x18,0x00,0x18,0x00}, // !
    {0x6C,0x6C,0x6C,0x00,0x00,0x00,0x00,0x00}, // "
    {0x6C,0x6C,0xFE,0x6C,0xFE,0x6C,0x6C,0x00}, // #
    {0x18,0x7E,0xC0,0x7C,0x06,0xFC,0x18,0x00}, // $
    {0x00,0xC6,0xCC,0x18,0x30,0x66,0xC6,0x00}, // %
    {0x38,0x6C,0x38,0x76,0xDC,0xCC,0x76,0x00}, // &
    {0x30,0x30,0x60,0x00,0x00,0x00,0x00,0x00}, // '
    {0x0C,0x18,0x30,0x30,0x30,0x18,0x0C,0x00}, // (
    {0x30,0x18,0x0C,0x0C,0x0C,0x18,0x30,0x00}, // )
    {0x00,0x66,0x3C,0xFF,0x3C,0x66,0x00,0x00}, // *
    {0x00,0x18,0x18,0x7E,0x18,0x18,0x00,0x00}, // +
    {0x00,0x00,0x00,0x00,0x00,0x18,0x18,0x30}, // ,
    {0x00,0x00,0x00,0x7E,0x00,0x00,0x00,0x00}, // -
    {0x00,0x00,0x00,0x00,0x00,0x18,0x18,0x00}, // .
    {0x06,0x0C,0x18,0x30,0x60,0xC0,0x80,0x00}, // /
    {0x7C,0xC6,0xCE,0xDE,0xF6,0xE6,0x7C,0x00}, // 0
    {0x10,0x30,0xF0,0x30,0x30,0x30,0xFC,0x00}, // 1
    {0x7C,0xC6,0x06,0x3C,0x60,0xC0,0xFE,0x00}, // 2
    {0x7C,0xC6,0x06,0x3C,0x06,0xC6,0x7C,0x00}, // 3
    {0x1C,0x3C,0x5C,0x9C,0xFE,0x1C,0x1C,0x00}, // 4
    {0xFE,0xC0,0xFC,0x06,0x06,0xC6,0x7C,0x00}, // 5
    {0x3C,0x60,0xC0,0xFC,0xC6,0xC6,0x7C,0x00}, // 6
    {0xFE,0xC6,0x06,0x0C,0x18,0x30,0x30,0x00}, // 7
    {0x7C,0xC6,0xC6,0x7C,0xC6,0xC6,0x7C,0x00}, // 8
    {0x7C,0xC6,0xC6,0x7E,0x06,0x0C,0x78,0x00}, // 9
    {0x00,0x18,0x18,0x00,0x00,0x18,0x18,0x00}, // :
    {0x00,0x18,0x18,0x00,0x00,0x18,0x18,0x30}, // ;
    {0x0C,0x18,0x30,0x60,0x30,0x18,0x0C,0x00}, // <
    {0x00,0x00,0x7E,0x00,0x7E,0x00,0x00,0x00}, // =
    {0x30,0x18,0x0C,0x06,0x0C,0x18,0x30,0x00}, // >
    {0x7C,0xC6,0x06,0x0C,0x18,0x00,0x18,0x00}, // ?
    {0x7C,0xC6,0xDE,0xDE,0xDC,0xC0,0x7E,0x00}, // @
    {0x10,0x38,0x6C,0xC6,0xFE,0xC6,0xC6,0x00}, // A
    {0xFC,0x66,0x66,0x7C,0x66,0x66,0xFC,0x00}, // B
    {0x3C,0x66,0xC0,0xC0,0xC0,0x66,0x3C,0x00}, // C
    {0xFC,0x66,0x66,0x66,0x66,0x66,0xFC,0x00}, // D
    {0xFE,0x62,0x68,0x78,0x68,0x62,0xFE,0x00}, // E
    {0xFE,0x62,0x68,0x78,0x68,0x60,0xF0,0x00}, // F
    {0x3C,0x66,0xC0,0xC0,0xCE,0x66,0x3E,0x00}, // G
    {0xC6,0xC6,0xC6,0xFE,0xC6,0xC6,0xC6,0x00}, // H
    {0x78,0x30,0x30,0x30,0x30,0x30,0x78,0x00}, // I
    {0x1E,0x06,0x06,0x06,0x06,0xC6,0x7C,0x00}, // J
    {0xC6,0xCC,0xD8,0xF0,0xD8,0xCC,0xC6,0x00}, // K
    {0xF0,0x60,0x60,0x60,0x60,0x62,0xFE,0x00}, // L
    {0xC6,0xEE,0xFE,0xFE,0xD6,0xC6,0xC6,0x00}, // M
    {0xC6,0xE6,0xF6,0xDE,0xCE,0xC6,0xC6,0x00}, // N
    {0x3C,0x66,0xC6,0xC6,0xC6,0x66,0x3C,0x00}, // O
    {0xFC,0x66,0x66,0x7C,0x60,0x60,0xF0,0x00}, // P
    {0x3C,0x66,0xC6,0xC6,0xD6,0xCC,0x7E,0x06}, // Q
    {0xFC,0x66,0x66,0x7C,0x6C,0x66,0xC6,0x00}, // R
    {0x7C,0xC6,0x60,0x3C,0x06,0xC6,0x7C,0x00}, // S
    {0x7E,0x5A,0x18,0x18,0x18,0x18,0x3C,0x00}, // T
    {0xC6,0xC6,0xC6,0xC6,0xC6,0x7C,0x00,0x00}, // U
    {0xC6,0xC6,0xC6,0xC6,0xC6,0x30,0x18,0x00}, // V
    {0xC6,0xC6,0xD6,0xFE,0xFE,0xEE,0xC6,0x00}, // W
    {0xC6,0xC6,0x6C,0x38,0x6C,0xC6,0xC6,0x00}, // X
    {0xC6,0xC6,0x6C,0x38,0x18,0x18,0x3C,0x00}, // Y
    {0xFE,0xC6,0x0C,0x18,0x30,0x62,0xFE,0x00}, // Z
    {0x3C,0x30,0x30,0x30,0x30,0x30,0x3C,0x00}, // [
    {0xC0,0x60,0x30,0x18,0x0C,0x06,0x02,0x00}, /* backslash */
    {0x3C,0x0C,0x0C,0x0C,0x0C,0x0C,0x3C,0x00}, // ]
    {0x10,0x38,0x6C,0x00,0x00,0x00,0x00,0x00}, // ^
    {0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0x00}, // _
    {0x30,0x30,0x18,0x00,0x00,0x00,0x00,0x00}, // `
    {0x00,0x00,0x78,0x0C,0x7C,0xCC,0x76,0x00}, // a
    {0xE0,0x60,0x7C,0x66,0x66,0x66,0xDC,0x00}, // b
    {0x00,0x00,0x7C,0xC6,0xC0,0xC6,0x7C,0x00}, // c
    {0x1C,0x0C,0x7C,0xCC,0xCC,0xCC,0x76,0x00}, // d
    {0x00,0x00,0x7C,0xC6,0xFE,0xC0,0x7E,0x00}, // e
    {0x38,0x6C,0x60,0xF0,0x60,0x60,0xF0,0x00}, // f
    {0x00,0x00,0x76,0xCC,0xCC,0x7C,0x0C,0xF8}, // g
    {0xE0,0x60,0x6C,0x76,0x66,0x66,0xE6,0x00}, // h
    {0x18,0x00,0x38,0x18,0x18,0x18,0x3C,0x00}, // i
    {0x06,0x00,0x06,0x06,0x06,0x06,0x66,0x3C}, // j
    {0xE0,0x60,0x66,0x6C,0x78,0x6C,0xE6,0x00}, // k
    {0x38,0x18,0x18,0x18,0x18,0x18,0x3C,0x00}, // l
    {0x00,0x00,0xEC,0xFE,0xD6,0xD6,0xC6,0x00}, // m
    {0x00,0x00,0xDC,0x66,0x66,0x66,0x66,0x00}, // n
    {0x00,0x00,0x7C,0xC6,0xC6,0xC6,0x7C,0x00}, // o
    {0x00,0x00,0xDC,0x66,0x66,0x7C,0x60,0xF0}, // p
    {0x00,0x00,0x76,0xCC,0xCC,0x7C,0x0C,0x1E}, // q
    {0x00,0x00,0xDC,0x76,0x62,0x60,0xF0,0x00}, // r
    {0x00,0x00,0x7E,0xC0,0x7C,0x06,0xFC,0x00}, // s
    {0x30,0x30,0xFC,0x30,0x30,0x34,0x18,0x00}, // t
    {0x00,0x00,0xCC,0xCC,0xCC,0xCC,0x76,0x00}, // u
    {0x00,0x00,0xC6,0xC6,0xC6,0x38,0x10,0x00}, // v
    {0x00,0x00,0xC6,0xD6,0xFE,0xFE,0x6C,0x00}, // w
    {0x00,0x00,0xC6,0x6C,0x38,0x6C,0xC6,0x00}, // x
    {0x00,0x00,0xC6,0xC6,0xCC,0x78,0x30,0xE0}, // y
    {0x00,0x00,0xFE,0x8C,0x18,0x32,0xFE,0x00}  // z
};

#include "esp_lcd_types.h"

/* ============================================================
   FRAMEBUFFER DMA — 128 x 160 x 2 bytes = 40 KB
   ============================================================ */
static uint16_t *s_fb = NULL;

static inline uint16_t rgb888_to_rgb565(uint8_t r, uint8_t g, uint8_t b) {
    uint16_t color = (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
    return (uint16_t)((color << 8) | (color >> 8));
}

static inline void fb_set_pixel(int x, int y, uint16_t color) {
    if (x >= 0 && x < LCD_WIDTH && y >= 0 && y < LCD_HEIGHT) {
        s_fb[y * LCD_WIDTH + x] = color;
    }
}

void fb_clear(uint16_t color) {
    for (int i = 0; i < LCD_WIDTH * LCD_HEIGHT; i++) {
        s_fb[i] = color;
    }
}

void fb_fill_rect(int x, int y, int w, int h, uint16_t color) {
    if (x < 0) { w += x; x = 0; }
    if (y < 0) { h += y; y = 0; }
    if (x + w > LCD_WIDTH)  w = LCD_WIDTH - x;
    if (y + h > LCD_HEIGHT) h = LCD_HEIGHT - y;
    if (w <= 0 || h <= 0) return;

    for (int row = y; row < y + h; row++) {
        for (int col = x; col < x + w; col++) {
            s_fb[row * LCD_WIDTH + col] = color;
        }
    }
}

void fb_draw_char(int x, int y, char c, uint16_t fg_color, uint16_t bg_color) {
    if (c < 32 || c > 122) c = '?';
    uint8_t c_idx = c - 32;
    for (int row = 0; row < 8; row++) {
        uint8_t bits = font8x8[c_idx][row];
        for (int col = 0; col < 8; col++) {
            fb_set_pixel(x + col, y + row,
                (bits & (1 << (7 - col))) ? fg_color : bg_color);
        }
    }
}

void fb_draw_string(int x, int y, const char *str, uint16_t fg_color, uint16_t bg_color) {
    while (*str) {
        fb_draw_char(x, y, *str++, fg_color, bg_color);
        x += 8;
        if (x + 8 > LCD_WIDTH) break;
    }
}

void fb_flush(void) {
    esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, LCD_WIDTH, LCD_HEIGHT, s_fb);
}

/* ============================================================
   TOUCHSCREEN (XPT2046 / HR2046)
   Decomenteaza pinii T_* de mai sus pentru a activa.
   ============================================================ */
#if defined(T_IRQ) && defined(T_CS)

static spi_device_handle_t s_touch_spi = NULL;

static void touch_spi_init(void) {
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = T_CS,
        .queue_size = 1,
        .pre_cb = NULL,
        .post_cb = NULL,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &s_touch_spi));

    gpio_config_t irq_conf = {
        .pin_bit_mask = (1ULL << T_IRQ),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&irq_conf);
}

static uint16_t touch_read_adc(uint8_t cmd) {
    if (!s_touch_spi) return 0;
    uint8_t tx[3] = {cmd, 0x00, 0x00};
    uint8_t rx[3] = {0};
    spi_transaction_t t = {
        .length = 24,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    spi_device_transmit(s_touch_spi, &t);
    uint16_t val = ((rx[1] << 8) | rx[2]) >> 3;
    return val & 0x0FFF;
}

static bool touch_is_pressed(void) {
    if (T_IRQ >= 0) {
        return (gpio_get_level(T_IRQ) == 0);
    }
    /* Fallback: citeste Z (pressure) */
    uint16_t z1 = touch_read_adc(0xB0);
    uint16_t z2 = touch_read_adc(0xC0);
    return (z1 > 50 && z2 < 4000);
}

/* Calibrare empirica — ajusteaza MIN/MAX conform modulului tau */
#define TOUCH_X_MIN  200
#define TOUCH_X_MAX  3800
#define TOUCH_Y_MIN  200
#define TOUCH_Y_MAX  3800

static bool touch_get_point(int *x, int *y) {
    if (!touch_is_pressed()) return false;

    uint32_t sx = 0, sy = 0;
    for (int i = 0; i < 4; i++) {
        sx += touch_read_adc(0x90); /* X */
        sy += touch_read_adc(0xD0); /* Y */
    }
    uint16_t raw_x = (uint16_t)(sx / 4);
    uint16_t raw_y = (uint16_t)(sy / 4);

    int tx = (int)((raw_x - TOUCH_X_MIN) * LCD_WIDTH  / (TOUCH_X_MAX - TOUCH_X_MIN));
    int ty = (int)((raw_y - TOUCH_Y_MIN) * LCD_HEIGHT / (TOUCH_Y_MAX - TOUCH_Y_MIN));

    if (tx < 0) tx = 0; if (tx >= LCD_WIDTH)  tx = LCD_WIDTH  - 1;
    if (ty < 0) ty = 0; if (ty >= LCD_HEIGHT) ty = LCD_HEIGHT - 1;

    *x = tx;
    *y = ty;
    return true;
}

#else /* T_IRQ / T_CS nu sunt definite */

static void touch_spi_init(void) {}
static bool touch_is_pressed(void) { return false; }
static bool touch_get_point(int *x, int *y) { (void)x; (void)y; return false; }

#endif

/* ============================================================
   Helpers
   ============================================================ */
static inline uint16_t channel_to_mhz(uint8_t ch) {
    return (uint16_t)(2405 + (ch - 11) * 5);
}

static bool point_in_rect(int px, int py, int x, int y, int w, int h) {
    return (px >= x && px < x + w && py >= y && py < y + h);
}

/* Forward declarations */
static void stats_reset_scan(void);
static void radio_set_channel(uint8_t ch);

/* ============================================================
   UI NAVIGATION — TOUCH HANDLER
   ============================================================ */
static uint32_t s_last_touch_ms = 0;

static void ui_handle_touch(void) {
    int x, y;
    if (!touch_get_point(&x, &y)) return;

    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    if (now - s_last_touch_ms < 250) return; /* debounce 250 ms */
    s_last_touch_ms = now;

    ESP_LOGI(TAG, "Touch: %d,%d screen=%d", x, y, (int)s_screen);

    switch (s_screen) {
        case SCREEN_MAIN:
            if (point_in_rect(x, y, 14, 30, 100, 22)) {
                s_screen = SCREEN_GENERATOR_LOCKED;
                s_prev_screen = SCREEN_MAIN;
            } else if (point_in_rect(x, y, 14, 58, 100, 22)) {
                s_screen = SCREEN_SNIFFER_LOCKED;
                s_prev_screen = SCREEN_MAIN;
                /* Pornim sniffer pe canalul din setari */
                radio_set_channel(s_settings.channel);
                esp_ieee802154_receive();
                s_sniffer_running = true;
            } else if (point_in_rect(x, y, 4, 92, 56, 22)) {
                s_screen = SCREEN_SETUP;
                s_prev_screen = SCREEN_MAIN;
            } else if (point_in_rect(x, y, 68, 92, 56, 22)) {
                s_screen = SCREEN_INFO;
                s_prev_screen = SCREEN_MAIN;
            }
            break;

        case SCREEN_GENERATOR_LOCKED:
            if (point_in_rect(x, y, 24, 88, 80, 22)) {
                s_gen_running = !s_gen_running;
            } else if (point_in_rect(x, y, 68, 128, 56, 22)) {
                s_screen = SCREEN_GENERATOR_SPREAD;
                s_prev_screen = SCREEN_GENERATOR_LOCKED;
            } else if (point_in_rect(x, y, 4, 128, 40, 22)) {
                s_gen_running = false;
                s_screen = SCREEN_MAIN;
            }
            break;

        case SCREEN_GENERATOR_SPREAD: {
            if (point_in_rect(x, y, 4, 142, 40, 16)) {
                s_gen_running = false;
                s_screen = SCREEN_GENERATOR_LOCKED;
            } else if (point_in_rect(x, y, 68, 120, 56, 18)) {
                s_gen_running = !s_gen_running;
            } else if (point_in_rect(x, y, 4, 120, 56, 18)) {
                /* Toggle ALL / NONE */
                if (s_spread_channels == 0xFFFF) {
                    s_spread_channels = 0;
                } else {
                    s_spread_channels = 0xFFFF;
                }
            } else {
                /* Detectare canal din matrice */
                for (int row = 0; row < 4; row++) {
                    for (int col = 0; col < 4; col++) {
                        int ch = 11 + row * 4 + col;
                        if (ch > 26) break;
                        int bx = 4 + col * 31;
                        int by = 30 + row * 22;
                        if (point_in_rect(x, y, bx, by, 28, 18)) {
                            s_spread_channels ^= (1 << (ch - 11));
                            return;
                        }
                    }
                }
            }
            break;
        }

        case SCREEN_SNIFFER_LOCKED:
            if (point_in_rect(x, y, 4, 132, 34, 18)) {
                s_screen = SCREEN_MAIN;
            } else if (point_in_rect(x, y, 44, 128, 40, 22)) {
                s_screen = SCREEN_LAST_FRAME;
                s_prev_screen = SCREEN_SNIFFER_LOCKED;
            } else if (point_in_rect(x, y, 88, 128, 36, 22)) {
                s_screen = SCREEN_SNIFFER_SCAN;
                s_prev_screen = SCREEN_SNIFFER_LOCKED;
                stats_reset_scan();
            }
            break;

        case SCREEN_LAST_FRAME:
            if (point_in_rect(x, y, 4, 132, 40, 18)) {
                s_screen = s_prev_screen;
            }
            break;

        case SCREEN_SNIFFER_SCAN:
            if (point_in_rect(x, y, 4, 132, 40, 18)) {
                s_screen = SCREEN_MAIN;
            } else if (point_in_rect(x, y, 54, 128, 70, 22)) {
                s_settings.channel = s_radio_stats.channel;
                s_screen = SCREEN_SNIFFER_LOCKED;
            }
            break;

        case SCREEN_SETUP:
        case SCREEN_INFO:
            if (point_in_rect(x, y, 4, 132, 40, 18)) {
                s_screen = SCREEN_MAIN;
            }
            break;

        default:
            s_screen = SCREEN_MAIN;
            break;
    }
}

static void stats_reset_scan(void) {
    memset(s_scan_packets, 0, sizeof(s_scan_packets));
    memset(s_scan_rssi, 0, sizeof(s_scan_rssi));
}

static void radio_set_channel(uint8_t ch) {
    if (ch < 11 || ch > 26) ch = 11;
    s_radio_stats.channel = ch;
    ESP_ERROR_CHECK(esp_ieee802154_set_channel(ch));
}

/* ============================================================
   IEEE 802.15.4 FRAME PARSER
   ============================================================ */
typedef enum {
    FRAME_TYPE_BEACON   = 0,
    FRAME_TYPE_DATA     = 1,
    FRAME_TYPE_ACK      = 2,
    FRAME_TYPE_MAC_CMD  = 3,
} frame_type_t;

typedef struct {
    frame_type_t type;
    uint8_t  seq;
    uint16_t pan_id;
    uint16_t dst_addr;
    uint16_t src_addr;
    uint8_t  payload_offset;
    uint8_t  payload_len;
} parsed_frame_t;

static const char *frame_type_name(frame_type_t type) {
    switch (type) {
        case FRAME_TYPE_BEACON:  return "BEACON";
        case FRAME_TYPE_DATA:    return "DATA";
        case FRAME_TYPE_ACK:     return "ACK";
        case FRAME_TYPE_MAC_CMD: return "MACCMD";
        default:                 return "UNKNOWN";
    }
}

static void parse_ieee802154_frame(const uint8_t *frame, uint8_t len, parsed_frame_t *out) {
    memset(out, 0, sizeof(*out));
    if (len < 3) return;

    uint16_t fcf = frame[1] | ((uint16_t)frame[2] << 8);
    out->type = (frame_type_t)(fcf & 0x07);
    out->seq = frame[3];

    uint8_t dst_addr_mode = (fcf >> 10) & 0x03;
    uint8_t src_addr_mode = (fcf >> 12) & 0x03;
    bool panid_comp = (fcf >> 6) & 0x01;

    uint8_t idx = 3; /* dupa seq */

    /* Dest addressing */
    if (dst_addr_mode == 2) {
        out->pan_id = frame[idx] | ((uint16_t)frame[idx + 1] << 8);
        idx += 2;
        out->dst_addr = frame[idx] | ((uint16_t)frame[idx + 1] << 8);
        idx += 2;
    } else if (dst_addr_mode == 3) {
        out->pan_id = frame[idx] | ((uint16_t)frame[idx + 1] << 8);
        idx += 2;
        idx += 8; /* extended address - skip */
    }

    /* Source addressing */
    if (src_addr_mode == 2) {
        if (!panid_comp && dst_addr_mode == 0) {
            out->pan_id = frame[idx] | ((uint16_t)frame[idx + 1] << 8);
            idx += 2;
        }
        out->src_addr = frame[idx] | ((uint16_t)frame[idx + 1] << 8);
        idx += 2;
    } else if (src_addr_mode == 3) {
        if (!panid_comp && dst_addr_mode == 0) {
            out->pan_id = frame[idx] | ((uint16_t)frame[idx + 1] << 8);
            idx += 2;
        }
        idx += 8; /* extended address - skip */
    }

    if (idx < len) {
        out->payload_offset = idx;
        out->payload_len = len - idx;
    }
}

/* ============================================================
   Callback IEEE 802.15.4 RX — IRAM_ATTR (ISR-safe)
   ============================================================ */
void IRAM_ATTR esp_ieee802154_receive_done(
    uint8_t *frame,
    esp_ieee802154_frame_info_t *frame_info)
{
    s_radio_stats.rx_total++;

    uint8_t len = frame[0];
    if (len > FRAME_MAX_LEN) len = FRAME_MAX_LEN;

    /* Copiem rapid frame-ul si metadata */
    memcpy(s_last_frame.data, frame, len + 1);
    s_last_frame.len = len;
    s_last_frame.rssi = frame_info->rssi;
    s_last_frame.lqi = frame_info->lqi;
    s_last_frame.channel = frame_info->channel;
    s_last_frame.timestamp_ms = xTaskGetTickCountFromISR() * portTICK_PERIOD_MS;

    s_radio_stats.last_rssi = frame_info->rssi;
    s_radio_stats.last_lqi = frame_info->lqi;

    /* Statistici scan pe canal */
    uint8_t idx = (frame_info->channel >= 11 && frame_info->channel <= 26)
                  ? (frame_info->channel - 11) : 0;
    s_scan_packets[idx]++;
    if (frame_info->rssi < s_scan_rssi[idx] || s_scan_rssi[idx] == 0) {
        s_scan_rssi[idx] = frame_info->rssi;
    }

    s_frame_ready = true;

    /* Eliberam bufferul RX catre driver dupa procesare */
    esp_ieee802154_receive_handle_done(frame);
}

/* ============================================================
   Callback IEEE 802.15.4 TX DONE
   ============================================================ */
void IRAM_ATTR esp_ieee802154_transmit_done(
    const uint8_t *frame,
    const uint8_t *ack,
    esp_ieee802154_frame_info_t *ack_frame_info)
{
    (void)frame;
    (void)ack;
    (void)ack_frame_info;
    s_radio_stats.tx_total++;
}

/* ============================================================
   RADIO TX
   ============================================================ */
static uint8_t s_tx_seq = 0;

static void radio_tx_test_frame(void) {
    uint8_t frame[64];
    uint8_t pos = 0;

    /* PHR: lungime MPDU + 2 (FCS) */
    uint8_t payload_len = s_settings.tx_payload_len;
    if (payload_len > sizeof(s_settings.tx_payload)) payload_len = sizeof(s_settings.tx_payload);

    uint8_t mpdu_len = 2 + 1 + 2 + 2 + 2 + payload_len; /* FCF + Seq + PAN + DST + SRC + payload */
    frame[pos++] = mpdu_len + 2;

    /* FCF: Data frame, panid compression, short addresses */
    frame[pos++] = 0x41; /* Data, no ACK request */
    frame[pos++] = 0x88; /* PAN id compression, short dst/src */

    /* Sequence */
    frame[pos++] = s_tx_seq++;

    /* PAN ID */
    frame[pos++] = (uint8_t)(s_settings.pan_id & 0xFF);
    frame[pos++] = (uint8_t)(s_settings.pan_id >> 8);

    /* DST */
    frame[pos++] = (uint8_t)(s_settings.dst_addr & 0xFF);
    frame[pos++] = (uint8_t)(s_settings.dst_addr >> 8);

    /* SRC */
    frame[pos++] = (uint8_t)(s_settings.src_addr & 0xFF);
    frame[pos++] = (uint8_t)(s_settings.src_addr >> 8);

    /* Payload */
    memcpy(&frame[pos], s_settings.tx_payload, payload_len);
    pos += payload_len;

    esp_err_t err = esp_ieee802154_transmit(frame, false);
    if (err != ESP_OK) {
        s_radio_stats.tx_errors++;
    }
}

/* ============================================================ */
void app_main(void)
{
    /* --- NVS ---------------------------------------------------- */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* --- Alocare framebuffer DMA -------------------------------- */
    s_fb = (uint16_t *)heap_caps_malloc(
        LCD_WIDTH * LCD_HEIGHT * sizeof(uint16_t),
        MALLOC_CAP_DMA);
    if (!s_fb) {
        ESP_LOGE(TAG, "Eroare alocare framebuffer DMA!");
        return;
    }

    /* --- SPI bus ------------------------------------------------ */
    spi_bus_config_t buscfg = {
        .sclk_io_num = TFT_SCLK,
        .mosi_io_num = TFT_MOSI,
        .miso_io_num = -1,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_WIDTH * LCD_HEIGHT * sizeof(uint16_t),
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));

    /* --- Panel IO ----------------------------------------------- */
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = TFT_DC,
        .cs_gpio_num = TFT_CS,
        .pclk_hz = 10 * 1000 * 1000,   // 10 MHz — echilibru viteza/stabilitate
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 0,
        .trans_queue_depth = 7,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(
        (esp_lcd_spi_bus_handle_t)SPI2_HOST, &io_config, &io_handle));

    /* --- Panel init --------------------------------------------- */
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = TFT_RST,
        .bits_per_pixel = 16,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7735(io_handle, &panel_config, &panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));

    vTaskDelay(pdMS_TO_TICKS(120));   // ST7735 iese din Sleep Out

    /* --- Offset & orientare -------------------------------------
       Daca textul e rasturnat / oglinzit / decalat, modifica aici.
       Valori comune: set_gap(0,0), (2,1), (2,3), (0,32)
       --- Orientare & offset -------------------------------------
       Daca textul e rasturnat / oglinzit / decalat,
       modifica aici. Valori comune pentru ST7735 128x160:
       - set_gap(0, 0)   -> majoritatea modulelor noi
       - set_gap(2, 1)   -> "Green Tab" vechi
       - set_gap(2, 3)   -> alte clone
       - set_gap(0, 32)  -> unele variante 160x128

       Pentru MADCTL (comanda 0x36), daca e necesar:
       0x00 = normal, 0xC0 = rotit 180, 0xC8 = oglinda+rotit
       ---------------------------------------------------------- */
    esp_lcd_panel_set_gap(panel_handle, 0, 0);
    // Decomenteaza DOAR daca imaginea e rasturnata:
    esp_lcd_panel_io_tx_param(io_handle, 0x36, (uint8_t[]){0xC0}, 1);
    
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, false));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
    vTaskDelay(pdMS_TO_TICKS(50));

    /* --- Culori ------------------------------------------------- */
    uint16_t c_black  = rgb888_to_rgb565(0,   0,   0);
    uint16_t c_white  = rgb888_to_rgb565(255, 255, 255);
    uint16_t c_green  = rgb888_to_rgb565(0,   255, 0);
    uint16_t c_yellow = rgb888_to_rgb565(255, 255, 0);
    uint16_t c_red    = rgb888_to_rgb565(255, 0,   0);
    uint16_t c_cyan   = rgb888_to_rgb565(0,   255, 255);
    uint16_t c_blue   = rgb888_to_rgb565(0,   0,   255);
    uint16_t c_orange = rgb888_to_rgb565(255, 165, 0);
    uint16_t c_gray   = rgb888_to_rgb565(128, 128, 128);

    ESP_LOGI(TAG, "Display initializat. Init touch...");
    touch_spi_init();

    /* --- Radio init --------------------------------------------- */
    esp_ieee802154_enable();
    esp_ieee802154_set_promiscuous(true);
    radio_set_channel(s_settings.channel);
    esp_ieee802154_receive();
    s_sniffer_running = true;

    /* --- Main UI loop ------------------------------------------- */
    char buff[48];
    uint32_t last_render = 0;
    uint32_t last_tx_tick = 0;
    uint32_t scan_tick = 0;

    while (1) {
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;

        /* Procesare touch */
        ui_handle_touch();

        /* Procesare frame primit in callback */
        if (s_frame_ready) {
            s_frame_ready = false;
            /* Actualizari care tin de frame in task principal */
        }

        /* Generator LOCKED: TX periodic */
        if (s_screen == SCREEN_GENERATOR_LOCKED && s_gen_running) {
            if (now - last_tx_tick >= s_settings.tx_interval_ms) {
                last_tx_tick = now;
                radio_tx_test_frame();
            }
        }

        /* Generator SPREAD: TX periodic pe canalele selectate */
        if (s_screen == SCREEN_GENERATOR_SPREAD && s_gen_running) {
            static uint8_t spread_idx = 0;
            if (now - last_tx_tick >= s_settings.dwell_time_ms) {
                last_tx_tick = now;
                radio_tx_test_frame();
                /* Avansam la urmatorul canal activ */
                uint8_t active_count = 0;
                for (int i = 0; i < 16; i++) {
                    if (s_spread_channels & (1 << i)) active_count++;
                }
                if (active_count > 0) {
                    do {
                        spread_idx = (spread_idx + 1) & 0x0F;
                    } while (!(s_spread_channels & (1 << spread_idx)));
                    s_settings.channel = 11 + spread_idx;
                    radio_set_channel(s_settings.channel);
                }
            }
        }

        /* Sniffer SCAN: channel hopping */
        if (s_screen == SCREEN_SNIFFER_SCAN && s_sniffer_running) {
            if (now - scan_tick >= s_settings.dwell_time_ms) {
                scan_tick = now;
                s_radio_stats.channel++;
                if (s_radio_stats.channel > 26) s_radio_stats.channel = 11;
                radio_set_channel(s_radio_stats.channel);
                esp_ieee802154_receive();
            }
        }

        /* Rendering la ~5 FPS e suficient pentru UI si reduce flicker */
        if (now - last_render >= 200) {
            last_render = now;

            switch (s_screen) {
                case SCREEN_MAIN:
                    fb_clear(c_black);
                    fb_draw_string(4, 4,  "ZIGBEE TESTER", c_yellow, c_black);
                    fb_fill_rect(4, 18, 120, 2, c_gray);

                    /* [ GENERATOR ] */
                    fb_fill_rect(14, 30, 100, 22, c_blue);
                    fb_draw_string(22, 38, "GENERATOR", c_white, c_blue);

                    /* [ SNIFFER ] */
                    fb_fill_rect(14, 58, 100, 22, c_green);
                    fb_draw_string(26, 66, "SNIFFER", c_black, c_green);

                    /* [ SETUP ] */
                    fb_fill_rect(4,  92, 56, 22, c_cyan);
                    fb_draw_string(10, 100, "SETUP", c_black, c_cyan);

                    /* [ INFO ] */
                    fb_fill_rect(68, 92, 56, 22, c_orange);
                    fb_draw_string(78, 100, "INFO", c_black, c_orange);
                    break;

                case SCREEN_GENERATOR_LOCKED:
                    fb_clear(c_black);
                    fb_draw_string(4, 4, "GEN / LOCKED", c_yellow, c_black);
                    snprintf(buff, sizeof(buff), "CH:%02u %04uMHz",
                             s_settings.channel, channel_to_mhz(s_settings.channel));
                    fb_draw_string(4, 18, buff, c_white, c_black);
                    snprintf(buff, sizeof(buff), "PAN:%04X DST:%04X",
                             s_settings.pan_id, s_settings.dst_addr);
                    fb_draw_string(4, 30, buff, c_white, c_black);
                    snprintf(buff, sizeof(buff), "TX:%lu", s_radio_stats.tx_total);
                    fb_draw_string(4, 44, buff, c_green, c_black);
                    snprintf(buff, sizeof(buff), "ERR:%lu", s_radio_stats.tx_errors);
                    fb_draw_string(4, 56, buff, c_red, c_black);
                    snprintf(buff, sizeof(buff), "INT:%ums", s_settings.tx_interval_ms);
                    fb_draw_string(4, 70, buff, c_white, c_black);

                    /* Buton START/STOP */
                    if (s_gen_running) {
                        fb_fill_rect(24, 88, 80, 22, c_red);
                        fb_draw_string(34, 95, "STOP", c_white, c_red);
                    } else {
                        fb_fill_rect(24, 88, 80, 22, c_green);
                        fb_draw_string(32, 95, "START", c_black, c_green);
                    }

                    /* Buton SPREAD */
                    fb_fill_rect(68, 128, 56, 22, c_orange);
                    fb_draw_string(70, 134, "SPRD", c_black, c_orange);

                    /* Buton BACK */
                    fb_fill_rect(4, 128, 40, 22, c_gray);
                    fb_draw_string(8, 134, "<", c_white, c_gray);
                    break;

                case SCREEN_GENERATOR_SPREAD:
                    fb_clear(c_black);
                    fb_draw_string(4, 4, "GEN / SPREAD", c_yellow, c_black);
                    snprintf(buff, sizeof(buff), "DWELL:%ums", s_settings.dwell_time_ms);
                    fb_draw_string(4, 16, buff, c_white, c_black);

                    /* Matrice canale 11..26 */
                    for (int row = 0; row < 4; row++) {
                        for (int col = 0; col < 4; col++) {
                            int ch = 11 + row * 4 + col;
                            if (ch > 26) break;
                            bool active = (s_spread_channels >> (ch - 11)) & 1;
                            int bx = 4 + col * 31;
                            int by = 30 + row * 22;
                            fb_fill_rect(bx, by, 28, 18, active ? c_green : c_gray);
                            snprintf(buff, sizeof(buff), "%02d", ch);
                            fb_draw_string(bx + 4, by + 5, buff,
                                           active ? c_black : c_white,
                                           active ? c_green : c_gray);
                        }
                    }

                    if (s_gen_running) {
                        fb_fill_rect(68, 120, 56, 18, c_red);
                        fb_draw_string(72, 125, "STOP", c_white, c_red);
                    } else {
                        fb_fill_rect(68, 120, 56, 18, c_green);
                        fb_draw_string(70, 125, "START", c_black, c_green);
                    }
                    fb_fill_rect(4, 120, 56, 18, c_blue);
                    fb_draw_string(6, 125, "ALL/NON", c_white, c_blue);
                    fb_fill_rect(4, 142, 40, 16, c_gray);
                    fb_draw_string(8, 146, "<", c_white, c_gray);
                    break;

                case SCREEN_SNIFFER_LOCKED:
                    fb_clear(c_black);
                    fb_draw_string(4, 4, "SNIFF / LOCKED", c_cyan, c_black);
                    snprintf(buff, sizeof(buff), "CH:%02u %04uMHz",
                             s_radio_stats.channel, channel_to_mhz(s_radio_stats.channel));
                    fb_draw_string(4, 18, buff, c_white, c_black);
                    snprintf(buff, sizeof(buff), "RX:%lu RSSI:%d",
                             s_radio_stats.rx_total, s_radio_stats.last_rssi);
                    fb_draw_string(4, 32, buff, c_green, c_black);
                    snprintf(buff, sizeof(buff), "LQI:%u CH:%u",
                             s_radio_stats.last_lqi, s_last_frame.channel);
                    fb_draw_string(4, 44, buff, c_green, c_black);

                    parsed_frame_t pf;
                    parse_ieee802154_frame(s_last_frame.data + 1,
                                           s_last_frame.len, &pf);
                    snprintf(buff, sizeof(buff), "T:%s SEQ:%u",
                             frame_type_name(pf.type), pf.seq);
                    fb_draw_string(4, 58, buff, c_yellow, c_black);
                    snprintf(buff, sizeof(buff), "SRC:%04X DST:%04X",
                             pf.src_addr, pf.dst_addr);
                    fb_draw_string(4, 70, buff, c_yellow, c_black);
                    snprintf(buff, sizeof(buff), "PAN:%04X LEN:%u",
                             pf.pan_id, s_last_frame.len);
                    fb_draw_string(4, 82, buff, c_yellow, c_black);

                    /* Butoane */
                    fb_fill_rect(4, 132, 34, 18, c_gray);
                    fb_draw_string(8, 138, "<", c_white, c_gray);
                    fb_fill_rect(44, 128, 40, 22, c_blue);
                    fb_draw_string(46, 134, "FRM", c_white, c_blue);
                    fb_fill_rect(88, 128, 36, 22, c_orange);
                    fb_draw_string(90, 134, "SCN", c_black, c_orange);
                    break;

                case SCREEN_SNIFFER_SCAN:
                    fb_clear(c_black);
                    fb_draw_string(4, 4, "SNIFF / SCAN", c_cyan, c_black);
                    snprintf(buff, sizeof(buff), "CH:%02u RSSI:%d PKT:%lu",
                             s_radio_stats.channel, s_radio_stats.last_rssi,
                             s_scan_packets[s_radio_stats.channel - 11]);
                    fb_draw_string(4, 18, buff, c_white, c_black);

                    /* Mini bar chart */
                    for (int i = 0; i < 16; i++) {
                        int h = (s_scan_packets[i] > 40) ? 40 : (int)s_scan_packets[i];
                        uint16_t col = (s_scan_packets[i] > 0) ? c_green : c_gray;
                        fb_fill_rect(4 + i * 7, 120 - h, 6, h, col);
                    }

                    fb_fill_rect(4, 132, 40, 18, c_gray);
                    fb_draw_string(8, 138, "<", c_white, c_gray);
                    fb_fill_rect(54, 128, 70, 22, c_green);
                    fb_draw_string(54, 134, "LOCK CH", c_black, c_green);
                    break;

                case SCREEN_SETUP:
                    fb_clear(c_black);
                    fb_draw_string(4, 4, "SETUP", c_cyan, c_black);
                    fb_draw_string(4, 18, "RADIO", c_white, c_black);
                    snprintf(buff, sizeof(buff), "PAN:%04X CH:%02u",
                             s_settings.pan_id, s_settings.channel);
                    fb_draw_string(4, 30, buff, c_yellow, c_black);
                    fb_draw_string(4, 46, "GENERATOR", c_white, c_black);
                    snprintf(buff, sizeof(buff), "INT:%ums DWELL:%ums",
                             s_settings.tx_interval_ms, s_settings.dwell_time_ms);
                    fb_draw_string(4, 58, buff, c_yellow, c_black);
                    fb_draw_string(4, 74, "DISPLAY", c_white, c_black);
                    fb_draw_string(4, 86, "SYSTEM", c_white, c_black);
                    fb_draw_string(4, 100, "TODO: edit params", c_gray, c_black);
                    fb_fill_rect(4, 132, 40, 18, c_gray);
                    fb_draw_string(8, 138, "<", c_white, c_gray);
                    break;

                case SCREEN_LAST_FRAME:
                    fb_clear(c_black);
                    fb_draw_string(4, 4, "LAST FRAME", c_yellow, c_black);
                    {
                        parsed_frame_t pf;
                        parse_ieee802154_frame(s_last_frame.data + 1,
                                               s_last_frame.len, &pf);
                        snprintf(buff, sizeof(buff), "%s SEQ:%u",
                                 frame_type_name(pf.type), pf.seq);
                        fb_draw_string(4, 18, buff, c_white, c_black);
                        snprintf(buff, sizeof(buff), "RSSI:%d LEN:%u",
                                 s_last_frame.rssi, s_last_frame.len);
                        fb_draw_string(4, 30, buff, c_white, c_black);
                        snprintf(buff, sizeof(buff), "PAN:%04X SRC:%04X",
                                 pf.pan_id, pf.src_addr);
                        fb_draw_string(4, 42, buff, c_yellow, c_black);
                        snprintf(buff, sizeof(buff), "DST:%04X",
                                 pf.dst_addr);
                        fb_draw_string(4, 54, buff, c_yellow, c_black);

                        int line = 70;
                        int off = pf.payload_offset;
                        int len = (pf.payload_len > 18) ? 18 : pf.payload_len;
                        for (int i = 0; i < len && line < 128; i += 6) {
                            char hex[32] = {0};
                            int pos = 0;
                            for (int j = 0; j < 6 && (i + j) < len; j++) {
                                pos += snprintf(hex + pos, sizeof(hex) - pos,
                                                "%02X ", s_last_frame.data[1 + off + i + j]);
                            }
                            fb_draw_string(4, line, hex, c_cyan, c_black);
                            line += 10;
                        }
                    }
                    fb_fill_rect(4, 132, 40, 18, c_gray);
                    fb_draw_string(8, 138, "<", c_white, c_gray);
                    break;

                case SCREEN_INFO:
                    fb_clear(c_black);
                    fb_draw_string(4, 4, "INFO", c_orange, c_black);
                    fb_draw_string(4, 18, "ESP32-C6", c_white, c_black);
                    fb_draw_string(4, 30, "FW: v0.2.0", c_white, c_black);
                    snprintf(buff, sizeof(buff), "CH:%02u RX:%lu",
                             s_radio_stats.channel, s_radio_stats.rx_total);
                    fb_draw_string(4, 44, buff, c_white, c_black);
                    fb_draw_string(4, 58, "Touch: not init", c_red, c_black);
                    fb_fill_rect(4, 132, 40, 18, c_gray);
                    fb_draw_string(8, 138, "<", c_white, c_gray);
                    break;

                default:
                    s_screen = SCREEN_MAIN;
                    break;
            }

            fb_flush();
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}











