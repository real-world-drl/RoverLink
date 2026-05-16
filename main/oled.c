#include "oled.h"
#include "i2c_bus.h"

#include <string.h>
#include "driver/i2c.h"
#include "esp_log.h"

static const char *TAG = "oled";

#define OLED_ADDR        0x3C
#define OLED_W           128
#define OLED_H           32       // Waveshare General Driver has the 0.91" panel
#define OLED_PAGES       (OLED_H / 8)
#define OLED_CTRL_CMD    0x00     // Co=0, D/C#=0 -> command stream
#define OLED_CTRL_DATA   0x40     // Co=0, D/C#=1 -> data stream

#define GLYPH_W          5         // bytes per glyph (columns)
#define CELL_W           6         // 5 pixels + 1 column spacing

static uint8_t s_fb[OLED_W * OLED_PAGES];

// --- Font: only the glyphs the display task uses; everything else falls
// back to a small block. Each byte is a column, LSB = top pixel.
typedef struct { char ch; uint8_t glyph[GLYPH_W]; } glyph_t;
static const glyph_t FONT[] = {
    {' ', {0x00,0x00,0x00,0x00,0x00}},
    {'+', {0x08,0x08,0x3E,0x08,0x08}},
    {'-', {0x08,0x08,0x08,0x08,0x08}},
    {'.', {0x00,0x60,0x60,0x00,0x00}},
    {':', {0x00,0x36,0x36,0x00,0x00}},
    {'0', {0x3E,0x51,0x49,0x45,0x3E}},
    {'1', {0x00,0x42,0x7F,0x40,0x00}},
    {'2', {0x42,0x61,0x51,0x49,0x46}},
    {'3', {0x21,0x41,0x45,0x4B,0x31}},
    {'4', {0x18,0x14,0x12,0x7F,0x10}},
    {'5', {0x27,0x45,0x45,0x45,0x39}},
    {'6', {0x3C,0x4A,0x49,0x49,0x30}},
    {'7', {0x01,0x71,0x09,0x05,0x03}},
    {'8', {0x36,0x49,0x49,0x49,0x36}},
    {'9', {0x06,0x49,0x49,0x29,0x1E}},
    {'H', {0x7F,0x08,0x08,0x08,0x7F}},
    {'L', {0x7F,0x40,0x40,0x40,0x40}},
    {'P', {0x7F,0x09,0x09,0x09,0x06}},
    {'R', {0x7F,0x09,0x19,0x29,0x46}},
    {'V', {0x07,0x18,0x60,0x18,0x07}},
    {'Y', {0x07,0x08,0x70,0x08,0x07}},
    {'d', {0x38,0x44,0x44,0x48,0x7F}},
    {'m', {0x7C,0x04,0x18,0x04,0x78}},
    {'r', {0x7C,0x08,0x04,0x04,0x08}},
    {'a', {0x20,0x54,0x54,0x54,0x78}},
    {'s', {0x48,0x54,0x54,0x54,0x24}},
    {'c', {0x38,0x44,0x44,0x44,0x20}},
    {'x', {0x44,0x28,0x10,0x28,0x44}},
    {'y', {0x0C,0x50,0x50,0x50,0x3C}},
    {'O', {0x3E,0x41,0x41,0x41,0x3E}},
    {'D', {0x7F,0x41,0x41,0x41,0x3E}},
    {'M', {0x7F,0x02,0x0C,0x02,0x7F}},
    {'/', {0x20,0x10,0x08,0x04,0x02}},
};
static const uint8_t MISSING[GLYPH_W] = {0x7E,0x42,0x42,0x42,0x7E};

static const uint8_t *find_glyph(char c) {
    for (size_t i = 0; i < sizeof(FONT)/sizeof(FONT[0]); i++) {
        if (FONT[i].ch == c) return FONT[i].glyph;
    }
    return MISSING;
}

// --- I²C helpers --------------------------------------------------------

static esp_err_t send_cmds(const uint8_t *cmds, size_t n) {
    // Each command goes out as [control=0x00, cmd_byte]; we can stream
    // multiple back-to-back by prefixing once with 0x00 then sending bytes
    // one at a time. For init brevity we just send single-byte commands.
    uint8_t buf[2] = { OLED_CTRL_CMD, 0 };
    for (size_t i = 0; i < n; i++) {
        buf[1] = cmds[i];
        esp_err_t e = i2c_master_write_to_device(
            I2C_NUM_0, OLED_ADDR, buf, 2, pdMS_TO_TICKS(100));
        if (e != ESP_OK) return e;
    }
    return ESP_OK;
}

// --- Public API ---------------------------------------------------------

esp_err_t oled_init(void) {
    static const uint8_t init_seq[] = {
        0xAE,                   // display off
        0xD5, 0x80,             // clock div / oscillator
        0xA8, OLED_H - 1,       // mux ratio (31 for 128x32, 63 for 128x64)
        0xD3, 0x00,             // display offset
        0x40,                   // start line = 0
        0x8D, 0x14,             // charge pump on
        0x20, 0x00,             // memory mode = horizontal addressing
        0xA1,                   // segment remap (flip x)  — matches stock
        0xC8,                   // COM scan inverted (flip y) — matches stock
        0xDA, 0x02,             // COM pins: sequential, no L/R remap (128x32)
        0x81, 0x8F,             // contrast
        0xD9, 0xF1,             // pre-charge
        0xDB, 0x40,             // VCOMH deselect
        0xA4,                   // entire display follows RAM
        0xA6,                   // normal (non-inverted)
        0x2E,                   // deactivate scroll
        0xAF,                   // display on
    };
    esp_err_t err = send_cmds(init_seq, sizeof(init_seq));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "init failed: %s", esp_err_to_name(err));
        return err;
    }
    oled_clear();
    err = oled_flush();
    ESP_LOGI(TAG, "ready at 0x%02X (128x64)", OLED_ADDR);
    return err;
}

void oled_clear(void) {
    memset(s_fb, 0, sizeof(s_fb));
}

void oled_text(int row, int col, const char *s) {
    if (row < 0 || row >= OLED_PAGES) return;
    int x_px = col * CELL_W;
    while (*s && x_px + GLYPH_W <= OLED_W) {
        const uint8_t *g = find_glyph(*s++);
        for (int i = 0; i < GLYPH_W; i++) {
            s_fb[row * OLED_W + x_px + i] = g[i];
        }
        // 1-column spacing (already zero from clear, but be explicit so
        // overwriting works correctly without an intervening clear).
        if (x_px + GLYPH_W < OLED_W) {
            s_fb[row * OLED_W + x_px + GLYPH_W] = 0x00;
        }
        x_px += CELL_W;
    }
}

esp_err_t oled_flush(void) {
    // Set full column and page address ranges so the data stream covers
    // the whole panel in one shot.
    static const uint8_t addr_cmds[] = {
        0x21, 0x00, OLED_W - 1,   // column 0..127
        0x22, 0x00, OLED_PAGES-1, // page   0..7
    };
    esp_err_t err = send_cmds(addr_cmds, sizeof(addr_cmds));
    if (err != ESP_OK) return err;

    // Data write: single transaction = [0x40][1024 bytes of GDDRAM].
    // Build it in a small scratch buffer on the stack to avoid heap churn.
    uint8_t hdr_and_data[1 + sizeof(s_fb)];
    hdr_and_data[0] = OLED_CTRL_DATA;
    memcpy(&hdr_and_data[1], s_fb, sizeof(s_fb));
    return i2c_master_write_to_device(
        I2C_NUM_0, OLED_ADDR,
        hdr_and_data, sizeof(hdr_and_data),
        pdMS_TO_TICKS(200));
}
