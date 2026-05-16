#pragma once

#include <stdbool.h>
#include "esp_err.h"

// Minimal SSD1306 128x64 driver on the shared I2C bus.
// Frame-buffered: oled_clear / oled_text fills a 1 KB RAM buffer, then
// oled_flush ships it as a single I²C transaction (~25 ms at 400 kHz —
// fine at the 1 Hz display refresh).
//
// Font is a small 5x7 glyph table covering only the chars used by the
// display task; unsupported chars render as a tiny solid block.

esp_err_t oled_init(void);

// Buffer ops (no I/O):
void oled_clear(void);
// Draws a string starting at the given character cell.
// row: 0..7 (each = 8 px tall page).
// col: 0..20 (each = 6 px wide glyph cell).
void oled_text(int row, int col, const char *s);

// Sends the whole framebuffer to the panel. ~25 ms at 400 kHz.
esp_err_t oled_flush(void);
