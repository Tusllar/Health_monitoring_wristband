// components/ssd1306/ssd1306.c
#include "ssd1306.h"
#include "i2c_bus.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "SSD1306";

// Frame buffer
static uint8_t ssd1306_buffer[SSD1306_WIDTH * SSD1306_HEIGHT / 8];

// Font 5x7 (basic ASCII)
static const uint8_t font5x7[][5] = {
    {0x00, 0x00, 0x00, 0x00, 0x00}, // Space (32)
    {0x00, 0x00, 0x5F, 0x00, 0x00}, // ! (33)
    {0x00, 0x07, 0x00, 0x07, 0x00}, // " (34)
    {0x14, 0x7F, 0x14, 0x7F, 0x14}, // # (35)
    {0x24, 0x2A, 0x7F, 0x2A, 0x12}, // $ (36)
    {0x23, 0x13, 0x08, 0x64, 0x62}, // % (37)
    {0x36, 0x49, 0x55, 0x22, 0x50}, // & (38)
    {0x00, 0x05, 0x03, 0x00, 0x00}, // ' (39)
    {0x00, 0x1C, 0x22, 0x41, 0x00}, // ( (40)
    {0x00, 0x41, 0x22, 0x1C, 0x00}, // ) (41)
    {0x08, 0x2A, 0x1C, 0x2A, 0x08}, // * (42)
    {0x08, 0x08, 0x3E, 0x08, 0x08}, // + (43)
    {0x00, 0x50, 0x30, 0x00, 0x00}, // , (44)
    {0x08, 0x08, 0x08, 0x08, 0x08}, // - (45)
    {0x00, 0x60, 0x60, 0x00, 0x00}, // . (46)
    {0x20, 0x10, 0x08, 0x04, 0x02}, // / (47)
    {0x3E, 0x51, 0x49, 0x45, 0x3E}, // 0 (48)
    {0x00, 0x42, 0x7F, 0x40, 0x00}, // 1 (49)
    {0x42, 0x61, 0x51, 0x49, 0x46}, // 2 (50)
    {0x21, 0x41, 0x45, 0x4B, 0x31}, // 3 (51)
    {0x18, 0x14, 0x12, 0x7F, 0x10}, // 4 (52)
    {0x27, 0x45, 0x45, 0x45, 0x39}, // 5 (53)
    {0x3C, 0x4A, 0x49, 0x49, 0x30}, // 6 (54)
    {0x01, 0x71, 0x09, 0x05, 0x03}, // 7 (55)
    {0x36, 0x49, 0x49, 0x49, 0x36}, // 8 (56)
    {0x06, 0x49, 0x49, 0x29, 0x1E}, // 9 (57)
    {0x00, 0x36, 0x36, 0x00, 0x00}, // : (58)
    {0x00, 0x56, 0x36, 0x00, 0x00}, // ; (59)
    {0x08, 0x14, 0x22, 0x41, 0x00}, // < (60)
    {0x14, 0x14, 0x14, 0x14, 0x14}, // = (61)
    {0x00, 0x41, 0x22, 0x14, 0x08}, // > (62)
    {0x02, 0x01, 0x51, 0x09, 0x06}, // ? (63)
    {0x32, 0x49, 0x59, 0x51, 0x3E}, // @ (64)
    {0x7C, 0x12, 0x11, 0x12, 0x7C}, // A (65)
    {0x7F, 0x49, 0x49, 0x49, 0x36}, // B (66)
    {0x3E, 0x41, 0x41, 0x41, 0x22}, // C (67)
    {0x7F, 0x41, 0x41, 0x22, 0x1C}, // D (68)
    {0x7F, 0x49, 0x49, 0x49, 0x41}, // E (69)
    {0x7F, 0x09, 0x09, 0x09, 0x01}, // F (70)
    {0x3E, 0x41, 0x49, 0x49, 0x7A}, // G (71)
    {0x7F, 0x08, 0x08, 0x08, 0x7F}, // H (72)
    {0x00, 0x41, 0x7F, 0x41, 0x00}, // I (73)
    {0x20, 0x40, 0x41, 0x3F, 0x01}, // J (74)
    {0x7F, 0x08, 0x14, 0x22, 0x41}, // K (75)
    {0x7F, 0x40, 0x40, 0x40, 0x40}, // L (76)
    {0x7F, 0x02, 0x0C, 0x02, 0x7F}, // M (77)
    {0x7F, 0x04, 0x08, 0x10, 0x7F}, // N (78)
    {0x3E, 0x41, 0x41, 0x41, 0x3E}, // O (79)
    {0x7F, 0x09, 0x09, 0x09, 0x06}, // P (80)
    {0x3E, 0x41, 0x51, 0x21, 0x5E}, // Q (81)
    {0x7F, 0x09, 0x19, 0x29, 0x46}, // R (82)
    {0x46, 0x49, 0x49, 0x49, 0x31}, // S (83)
    {0x01, 0x01, 0x7F, 0x01, 0x01}, // T (84)
    {0x3F, 0x40, 0x40, 0x40, 0x3F}, // U (85)
    {0x1F, 0x20, 0x40, 0x20, 0x1F}, // V (86)
    {0x3F, 0x40, 0x38, 0x40, 0x3F}, // W (87)
    {0x63, 0x14, 0x08, 0x14, 0x63}, // X (88)
    {0x07, 0x08, 0x70, 0x08, 0x07}, // Y (89)
    {0x61, 0x51, 0x49, 0x45, 0x43}, // Z (90)
    {0x00, 0x7F, 0x41, 0x41, 0x00}, // [ (91)
    {0x02, 0x04, 0x08, 0x10, 0x20}, // \ (92)
    {0x00, 0x41, 0x41, 0x7F, 0x00}, // ] (93)
    {0x04, 0x02, 0x01, 0x02, 0x04}, // ^ (94)
    {0x40, 0x40, 0x40, 0x40, 0x40}, // _ (95)
    {0x00, 0x01, 0x02, 0x04, 0x00}, // ` (96)
    {0x20, 0x54, 0x54, 0x54, 0x78}, // a (97)
    {0x7F, 0x48, 0x44, 0x44, 0x38}, // b (98)
    {0x38, 0x44, 0x44, 0x44, 0x20}, // c (99)
    {0x38, 0x44, 0x44, 0x48, 0x7F}, // d (100)
    {0x38, 0x54, 0x54, 0x54, 0x18}, // e (101)
    {0x08, 0x7E, 0x09, 0x01, 0x02}, // f (102)
    {0x18, 0xA4, 0xA4, 0xA4, 0x7C}, // g (103)
    {0x7F, 0x08, 0x04, 0x04, 0x78}, // h (104)
    {0x00, 0x44, 0x7D, 0x40, 0x00}, // i (105)
    {0x40, 0x80, 0x84, 0x7D, 0x00}, // j (106)
    {0x7F, 0x10, 0x28, 0x44, 0x00}, // k (107)
    {0x00, 0x41, 0x7F, 0x40, 0x00}, // l (108)
    {0x7C, 0x04, 0x18, 0x04, 0x78}, // m (109)
    {0x7C, 0x08, 0x04, 0x04, 0x78}, // n (110)
    {0x38, 0x44, 0x44, 0x44, 0x38}, // o (111)
    {0xFC, 0x24, 0x24, 0x24, 0x18}, // p (112)
    {0x18, 0x24, 0x24, 0x18, 0xFC}, // q (113)
    {0x7C, 0x08, 0x04, 0x04, 0x08}, // r (114)
    {0x48, 0x54, 0x54, 0x54, 0x20}, // s (115)
    {0x04, 0x3F, 0x44, 0x40, 0x20}, // t (116)
    {0x3C, 0x40, 0x40, 0x20, 0x7C}, // u (117)
    {0x1C, 0x20, 0x40, 0x20, 0x1C}, // v (118)
    {0x3C, 0x40, 0x30, 0x40, 0x3C}, // w (119)
    {0x44, 0x28, 0x10, 0x28, 0x44}, // x (120)
    {0x1C, 0xA0, 0xA0, 0xA0, 0x7C}, // y (121)
    {0x44, 0x64, 0x54, 0x4C, 0x44}, // z (122)
};

// Send command to SSD1306
static esp_err_t ssd1306_send_command(uint8_t cmd)
{
    uint8_t data[2] = {0x00, cmd}; // 0x00 = command mode
    return i2c_bus_write_raw(SSD1306_I2C_ADDR, data, 2);
}

// Send data to SSD1306
static esp_err_t ssd1306_send_data(const uint8_t *data, size_t len)
{
    uint8_t buffer[len + 1];
    buffer[0] = 0x40; // 0x40 = data mode
    memcpy(&buffer[1], data, len);
    
    return i2c_bus_write_raw(SSD1306_I2C_ADDR, buffer, len + 1);
}

esp_err_t ssd1306_init(void)
{
    esp_err_t ret;
    
    // Initialization sequence for 128x64 OLED
    const uint8_t init_cmds[] = {
        SSD1306_CMD_DISPLAY_OFF,        // Display OFF
        0x20, 0x00,                     // Memory addressing mode: horizontal
        0xB0,                           // Set page start address
        0xC8,                           // COM scan direction
        0x00,                           // Set low column address
        0x10,                           // Set high column address
        0x40,                           // Set start line address
        SSD1306_CMD_SET_CONTRAST, 0xFF, // Set contrast
        0xA1,                           // Segment re-map
        SSD1306_CMD_INVERT_OFF,         // Normal display
        0xA8, 0x3F,                     // Multiplex ratio
        0xA4,                           // Display all on resume
        0xD3, 0x00,                     // Display offset
        0xD5, 0x80,                     // Display clock divide
        0xD9, 0xF1,                     // Pre-charge period
        0xDA, 0x12,                     // COM pins configuration
        0xDB, 0x40,                     // VCOMH deselect level
        0x8D, 0x14,                     // Charge pump enable
        SSD1306_CMD_DISPLAY_ON          // Display ON
    };
    
    for (size_t i = 0; i < sizeof(init_cmds); i++) {
        ret = ssd1306_send_command(init_cmds[i]);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to send init command[%d]", i);
            return ret;
        }
    }
    
    // Clear display
    memset(ssd1306_buffer, 0, sizeof(ssd1306_buffer));
    ret = ssd1306_display();
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "SSD1306 initialized successfully");
    }
    
    return ret;
}

esp_err_t ssd1306_clear(void)
{
    memset(ssd1306_buffer, 0, sizeof(ssd1306_buffer));
    return ESP_OK;
}

esp_err_t ssd1306_display(void)
{
    esp_err_t ret;
    
    // Set column address
    ret = ssd1306_send_command(0x21); // Column addr
    if (ret != ESP_OK) return ret;
    ret = ssd1306_send_command(0);    // Column start
    if (ret != ESP_OK) return ret;
    ret = ssd1306_send_command(SSD1306_WIDTH - 1); // Column end
    if (ret != ESP_OK) return ret;
    
    // Set page address
    ret = ssd1306_send_command(0x22); // Page addr
    if (ret != ESP_OK) return ret;
    ret = ssd1306_send_command(0);    // Page start
    if (ret != ESP_OK) return ret;
    ret = ssd1306_send_command((SSD1306_HEIGHT / 8) - 1); // Page end
    if (ret != ESP_OK) return ret;
    
    // Send buffer in chunks (I2C has size limit)
    // Giảm chunk size để tránh timeout và tăng độ ổn định
    const size_t chunk_size = 32;  // Giảm từ 128 xuống 32 bytes
    for (size_t i = 0; i < sizeof(ssd1306_buffer); i += chunk_size) {
        size_t len = (i + chunk_size > sizeof(ssd1306_buffer)) ?
                     (sizeof(ssd1306_buffer) - i) : chunk_size;
        ret = ssd1306_send_data(&ssd1306_buffer[i], len);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to send buffer chunk at %d", i);
            // Retry once
            vTaskDelay(pdMS_TO_TICKS(10));
            ret = ssd1306_send_data(&ssd1306_buffer[i], len);
            if (ret != ESP_OK) {
                return ret;
            }
        }
        // Small delay between chunks để tránh quá tải I2C bus
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    
    return ESP_OK;
}

void ssd1306_draw_pixel(int16_t x, int16_t y, bool color)
{
    if (x < 0 || x >= SSD1306_WIDTH || y < 0 || y >= SSD1306_HEIGHT) {
        return;
    }
    
    if (color) {
        ssd1306_buffer[x + (y / 8) * SSD1306_WIDTH] |= (1 << (y % 8));
    } else {
        ssd1306_buffer[x + (y / 8) * SSD1306_WIDTH] &= ~(1 << (y % 8));
    }
}

void ssd1306_draw_line(int16_t x0, int16_t y0, int16_t x1, int16_t y1, bool color)
{
    int16_t dx = abs(x1 - x0);
    int16_t dy = abs(y1 - y0);
    int16_t sx = (x0 < x1) ? 1 : -1;
    int16_t sy = (y0 < y1) ? 1 : -1;
    int16_t err = dx - dy;
    
    while (1) {
        ssd1306_draw_pixel(x0, y0, color);
        
        if (x0 == x1 && y0 == y1) break;
        
        int16_t e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x0 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y0 += sy;
        }
    }
}

void ssd1306_draw_rect(int16_t x, int16_t y, int16_t w, int16_t h, bool color)
{
    ssd1306_draw_line(x, y, x + w - 1, y, color);
    ssd1306_draw_line(x + w - 1, y, x + w - 1, y + h - 1, color);
    ssd1306_draw_line(x + w - 1, y + h - 1, x, y + h - 1, color);
    ssd1306_draw_line(x, y + h - 1, x, y, color);
}

void ssd1306_fill_rect(int16_t x, int16_t y, int16_t w, int16_t h, bool color)
{
    for (int16_t i = x; i < x + w; i++) {
        for (int16_t j = y; j < y + h; j++) {
            ssd1306_draw_pixel(i, j, color);
        }
    }
}

void ssd1306_draw_char(int16_t x, int16_t y, char c, 
                       ssd1306_text_size_t size, bool color)
{
    if (c < ' ' || c > 'z') return;
    
    const uint8_t *glyph = font5x7[c - ' '];
    
    // Font 5x7: 5 columns, 7 rows
    // Numbers use LSB at bottom, letters use MSB at top
    bool is_letter = (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z');
    
    for (uint8_t i = 0; i < 5; i++) {
        uint8_t line = glyph[i];
        
        // Draw 7 rows
        for (uint8_t j = 0; j < 7; j++) {
            bool pixel_on = false;
            uint8_t draw_y = y + j; // Default: draw from top to bottom
            
            if (is_letter) {
                // For letters: read from MSB (bit 6) and flip vertically
                // Read bit (6-j) but draw at position (6-j) to flip
                pixel_on = (line & (1 << (6 - j))) != 0;
                draw_y = y + (6 - j); // Flip vertically: draw from bottom to top
            } else {
                // For numbers and special chars: read from LSB (bit 0) up
                pixel_on = (line & (1 << j)) != 0;
            }
            
            if (pixel_on) {
                if (size == 1) {
                    ssd1306_draw_pixel(x + i, draw_y, color);
                } else {
                    ssd1306_fill_rect(x + i * size, draw_y * size, 
                                     size, size, color);
                }
            }
        }
    }
}

void ssd1306_draw_string(int16_t x, int16_t y, const char *str,
                         ssd1306_text_size_t size, bool color)
{
    int16_t cursor_x = x;
    
    while (*str) {
        if (*str == '\n') {
            cursor_x = x;
            y += 8 * size;
        } else {
            ssd1306_draw_char(cursor_x, y, *str, size, color);
            cursor_x += 6 * size;
            
            if (cursor_x > SSD1306_WIDTH - 6 * size) {
                cursor_x = x;
                y += 8 * size;
            }
        }
        str++;
    }
}

esp_err_t ssd1306_set_display(bool on)
{
    return ssd1306_send_command(on ? SSD1306_CMD_DISPLAY_ON : 
                                     SSD1306_CMD_DISPLAY_OFF);
}

esp_err_t ssd1306_set_contrast(uint8_t contrast)
{
    esp_err_t ret = ssd1306_send_command(SSD1306_CMD_SET_CONTRAST);
    if (ret == ESP_OK) {
        ret = ssd1306_send_command(contrast);
    }
    return ret;
}