// components/ssd1306/include/ssd1306.h
#ifndef SSD1306_H
#define SSD1306_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

// Địa chỉ I2C của SSD1306
#define SSD1306_I2C_ADDR            0x3C

// Screen dimensions (adjust based on your OLED)
#define SSD1306_WIDTH               128
#define SSD1306_HEIGHT              64

// Commands
#define SSD1306_CMD_DISPLAY_OFF     0xAE
#define SSD1306_CMD_DISPLAY_ON      0xAF
#define SSD1306_CMD_SET_CONTRAST    0x81
#define SSD1306_CMD_INVERT_OFF      0xA6
#define SSD1306_CMD_INVERT_ON       0xA7

// Text size
typedef enum {
    SSD1306_TEXT_SIZE_SMALL  = 1,  // 8x8
    SSD1306_TEXT_SIZE_MEDIUM = 2,  // 16x16
    SSD1306_TEXT_SIZE_LARGE  = 3   // 24x24
} ssd1306_text_size_t;

/**
 * @brief Khởi tạo màn hình OLED SSD1306
 * @return ESP_OK nếu thành công
 */
esp_err_t ssd1306_init(void);

/**
 * @brief Xóa toàn bộ màn hình
 * @return ESP_OK nếu thành công
 */
esp_err_t ssd1306_clear(void);

/**
 * @brief Cập nhật màn hình từ buffer
 * @return ESP_OK nếu thành công
 */
esp_err_t ssd1306_display(void);

/**
 * @brief Vẽ pixel tại vị trí (x, y)
 * @param x Tọa độ X
 * @param y Tọa độ Y
 * @param color true = white, false = black
 */
void ssd1306_draw_pixel(int16_t x, int16_t y, bool color);

/**
 * @brief Vẽ đường thẳng
 * @param x0, y0 Điểm bắt đầu
 * @param x1, y1 Điểm kết thúc
 * @param color true = white, false = black
 */
void ssd1306_draw_line(int16_t x0, int16_t y0, int16_t x1, int16_t y1, bool color);

/**
 * @brief Vẽ hình chữ nhật
 * @param x, y Góc trên bên trái
 * @param w Chiều rộng
 * @param h Chiều cao
 * @param color true = white, false = black
 */
void ssd1306_draw_rect(int16_t x, int16_t y, int16_t w, int16_t h, bool color);

/**
 * @brief Vẽ hình chữ nhật đặc
 */
void ssd1306_fill_rect(int16_t x, int16_t y, int16_t w, int16_t h, bool color);

/**
 * @brief Hiển thị ký tự
 * @param x, y Vị trí
 * @param c Ký tự
 * @param size Kích thước
 * @param color true = white, false = black
 */
void ssd1306_draw_char(int16_t x, int16_t y, char c, 
                       ssd1306_text_size_t size, bool color);

/**
 * @brief Hiển thị chuỗi
 * @param x, y Vị trí
 * @param str Chuỗi cần hiển thị
 * @param size Kích thước
 * @param color true = white, false = black
 */
void ssd1306_draw_string(int16_t x, int16_t y, const char *str,
                         ssd1306_text_size_t size, bool color);

/**
 * @brief Bật/tắt màn hình
 * @param on true = bật, false = tắt
 * @return ESP_OK nếu thành công
 */
esp_err_t ssd1306_set_display(bool on);

/**
 * @brief Đặt độ tương phản
 * @param contrast Giá trị 0-255
 * @return ESP_OK nếu thành công
 */
esp_err_t ssd1306_set_contrast(uint8_t contrast);

#endif // SSD1306_H