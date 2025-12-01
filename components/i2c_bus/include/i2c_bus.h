// components/i2c_bus/include/i2c_bus.h
#ifndef I2C_BUS_H
#define I2C_BUS_H

#include "driver/i2c.h"
#include "esp_err.h"

// Cấu hình I2C cho ESP32-C3 Mini-1
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_SDA_IO           4
#define I2C_MASTER_SCL_IO           5
#define I2C_MASTER_FREQ_HZ          100000  // 100kHz cho ổn định
#define I2C_MASTER_TIMEOUT_MS       1000

/**
 * @brief Khởi tạo I2C master bus
 * @return ESP_OK nếu thành công
 */
esp_err_t i2c_bus_init(void);

/**
 * @brief Đọc dữ liệu từ I2C device
 * @param device_addr Địa chỉ I2C của device (7-bit)
 * @param reg_addr Địa chỉ register cần đọc
 * @param data Con trỏ buffer để lưu dữ liệu đọc được
 * @param len Số byte cần đọc
 * @return ESP_OK nếu thành công
 */
esp_err_t i2c_bus_read_bytes(uint8_t device_addr, uint8_t reg_addr, 
                              uint8_t *data, size_t len);

/**
 * @brief Ghi dữ liệu vào I2C device
 * @param device_addr Địa chỉ I2C của device (7-bit)
 * @param reg_addr Địa chỉ register cần ghi
 * @param data Con trỏ dữ liệu cần ghi
 * @param len Số byte cần ghi
 * @return ESP_OK nếu thành công
 */
esp_err_t i2c_bus_write_bytes(uint8_t device_addr, uint8_t reg_addr,
                               const uint8_t *data, size_t len);

/**
 * @brief Ghi 1 byte vào register
 */
esp_err_t i2c_bus_write_byte(uint8_t device_addr, uint8_t reg_addr, uint8_t data);

/**
 * @brief Đọc 1 byte từ register
 */
esp_err_t i2c_bus_read_byte(uint8_t device_addr, uint8_t reg_addr, uint8_t *data);

/**
 * @brief Ghi dữ liệu trực tiếp (không có register address)
 * @param device_addr Địa chỉ I2C của device (7-bit)
 * @param data Con trỏ dữ liệu cần ghi
 * @param len Số byte cần ghi
 * @return ESP_OK nếu thành công
 */
esp_err_t i2c_bus_write_raw(uint8_t device_addr, const uint8_t *data, size_t len);

/**
 * @brief Scan tất cả địa chỉ I2C trên bus
 */
void i2c_bus_scan(void);

#endif // I2C_BUS_H