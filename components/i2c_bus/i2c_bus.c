// components/i2c_bus/i2c_bus.c
#include "i2c_bus.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/i2c.h"
#include <string.h>

static const char *TAG = "I2C_BUS";
static SemaphoreHandle_t i2c_mutex = NULL;

esp_err_t i2c_bus_init(void)
{
    // Tạo mutex để bảo vệ I2C bus khỏi truy cập đồng thời
    if (i2c_mutex == NULL) {
        i2c_mutex = xSemaphoreCreateMutex();
        if (i2c_mutex == NULL) {
            ESP_LOGE(TAG, "Failed to create I2C mutex!");
            return ESP_ERR_NO_MEM;
        }
    }
    
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    
    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C param config failed: %s", esp_err_to_name(err));
        return err;
    }
    
    err = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(err));
        return err;
    }
    
    ESP_LOGI(TAG, "I2C initialized successfully");
    ESP_LOGI(TAG, "SDA: GPIO%d, SCL: GPIO%d, Freq: %d Hz", 
             I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO, I2C_MASTER_FREQ_HZ);
    
    return ESP_OK;
}

esp_err_t i2c_bus_read_bytes(uint8_t device_addr, uint8_t reg_addr, 
                              uint8_t *data, size_t len)
{
    if (data == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Lấy mutex để đảm bảo truy cập độc quyền
    if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take I2C mutex");
        return ESP_ERR_TIMEOUT;
    }
    
    esp_err_t ret = i2c_master_write_read_device(I2C_MASTER_NUM, 
                                                  device_addr,
                                                  &reg_addr, 1,
                                                  data, len,
                                                  pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    
    // Giải phóng mutex
    xSemaphoreGive(i2c_mutex);
    
    return ret;
}

esp_err_t i2c_bus_write_bytes(uint8_t device_addr, uint8_t reg_addr,
                               const uint8_t *data, size_t len)
{
    if (data == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Lấy mutex để đảm bảo truy cập độc quyền
    if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take I2C mutex");
        return ESP_ERR_TIMEOUT;
    }
    
    uint8_t write_buf[len + 1];
    write_buf[0] = reg_addr;
    memcpy(&write_buf[1], data, len);
    
    esp_err_t ret = i2c_master_write_to_device(I2C_MASTER_NUM,
                                                device_addr,
                                                write_buf, len + 1,
                                                pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    
    // Giải phóng mutex
    xSemaphoreGive(i2c_mutex);
    
    return ret;
}

esp_err_t i2c_bus_write_byte(uint8_t device_addr, uint8_t reg_addr, uint8_t data)
{
    return i2c_bus_write_bytes(device_addr, reg_addr, &data, 1);
}

esp_err_t i2c_bus_read_byte(uint8_t device_addr, uint8_t reg_addr, uint8_t *data)
{
    return i2c_bus_read_bytes(device_addr, reg_addr, data, 1);
}

// Hàm ghi dữ liệu trực tiếp (không có register address) - dùng cho SSD1306
esp_err_t i2c_bus_write_raw(uint8_t device_addr, const uint8_t *data, size_t len)
{
    if (data == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Lấy mutex để đảm bảo truy cập độc quyền
    if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take I2C mutex");
        return ESP_ERR_TIMEOUT;
    }
    
    esp_err_t ret = i2c_master_write_to_device(I2C_MASTER_NUM,
                                                device_addr,
                                                data, len,
                                                pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    
    // Giải phóng mutex
    xSemaphoreGive(i2c_mutex);
    
    return ret;
}

void i2c_bus_scan(void)
{
    ESP_LOGI(TAG, "Scanning I2C bus...");
    uint8_t found = 0;
    
    // Lấy mutex để scan an toàn
    if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(5000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take I2C mutex for scan");
        return;
    }
    
    // Scan từ 0x08 đến 0x77 (địa chỉ I2C hợp lệ, bỏ qua 0x00-0x07 và 0x78-0x7F)
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        
        esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(50));
        i2c_cmd_link_delete(cmd);
        
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Found device at address: 0x%02X", addr);
            found++;
        }
    }
    
    xSemaphoreGive(i2c_mutex);
    
    if (found == 0) {
        ESP_LOGW(TAG, "No I2C devices found!");
    } else {
        ESP_LOGI(TAG, "Found %d I2C device(s)", found);
    }
}