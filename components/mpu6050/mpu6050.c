// components/mpu6050/mpu6050.c
#include "mpu6050.h"
#include "i2c_bus.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "MPU6050";

// Scaling factors
static float accel_scale = 16384.0f;  // For ±2g
static float gyro_scale = 131.0f;     // For ±250°/s

// Địa chỉ I2C thực tế của MPU6050 (có thể là 0x68 hoặc 0x69)
static uint8_t mpu6050_i2c_addr = MPU6050_I2C_ADDR;

esp_err_t mpu6050_init(void)
{
    esp_err_t ret;
    uint8_t who_am_i;
    bool found = false;
    
    // Delay để đảm bảo sensor đã sẵn sàng
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Thử cả 2 địa chỉ I2C có thể của MPU6050 (0x68 và 0x69)
    uint8_t possible_addrs[] = {0x68, 0x69};
    
    ESP_LOGI(TAG, "Searching for MPU6050...");
    
    for (int addr_idx = 0; addr_idx < 2; addr_idx++) {
        uint8_t test_addr = possible_addrs[addr_idx];
        ESP_LOGI(TAG, "Trying address 0x%02X...", test_addr);
        
        // Retry đọc WHO_AM_I (tối đa 3 lần cho mỗi địa chỉ)
        int retry_count = 3;
        for (int i = 0; i < retry_count; i++) {
            ret = i2c_bus_read_byte(test_addr, MPU6050_REG_WHO_AM_I, &who_am_i);
            if (ret == ESP_OK) {
                // Kiểm tra WHO_AM_I: 0x68, 0x71 (MPU6050) hoặc 0x70 (MPU6500/tương thích)
                if (who_am_i == 0x68 || who_am_i == 0x71 || who_am_i == 0x70) {
                    mpu6050_i2c_addr = test_addr;
                    found = true;
                    if (who_am_i == 0x70) {
                        ESP_LOGI(TAG, "MPU-compatible sensor found at 0x%02X (WHO_AM_I=0x%02X, likely MPU6500)", 
                                 test_addr, who_am_i);
                    } else {
                        ESP_LOGI(TAG, "MPU6050 found at address 0x%02X, WHO_AM_I=0x%02X", 
                                 test_addr, who_am_i);
                    }
                    break;
                } else {
                    // Thiết bị khác ở địa chỉ này, không phải MPU6050
                    // Chỉ log một lần, không spam
                    if (i == 0) {
                        ESP_LOGI(TAG, "Device at 0x%02X is not MPU6050 (WHO_AM_I=0x%02X)", 
                                 test_addr, who_am_i);
                    }
                }
            }
            if (i < retry_count - 1) {
                // Chỉ log retry nếu đây là lần thử đầu tiên
                if (i == 0) {
                    ESP_LOGD(TAG, "Retrying WHO_AM_I read from 0x%02X...", test_addr);
                }
                vTaskDelay(pdMS_TO_TICKS(50));
            }
        }
        
        if (found) {
            break;
        }
    }
    
    if (!found) {
        ESP_LOGI(TAG, "MPU6050/MPU6500 sensor not found");
        ESP_LOGI(TAG, "Motion data will be unavailable");
        ESP_LOGI(TAG, "Note: This is normal if motion sensor is not installed. System will continue without it.");
        return ESP_ERR_NOT_FOUND;
    }
    
    // Wake up sensor (clear sleep bit)
    ret = i2c_bus_write_byte(mpu6050_i2c_addr, MPU6050_REG_PWR_MGMT_1, 0x00);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to wake up sensor");
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Set sample rate divider (1kHz / (1 + 9) = 100Hz)
    ret = i2c_bus_write_byte(mpu6050_i2c_addr, MPU6050_REG_SMPLRT_DIV, 0x09);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set sample rate");
        return ret;
    }
    
    // Configure DLPF (Digital Low Pass Filter) - 44Hz bandwidth
    ret = i2c_bus_write_byte(mpu6050_i2c_addr, MPU6050_REG_CONFIG, 0x03);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure DLPF");
        return ret;
    }
    
    // Set gyro range to ±250°/s
    ret = mpu6050_set_gyro_range(MPU6050_GYRO_FS_250);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set gyro range");
        return ret;
    }
    
    // Set accel range to ±2g
    ret = mpu6050_set_accel_range(MPU6050_ACCEL_FS_2);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set accel range");
        return ret;
    }
    
    ESP_LOGI(TAG, "MPU6050 initialized successfully");
    return ESP_OK;
}

esp_err_t mpu6050_set_gyro_range(mpu6050_gyro_fs_t fs_range)
{
    esp_err_t ret = i2c_bus_write_byte(mpu6050_i2c_addr, 
                                       MPU6050_REG_GYRO_CONFIG, 
                                       fs_range);
    if (ret == ESP_OK) {
        switch (fs_range) {
            case MPU6050_GYRO_FS_250:  gyro_scale = 131.0f; break;
            case MPU6050_GYRO_FS_500:  gyro_scale = 65.5f; break;
            case MPU6050_GYRO_FS_1000: gyro_scale = 32.8f; break;
            case MPU6050_GYRO_FS_2000: gyro_scale = 16.4f; break;
        }
    }
    return ret;
}

esp_err_t mpu6050_set_accel_range(mpu6050_accel_fs_t fs_range)
{
    esp_err_t ret = i2c_bus_write_byte(mpu6050_i2c_addr,
                                       MPU6050_REG_ACCEL_CONFIG,
                                       fs_range);
    if (ret == ESP_OK) {
        switch (fs_range) {
            case MPU6050_ACCEL_FS_2:  accel_scale = 16384.0f; break;
            case MPU6050_ACCEL_FS_4:  accel_scale = 8192.0f; break;
            case MPU6050_ACCEL_FS_8:  accel_scale = 4096.0f; break;
            case MPU6050_ACCEL_FS_16: accel_scale = 2048.0f; break;
        }
    }
    return ret;
}

esp_err_t mpu6050_read_raw(mpu6050_raw_data_t *data)
{
    if (data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    uint8_t buffer[14];
    
    // Đọc tất cả dữ liệu từ ACCEL_XOUT_H (14 bytes liên tiếp)
    esp_err_t ret = i2c_bus_read_bytes(mpu6050_i2c_addr, 
                                       MPU6050_REG_ACCEL_XOUT_H,
                                       buffer, 14);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Parse data
    data->accel_x = (int16_t)((buffer[0] << 8) | buffer[1]);
    data->accel_y = (int16_t)((buffer[2] << 8) | buffer[3]);
    data->accel_z = (int16_t)((buffer[4] << 8) | buffer[5]);
    data->temp_raw = (int16_t)((buffer[6] << 8) | buffer[7]);
    data->gyro_x = (int16_t)((buffer[8] << 8) | buffer[9]);
    data->gyro_y = (int16_t)((buffer[10] << 8) | buffer[11]);
    data->gyro_z = (int16_t)((buffer[12] << 8) | buffer[13]);
    
    return ESP_OK;
}

esp_err_t mpu6050_read_data(mpu6050_data_t *data)
{
    if (data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    mpu6050_raw_data_t raw;
    esp_err_t ret = mpu6050_read_raw(&raw);
    
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Convert to g and °/s
    data->accel_x = (float)raw.accel_x / accel_scale;
    data->accel_y = (float)raw.accel_y / accel_scale;
    data->accel_z = (float)raw.accel_z / accel_scale;
    
    data->gyro_x = (float)raw.gyro_x / gyro_scale;
    data->gyro_y = (float)raw.gyro_y / gyro_scale;
    data->gyro_z = (float)raw.gyro_z / gyro_scale;
    
    // Convert temperature: Temp = (TEMP_OUT / 340) + 36.53
    data->temperature = ((float)raw.temp_raw / 340.0f) + 36.53f;
    
    return ESP_OK;
}

esp_err_t mpu6050_read_temperature(float *temp_c)
{
    if (temp_c == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    uint8_t buffer[2];
    esp_err_t ret = i2c_bus_read_bytes(mpu6050_i2c_addr,
                                       MPU6050_REG_TEMP_OUT_H,
                                       buffer, 2);
    if (ret != ESP_OK) {
        return ret;
    }
    
    int16_t temp_raw = (int16_t)((buffer[0] << 8) | buffer[1]);
    *temp_c = ((float)temp_raw / 340.0f) + 36.53f;
    
    return ESP_OK;
}