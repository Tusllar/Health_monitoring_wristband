// components/mpu6050/include/mpu6050.h
#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>
#include "esp_err.h"

// Địa chỉ I2C của MPU6050
#define MPU6050_I2C_ADDR            0x68

// Registers
#define MPU6050_REG_SMPLRT_DIV      0x19
#define MPU6050_REG_CONFIG          0x1A
#define MPU6050_REG_GYRO_CONFIG     0x1B
#define MPU6050_REG_ACCEL_CONFIG    0x1C
#define MPU6050_REG_INT_ENABLE      0x38
#define MPU6050_REG_ACCEL_XOUT_H    0x3B
#define MPU6050_REG_TEMP_OUT_H      0x41
#define MPU6050_REG_GYRO_XOUT_H     0x43
#define MPU6050_REG_PWR_MGMT_1      0x6B
#define MPU6050_REG_WHO_AM_I        0x75

// Gyroscope full scale range
typedef enum {
    MPU6050_GYRO_FS_250  = 0x00,  // ±250°/s
    MPU6050_GYRO_FS_500  = 0x08,  // ±500°/s
    MPU6050_GYRO_FS_1000 = 0x10,  // ±1000°/s
    MPU6050_GYRO_FS_2000 = 0x18   // ±2000°/s
} mpu6050_gyro_fs_t;

// Accelerometer full scale range
typedef enum {
    MPU6050_ACCEL_FS_2  = 0x00,   // ±2g
    MPU6050_ACCEL_FS_4  = 0x08,   // ±4g
    MPU6050_ACCEL_FS_8  = 0x10,   // ±8g
    MPU6050_ACCEL_FS_16 = 0x18    // ±16g
} mpu6050_accel_fs_t;

// Raw data structure
typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    int16_t temp_raw;
} mpu6050_raw_data_t;

// Processed data structure
typedef struct {
    float accel_x;      // g
    float accel_y;      // g
    float accel_z;      // g
    float gyro_x;       // °/s
    float gyro_y;       // °/s
    float gyro_z;       // °/s
    float temperature;  // °C
} mpu6050_data_t;

/**
 * @brief Khởi tạo cảm biến MPU6050
 * @return ESP_OK nếu thành công
 */
esp_err_t mpu6050_init(void);

/**
 * @brief Đọc dữ liệu thô từ cảm biến
 * @param data Con trỏ lưu dữ liệu thô
 * @return ESP_OK nếu thành công
 */
esp_err_t mpu6050_read_raw(mpu6050_raw_data_t *data);

/**
 * @brief Đọc và xử lý dữ liệu từ cảm biến
 * @param data Con trỏ lưu dữ liệu đã xử lý
 * @return ESP_OK nếu thành công
 */
esp_err_t mpu6050_read_data(mpu6050_data_t *data);

/**
 * @brief Đọc nhiệt độ từ cảm biến
 * @param temp_c Con trỏ lưu nhiệt độ (°C)
 * @return ESP_OK nếu thành công
 */
esp_err_t mpu6050_read_temperature(float *temp_c);

/**
 * @brief Cấu hình gyroscope range
 * @param fs_range Full scale range
 * @return ESP_OK nếu thành công
 */
esp_err_t mpu6050_set_gyro_range(mpu6050_gyro_fs_t fs_range);

/**
 * @brief Cấu hình accelerometer range
 * @param fs_range Full scale range
 * @return ESP_OK nếu thành công
 */
esp_err_t mpu6050_set_accel_range(mpu6050_accel_fs_t fs_range);

#endif // MPU6050_H