// main/main.c
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_netif.h"
#include "driver/gpio.h"

// Include tất cả components
#include "i2c_bus.h"
#include "max30100.h"
#include "mpu6050.h"
#include "ssd1306.h"
#include "esp32c3_wifi.h"
#include "http_server_app.h"
// #include "mqtt_client_app.h"





static const char *TAG = "MAIN";

// ========================= BUZZER CONFIGURATION =========================
#define BUZZER_GPIO           10      // GPIO pin cho buzzer (có thể thay đổi)
#define HR_MIN_THRESHOLD      50     // Nhịp tim tối thiểu (BPM)
#define HR_MAX_THRESHOLD      120    // Nhịp tim tối đa (BPM)
#define SPO2_MIN_THRESHOLD    90     // SpO2 tối thiểu (%)
#define BUZZER_BEEP_DURATION  200    // Thời gian beep (ms)
#define BUZZER_SILENCE_DURATION 100  // Thời gian im lặng giữa các beep (ms)

// ========================= MOTION DETECTION CONFIGURATION =========================
#define FREE_FALL_THRESHOLD       0.5f    // G - phát hiện rơi tự do
#define IMPACT_THRESHOLD          2.5f    // G - phát hiện va đập
#define IMPACT_JERK_THRESHOLD      5.0f    // G/s - tốc độ thay đổi acceleration
#define RUNNING_ACCEL_THRESHOLD   1.5f    // G - ngưỡng chạy
#define RUNNING_FREQ_MIN          2.0f    // Hz - tần số tối thiểu cho chạy
#define RUNNING_FREQ_MAX          5.0f    // Hz - tần số tối đa cho chạy
#define TREMOR_ACCEL_MIN          0.1f    // G - biên độ tối thiểu cho rung tay
#define TREMOR_ACCEL_MAX          0.5f    // G - biên độ tối đa cho rung tay
#define TREMOR_FREQ_MIN           3.0f    // Hz - tần số tối thiểu cho rung tay
#define TREMOR_FREQ_MAX           12.0f   // Hz - tần số tối đa cho rung tay
#define MOTION_HISTORY_SIZE       20      // Số mẫu lưu trữ cho phân tích tần số
#define MOTION_SAMPLE_RATE        10      // Hz - tần số lấy mẫu (100ms)

// ========================= SHARED DATA STRUCTURE =========================
typedef struct {
    float heart_rate;
    float spo2;
    bool valid;
    uint32_t last_update;
    // MPU6050 data
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
    float mpu_temp;
    bool mpu_valid;
} shared_sensor_data_t;

// Export shared data for HTTP server
shared_sensor_data_t shared_data = {0};
SemaphoreHandle_t data_mutex = NULL;

// Task handle for buzzer task to use Task Notifications
static TaskHandle_t buzzer_task_handle = NULL;

// ========================= ALERT HISTORY (FOR WEB UI) =========================
#define ALERT_HISTORY_SIZE 20

typedef struct {
    uint32_t timestamp_ms;      // thời gian theo ms từ khi boot (xTaskGetTickCount)
    char message[64];           // nội dung cảnh báo (té ngã, va đập, ...)
} alert_event_t;

// Vòng đệm lưu history, được đọc từ HTTP server và hiển thị trên web
alert_event_t alert_history[ALERT_HISTORY_SIZE] = {0};
uint32_t alert_history_count = 0;
uint32_t alert_history_head = 0;   // vị trí phần tử mới nhất sẽ được ghi tiếp theo

static void add_alert_event(const char *msg)
{
    if (msg == NULL || data_mutex == NULL) {
        return;
    }

    if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        uint32_t idx = alert_history_head;
        alert_history[idx].timestamp_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
        strncpy(alert_history[idx].message, msg, sizeof(alert_history[idx].message) - 1);
        alert_history[idx].message[sizeof(alert_history[idx].message) - 1] = '\0';

        alert_history_head = (alert_history_head + 1) % ALERT_HISTORY_SIZE;
        if (alert_history_count < ALERT_HISTORY_SIZE) {
            alert_history_count++;
        }

        xSemaphoreGive(data_mutex);
    }
}



// ========================= MAX30100 TASK =========================
void max30100_task(void *pvParameters)
{
    ESP_LOGI(TAG, "MAX30100 task started");

    while (1) {

        max30100_reading_t reading;
        esp_err_t ret = max30100_get_reading(&reading);

        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "MAX30100 read error: %d", ret);
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }


        if (reading.valid) {

            // // ⭐ Log final computed output
            // ESP_LOGI("MAX30100",
            //          "FINAL ==> SpO2: %.1f%%   HR: %.0f BPM",
            //          reading.spo2, reading.heart_rate);

            // Update shared data
            if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                shared_data.heart_rate = reading.heart_rate;
                shared_data.spo2 = reading.spo2;
                shared_data.valid = true;
                shared_data.last_update = xTaskGetTickCount() * portTICK_PERIOD_MS;
                xSemaphoreGive(data_mutex);
            }

        } else {
            // ESP_LOGW("MAX30100", "⚠ Data invalid — No pulse or weak signal");

            if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                shared_data.valid = false;
                xSemaphoreGive(data_mutex);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100)); // 10 Hz
    }
}

// ========================= MPU6050 TASK =========================
void mpu6050_task(void *pvParameters)
{
    ESP_LOGI(TAG, "MPU6050 task started");
    
    mpu6050_data_t mpu_data = {0};
    uint32_t error_count = 0;
    bool logged_initial_error = false;
    
    while (1) {
        esp_err_t ret = mpu6050_read_data(&mpu_data);
        
        if (ret == ESP_OK) {
            // Update shared data
            if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                shared_data.accel_x = mpu_data.accel_x;
                shared_data.accel_y = mpu_data.accel_y;
                shared_data.accel_z = mpu_data.accel_z;
                shared_data.gyro_x = mpu_data.gyro_x;
                shared_data.gyro_y = mpu_data.gyro_y;
                shared_data.gyro_z = mpu_data.gyro_z;
                shared_data.mpu_temp = mpu_data.temperature;
                shared_data.mpu_valid = true;
                xSemaphoreGive(data_mutex);
            }

            // Notify buzzer task that new motion data is available
            if (buzzer_task_handle != NULL) {
                xTaskNotifyGive(buzzer_task_handle);
            }

            error_count = 0; // Reset error counter on success
            logged_initial_error = false;
        } else {
            // Chỉ log lỗi lần đầu, sau đó im lặng để tránh spam log
            if (!logged_initial_error) {
                ESP_LOGI(TAG, "MPU6050 not available (sensor not connected)");
                logged_initial_error = true;
            }
            
            if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                shared_data.mpu_valid = false;
                xSemaphoreGive(data_mutex);
            }
            error_count++;
        }
        
        vTaskDelay(pdMS_TO_TICKS(100)); // 10 Hz
    }
}

// ========================= DISPLAY TASK =========================
void display_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Display task started");

    char buffer[32];
    shared_sensor_data_t local_data = {0};

    while (1) {

        if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            local_data = shared_data;
            xSemaphoreGive(data_mutex);
        }

        ssd1306_clear();

        // Header
        ssd1306_draw_string(0, 0, "Health Monitor",
                            SSD1306_TEXT_SIZE_SMALL, true);
        ssd1306_draw_line(0, 10, 127, 10, true);

        // Phần 1: Health Data (MAX30100)
        if (local_data.valid) {
            // Dòng 1: SpO2
            sprintf(buffer, "SpO2: %.0f%%", local_data.spo2);
            ssd1306_draw_string(0, 16, buffer, SSD1306_TEXT_SIZE_SMALL, true);
            
            // Dòng 2: Heart Rate
            if (local_data.heart_rate > 0) {
                sprintf(buffer, "HR:   %.0f BPM", local_data.heart_rate);
            } else {
                sprintf(buffer, "HR:   -- BPM");
            }
            ssd1306_draw_string(0, 24, buffer, SSD1306_TEXT_SIZE_SMALL, true);
        } else {
            ssd1306_draw_string(0, 16, "Place finger", SSD1306_TEXT_SIZE_SMALL, true);
            ssd1306_draw_string(0, 24, "on sensor...", SSD1306_TEXT_SIZE_SMALL, true);
        }

        // Đường kẻ phân cách
        ssd1306_draw_line(0, 33, 127, 33, true);

        // Phần 2: Motion Data (MPU6050)
        if (local_data.mpu_valid) {
            // Dòng 3: Accelerometer - format gọn (chỉ X, Y để vừa màn hình)
            sprintf(buffer, "Acc: X%.2f Y%.2f Z%.2f", 
                    local_data.accel_x, local_data.accel_y, local_data.accel_z);
            // Kiểm tra độ dài, nếu quá dài thì rút gọn
            if (strlen(buffer) > 20) {
                sprintf(buffer, "Acc: X%.1f Y%.1f Z%.1f", 
                        local_data.accel_x, local_data.accel_y, local_data.accel_z);
            }
            ssd1306_draw_string(0, 40, buffer, SSD1306_TEXT_SIZE_SMALL, true);
            
            // Dòng 4: Gyroscope - format gọn
            sprintf(buffer, "Gyr: X%.0f Y%.0f Z%.0f", 
                    local_data.gyro_x, local_data.gyro_y, local_data.gyro_z);
            // Kiểm tra độ dài
            if (strlen(buffer) > 20) {
                sprintf(buffer, "Gyr: X%.0f Y%.0f", 
                        local_data.gyro_x, local_data.gyro_y);
            }
            ssd1306_draw_string(0, 48, buffer, SSD1306_TEXT_SIZE_SMALL, true);
        } else {
            ssd1306_draw_string(0, 40, "Motion: N/A", SSD1306_TEXT_SIZE_SMALL, true);
            ssd1306_draw_string(0, 48, "MPU6050 offline", SSD1306_TEXT_SIZE_SMALL, true);
        }

        ssd1306_display();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ========================= MOTION DETECTION STRUCTURES =========================
typedef struct {
    float accel_magnitude[MOTION_HISTORY_SIZE];
    float accel_x[MOTION_HISTORY_SIZE];
    float accel_y[MOTION_HISTORY_SIZE];
    float accel_z[MOTION_HISTORY_SIZE];
    uint32_t index;
    uint32_t count;
    float last_magnitude;
    bool free_fall_detected;
    uint32_t free_fall_start_time;
} motion_history_t;

// ========================= MOTION DETECTION FUNCTIONS =========================
static float calculate_magnitude(float x, float y, float z)
{
    return sqrtf(x*x + y*y + z*z);
}

static float calculate_frequency(float *history, uint32_t count)
{
    if (count < 4) return 0.0f;
    
    // Đếm số lần đổi dấu (zero crossing) để ước tính tần số
    uint32_t zero_crossings = 0;
    float mean = 0.0f;
    
    // Tính giá trị trung bình
    for (uint32_t i = 0; i < count; i++) {
        mean += history[i];
    }
    mean /= count;
    
    // Đếm zero crossings
    for (uint32_t i = 1; i < count; i++) {
        if ((history[i-1] - mean) * (history[i] - mean) < 0) {
            zero_crossings++;
        }
    }
    
    // Tần số = số zero crossings / 2 / thời gian
    float time_window = (float)count / MOTION_SAMPLE_RATE;
    return (float)zero_crossings / 2.0f / time_window;
}

static const char* detect_motion_events(motion_history_t *motion, 
                                         float accel_x, float accel_y, float accel_z)
{
    if (motion == NULL) return NULL;
    
    // Tính magnitude hiện tại
    float magnitude = calculate_magnitude(accel_x, accel_y, accel_z);
    
    // Lưu vào history
    motion->accel_magnitude[motion->index] = magnitude;
    motion->accel_x[motion->index] = accel_x;
    motion->accel_y[motion->index] = accel_y;
    motion->accel_z[motion->index] = accel_z;
    motion->index = (motion->index + 1) % MOTION_HISTORY_SIZE;
    if (motion->count < MOTION_HISTORY_SIZE) {
        motion->count++;
    }
    
    // Tính jerk (tốc độ thay đổi acceleration)
    float jerk = 0.0f;
    if (motion->count > 1) {
        uint32_t prev_idx = (motion->index - 2 + MOTION_HISTORY_SIZE) % MOTION_HISTORY_SIZE;
        jerk = fabsf(magnitude - motion->accel_magnitude[prev_idx]) * MOTION_SAMPLE_RATE;
    }
    
    // 1. PHÁT HIỆN RƠI TỰ DO (Free Fall)
    if (magnitude < FREE_FALL_THRESHOLD) {
        if (!motion->free_fall_detected) {
            motion->free_fall_detected = true;
            motion->free_fall_start_time = xTaskGetTickCount();
        }
    } else {
        // 2. PHÁT HIỆN VA ĐẬP (Impact) - sau khi rơi tự do
        if (motion->free_fall_detected && magnitude > IMPACT_THRESHOLD) {
            motion->free_fall_detected = false;
            return "TÉ NGÃ - Va đập phát hiện!";
        }
        motion->free_fall_detected = false;
    }
    
    // 3. PHÁT HIỆN VA ĐẬP ĐỘT NGỘT (Sudden Impact)
    if (jerk > IMPACT_JERK_THRESHOLD && magnitude > IMPACT_THRESHOLD) {
        return "VA ĐẬP - Chấn động mạnh!";
    }
    
    // 4. PHÁT HIỆN CHẠY (Running) - cần đủ dữ liệu để phân tích tần số
    if (motion->count >= 10) {
        float freq = calculate_frequency(motion->accel_magnitude, motion->count);
        if (magnitude > RUNNING_ACCEL_THRESHOLD && 
            freq >= RUNNING_FREQ_MIN && freq <= RUNNING_FREQ_MAX) {
            return "ĐANG CHẠY - Hoạt động mạnh";
        }
    }
    
    // 5. PHÁT HIỆN RUNG TAY (Tremor) - cần đủ dữ liệu
    if (motion->count >= 10) {
        float freq = calculate_frequency(motion->accel_magnitude, motion->count);
        float avg_magnitude = 0.0f;
        for (uint32_t i = 0; i < motion->count; i++) {
            avg_magnitude += fabsf(motion->accel_magnitude[i] - 1.0f); // Trừ 1g (gravity)
        }
        avg_magnitude /= motion->count;
        
        if (avg_magnitude >= TREMOR_ACCEL_MIN && avg_magnitude <= TREMOR_ACCEL_MAX &&
            freq >= TREMOR_FREQ_MIN && freq <= TREMOR_FREQ_MAX) {
            return "RUNG TAY - Tremor phát hiện";
        }
    }
    
    motion->last_magnitude = magnitude;
    return NULL;
}

// ========================= BUZZER TASK =========================
static void buzzer_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Buzzer task started");
    
    // Cấu hình GPIO cho buzzer
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BUZZER_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    gpio_set_level(BUZZER_GPIO, 0);  // Tắt buzzer ban đầu
    
    ESP_LOGI(TAG, "Buzzer initialized on GPIO %d", BUZZER_GPIO);
    
    // Khởi tạo motion detection
    motion_history_t motion = {0};
    motion.free_fall_detected = false;
    
    bool last_alert_state = false;
    uint32_t alert_count = 0;
    const char *last_motion_alert = NULL;
    uint32_t motion_alert_cooldown = 0;
    
    while (1) {
        // Wait for notification from MPU6050 task (new motion data),
        // or timeout periodically to avoid getting stuck if notifications stop.
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(200));

        shared_sensor_data_t local_data = {0};
        bool alert_triggered = false;
        const char *alert_reason = NULL;
        
        // Đọc dữ liệu từ shared memory
        if (data_mutex != NULL && xSemaphoreTake(data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            local_data = shared_data;
            xSemaphoreGive(data_mutex);
            
            // PHÁT HIỆN MOTION EVENTS (Fall, Impact, Running, Tremor)
            if (local_data.mpu_valid) {
                const char *motion_event = detect_motion_events(&motion,
                    local_data.accel_x, local_data.accel_y, local_data.accel_z);
                
                if (motion_event != NULL) {
                    // Cooldown để tránh spam cảnh báo
                    if (motion_alert_cooldown == 0 || 
                        (xTaskGetTickCount() - motion_alert_cooldown) > pdMS_TO_TICKS(2000)) {
                        alert_triggered = true;
                        alert_reason = motion_event;
                        last_motion_alert = motion_event;
                        motion_alert_cooldown = xTaskGetTickCount();
                    }
                }
            }
        }
        
        // Điều khiển buzzer
        if (alert_triggered) {
            if (!last_alert_state) {
                // Log cảnh báo với thông tin chi tiết
                if (local_data.mpu_valid) {
                    float mag = calculate_magnitude(local_data.accel_x, 
                                                    local_data.accel_y, 
                                                    local_data.accel_z);
                    ESP_LOGW(TAG, "⚠️ CẢNH BÁO: %s | Accel: %.2fg", 
                             alert_reason, mag);
                } else {
                    ESP_LOGW(TAG, "⚠️ CẢNH BÁO: %s", alert_reason);
                }
                last_alert_state = true;
                alert_count = 0;

                // Lưu sự kiện cảnh báo vào history để hiển thị trên web
                if (alert_reason != NULL) {
                    add_alert_event(alert_reason);
                }
            }
            
            // Beep pattern: 3 beep ngắn, sau đó im lặng
            alert_count++;
            if (alert_count <= 3) {
                // Beep
                gpio_set_level(BUZZER_GPIO, 1);
                vTaskDelay(pdMS_TO_TICKS(BUZZER_BEEP_DURATION));
                gpio_set_level(BUZZER_GPIO, 0);
                vTaskDelay(pdMS_TO_TICKS(BUZZER_SILENCE_DURATION));
            } else if (alert_count >= 10) {
                // Reset sau 10 chu kỳ (khoảng 3 giây)
                alert_count = 0;
                vTaskDelay(pdMS_TO_TICKS(1000));  // Im lặng 1 giây
            } else {
                vTaskDelay(pdMS_TO_TICKS(100));
            }
        } else {
            if (last_alert_state) {
                ESP_LOGI(TAG, "✅ Cảnh báo đã tắt - Giá trị trở về bình thường");
                last_alert_state = false;
                alert_count = 0;
            }
            gpio_set_level(BUZZER_GPIO, 0);  // Tắt buzzer
            // Không cần delay lớn ở đây vì task đã chờ bằng Task Notification phía trên.
        }
    }
}

// ========================= HTTP SERVER TASK =========================
static void http_server_task(void *pvParameters)
{
    ESP_LOGI(TAG, "HTTP Server task started");
    
    // Wait for WiFi to be connected (wifi_init_sta blocks until connected)
    // Thêm delay để đảm bảo network stack hoàn toàn sẵn sàng
    vTaskDelay(pdMS_TO_TICKS(3000));  // Tăng delay lên 3 giây
    
    // Kiểm tra lại IP address trước khi start server
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (netif != NULL) {
        esp_netif_ip_info_t ip_info;
        if (esp_netif_get_ip_info(netif, &ip_info) == ESP_OK && ip_info.ip.addr != 0) {
            ESP_LOGI(TAG, "IP confirmed: " IPSTR, IP2STR(&ip_info.ip));
        } else {
            ESP_LOGW(TAG, "IP not ready yet, waiting...");
            vTaskDelay(pdMS_TO_TICKS(2000));  // Đợi thêm 2 giây
        }
    }
    
    // Start HTTP server
    ESP_LOGI(TAG, "Starting web server...");
    start_webserver();
    ESP_LOGI(TAG, "HTTP Server started successfully");
    
    // Keep task running (monitor server status periodically)
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(60000));  // Check every 60 seconds
        
        // Log server status with sensor data
        shared_sensor_data_t local_data = {0};
        if (data_mutex != NULL && xSemaphoreTake(data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            local_data = shared_data;
            xSemaphoreGive(data_mutex);
        }
        
        ESP_LOGI(TAG, "HTTP Server running - HR: %.1f BPM, SpO2: %.1f%%, Valid: %s", 
                 local_data.heart_rate, 
                 local_data.spo2,
                 local_data.valid ? "Yes" : "No");
    }
}
// ========================= APP MAIN =========================
void app_main(void)
{
    ESP_LOGI(TAG, "=================================");
    ESP_LOGI(TAG, "ESP32-C3 Health Monitor Starting");
    ESP_LOGI(TAG, "=================================");


    ESP_LOGI(TAG, "Initializing WiFi...");
    wifi_init_sta();



    data_mutex = xSemaphoreCreateMutex();

    // Init I2C
    if (i2c_bus_init() != ESP_OK) {
        ESP_LOGE(TAG, "I2C init failed");
        return;
    }

    ESP_LOGI(TAG, "Scanning I2C bus...");
    i2c_bus_scan();
    vTaskDelay(pdMS_TO_TICKS(500));

    // Init OLED
    ESP_LOGI(TAG, "Initializing SSD1306...");
    ssd1306_init();

    // Init MAX30100
    ESP_LOGI(TAG, "Initializing MAX30100...");
    max30100_init();
    vTaskDelay(pdMS_TO_TICKS(500));

    // Init MPU6050
    ESP_LOGI(TAG, "Initializing MPU6050...");
    esp_err_t mpu_ret = mpu6050_init();
    if (mpu_ret == ESP_OK) {
        ESP_LOGI(TAG, "MPU6050 initialized successfully");
    } else {
        ESP_LOGI(TAG, "MPU6050 not connected - system will continue without motion sensor");
    }
    vTaskDelay(pdMS_TO_TICKS(500));

    ESP_LOGI(TAG, "Sensors initialized — starting tasks...");

    // Tạo các tasks trước để chúng chạy ngay
    xTaskCreate(display_task, "display_task", 4096, NULL, 5, NULL);
    xTaskCreate(max30100_task, "max30100_task", 4096, NULL, 4, NULL);
    xTaskCreate(mpu6050_task, "mpu6050_task", 4096, NULL, 4, NULL);
    xTaskCreate(buzzer_task, "buzzer_task", 6144, NULL, 3, &buzzer_task_handle);  // Lưu handle để dùng Task Notification
    xTaskCreate(http_server_task, "http_server_task", 4096, NULL, 2, NULL);



    ESP_LOGI(TAG, "System running...");
}
