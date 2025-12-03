#include "http_server_app.h"
#include <string.h>
#include "esp_log.h"
#include "esp_http_server.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "max30100.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <inttypes.h>

static const char *TAG = "HTTP_SERVER";
static httpd_handle_t server = NULL;

extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[] asm("_binary_index_html_end");

// External shared sensor data (from main)
// MUST match the definition in main/health_monitoring_wristband.c
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

extern shared_sensor_data_t shared_data;
extern SemaphoreHandle_t data_mutex;

// Alert history từ main (vòng đệm các cảnh báo buzzer)
#define ALERT_HISTORY_SIZE 20
typedef struct {
    uint32_t timestamp_ms;
    char message[64];
} alert_event_t;

extern alert_event_t alert_history[];
extern uint32_t alert_history_count;
extern uint32_t alert_history_head;

/* Serve index.html at root path */
static esp_err_t root_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, (const char *)index_html_start, index_html_end - index_html_start);
    return ESP_OK;
}

static const httpd_uri_t root = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = root_get_handler,
    .user_ctx  = NULL
};

/* MAX30100 Sensor handler - trả về dữ liệu theo cấu trúc max30100_reading_t */
static esp_err_t sensor_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "Sensor handler called");
    
    shared_sensor_data_t local_data = {0};
    
    // Đọc dữ liệu từ shared memory với mutex
    if (data_mutex != NULL && xSemaphoreTake(data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        local_data = shared_data;
        xSemaphoreGive(data_mutex);
        ESP_LOGI(TAG, "Data read: HR=%.1f, SpO2=%.1f, valid=%d", 
                 local_data.heart_rate, local_data.spo2, local_data.valid);
    } else {
        ESP_LOGW(TAG, "Failed to acquire mutex or mutex is NULL");
    }
    
    // Trả về JSON: dữ liệu MAX30100 + MPU6050
    // {
    //   "heart_rate": ...,
    //   "spo2": ...,
    //   "valid": true/false,
    //   "last_update": ...,
    //   "accel": {"x":...,"y":...,"z":...},
    //   "gyro": {"x":...,"y":...,"z":...},
    //   "mpu_temp": ...,
    //   "mpu_valid": true/false
    // }
    char resp[256];
    int len = snprintf(resp, sizeof(resp), 
             "{\"heart_rate\":%.1f,"
             "\"spo2\":%.1f,"
             "\"valid\":%s,"
             "\"last_update\":%" PRIu32 ","
             "\"accel\":{\"x\":%.3f,\"y\":%.3f,\"z\":%.3f},"
             "\"gyro\":{\"x\":%.3f,\"y\":%.3f,\"z\":%.3f},"
             "\"mpu_temp\":%.2f,"
             "\"mpu_valid\":%s}",
             local_data.heart_rate,
             local_data.spo2,
             local_data.valid ? "true" : "false",
             local_data.last_update,
             local_data.accel_x, local_data.accel_y, local_data.accel_z,
             local_data.gyro_x, local_data.gyro_y, local_data.gyro_z,
             local_data.mpu_temp,
             local_data.mpu_valid ? "true" : "false");
    
    if (len < 0 || len >= sizeof(resp)) {
        ESP_LOGE(TAG, "Failed to format JSON response");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    
    // Đảm bảo JSON hợp lệ - thêm CORS headers nếu cần
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store, no-cache, must-revalidate");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_send(req, resp, len);
    
    ESP_LOGI(TAG, "Response sent: %s (len=%d)", resp, len);
    return ESP_OK;
}

static const httpd_uri_t sensor = {
    .uri       = "/sensor",
    .method    = HTTP_GET,
    .handler   = sensor_handler,
    .user_ctx  = NULL
};

// Handler trả về danh sách cảnh báo (fall, impact, chạy, tremor, ...) để hiển thị trên web
static esp_err_t alerts_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "Alerts handler called");

    // Dùng buffer static để tránh dùng quá nhiều stack của task httpd
    static alert_event_t local_history[ALERT_HISTORY_SIZE];
    static char resp[1024];
    uint32_t count = 0;

    // Sao chép dữ liệu alert history dưới mutex để tránh race condition
    if (data_mutex != NULL && xSemaphoreTake(data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        uint32_t head = alert_history_head;
        count = alert_history_count;

        if (count > ALERT_HISTORY_SIZE) {
            count = ALERT_HISTORY_SIZE;
        }

        // Lấy theo thứ tự thời gian tăng dần (cũ → mới)
        for (uint32_t i = 0; i < count; i++) {
            uint32_t idx = (head + ALERT_HISTORY_SIZE - count + i) % ALERT_HISTORY_SIZE;
            local_history[i] = alert_history[idx];
        }

        xSemaphoreGive(data_mutex);
    }

    int len = snprintf(resp, sizeof(resp), "{\"alerts\":[");

    for (uint32_t i = 0; i < count && len > 0 && (size_t)len < sizeof(resp) - 1; i++) {
        int written = snprintf(resp + len, sizeof(resp) - len,
                               "%s{\"t\":%" PRIu32 ",\"msg\":\"%s\"}",
                               (i == 0) ? "" : ",",
                               local_history[i].timestamp_ms,
                               local_history[i].message);
        if (written < 0 || (size_t)written >= sizeof(resp) - len) {
            break;
        }
        len += written;
    }

    if ((size_t)len < sizeof(resp) - 2) {
        resp[len++] = ']';
        resp[len++] = '}';
        resp[len] = '\0';
    } else {
        // Không đủ buffer, đóng JSON đơn giản
        snprintf(resp, sizeof(resp), "{\"alerts\":[]}");
        len = strlen(resp);
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store, no-cache, must-revalidate");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_send(req, resp, len);

    ESP_LOGI(TAG, "Alerts response sent (count=%" PRIu32 ")", count);
    return ESP_OK;
}

static const httpd_uri_t alerts = {
    .uri       = "/alerts",
    .method    = HTTP_GET,
    .handler   = alerts_handler,
    .user_ctx  = NULL
};

void start_webserver(void)
{
    // Kiểm tra và log IP address
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (netif != NULL) {
        esp_netif_ip_info_t ip_info;
        if (esp_netif_get_ip_info(netif, &ip_info) == ESP_OK) {
            ESP_LOGI(TAG, "Network ready - IP: " IPSTR, IP2STR(&ip_info.ip));
        } else {
            ESP_LOGW(TAG, "Failed to get IP info");
        }
    } else {
        ESP_LOGW(TAG, "Network interface not found");
    }
    
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;
    config.max_open_sockets = 7;  // Tăng số lượng connections
    config.max_uri_handlers = 8;   // Tăng số lượng handlers

    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    esp_err_t ret = httpd_start(&server, &config);
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "HTTP server started successfully");
        ESP_LOGI(TAG, "Registering URI handlers");
        esp_err_t ret1 = httpd_register_uri_handler(server, &root);
        esp_err_t ret2 = httpd_register_uri_handler(server, &sensor);
        esp_err_t ret3 = httpd_register_uri_handler(server, &alerts);
        
        if (ret1 == ESP_OK && ret2 == ESP_OK && ret3 == ESP_OK) {
            ESP_LOGI(TAG, "URI handlers registered successfully");
            ESP_LOGI(TAG, "  - GET /");
            ESP_LOGI(TAG, "  - GET /sensor");
            ESP_LOGI(TAG, "  - GET /alerts");
            ESP_LOGI(TAG, "Server is ready to accept connections");
        } else {
            ESP_LOGE(TAG, "Failed to register URI handlers: root=%d, sensor=%d, alerts=%d",
                     ret1, ret2, ret3);
        }
    } else {
        ESP_LOGE(TAG, "Error starting server! Error code: %d", ret);
    }
}

void stop_webserver(void)
{
    if (server != NULL) {
        httpd_stop(server);
        server = NULL;
    }
}
