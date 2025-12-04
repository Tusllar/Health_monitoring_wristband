## ESP32-C3 Health Monitoring Wristband

Dự án vòng tay giám sát sức khỏe thời gian thực sử dụng **ESP32-C3 Mini-1** và **ESP-IDF v5.5**, kết hợp:

- Đo **nhịp tim** & **SpO2** bằng **MAX30100**
- Đo **gia tốc 3 trục** & **con quay hồi chuyển 3 trục** bằng **MPU6050**
- Hiển thị tức thời trên **OLED SSD1306**
- Còi **buzzer (GPIO10)** cảnh báo té ngã/chấn động/rung tay
- Truy cập **web dashboard** (Chart.js + Three.js) để xem:
  - Biểu đồ nhịp tim & SpO2 theo thời gian
  - Trạng thái kết nối & thông tin thiết bị
  - **Mô phỏng 3D hướng cổ tay** dựa trên dữ liệu MPU6050
- Truy cập **App** được xây dựng từ dự án [Health_monitor_app](https://github.com/Tusllar/Health_monitor_app.git) để xem:
  - Biểu đồ nhịp tim & SpO2 theo thời gian
  - Trạng thái kết nối & thông tin thiết bị
---

## 1. Phần cứng

- **ESP32-C3 Mini-1**
- **MAX30100** – Cảm biến SpO2 và nhịp tim
- **MPU6050** – IMU 6 trục (Accelerometer + Gyroscope) + nhiệt độ
- **SSD1306** – Màn hình OLED 128x64 I2C
- **Buzzer** – điều khiển bởi **GPIO10** (nên qua transistor/driver nếu là còi tiêu thụ dòng lớn)

### 1.1. Kết nối I2C

Tất cả module dùng chung bus I2C:

| Module   | Địa chỉ I2C       | SDA   | SCL   |
|----------|-------------------|-------|-------|
| SSD1306  | 0x3C              | GPIO4 | GPIO5 |
| MAX30100 | 0x57              | GPIO4 | GPIO5 |
| MPU6050  | 0x68 (hoặc 0x69)  | GPIO4 | GPIO5 |

**Lưu ý:**

- Cần 2 điện trở pull-up ~4.7kΩ trên SDA và SCL.
- Nếu MPU6050 nối chân AD0 = HIGH thì địa chỉ có thể là **0x69** (được xử lý tự động trong `mpu6050.c`).
- Buzzer: chân tín hiệu nối **GPIO10**, chân còn lại qua transistor + nguồn; thêm diode bảo vệ nếu là loa từ.

---

## 2. Cấu trúc dự án

```text
health_monitoring_wristband/
├── CMakeLists.txt
├── sdkconfig
├── sdkconfig.defaults
├── README.MD                     # Tài liệu này
│
├── main/
│   ├── CMakeLists.txt
│   ├── health_monitoring_wristband.c  # Logic chính, FreeRTOS tasks
│   └── index.html                # Web dashboard (Chart.js + Three.js)
│
└── components/
    ├── i2c_bus/                  # Abstraction cho I2C
    │   ├── i2c_bus.c
    │   └── include/i2c_bus.h
    │
    ├── max30100/                 # Driver & xử lý tín hiệu MAX30100
    │   ├── max30100.c
    │   └── include/max30100.h
    │
    ├── mpu6050/                  # Driver MPU6050 (IMU)
    │   ├── mpu6050.c
    │   └── include/mpu6050.h
    │
    ├── ssd1306/                  # Driver màn hình OLED
    │   ├── ssd1306.c
    │   └── include/ssd1306.h
    │
    ├── esp32c3_wifi/             # Kết nối WiFi Station (Event Group)
    │   ├── esp32c3_wifi.c
    │   └── include/esp32c3_wifi.h
    │
    └── http_server_app/          # HTTP server + REST API /sensor
        ├── http_server_app.c
        └── include/http_server_app.h
```

---

## 3. Kiến trúc FreeRTOS & luồng dữ liệu

### 3.1. Các FreeRTOS object chính

- **Tasks:**
  - `max30100_task` – đọc nhịp tim & SpO2, xử lý tín hiệu, ghi vào `shared_data`.
  - `mpu6050_task` – đọc gia tốc/gyro/nhiệt độ, ghi vào `shared_data`, **xTaskNotifyGive()** cho `buzzer_task`.
  - `display_task` – đọc `shared_data` và vẽ thông tin lên OLED SSD1306.
  - `buzzer_task` – nhận notify từ `mpu6050_task`, chạy thuật toán phát hiện té ngã/chấn động/rung tay và điều khiển buzzer GPIO10.
  - `http_server_task` – khởi động HTTP server, kiểm tra định kỳ và log trạng thái web server + dữ liệu sensor.

- **Mutex / Semaphore:**
  - `data_mutex` – bảo vệ struct `shared_data` (dùng chung giữa tất cả tasks + HTTP server).
  - `i2c_mutex` (trong `i2c_bus.c`) – bảo vệ truy cập bus I2C cho MAX30100, MPU6050, SSD1306.

- **Event Group (WiFi):**
  - `s_wifi_event_group` (trong `esp32c3_wifi.c`) – bit:
    - `WIFI_CONNECTED_BIT` – đã kết nối WiFi & có IP.
    - `WIFI_FAIL_BIT` – kết nối thất bại sau số lần retry.

- **Task Notification:**
  - `buzzer_task_handle` – handle task của buzzer.
  - `mpu6050_task` gọi `xTaskNotifyGive(buzzer_task_handle)` mỗi khi có dữ liệu motion mới hợp lệ.
  - `buzzer_task` dùng `ulTaskNotifyTake(pdTRUE, timeout)` để ngủ và chỉ thức khi có notify (giảm polling CPU).

### 3.2. Dữ liệu chia sẻ (`shared_data`)

Định nghĩa tại `main/health_monitoring_wristband.c`:

```c
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
```

Được cập nhật bởi:

- `max30100_task`: `heart_rate`, `spo2`, `valid`, `last_update`.
- `mpu6050_task`: `accel_*`, `gyro_*`, `mpu_temp`, `mpu_valid`.

Được đọc bởi:

- `display_task`, `buzzer_task`, `http_server_task`.
- HTTP handler `/sensor` trong `components/http_server_app/http_server_app.c`.

---

## 4. Web Dashboard (main/index.html)

Frontend single-page:

- **Chart.js**: vẽ biểu đồ:
  - Heart Rate (BPM)
  - SpO2 (%)
- **Three.js**: mô phỏng 3D một “hộp” đại diện cổ tay, xoay theo vector gia tốc (ước lượng pitch/roll).
- **UI**:
  - Card Health Monitor: giá trị số HR, SpO2, trạng thái dữ liệu.
  - Card Performance: điều chỉnh tốc độ cập nhật (ms) và số điểm data trên chart.
  - Device Info: IP, thời gian cập nhật cuối.
  - Card Motion Monitor:
    - Bảng giá trị accel/gyro/temp.
    - Trạng thái MPU6050 (online/offline).
    - Khung 3D hiển thị orientation.

### 4.1. Endpoint `/sensor`

Được cung cấp bởi `http_server_app.c`, trả về JSON tương ứng với `shared_sensor_data_t`:

```json
{
  "heart_rate": 75.0,
  "spo2": 98.5,
  "valid": true,
  "last_update": 1234567,
  "accel": { "x": 0.01, "y": 0.02, "z": 0.98 },
  "gyro":  { "x": 0.1,  "y": -0.2, "z": 0.0  },
  "mpu_temp": 36.8,
  "mpu_valid": true
}
```

Trang web định kỳ gọi `fetch('/sensor')`:

- Cập nhật số HR, SpO2, trạng thái sensor.
- Push dữ liệu vào `heartRateData`, `spo2Data`, `timeLabels` để vẽ chart.
- Cập nhật `accel/gyro/temp` và xoay cube 3D tương ứng.
- Dùng `maxDataPoints` để giới hạn độ dài buffer trên trình duyệt (không lưu vĩnh viễn).

### 4.2. Endpoint `/alerts`

Được cung cấp bởi `http_server_app.c`, trả về JSON tương ứng với `alert_event_t`:

```json
{
  "alerts": [
    {
      "t": 89284,
      "msg": "VA ĐẬP - Chấn động mạnh!"
    },
    {
      "t": 100041,
      "msg": "VA ĐẬP - Chấn động mạnh!"
    },
    {
      "t": 110273,
      "msg": "ĐANG CHẠY - Hoạt động mạnh"
    },
    {
      "t": 122717,
      "msg": "VA ĐẬP - Chấn động mạnh!"
    }
  ]
}
```

Trang web định kỳ gọi `fetch('/alerts')`:

- Hiển thị danh sách số lần va đập, ngã,....

---

## 5. Build, Flash & Chạy

### 5.1. Yêu cầu

- ESP-IDF v5.5 (hoặc tương đương).
- Python 3.8+.
- USB driver cho ESP32-C3.

### 5.2. Build & Flash

```bash
# 1. Thiết lập môi trường ESP-IDF
. $HOME/esp/esp-idf/export.sh

# 2. Chọn target & cấu hình
idf.py set-target esp32c3
idf.py menuconfig

# 3. Build
idf.py build

# 4. Flash (thay /dev/ttyUSB0 bằng cổng tương ứng)
idf.py -p /dev/ttyUSB0 flash

# 5. Monitor log serial
idf.py -p /dev/ttyUSB0 monitor
```

### 5.3. Truy cập Web

1. ESP32-C3 kết nối WiFi theo cấu hình trong `esp32c3_wifi.c`  
   (SSID/password có thể hard-code hoặc cấu hình qua `menuconfig`).
2. Khi đã nhận IP, serial log sẽ in dạng `got ip: 192.168.x.y`.
3. Trên máy/điện thoại cùng mạng:
   - Mở trình duyệt tới: `http://<IP_ESP32_C3>/`
4. Dashboard sẽ tự động fetch `/sensor` và cập nhật theo thời gian thực.

---

## 6. Các component & API chính

### 6.1. `i2c_bus`

- Khởi tạo I2C master.
- API:
  - `i2c_bus_init()`
  - `i2c_bus_read_bytes()`, `i2c_bus_write_bytes()`
  - `i2c_bus_write_raw()` (dùng cho SSD1306)
  - `i2c_bus_scan()`
- Dùng `i2c_mutex` để thread-safe giữa nhiều task.

### 6.2. `max30100`

- API:
  - `max30100_init()`
  - `max30100_get_reading(max30100_reading_t *r)`
  - `max30100_set_mode()`
  - `max30100_reset_fifo()`
- Xử lý:
  - Lọc DC/AC, phát hiện đỉnh, tính BPM.
  - Tính SpO2 từ tỷ lệ AC/DC theo đường cong thực nghiệm.

Ví dụ:

```c
max30100_reading_t reading;
if (max30100_get_reading(&reading) == ESP_OK && reading.valid) {
    printf("SpO2: %.1f%%, HR: %.0f BPM\n",
           reading.spo2, reading.heart_rate);
}
```

### 6.3. `mpu6050`

- Tự dò địa chỉ (0x68/0x69) qua WHO_AM_I.
- API:
  - `mpu6050_init()`
  - `mpu6050_read_data(mpu6050_data_t *data)`
  - `mpu6050_set_gyro_range()`
  - `mpu6050_set_accel_range()`
- Trả về:
  - `accel_x/y/z` (g)
  - `gyro_x/y/z` (°/s)
  - `temperature` (°C)

Ví dụ:

```c
mpu6050_data_t data;
if (mpu6050_read_data(&data) == ESP_OK) {
    printf("Accel: [%.2f, %.2f, %.2f] g\n",
           data.accel_x, data.accel_y, data.accel_z);
}
```

### 6.4. `ssd1306`

- Frame buffer 1024 bytes, vẽ xong gọi `ssd1306_display()` để đẩy lên màn hình.
- API:
  - `ssd1306_init()`, `ssd1306_clear()`, `ssd1306_display()`
  - `ssd1306_draw_pixel()`, `ssd1306_draw_line()`, `ssd1306_draw_rect()`, `ssd1306_fill_rect()`
  - `ssd1306_draw_string()` – text font 5x7, nhiều kích thước.

### 6.5. `esp32c3_wifi`

- Kết nối WiFi chế độ station.
- Dùng Event Group để chờ kết quả kết nối.
- Đăng ký event cho `WIFI_EVENT` và `IP_EVENT`, log IP khi nhận được.

### 6.6. `http_server_app`

- Dựa trên `esp_http_server`.
- URI:
  - `GET /` → trả về nội dung `index.html` nhúng trong flash.
  - `GET /sensor` → đọc `shared_data` (qua `data_mutex`) và trả JSON.
  - `GET /alerts` → đọc `alert_data` và trả JSON.

---

---

## 7. Xử lý lỗi thường gặp

- **I2C Communication Failed**
  - Kiểm tra SDA/SCL, điện trở pull-up, nguồn 3.3V.
  - Gọi `i2c_bus_scan()` để xem thiết bị có xuất hiện trên bus hay không.

- **MAX30100 không có dữ liệu hợp lệ**
  - Đảm bảo ngón tay đặt đúng, chờ vài giây để ổn định.
  - Kiểm tra nguồn và mức sáng LED IR/Red.

- **MPU6050 WHO_AM_I sai**
  - Kiểm tra lại địa chỉ (0x68/0x69).
  - Kiểm tra wiring, đặc biệt chân AD0.

- **SSD1306 không hiển thị**
  - Kiểm tra kích thước (128x64 vs 128x32).
  - Xem lại init sequence trong `ssd1306_init()`.
  - Thử tăng contrast: `ssd1306_set_contrast(255)`.

---

## 8. License & Credits

- **License**: MIT – Tự do sử dụng cho dự án cá nhân và thương mại.
- Thiết kế cho **ESP32-C3 Mini-1 + ESP-IDF v5.5**, kiến trúc chia component rõ ràng, dễ mở rộng và tái sử dụng.


