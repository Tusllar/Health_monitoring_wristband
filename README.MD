# ESP32-C3 Health Monitor

Dự án giám sát sức khỏe sử dụng ESP32-C3 Mini-1 với ESP-IDF v5.5.

## Phần cứng

- **ESP32-C3 Mini-1**
- **MAX30100**: Cảm biến SpO2 và nhịp tim
- **MPU6050**: Cảm biến gia tốc và con quay hồi chuyển
- **SSD1306**: Màn hình OLED 128x64

## Kết nối I2C

Tất cả module dùng chung bus I2C:

| Module | Địa chỉ I2C | SDA | SCL |
|--------|------------|-----|-----|
| SSD1306 | 0x3C | GPIO4 | GPIO5 |
| MAX30100 | 0x57 | GPIO4 | GPIO5 |
| MPU6050 | 0x68 | GPIO4 | GPIO5 |

**Lưu ý**: Cần điện trở pull-up 4.7kΩ trên SDA và SCL.

## Cấu trúc dự án

```
esp32_health_monitor/
├── CMakeLists.txt              # Root CMake
├── sdkconfig.defaults          # Cấu hình mặc định
├── README.md                   # File này
│
├── main/
│   ├── CMakeLists.txt
│   └── main.c                  # Code chính
│
└── components/
    ├── i2c_bus/               # Module quản lý I2C
    │   ├── CMakeLists.txt
    │   ├── i2c_bus.c
    │   └── include/
    │       └── i2c_bus.h
    │
    ├── max30100/              # Driver MAX30100
    │   ├── CMakeLists.txt
    │   ├── max30100.c
    │   └── include/
    │       └── max30100.h
    │
    ├── mpu6050/               # Driver MPU6050
    │   ├── CMakeLists.txt
    │   ├── mpu6050.c
    │   └── include/
    │       └── mpu6050.h
    │
    └── ssd1306/               # Driver SSD1306
        ├── CMakeLists.txt
        ├── ssd1306.c
        └── include/
            └── ssd1306.h
```

## Cài đặt

### Yêu cầu

- ESP-IDF v5.5
- Python 3.8+
- USB driver cho ESP32-C3

### Build và Flash

```bash
# 1. Set up ESP-IDF environment
. $HOME/esp/esp-idf/export.sh

# 2. Configure project
idf.py set-target esp32c3
idf.py menuconfig

# 3. Build
idf.py build

# 4. Flash
idf.py -p /dev/ttyUSB0 flash

# 5. Monitor
idf.py -p /dev/ttyUSB0 monitor
```

**Windows**: Thay `/dev/ttyUSB0` bằng `COM3` (hoặc COM port tương ứng)

## Tính năng

### Component i2c_bus
- Khởi tạo I2C master ở 100kHz
- Đọc/ghi byte và multi-byte
- Scan bus để phát hiện devices
- Timeout handling

### Component max30100
- Đo SpO2 (độ bão hòa oxy trong máu)
- Đo nhịp tim (BPM)
- Đọc nhiệt độ
- FIFO buffer management

### Component mpu6050
- Đọc gia tốc 3 trục (±2g mặc định)
- Đọc gyroscope 3 trục (±250°/s mặc định)
- Đọc nhiệt độ
- Digital Low Pass Filter

### Component ssd1306
- Vẽ pixel, line, rectangle
- Hiển thị text với font 5x7
- Frame buffer (1024 bytes)
- Điều chỉnh độ tương phản

## API Examples

### Đọc SpO2 và nhịp tim

```c
max30100_reading_t reading;
esp_err_t ret = max30100_get_reading(&reading);

if (ret == ESP_OK && reading.valid) {
    printf("SpO2: %.1f%%, HR: %.0f BPM\n", 
           reading.spo2, reading.heart_rate);
}
```

### Đọc IMU data

```c
mpu6050_data_t data;
esp_err_t ret = mpu6050_read_data(&data);

if (ret == ESP_OK) {
    printf("Accel: [%.2f, %.2f, %.2f]g\n",
           data.accel_x, data.accel_y, data.accel_z);
    printf("Gyro: [%.1f, %.1f, %.1f]°/s\n",
           data.gyro_x, data.gyro_y, data.gyro_z);
}
```

### Hiển thị text trên OLED

```c
ssd1306_clear();
ssd1306_draw_string(0, 0, "Hello ESP32", 
                   SSD1306_TEXT_SIZE_MEDIUM, true);
ssd1306_display();
```

## Xử lý lỗi thường gặp

### I2C Communication Failed

1. Kiểm tra kết nối SDA/SCL
2. Kiểm tra điện trở pull-up (4.7kΩ)
3. Kiểm tra nguồn 3.3V cho các module
4. Dùng `i2c_bus_scan()` để kiểm tra địa chỉ

### MAX30100 No Valid Data

- Đảm bảo ngón tay đặt chính xác lên sensor
- Chờ 5-10 giây để sensor ổn định
- Kiểm tra LED sáng (IR và Red)

### MPU6050 Wrong WHO_AM_I

- Địa chỉ I2C có thể là 0x69 nếu AD0 = HIGH
- Sửa `MPU6050_I2C_ADDR` trong `mpu6050.h`

### SSD1306 Nothing Displayed

- Kiểm tra kích thước màn hình (128x64 hoặc 128x32)
- Điều chỉnh init sequence trong `ssd1306_init()`
- Thử tăng contrast: `ssd1306_set_contrast(255)`

## Performance

- **Task priorities**: Display (5) > Sensors (4)
- **Update rates**: 
  - MAX30100: 10Hz
  - MPU6050: 2Hz  
  - Display: 1Hz
- **Memory usage**: ~50KB RAM

## License

MIT License - Tự do sử dụng cho dự án cá nhân và thương mại.

## Credits

Phát triển cho ESP32-C3 Mini-1 với ESP-IDF v5.5