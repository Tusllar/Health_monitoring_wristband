# idf_component_register(SRCS "health_monitoring_wristband.c"
#                     INCLUDE_DIRS "."
#                     )
# main/CMakeLists.txt
idf_component_register(
    SRCS "health_monitoring_wristband.c"
    INCLUDE_DIRS "."
    REQUIRES 
        i2c_bus
        max30100
        mpu6050
        ssd1306
        esp32c3_wifi
        http_server_app
)