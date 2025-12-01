# components/max30100/CMakeLists.txt
idf_component_register(
    SRCS "max30100.c"
    INCLUDE_DIRS "include"
    REQUIRES i2c_bus esp_timer
)