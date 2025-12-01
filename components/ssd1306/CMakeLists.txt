
# components/ssd1306/CMakeLists.txt
idf_component_register(
    SRCS "ssd1306.c"
    INCLUDE_DIRS "include"
    REQUIRES i2c_bus freertos
)