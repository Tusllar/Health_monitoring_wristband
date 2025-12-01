# components/mpu6050/CMakeLists.txt
idf_component_register(
    SRCS "mpu6050.c"
    INCLUDE_DIRS "include"
    REQUIRES i2c_bus
)